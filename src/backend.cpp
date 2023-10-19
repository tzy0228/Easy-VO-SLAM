#include "MYSLAM/backend.h"
#include "MYSLAM/algorithm.h"
#include "MYSLAM/feature.h"
#include "MYSLAM/g2o_types.h"
#include "MYSLAM/map.h"
#include "MYSLAM/mappoint.h"

namespace MYSLAM{

// 构造函数 启动优化线程并挂起
Backend::Backend(){
    // 创建一个线程，线程执行的函数是BackendLoop，并将this绑定到函数，
    // 即这是this指向的类的成员函数
    backend_running_.store(true);// 设置原子类型值
    backend_thread_ = std::thread(std::bind(&Backend::BackendLoop, this));//上锁,唤醒一个wait的线程
}

// 关闭后端线程
void Backend::Stop(){
    // backend_running标志为false，唤醒一个wait的线程，等待后端线程结束
    backend_running_.store(false);
    map_update_.notify_one();    // 后端结束时最后一次更新地图
    backend_thread_.join();
}

// 触发地图更新，启动优化
void Backend::UpdateMap(){
    std::unique_lock<std::mutex> lock(data_mutex_);   
    map_update_.notify_one(); // 唤醒一个正在等待的线程
}

// 后端线程
void Backend::BackendLoop(){
    // load读取backend_running的值
    // 实际上当后端在运行时，这是一个死循环函数，但是会等待前端的激活,即前端激活一次，就运行此函数，进行一次后端优化
    while (backend_running_.load())// 读取原子类型值
    {
        std::unique_lock<std::mutex>lock(data_mutex_);
        map_update_.wait(lock);//锁住当前线程
    }

    // 后端仅优化激活的Frames和mappoints
    Map::KeyframesType active_kfs = map_->GetActiveKeyFrames();    // 获取激活关键帧
    Map::LandmarksType active_landmarks =map_->GetActiveMapPoints();    // 获取激活地图点

    Optimize(active_kfs,active_landmarks); //执行优化
}

//后端优化函数
void Backend::Optimize(Map::KeyframesType &keyframes, Map::LandmarksType &landmarks){
    // 设定g2o
    typedef::g2o::BlockSolver_6_3 BlockSolverType;
    typedef::g2o::LinearSolverCSparse<BlockSolverType::PoseMatrixType>LinearSolverType; //线性求解器类型

     // 块求解器BlockSolver
    auto solver = new g2o::OptimizationAlgorithmLevenberg (
        g2o::make_unique<BlockSolverType>( g2o::make_unique<LinearSolverType>()));      // 选择梯度下降法
    g2o::SparseOptimizer optimizer;  //稀疏求解
    optimizer.setAlgorithm(solver);  //设置求解器

    // vertex（优化量 顶点）
    // pose 顶点 使用Keyframe id
    std::map<unsigned long ,VertexPose *>vertices;    // 定义了一个名为vertices的std::map，它的键是unsigned long类型，值是指向位姿VertexPose对象的指针。
    unsigned long max_kf_id = 0;
    //  遍历关键帧,确定第一个顶点
    for(auto &keyframe : keyframes){
        auto kf=keyframe.second;

        VertexPose *vertex_pose=new VertexPose();
        vertex_pose->setId(kf->keyframe_id_);// 定义节点编号
        vertex_pose->setEstimate(kf->Pose());//设置初值
        optimizer.addVertex(vertex_pose);//把节点添加到图中

        if (kf->keyframe_id_>max_kf_id)
        {
            max_kf_id=kf->keyframe_id_;
        }   
        vertices.insert({kf->keyframe_id_,vertex_pose});
    }

    // 路标顶点，使用路标id索引
    std::map<unsigned long, VertexXYZ *> vertices_landmarks;

    // 内参和左右外参
    Mat33 K=cam_left_->K();
    SE3 left_ext =cam_left_->pose();
    SE3 right_ext =cam_right_->pose();

    // edge边
    int index =1 ;
    double chi2_th = 5.991;  // robust kernel 阈值
    std::map<EdgeProjection*,Feature::Ptr>edges_and_features;
    // 每一个landmark均需要建立一条边，所以landmark vertex的定义与edge的定义同步进行
    // 遍历地图点，取出观测到该路标点的特征
    for(auto &landmark : landmarks){
        if (landmark.second->is_outlier_)         //外点不优化
        {
            continue;
        }
        unsigned long landmark_id = landmark.second->id_;        //路标点id
        auto observations=landmark.second->GetObs();// 取出观测到该路标点的特征
        // 再对特征进行遍历，得到该特征所处的帧
        for(auto obs: observations){
            if (obs.lock()=nullptr)
            {
                continue;
            }
            auto feat=obs.lock();
            if (feat->is_outlier_ || feat->frame_.lock() ==nullptr)
            {
                continue;
            }
            auto frame=feat->frame_.lock();
            EdgeProjection *edge=nullptr;

            // 提供内参矩阵K，和初始化的初值
            if (feat->is_on_left_image_)
            {
                edge=new EdgeProjection(K,left_ext);
            }else
            {
                edge=new EdgeProjection(K,right_ext);
            }
            
            // 如果landmark还没有被加入优化，则新加一个顶点
            if (vertices_landmarks.find(landmark_id) == vertices_landmarks.end())
            {
                VertexXYZ *v = new VertexXYZ;
                v->setId(landmark_id + max_kf_id + 1);// 定义节点编号
                v->setEstimate(landmark.second->Pos());//设置初值
                v->setMarginalized(true); //边缘化
                vertices_landmarks.insert({landmark_id, v});
                optimizer.addVertex(v);//把节点添加到图中
            }
            
            // 设置edge的参数
            edge->setId(index);
            edge->setVertex(0,vertices.at(frame->keyframe_id_));// 设置连接的顶点:pose
            edge->setVertex(1, vertices_landmarks.at(landmark_id));  // 设置连接的顶点:landmark
            edge->setMeasurement(toVec2(feat->position_.pt));//传入观测值
            edge->setInformation(Mat22::Identity());// 信息矩阵
            //鲁棒核函数
            auto rk =new g2o::RobustKernelHuber(); 
            rk->setDelta(chi2_th);
            edge->setRobustKernel(rk);

            edges_and_features.insert({edge, feat});

            optimizer.addEdge(edge);//把边添加到图中
            index++;
        }
    }

    // do optimization and eliminate the outliers
    // 执行优化并去除外点
    optimizer.initializeOptimization();// 设置优化初始值
    optimizer.optimize(10);// 设置优化次数
    
    //设置内点和外点数量 
    int cnt_outlier = 0, cnt_inlier = 0;
    int iteration = 0;

    while (iteration<5)
    {
        cnt_outlier = 0;
        cnt_inlier = 0;
        // 统计内点和外点
        for(auto &ef : edges_and_features){
            if (ef.first->chi2()>chi2_th)
            {
                cnt_outlier++;
            }else{
                cnt_inlier++;
            }
        }
        //确保内点占1/2以上，否则调整阈值*2，直到迭代结束
        double inlier_ratio = cnt_inlier/ double(cnt_inlier + cnt_outlier);
        if (inlier_ratio>0.5)
        {
            break;
        }else{
            chi2_th *= 2;
             iteration++;
        }
    }

    // 记录是否为异常特征
    for(auto &ef : edges_and_features){
        if (ef.first->chi2()>chi2_th){
            ef.second->is_outlier_=true;  //记为为异常特征
            // remove the observation
            ef.second->map_point_.lock()->RemoveObservation(ef.second);
        }else{
            ef.second->is_outlier_=false;//记为为正常特征
        }
    }

    LOG(INFO) << "Outlier/Inlier in optimization: " << cnt_outlier << "/"<< cnt_inlier;

    // Set pose and lanrmark position
    for(auto &v: vertices){
        keyframes.at(v.first)->SetPose(v.second->estimate());
    }
    for (auto &v : vertices_landmarks) {
        landmarks.at(v.first)->SetPos(v.second->estimate());
    }
}

}//name MYSLAM