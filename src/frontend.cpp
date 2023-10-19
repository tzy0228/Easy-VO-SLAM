#include <opencv2/opencv.hpp>
#include"MYSLAM/algorithm.h"
#include "MYSLAM/g2o_types.h"

#include "MYSLAM/backend.h"
#include "MYSLAM/config.h"
#include "MYSLAM/feature.h"
#include "MYSLAM/frontend.h"

#include "MYSLAM/map.h"
#include "MYSLAM/viewer.h"

namespace MYSLAM{

// 构造函数 ：根据配置文件（Config类） 的参数来创建GFTT的特征点提取器。
    // num_features_是每帧最多提取的特征点数量，此外还保存一个参数num_features_init_，在后面的地图初始化中会用到
Frontend::Frontend() {

// 以下是 cv::GFTTDetector 的构造函数：
//     static Ptr<GFTTDetector> create( int maxCorners=1000, double qualityLevel=0.01, double minDistance=1,
//                                              int blockSize=3, bool useHarrisDetector=false, double k=0.04 );

//    其中，maxCorners 决定提取关键点最大个数；    qualityLevel 表示关键点强度阈值，只保留大于该阈值的关键点（阈值 = 最强关键点强度 * qualityLevel）；
//     minDistance 决定关键点间的最小距离；blockSize 决定自相关函数累加区域，也决定了 Sobel 梯度时窗口尺寸；
//     useHarrisDetector 决定使用 Harris 判定依据还是 Shi-Tomasi 判定依据；k 仅在使用 Harris 判定依据时有效；
    gftt_=
        cv::GFTTDetector::create(Config::Get<int>("num_features"),0.01, 20);
        num_features_init_=Config::Get<int>("num_features_init");
        num_features_= Config::Get<int>("num_features");
}

//  外部接口，添加一个帧并计算其定位结果
bool Frontend::AddFrame(MYSLAM::Frame::Ptr frame ){//用frame接收传入的new_frame
    current_frame_ = frame;//令当前帧等于上一行的frame
    // 判断跟踪状态
     switch(status_){
        // 如果状态为初始化,进入双目初始化
        case FrontendStatus::INITING: 
            StereoInit();
            break;

        // 跟踪状态好 或者  跟踪状态差，进入跟踪
        case FrontendStatus::TRACKING_GOOD:
        case FrontendStatus::TRACKING_BAD:
            Track();
            break;

        // 如果跟踪丢失, 重定位
        case FrontendStatus::LOST:
            Reset();
            break;
     }
    //对当前帧操作完毕，更新帧：令当前帧变为上一帧
    last_frame_ =current_frame_;
    return true;
}


//双目初始化
 //根据左右目之间的光流匹配，寻找可以三角化的地图点，成功时建立初始地图
bool Frontend::StereoInit(){
    //提取左目的GFTT特征点（数量）
    int num_features_left = DetectFeatures();
    // 右目光流追踪
    int num_coor_features=FindFeaturesInRight();

    //如果匹配到的特征点数量小于num_features_init_，默认100个，就返回false
    if (num_coor_features < num_features_init_)
    {
        return false;
    }

    //初始化地图
    bool build_map_success= BuildInitMap();

    //如果地图初始化成功就改前端状态为TRACKING_GOOD，并把当前帧和地图点传到viewer_线程里
    if (build_map_success)
    {
        status_=FrontendStatus::TRACKING_GOOD;
        //地图在可视化端的操作，添加当前帧并更新整个地图
        if (viewer_)
        {
            viewer_->AddCurrentFrame(current_frame_);
            viewer_->UpdateMap();
        }
        return true; //地图初始化成功，返回ture
    }
    return false; //地图初始化失败，返回false
}

/*提取左目的GFTT特征点（数量）
    //opencv中mask的作用就是创建感兴趣区域，即待处理的区域。
         通常，mask大小创建分为两步，先创建与原图一致，类型为CV_8UC1或者CV_8UC3的全零图（即黑色图）。如mask = Mat::zeros(image.size(),CV_8UC1); 
        然后用rect类或者fillPoly()函数将原图中待处理的区域（感兴趣区域）置为1。
*/
int Frontend::DetectFeatures(){
    cv::Mat mask(current_frame_->left_img_.size(),CV_8UC1, 255);
    //循环遍历当前帧上的左侧图像特征,并绘制边框
    for(auto feat : current_frame_->features_left_){
        /*
            void cv::rectangle(cv::InputOutputArray img, cv::Point pt1, cv::Point pt2, 
                                const cv::Scalar &color, int thickness = 1, int lineType = 8, int shift = 0)
            绘制一个简单的、厚的或向上填充的矩形。函数cv::rectangle绘制一个矩形轮廓或填充矩形，其两个相对的角分别为pt1和pt2。
            position_.pt————————>关键点坐标
            inline cv::Point2f::Point_(float _x, float _y)------>关键点上下左右10距离的矩形，color为0，就是黑色填充满
        */
       cv::rectangle(mask,feat->position_.pt-cv::Point2f(10,10),feat->position_.pt+cv::Point2f(10,10),0,CV_FILLED);
    }
    //建立一个关键点的vector
    std::vector<cv::KeyPoint>keypoints;

    //提取左图特征点 
    /*
        virtual void cv::Feature2D::detect(cv::InputArray image, std::vector<cv::KeyPoint> &keypoints, cv::InputArray mask = noArray())
        重载：
            检测图像(第一种变体)或图像集(第二种变体)中的关键点。
        参数：
            image – 图像.
            keypoints – The detected keypoints. In the second variant of the method keypoints[i] is a set of keypoints detected in images[i] .
                        检测到的关键点。在该方法的第二种变体中，keypoints[i]是在图像[i]中检测到的一组关键点。
            mask – Mask specifying where to look for keypoints (optional). It must be a 8-bit integer matrix with non-zero values in the region of interest.
                        指定在何处查找关键点的掩码(可选)。它必须是一个8位整数矩阵，在感兴趣的区域中具有非零值。
        GFTT角点
    */        
    gftt_->detect(current_frame_->left_img_,keypoints,mask);

    //关联current_frame_和kp，同时统计这两次检测到的特征点数量
    int cnt_detected = 0;
    for(auto &kp : keypoints){
        current_frame_->features_left_.push_back(
            Feature::Ptr(new Feature(current_frame_,kp))
        );
        cnt_detected++;
    }
    LOG(INFO) << "Detect " << cnt_detected << " new features";
    return cnt_detected;
}

// 右目光流追踪
int Frontend::FindFeaturesInRight(){
    //赋初值
    std::vector<cv::Point2f>kps_left,kps_right;
    //  这里把当前帧左目的特征点位置放到 kps_left 这个临时变量里面。
    //  如果当前特征点有在地图上有对应的点，那么将根据特征点的3D POSE和当前帧的位姿反求出特征点在当前帧的像素坐标。如果没有对应特征点，右目的特征点初值就是和左目一样。
    for(auto &kp : current_frame_->features_left_){
        kps_left.push_back(kp->position_.pt);
        auto mp =kp->map_point_.lock();//检查是否上锁
        // 如果当前特征点有在地图上有对应的点
        if (mp)
        {
             // use projected points as initial guess（使用投影点作为初始估计值）//tzy
            auto px=camera_right_->world2pixel(mp->Pos(),current_frame_->Pose());
            //存入右侧图像的关键点
            kps_right.push_back(cv::Point2f(px[0],px[1]));
        }
        //否则，使用左侧相机特征点坐标为初始值
        else{
            kps_right.push_back(kp->position_.pt);
        }
    }
    
    // 开始使用LK流来估计右侧图像中的点
    std::vector<uchar> status;
    Mat error;
        //opencv自带的光流跟踪函数
         /*
        CV_EXPORTS_W void calcOpticalFlowPyrLK( InputArray prevImg, InputArray nextImg,
                                        InputArray prevPts, InputOutputArray nextPts,
                                        OutputArray status, OutputArray err,
                                        Size winSize = Size(21,21), int maxLevel = 3,
                                        TermCriteria criteria = TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01),
                                        int flags = 0, double minEigThreshold = 1e-4 );
            Calculates an optical flow for a sparse feature set using the iterative Lucas-Kanade method with pyramids.
        */
    cv::calcOpticalFlowPyrLK(current_frame_->left_img_,
                             current_frame_->right_img_, kps_left, kps_right,
                             status, error,
                             cv::Size(11, 11), 3,
                             cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
                             cv::OPTFLOW_USE_INITIAL_FLOW); // 最后一个参数flag，使用初始估计，存储在nextPts中;如果未设置标志，则将prevPts复制到nextPts并将其视为初始估计

    // 统计匹配上的特征点个数，并存储
    int num_good_pts;
    for (size_t i = 0; i < status.size(); ++i)
    {
        // 为真表示光流追踪成功
        if (status[i])
        {
            cv::KeyPoint kp(kps_right[i], 7); // 右目关键点的直径为7
            Feature::Ptr feat(new Feature(current_frame_, kp));// 有一个关键点，就要有一个特征类
            feat->is_on_left_image_ = false;
            current_frame_->features_right_.push_back(feat);
            num_good_pts++;
        }
        else
        { // 光流追踪未成功，就放个空指针
            current_frame_->features_right_.push_back(nullptr);
        }
    }
    // 输出操作日志
    LOG(INFO) << "Find " << num_good_pts << " in the right image.";
    return num_good_pts; // 返回成功匹配的点对数量
}

// 使用单个图像中的双目相机构建初始地图
bool Frontend::BuildInitMap(){
    //设置左右两个相机的位置（获取pose）
    std::vector<SE3>poses{camera_left_->pose(),camera_right_->pose()};
    size_t cnt_init_landmarks= 0;    //设置地图标志初始值
    //循环左侧图像特征点 (获取 特征点归一化坐标)
    for (size_t i = 0; i < current_frame_->features_left_.size(); ++i)
    {
        //判断右侧图像是否有对应特征点，有则继续向下执行，没有就continue
        if (current_frame_->features_right_[i]== nullptr)    continue;
        // create map point from triangulation  (开始获取三角化所需的 特征点归一化坐标)
        std::vector<Vec3>points{
            camera_left_->pixel2camera(Vec2(current_frame_->features_left_[i]->position_.pt.x,current_frame_->features_left_[i]->position_.pt.y)),
            camera_right_->pixel2camera(   Vec2(current_frame_->features_right_[i]->position_.pt.x,current_frame_->features_right_[i]->position_.pt.y))};
        //新建一个三维0矩阵
        Vec3 pworld =Vec3::Zero();

        // 尝试对每一对匹配点进行三角化
        /*
            inline bool myslam::triangulation(const std::vector<SE3> &poses, std::vector<Vec3> points, Vec3 &pt_world)
            linear triangulation with SVD 线性三角测量
            参数:
                poses – poses,
                points – points in normalized plane
                pt_world – triangulated point in the world
            返回:
                true if success
        */
        if (triangulation(poses,points,pworld) &&pworld[2]>0)
        {
            //创建地图存储数据
            //创建一个新地图，用于信息更新
            auto new_map_point = MapPoint::CreateNewMappoint();
            new_map_point->SetPos(pworld);
        
             //将观测到的特征点加入新地图
             new_map_point->AddObservation(current_frame_->features_left_[i]);
             new_map_point->AddObservation(current_frame_->features_right_[i]);

             //当前帧的地图点指向新的地图点—>更新当前帧上的地图点
            current_frame_->features_left_[i]->map_point_ = new_map_point;
            current_frame_->features_right_[i]->map_point_ = new_map_point;

             //初始化地图点makr+1
             cnt_init_landmarks++;
             //在地图中插入当前新的更新点
             map_->InsertMapPoint(new_map_point);
        }
    }
    //把初始化的这一帧设置为关键帧
    current_frame_->SetKeyFrame();
    //在地图中插入关键帧
    map_->InsertKeyFrame(current_frame_);

    //后端更新地图（在后端更新）
    backend_->UpdateMap();

    //输出操作日志
    LOG(INFO) << "Initial map created with " << cnt_init_landmarks
              << " map points";
    //返回：单个图像构建初始地图成功
    return true;
}

//正常状态下的跟踪
bool Frontend::Track(){
    // 用上一帧位姿获得当前位姿估计的初始值
    //  假设匀速运动，即每两帧之间的位姿变换差不多，
    //  以【上一帧位姿】乘以【上上帧与上一帧之间的相对运动】作为这次位姿的估计初值
    if (last_frame_)
    {
        current_frame_->SetPose(relative_motion_ * last_frame_->Pose());
    }
    //用光流法匹配上一帧与当前帧，并返回光流法匹配到的点的数量
    int num_track_last= TrackLastFrame();

    // 修正当前帧的位姿估计，并返回追踪成功点数量（在光流匹配成功基础上，更信任的点）
    tracking_inliers_=EstimateCurrentPose();

     // 改变状态，为下次的添加帧做准备
        // tracking good
    if (tracking_inliers_>num_features_tracking_)
    {
        status_=FrontendStatus::TRACKING_GOOD;
    }
        // tracking bad
    else if (tracking_inliers_>num_features_tracking_bad_)
    {
        status_=FrontendStatus::TRACKING_BAD;
    }else  // lost
    {
        status_=FrontendStatus::LOST;
    }
    
    //将当前帧插入关键帧
    InsertKeyframe();
    
    //当前帧位姿 =  上一帧的位姿 左乘 当前帧与上一帧的相对变换
    //相对位姿变换 = 当前帧的位姿 * 上一帧位姿的逆
    relative_motion_=current_frame_->Pose() * last_frame_->Pose().inverse();

    if (viewer_)
    {
        viewer_->AddCurrentFrame(current_frame_);
        return true;
    }
}

//用光流法匹配上一帧与当前帧，并返回光流法匹配到的点的数量
int Frontend::TrackLastFrame(){
    // 赋初值
    std::vector<cv::Point2f>  kps_last,  kps_current;
    for(auto &kp : last_frame_->features_left_){
        // 如果这个点有对应的路标点，则当前帧的光流法初值为路标点对当前帧位姿（初值）的投影
        if (kp->map_point_.lock())
        {
            auto mp = kp->map_point_.lock();//检查是否上锁
            // 将世界坐标系中的坐标 投影 到像素坐标系上 //tzy
            auto px = camera_left_->world2pixel(mp->Pos(),current_frame_->Pose());
            //关键帧的上一个容器中推入匹配到的关键点
            kps_last.push_back(kp->position_.pt);
            //当前帧中推入对应的像素坐标系上的坐标
            kps_current.push_back(cv::Point2f(px[0],px[1]));
        }
        //没有上锁，直接推入对应的地图点
        else{
            kps_last.push_back(kp->position_.pt);
            kps_current.push_back(kp->position_.pt);
        }
    }
    // 光流追踪
    std::vector<uchar> status;// 设置一个状态的容器
    Mat error; //设置一个error矩阵
    cv::calcOpticalFlowPyrLK(last_frame_->left_img_,
                             current_frame_->left_img_, kps_last, kps_current,
                             status, error,
                             cv::Size(11, 11), 3,
                             cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
                             cv::OPTFLOW_USE_INITIAL_FLOW); // 最后一个参数flag，使用初始估计，存储在nextPts中;如果未设置标志，则将prevPts复制到nextPts并将其视为初始估计

    //统计匹配上的特征点个数，并存储
    int num_good_pts = 0;
    for (size_t i = 0; i < status.size(); ++i)
    {
        //如果匹配到对应的流则个数加一；并指针继续向后指
        if (status[i])
        {
            cv::KeyPoint kp(kps_current[i],7);  // 关键点代表的区域直径大小为7
            Feature::Ptr feature(new Feature(current_frame_,kp));  // 有一个关键点，就要有一个特征类
            feature->map_point_=last_frame_->features_left_[i]->map_point_;// 关键点对应的地图点就是上一帧的点对应的地图点
            current_frame_->features_left_.push_back(feature);// 填充当前帧的左图特征点容器 
            num_good_pts++;// 匹配成功点计数
        }
    }
    // 匹配成功的点数量记录到日志，并返回
    LOG(INFO) << "Find " << num_good_pts << " in the last image.";
    return num_good_pts;
}
    
/*估计当前帧位姿；与backend.cpp中的Optimize中的g2o优化相似，区别在于此处无需优化，只是估计出来，新建一些信息
 *返回计算成功匹配的数量
 *https://blog.csdn.net/wujianing_110117/article/details/116253914
 */
int Frontend::EstimateCurrentPose(){
// setup g2o（g2o过程）     图优化：单节点+多条一元边
    // 设定g2o
    typedef ::g2o::BlockSolver_6_3 BlockSolverType;//pose 是 6 ，mappoint是 3
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>  LinearSolverType;    //线性求解器类型

    // 块求解器BlockSolver
    auto solver = new g2o::OptimizationAlgorithmLevenberg (
        g2o::make_unique<BlockSolverType>( g2o::make_unique<LinearSolverType>()));      // 选择梯度下降法
    g2o::SparseOptimizer optimizer;  //稀疏求解
    optimizer.setAlgorithm(solver);  //设置求解器

    // vertex（优化量 顶点）
    VertexPose *vertex_pose= new VertexPose();// camera vertex_pose
    vertex_pose->setId(0);// 定义节点编号
    vertex_pose->setEstimate(current_frame_->Pose());//设置初值
    optimizer.addVertex(vertex_pose);//把节点添加到图中

    // K
    Mat33 K = camera_left_->K();

    // edges（约束 边）（每对匹配点就是一个边）
    int index =1;
    std::vector<EdgeProjectionPoseOnly*>edges;
    std::vector<Feature::Ptr>features;

    //遍历每一对点
    for (size_t i = 0; i < current_frame_->features_left_.size(); ++i)
    {
        auto mp = current_frame_->features_left_[i]->map_point_.lock();
        if (mp)
        {
            features.push_back(current_frame_->features_left_[i]);
            EdgeProjectionPoseOnly *edge = new EdgeProjectionPoseOnly(mp->pos_,K);
            edge->setId(index);
            edge->setVertex(0,vertex_pose);// 设置连接的顶点
            edge->setMeasurement(toVec2(current_frame_->features_left_[i]->position_.pt));//传入观测值
            edge->setInformation(Eigen::Matrix2d::Identity());// 信息矩阵
            edge->setRobustKernel(new g2o::RobustKernelHuber); //鲁棒核函数
            edges.push_back(edge);
            optimizer.addEdge(edge);
            index++;
        }
    }

    // estimate the Pose the determine the outliers
    // 要思路是首先定义误差（误差要归一化，也就是去量纲化），然后选择置信度，一般为95%，如果这个特征点的误差在区间内我们认为是内外，否则是外点
    const double chi2_th = 5.991;
    int cnt_outlier = 0;
    for (int iteration = 0; iteration < 4; ++iteration) {
        vertex_pose->setEstimate(current_frame_->Pose());
        optimizer.initializeOptimization();// 设置优化初始值
        optimizer.optimize(10);// 设置优化次数
        cnt_outlier = 0;

        // count the outliers
        for (size_t i = 0; i < edges.size(); ++i) {
            auto e = edges[i];
            if (features[i]->is_outlier_) {
                e->computeError();
            }
            if (e->chi2() > chi2_th) {
                //设置等级  一般情况下g2o只处理level = 0的边，设置等级为1，下次循环g2o不再优化这个边
                features[i]->is_outlier_ = true;
                e->setLevel(1);
                cnt_outlier++;
            } else {
                features[i]->is_outlier_ = false;
                e->setLevel(0);
            };
            // 在第三次迭代之后，它会将所有边的鲁棒核函数设置为nullptr，以便在后续的迭代中不再考虑outlier
            if (iteration == 2) {
                e->setRobustKernel(nullptr);
            }
        }
    }

    LOG(INFO) << "Outlier/Inlier in pose estimating: " << cnt_outlier << "/"
              << features.size() - cnt_outlier;

    // Set pose and outlier
    current_frame_->SetPose(vertex_pose->estimate());   

    LOG(INFO) << "Current Pose = \n" << current_frame_->Pose().matrix();

    for (auto &feat : features) {
        if (feat->is_outlier_) {
            feat->map_point_.reset();
            feat->is_outlier_ = false;  // maybe we can still use it in future
        }
    }
    return features.size() - cnt_outlier;
}

//将当前帧插入关键帧
bool Frontend::InsertKeyframe(){
    // 如果追踪到的点很多（说明两帧差异不大），则不需要插入关键帧
    if (tracking_inliers_>num_features_needed_for_keyframe_)
    {
        return false;
    }
    // 如果追踪到的点很少（说明两帧差异较大），判定该帧为关键帧
    // 当前帧为关键帧并分配关键帧id
    current_frame_->SetKeyFrame();
    // 地图中插入当前帧
    map_->InsertKeyFrame(current_frame_);

    LOG(INFO) << "Set frame " << current_frame_->id_ << " as keyframe "
              << current_frame_->keyframe_id_;

    // 将关键帧中特征点对应的地图点加入到观测容器中
    SetObservationsForKeyFrame();

    // 关键帧重新提取特征点
    DetectFeatures();

    // track in right image 在关键帧的右目图像中找与左目对应的特征点
    FindFeaturesInRight();

    // triangulate map points 三角化新的地图点
    TriangulateNewPoints();

    // update backend because we have a new keyframe更新地图,触发后端优化
    backend_->UpdateMap();

    // 可视化模块中更新视图
    if (viewer_)
    {
        viewer_->UpdateMap();
    }
}

// 将关键帧中特征点对应的地图点加入到观测容器中
void Frontend::SetObservationsForKeyFrame(){
    for(auto &feat : current_frame_->features_left_){
        auto mp = feat->map_point_.lock();
        if (mp)
        {
            mp->AddObservation(feat);
        }
    }
}

// triangulate map points 三角化新的地图点
int Frontend::TriangulateNewPoints(){
    // 1.左右目位姿（pose）
    std::vector<SE3>poses{camera_left_->pose(),camera_right_->pose()};
    SE3 current_pose_Twc=current_frame_->Pose().inverse();
    int cnt_triangulated_pts = 0;
     for (size_t i = 0; i < current_frame_->features_left_.size(); ++i)
     {
        //expired()函数用于检查智能指针指向的对象是否为空   ,expired()==true  存储空指针
        if (current_frame_->features_left_[i]->map_point_.expired() && current_frame_->features_right_[i] !=nullptr)
        {
            // 左图的特征点未关联地图点且存在右图匹配点
    //  2.左右目匹配点（point）
            std::vector<Vec3>points{
                camera_left_->pixel2camera(
                    Vec2(current_frame_->features_left_[i]->position_.pt.x, current_frame_->features_left_[i]->position_.pt.y)),
                camera_right_->pixel2camera(
                    Vec2(current_frame_->features_right_[i]->position_.pt.x,current_frame_->features_right_[i]->position_.pt.y))
            };
   //  3 .pworld
            Vec3 pworld = Vec3::Zero();

            // 三角化成功并且深度为正
             if (triangulation(poses,points,pworld) && pworld[2]>0)
             {
                //创建一个新地图，用于信息更新
                auto new_map_point = MapPoint::CreateNewMappoint();
                //三角化得到的坐标是左目相机系坐标,这里和初始化中的triangulation区分： 
                            // 这里计算出的pworld是相机系坐标，左乘Twc才是世界系坐标, 而初始化是第一帧，一般以第一帧为世界系，因此得到的pworld直接当世界系坐标使用
                pworld = current_pose_Twc * pworld;
                new_map_point->SetPos(pworld);
                //将观测到的特征点加入新地图
                new_map_point->AddObservation(current_frame_->features_left_[i]);
                new_map_point->AddObservation(current_frame_->features_right_[i]);
                //为特征类Feature对象填写地图点成员
                current_frame_->features_left_[i]->map_point_=new_map_point;
                current_frame_->features_right_[i]->map_point_=new_map_point;
                //对Map类对象来说，地图里面应当多了一个地图点，所以要将这个地图点加到地图中去
                map_->InsertMapPoint(new_map_point);
                cnt_triangulated_pts++;
             }
        }
     }

    LOG(INFO) << "new landmarks: " << cnt_triangulated_pts;
    return cnt_triangulated_pts;
}

// 重定位
bool Frontend::Reset() {
    LOG(INFO) << "Reset is not implemented. ";
    return true;
}

}//namespace MYSLAM