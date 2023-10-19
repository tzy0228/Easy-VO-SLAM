#include"MYSLAM/visual_odometry.h"
#include <chrono>
#include"MYSLAM/config.h"

namespace MYSLAM{

// 构造函数  参数是配置文件路径
VisualOdometry::VisualOdometry(std::string &config_path) : config_file_path_(config_path) {}

// 初始化，return true if success
bool VisualOdometry::Init(){
    //  1. 调用Config::SetParameterFile()函数读取配置文件，如果找不到对应文件就返回false。
    if ( Config::SetParameterFile(config_file_path_)==false)
    {
        return false;
    }else{
        LOG(INFO) << "Found file";
    }
    
    // 2. 通过Dataset::Init()函数设置相机的内参数，以及双目相机的外参数。
    dataset_=Dataset::Ptr(new Dataset(Config::Get<std::string>("dataset_dir")));
    LOG(INFO) << "1";
    CHECK_EQ(dataset_->Init(), true);
    LOG(INFO) << "2";
    // 3. 实例化并初始化frontend_，backend_，map_，viewer_四个对象。并且创建3个线程之间的联系，
    frontend_=Frontend::Ptr(new Frontend);
    backend_=Backend::Ptr(new Backend);
    map_=Map::Ptr(new Map);
    viewer_=Viewer::Ptr(new Viewer);


    frontend_->SetBackend(backend_);
    frontend_->SetMap(map_);
    frontend_->SetViewer(viewer_);
    frontend_->SetCameras(dataset_->GetCamera(0),dataset_->GetCamera(1));
 
    backend_->SetMap(map_);
    backend_->SetCameras(dataset_->GetCamera(0), dataset_->GetCamera(1));

    viewer_->SetMap(map_);
    return true;
}

// 开始运行 ：一直循环step函数
void VisualOdometry::Run(){
    while (1)
    {
        LOG(INFO) << "VO is running";
        if (Step()==false)
        {
           break;
        }
    }
    // 关闭后端线程
    backend_->Stop();

    // 关闭显示线程
    viewer_->Close();

    LOG(INFO) << "VO exit";
}

// Step() 函数: 其实也一直在循环两个函数 Dataset::NextFrame() 和 Frontend::AddFrame() 。 
bool VisualOdometry::Step(){

    // 从数据集里面读取一下帧图像,然后调用 Frame::CreateFrame() 函数为每一帧赋一个id号
    Frame::Ptr new_frame = dataset_->NextFrame();
    if (new_frame == nullptr) return false;

    // 调用AddFrame()函数：去根据前端状态变量FrontendStatus，运行不同的三个函数，StereoInit()，Track()和Reset()
    auto t1 = std::chrono::steady_clock::now();        //计时 时间戳t1
    bool success = frontend_->AddFrame(new_frame);
    auto t2 = std::chrono::steady_clock::now();     //计时 时间戳t2
    auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);   //计算用时 （t2-t1）
    LOG(INFO) << "VO cost time: " << time_used.count() << " seconds.";
    return success;
}

}//namespace MYSLAM