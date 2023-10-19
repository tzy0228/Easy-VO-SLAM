//  VO 对外接口
// 1.读取配置文件 
// 2.通过Dataset::Init()函数设置相机的内参数，以及双目相机的外参数 
// 3.实例化并初始化frontend_，backend_，map_，viewer_四个对象。并且创建3个线程之间的联系

#pragma once
#ifndef MYSLAM_VISUAL_ODOMETRY_H
#define MYSLAM_VISUAL_ODOMETRY_H

#include"MYSLAM/frontend.h"
#include"MYSLAM/backend.h"
#include"MYSLAM/viewer.h"
#include"MYSLAM/common_include.h"
#include"MYSLAM/dataset.h"

namespace MYSLAM{

//VO类
class VisualOdometry{

public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW; //内存对齐
    typedef std::shared_ptr<VisualOdometry>Ptr;  //智能指针

    // 构造函数:   参数是配置文件路径
    VisualOdometry(std::string &config_path); 

    // 初始化，return true if success
    bool Init();

    // 开始运行 start vo in the dataset
    void Run();

    // 从dataset开始逐帧读取、运行
    bool Step();

    // 获取前端状态
    FrontendStatus  GetFrontendStatus() const {return frontend_->GetStatus();}

private:
    bool inited_=false;// 是否初始化
    std::string config_file_path_;// 参数文件路径   

    //实例化并初始化frontend_，backend_，map_，viewer_四个对象
    Frontend::Ptr frontend_=nullptr;
    Backend::Ptr backend_=nullptr;
    Map::Ptr map_=nullptr;
    Viewer::Ptr viewer_=nullptr;

    //实例化并初始化dataset对象
    Dataset::Ptr dataset_=nullptr;
};


}//namespace MYSLAM

#endif  // MYSLAM_VISUAL_ODOMETRY_H