// 前端需要根据双目图像确定该帧的位姿
// 前端的处理逻辑：1.初始化 2.正常跟踪 3.追踪丢失
// 具体为：
// 1.初始化：根据左右目光流匹配寻找可以三角化的地图点，成功时建立初始地图
// 2.正常跟踪：前端计算上一帧的特征点到当前帧的光流，根据光流结果计算图像位姿（该阶段只使用左目，不用右目），
        //  且如果追踪的点较少，就判定当前帧为当前帧，并对其提取新的特征点，并三角化新的地图点，加入地图同时进行一次后端优化 
// 3.追踪丢失：重置前端系统，重新初始化

#pragma once
#ifndef MYSLAM_FRONTEND_H
#define MYSLAM_FRONTEND_H

#include <opencv2/features2d.hpp>
#include"MYSLAM/frame.h"
#include"MYSLAM/map.h"
#include "MYSLAM/common_include.h"

namespace MYSLAM{

class Backend;
class Viewer;

// 前端的三种处理逻辑:1.初始化 2.正常跟踪 3.追踪丢失
enum class FrontendStatus{INITING, TRACKING_GOOD, TRACKING_BAD, LOST};

// 前端类
class Frontend{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

// 参数：
    typedef std::shared_ptr<Frontend>Ptr;//智能指针

//函数 

    //构造函数
    Frontend();

    //获取前端状态
    FrontendStatus GetStatus()const {return status_;}

    // Set函数
    void SetMap(Map::Ptr map) {map_=map;}
    void SetBackend(std::shared_ptr<Backend> backend){ backend_=backend;}
    void SetViewer(std::shared_ptr<Viewer>viewer){viewer_=viewer;}
    void SetCameras(Camera::Ptr left , Camera::Ptr right ){
        camera_left_=left;
        camera_right_=right;
    }

    //  外部接口，添加一个帧并计算其定位结果
    bool AddFrame(Frame::Ptr frame);

    // 双目初始化
    bool StereoInit();

    // track 正常跟踪函数
    bool Track();

    // 重定位
    bool Reset();

    // 检测当前帧中左侧图像的特征并将关键点将保存在current_frame_中
    int DetectFeatures();

    // 右目光流追踪
    int FindFeaturesInRight();

    // 初始化地图
    bool BuildInitMap();

    //跟踪上一帧
    int TrackLastFrame(); 

    // 修正当前帧的位姿估计，并返回追踪成功点数量（在光流匹配成功基础上，更信任的点）
    int EstimateCurrentPose();

    //将当前帧插入关键帧
    bool InsertKeyframe();

    //将关键帧中特征点对应的地图点加入到观测容器中
    void SetObservationsForKeyFrame();

    // triangulate map points 三角化新的地图点
    int TriangulateNewPoints();


private:
// 参数：
    FrontendStatus status_ = FrontendStatus::INITING ;//将前端置于初始化状态

    Frame::Ptr current_frame_=nullptr;  // 当前帧
    Frame::Ptr  last_frame_=nullptr;// 上一帧
    Camera::Ptr camera_left_=nullptr; // 左相机
    Camera::Ptr camera_right_=nullptr;// 右相机
    Map::Ptr map_=nullptr; // 地图
    std::shared_ptr<Backend>backend_=nullptr; // 后端
    std::shared_ptr<Viewer>viewer_=nullptr;// 可视化

    SE3 relative_motion_;// 当前帧与上一帧的相对运动，用于估计当前帧pose初值
    int tracking_inliers_;// 用来判断插入关键帧的内点数量

    cv::Ptr<cv::GFTTDetector>gftt_;// GFTT角点检测器

    // params
    int num_features_ = 200;
    int num_features_init_ = 100;
    int num_features_tracking_ = 50;
    int num_features_tracking_bad_ = 20;
    int num_features_needed_for_keyframe_ = 80;
};

}//namespace MYSALM

#endif  // MYSLAM_FRONTEND_H