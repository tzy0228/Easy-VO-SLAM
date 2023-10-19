// Frame类含有id，位姿，图像，左右目特征点

#pragma once
#ifndef MYSLAM_FRAME_H
#define MYSLAM_FRAME_H

#include "MYSLAM/camera.h"
#include"MYSLAM/common_include.h"

namespace MYSLAM {

// 前向声明
struct MapPoint;
struct Feature;

// 开始定义Frame类
struct Frame
{
// 1.定义所需的参数
        public :
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW; // 用于在Eigen C++库中启用对齐内存分配
                typedef std::shared_ptr<Frame> Ptr; // 定义了一个shared_ptr类型的指针
                unsigned long id_ = 0;          // 该帧的id
                unsigned long keyframe_id_ = 0; // 该帧作为keyframe的id
                bool is_keyframe_ = true;       // 是否为关键帧
                double time_stamp_;             // 时间戳
                SE3 pose_;                      // TCW类型的pose
                std::mutex pose_mutex_;         // pose的数据锁
                Mat left_img_, right_img_;      // 该帧能看到的双目图像
                 // 该帧能看到的双目特征点(定义存放左图、右图特征点指针的容器)
                std::vector<std::shared_ptr<Feature>> features_left_;
                std::vector<std::shared_ptr<Feature>> features_right_;

 // 2.定义构造函数和一些成员函数
        public:
                Frame() {}
                // 构造函数，将各个参数初始化（输入id，时间戳，位姿，左右目图像）
                Frame(long id, double time_stamp, const SE3 &pose, const Mat &left, const Mat &right);

                // 取出帧的位姿，并保证线程安全
                SE3 Pose()
                {
                        std::unique_lock<std::mutex> lck(pose_mutex_);
                        return pose_;
        }

        // 设置帧的位姿，并保证线程安全
        void SetPose(const SE3 &pose){
            std::unique_lock <std::mutex> lck(pose_mutex_) ;
            pose_=pose;
        }

        // 设置关键帧并分配并键帧id
        void SetKeyFrame();

        // 工厂构建模式，分配id 
        static  std::shared_ptr<Frame>CreateFrame();
};
}

#endif  // MYSLAM_FRAME_H