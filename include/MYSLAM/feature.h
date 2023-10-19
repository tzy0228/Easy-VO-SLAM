//  feature类 含有自身的2d位置，是否异常，是否被左目相机提取

#pragma once
#ifndef MYSLAM_FEATURE_H
#define MYSLAM_FEATURE_H

#include "memory"
#include"opencv2/features2d.hpp"
#include"MYSLAM/common_include.h"


namespace MYSLAM{

struct Frame;
struct MapPoint;

// 2d特征点，三角化后会关联一个地图点
struct Feature
{
// 1.定义所需的参数
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    // 定义一个无符号类型的智能指针
        typedef  std::shared_ptr<Feature>Ptr;
      
        std::weak_ptr<Frame>frame_;// 持有该feature的frame
        std::weak_ptr<MapPoint>map_point_;// 关联地图点
        cv::KeyPoint position_;// 自身2d位置
        bool is_outlier_ =false;// 是否异常
        bool is_on_left_image_=true;//  是否被左目相机提取


// 2.定义构造函数
    public:
        Feature(){};
        // 构造函数
        Feature(std::weak_ptr<Frame>frame , const cv::KeyPoint &kp): frame_(frame),position_(kp){};
};
}

#endif  // MYSLAM_FEATURE_H