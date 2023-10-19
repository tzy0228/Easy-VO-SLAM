//map类与 frame、mappoint进行交互 ，维护一个被激活的关键帧和地图点
// 和地图的交互：前端调用InsertKeyframe和InsertMapPoint插入新帧和地图点，后端维护地图的结构，判定outlier/剔除等等

#pragma once
#ifndef MAP_H
#define MAP_H

#include "MYSLAM/common_include.h"
#include "MYSLAM/frame.h"
#include "MYSLAM/mappoint.h"

namespace MYSLAM{

// 开始定义Frame类
class Map{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    typedef std::shared_ptr<Map>Ptr;// 无符号指针
    // 为了方便查找，用哈希表的方式(容器)记录路标点、关键帧和被激活的关键帧，
    // 输入id可以在O(1)时间内找到
    typedef std::unordered_map<unsigned long,MapPoint::Ptr>LandmarksType;
    typedef std::unordered_map<unsigned long,Frame::Ptr>KeyframesType;
    
    // 无参构造
    Map(){}

    // 增加一个关键帧
    void InsertKeyFrame(Frame::Ptr frame);

    // 增加一个地图顶点
    void InsertMapPoint(MapPoint::Ptr map_point);

    // 获取所有地图点
    LandmarksType GetAllMapPoints()
    {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return landmarks_;
    }

    // 获取激活的地图点
    LandmarksType GetActiveMapPoints()
    {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return active_landmarks_;
    }

    // 获取激活关键帧
    KeyframesType GetActiveKeyFrames()
    {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return active_keyframes_;
    }

    // 清理map中观测数量为0的点
    void CleanMap();

private:
    //将旧的关键帧置于不活跃的状态
    void RemoveOldKeyframe();

    std::mutex data_mutex_;// 数据锁
    LandmarksType landmarks_;// 所有地图点
    LandmarksType active_landmarks_;// 被激活的地图点
    KeyframesType keyframes_;// 所有关键帧
    KeyframesType active_keyframes_;// 被激活的关键帧

    Frame::Ptr current_frame_ = nullptr;

    int num_active_keyframes_=7;// 激活的关键帧数量
};
}//namespace MYSLAM

#endif  // MAP_H