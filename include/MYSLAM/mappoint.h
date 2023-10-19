// mappoint类包含 3d位置，被哪些feature观察

#pragma once
#ifndef MYSLAM_MAPPOINT_H
#define MYSLAM_MAPPOINT_H

#include"MYSLAM/common_include.h"

namespace MYSLAM{

struct Frame;
struct Feature;

// 地图点类，在三角化之后形成地图点
struct MapPoint
{
// 1.定义参数
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        typedef std::shared_ptr<MapPoint>Ptr;// 定义了一个shared_ptr类型的指针
        unsigned long id_ = 0;// ID
        Vec3 pos_= Vec3::Zero();// 3D位置
        bool is_outlier_=false;// 是否是外点
        std::mutex data_mutex_;// 数据锁
        std::list<std::weak_ptr<Feature>> observations_;//被哪些feature观察
        int observed_times_=0;//被观测到的次数


// 2.构造函数和其他函数
    public:
    // （1）无参构造
        MapPoint(){};
    // （2）有参构造，输入id，3d位置
        MapPoint(long id,  Vec3 position);

    // 取出地图点的位置，并保证线程安全
        Vec3 Pos(){
            std::unique_lock<std::mutex>lck(data_mutex_);
            return  pos_;
        };

    //  设置地图点的位置，并保证线程安全
        void SetPos (const Vec3 &pos){
            std::unique_lock<std::mutex>lck(data_mutex_);
            pos_=pos;
        }

    // 增加新的观测到这个路标点，并且特征点数量+1
    void AddObservation(std::shared_ptr<Feature> feature) {
        std::unique_lock<std::mutex> lck(data_mutex_);
        observations_.push_back(feature);
        observed_times_++;
    }

    //获取 观测到这个地图点的特征 
    std::list<std::weak_ptr<Feature>> GetObs()
    {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return observations_;
    }

    // 可能是异常点，也可能将要删除某个关键帧，所以要移除某个特征点，并且特征点数量-1
        void RemoveObservation(std::shared_ptr<Feature>feat);

     // 工厂构建模式，分配id
        static MapPoint::Ptr  CreateNewMappoint();
};
} // namespace MYSLAM

#endif  // MYSLAM_MAPPOINT_H