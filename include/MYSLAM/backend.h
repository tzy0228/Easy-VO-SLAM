//  * 有单独优化线程，在Map更新时启动优化
//  * Map更新由前端触发

#ifndef MYSLAM_BACKEND_H
#define MYSLAM_BACKEND_H

#include "MYSLAM/common_include.h"
#include "MYSLAM/frame.h"
#include "MYSLAM/map.h"


namespace MYSLAM{

class Map;

//  后端
class Backend {

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Backend> Ptr;//智能指针

    /// 构造函数中启动优化线程并挂起
    Backend();

    // 设置左右目的相机，用于获得内外参
    void SetCameras(Camera::Ptr left , Camera::Ptr right){
        cam_left_=left;
        cam_right_=right;
    }

    // 设置地图，让backend自己的地图指针指向当前的地图，而不是对当前地图进行修改，不需要锁
    void  SetMap(std::shared_ptr<Map>map){
        map_=map;
    }

    // 关闭后端线程
    void Stop();

    // 触发地图更新，启动优化 （notify），
    // 主要应该由前端触发，当追踪点少时，添加关键帧并触发更新地图
    void UpdateMap();

private:

    // 后端线程
    void BackendLoop();

    // 对给定关键帧和路标点进行优化
    void Optimize(Map::KeyframesType &keyframes,Map::LandmarksType &landmarks);
    
    std::shared_ptr<Map>map_; //地图
    Camera::Ptr cam_left_ =nullptr , cam_right_=nullptr; // 左右目相机
    std::thread backend_thread_;//后端线程
    std::mutex data_mutex_;//线程锁，与std::unique_lock <std::mutex> lck(mtx)来对变量上锁

    // 线程同步的工具，用于实现线程间的条件变量等待和通知机制。   在多线程编程中，条件变量通常和互斥锁（std::mutex）一起使用，以避免死锁等问题
    //用于对运行中的线程进行管理，wait和notify
    std::condition_variable map_update_;

    // std::atomic 是模板类，一个模板类型为 T 的原子对象中封装了一个类型为 T 的值。
    // 原子类型对象不同线程同时访问不会产生数据竞争。
    std::atomic<bool> backend_running_;//后端是否没有上锁

};










}//namespace MYSLAM
#endif  // MYSLAM_BACKEND_H