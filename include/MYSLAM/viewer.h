#ifndef MYSLAM_VIEWER_H
#define MYSLAM_VIEWER_H

#include <thread>
#include <pangolin/pangolin.h>

#include "MYSLAM/common_include.h"
#include "MYSLAM/frame.h"
#include "MYSLAM/map.h"

namespace MYSLAM{

class Viewer{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Viewer> Ptr;


    // 构造函数
    Viewer() ;

    // 设置地图
    void SetMap(Map::Ptr map) { map_ = map; }

    // 关闭显示线程
    void Close();

    // 添加当前帧
    void AddCurrentFrame(Frame::Ptr current_frame);

    // 更新地图
    void UpdateMap();
    

private:

    void ThreadLoop();//显示线程函数

    void DrawFrame(Frame::Ptr frame, const float* color); //画出帧

    void FollowCurrentFrame(pangolin::OpenGlRenderState& vis_camera); //follow camera

    void DrawMapPoints(); //画出地图点


    // plot the features in current frame into an image
    cv::Mat PlotFrameImage();

    Frame::Ptr current_frame_ = nullptr; //当前帧
    Map::Ptr map_ = nullptr;//地图

    std::thread viewer_thread_; //显示线程
    bool viewer_running_ = true; //显示线程是否在运行
    std::mutex viewer_data_mutex_ ; //数据锁

    std::unordered_map<unsigned long, Frame::Ptr> active_keyframes_; //活跃关键帧
    std::unordered_map<unsigned long, MapPoint::Ptr> active_landmarks_; //活跃地图点
    bool map_updated_ = false;//地图是否更新完毕


};











}//namespace MYSLAM


#endif  // MYSLAM_VIEWER_H