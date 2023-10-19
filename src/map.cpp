
#include "MYSLAM/map.h"
#include "MYSLAM/feature.h"

namespace MYSLAM{

// 增加一个关键帧
void  Map::InsertKeyFrame(Frame::Ptr frame){
    current_frame_=frame;
    //先在keyframe哈希表里找一下id，看看有没有添加过
    // 如果没找到，就添加到keyframe和activeframe的哈希表中
    if(keyframes_.find(frame->keyframe_id_)==keyframes_.end()){
        keyframes_.insert( make_pair(frame->keyframe_id_,frame));
        active_keyframes_.insert(make_pair(frame->keyframe_id_,frame));
    }
    // 如果该帧在之前已经添加进keyframe了，更新一下关键帧哈希表中的id
    else{
        keyframes_[frame->keyframe_id_]=frame;
        active_keyframes_[frame->keyframe_id_] = frame;
    }

    // 如果活跃keyframe数量大于窗口限定数量7，则需要清除窗口内最old的那帧keyframe
    if (active_keyframes_.size()>num_active_keyframes_)
    {
          RemoveOldKeyframe();//清除窗口内最old的那帧keyframe
    }
}

// 增加一个地图顶点
void Map::InsertMapPoint(MapPoint::Ptr map_point){
     //先在Landmarks哈希表里找一下id，看看有没有添加过
    // 如果没找到，就添加到Landmarks和activeLandmarks的哈希表中
    if (landmarks_.find(map_point->id_)==landmarks_.end())
    {
        landmarks_.insert(make_pair(map_point->id_,map_point));
        active_landmarks_.insert(make_pair(map_point->id_,map_point));
    }
    //如果该地图点已经添加过了，就更新一下id
    else{
        landmarks_[map_point->id_]=map_point;
        active_landmarks_[map_point->id_]=map_point;
    }
}

//清除窗口内最old的那帧keyframe
void  Map::RemoveOldKeyframe(){
     if (current_frame_ == nullptr) return;
    // 寻找与当前帧最近与最远的两个关键帧
    
    int max_dis=0 , min_dis=9999; //定义最近距离和最远距离
    int max_kf_id=0 , min_kf_id=0;//定义最近帧的id和最远帧的id
    auto Twc=current_frame_->Pose().inverse();//定义Twc （）

    // 遍历activekeyframe哈希表，计算每帧与当前帧的距离
    for (auto &kf : active_keyframes_)
    {
          if (kf.second == current_frame_)
              continue; // 如果遍历到当前帧自动跳过
          // 计算每帧与当前帧的距离
          auto dis = (kf.second->Pose() * Twc).log().norm();
          // 如果距离>最远距离，则更新
          if (dis > max_dis)
          {
              max_dis = dis;
              max_kf_id = kf.first;
          }
          // 如果距离<最近距离，则更新
          if (dis < min_dis)
          {
              min_dis = dis;
              min_kf_id = kf.first;
          }
    }
    const double min_dis_th = 0.2;  // 设定一个最近阈值
    Frame::Ptr frame_to_remove=nullptr;
    if (min_dis<min_dis_th)
    {
        // 如果存在很近的帧，优先删掉最近的
        frame_to_remove=keyframes_.at(min_kf_id);
    }
        //  否则 删掉最远的
    else{
        frame_to_remove=keyframes_.at(max_kf_id);
    }
    LOG(INFO) << "remove keyframe " << frame_to_remove->keyframe_id_;//打印删除的是哪一帧

    // 确定好删除窗口中的哪一帧后，开始删除对应的关键帧和与之相关的地图点
    active_keyframes_.erase(frame_to_remove->keyframe_id_);//删除窗口中的关键帧
    // 遍历左目的特征点，将其删除
    for (auto feat : frame_to_remove->features_left_)
    {
        auto mp = feat->map_point_.lock();
        if (mp)
        {
              mp->RemoveObservation(feat);//移除左目特征点，并且特征点数量-1
        }
    }
     // 遍历右目的特征点，将其删除
    for (auto feat : frame_to_remove->features_right_)
     {
        if (feat == nullptr) continue;
        auto mp = feat->map_point_.lock();//查看是否上锁
        if (mp)
        {
            mp->RemoveObservation(feat);//移除右边目特征点，并且特征点数量-1
        }
    }
    CleanMap();// 清理map中观测数量为0的点
}

// 清理map中观测数量为0的点
void Map::CleanMap(){
    int cnt_landmark_removed = 0;//设置被删除的点的次数
    // 遍历窗口所有帧，如果该帧被观测的次数为0，则删除该帧
    for(auto iter =active_landmarks_.begin(); iter != active_landmarks_.end();){
        if (iter->second->observed_times_==0)
        {
            iter = active_landmarks_.erase(iter);
            cnt_landmark_removed++;//记录次数+1
        }
        // 否则继续遍历
        else{
            ++iter;
        }
    }
    LOG(INFO) << "Removed " << cnt_landmark_removed << " active landmarks";//打印被删除的数量
}

} // namespace MYSLAM