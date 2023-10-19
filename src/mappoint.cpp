#include"MYSLAM/mappoint.h"
#include"MYSLAM/feature.h"

namespace MYSLAM{

// 构造函数
MapPoint::MapPoint( long id,  Vec3 position) :  id_(id),pos_(position) {};

// 工厂模式
MapPoint::Ptr MapPoint::CreateNewMappoint() {
    static long factory_id=0;
    MapPoint::Ptr new_mappoint(new MapPoint);
    new_mappoint->id_=factory_id++;
    return new_mappoint;
}


// 可能是异常点，也可能将要删除某个关键帧，所以要移除某个特征点，并且特征点数量-1
void MapPoint::RemoveObservation(std::shared_ptr<Feature>feat){
    std::unique_lock<std::mutex> lck(data_mutex_);//上锁
    // 遍历observations_,找到被对应异常点观察到的那个feature
    for(auto iter=observations_.begin();iter!=observations_.end() ; iter++){
        if (iter->lock()==feat)
        {
            observations_.erase(iter);//从observations_,中删除
            feat->map_point_.reset();//把对应的地图点删除
            observed_times_--;//观察次数-1
            break;//找到之后，删除完就可以跳出循环了
        }
    }
 }
}// namespace MYSLAM