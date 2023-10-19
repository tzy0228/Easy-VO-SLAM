// 数据集读取
//  构造时传入配置文件路径，配置文件的dataset_dir为数据集路径
//  Init之后可获得相机和下一帧图像

#ifndef MYSLAM_DATASET_H
#define MYSLAM_DATASET_H

#include "MYSLAM/camera.h"
#include "MYSLAM/common_include.h"
#include "MYSLAM/frame.h"

namespace MYSLAM{

// 数据集读取类
class Dataset{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Dataset>Ptr;//智能指针

    // 构造函数
    Dataset(const std::string &dataset_path);

    // 初始化，返回是否成功
    bool Init();

    //创建并返回下一帧
    Frame::Ptr NextFrame();

    //根据id返回相机参数
    Camera::Ptr GetCamera(int camera_id) const { 
        return cameras_.at(camera_id);
    };

private:
// 参数
    std::string dataset_path_; //数据集路径
    int current_image_index_ =0 ;//当前图像id
    std::vector<Camera::Ptr>cameras_;//每一帧对应的相机参数（应该都是一样的
};
}//MYSLAM
#endif