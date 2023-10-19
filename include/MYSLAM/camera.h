// Camera类存储相机的内参和外参,并完成相机坐标系、像素坐标系、和世界坐标系之间的坐标变换。

#pragma once

#ifndef MYSLAM_CAMERA_H
#define MYSLAM_CAMERA_H

#include"MYSLAM/common_include.h"

namespace MYSLAM{

class Camera{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;//内存对齐
    typedef std::shared_ptr<Camera>Ptr;//智能指针
    double fx_=0,fy_=0,cx_=0, cy_=0,//定义内参
    baseline_=0;//基线
    SE3 pose_;//外参，双目到单目位姿变换 
    SE3 pose_inv_;

    // 构造函数（无参）
    Camera();
    // 构造函数（有参，初始化参数量）
    Camera(double fx, double fy, double cx, double cy, double baseline,
           const SE3 &pose  ) : fx_(fx), fy_(fy), cx_(cx), cy_(cy), baseline_(baseline), pose_(pose) {
            pose_inv_ = pose_.inverse();
           };

    // 取出位姿
    SE3 pose()const{
        return pose_;
    }

    //读取内参矩阵K
    Mat33 K()const{
        Mat33 k;
        k<<fx_,0,cx_,0,fy_,cy_,0,0,1;
        return k;
    }

    //世界to相机：T_c_w变换矩阵 乘 世界坐标系的点位置
    Vec3 world2camera(const Vec3 &p_w, const SE3 &T_c_w);
    
    //相机to世界：T_c_w变换矩阵的逆 乘 相机坐标系的点位置
    Vec3 camera2world(const Vec3 &p_c, const SE3 &T_c_w);

    //相机to像素
    Vec2 camera2pixel(const Vec3 &p_c);

    //像素to相机
    Vec3 pixel2camera(const Vec2 &p_p, double depth=1 );

    //像素to世界
    Vec3 pixel2world(const Vec2 &p_p,const SE3 &T_c_w, double depth=1);

    //世界to像素
    Vec2 world2pixel(const Vec3 &p_w,const SE3 &T_c_w);

};

}//namespace MYSLAM

#endif  // MYSLAM_CAMERA_H
