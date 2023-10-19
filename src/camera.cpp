#include"MYSLAM/camera.h"

namespace MYSLAM{

Camera::Camera(){};

//世界to相机：T_c_w变换矩阵 乘 世界坐标系的点位置
Vec3  Camera::world2camera(const Vec3 &p_w, const SE3 &T_c_w){
    return pose_*T_c_w*p_w;
}

//相机to世界：T_c_w变换矩阵的逆 乘 相机坐标系的点位置
Vec3 Camera::camera2world(const Vec3 &p_c, const SE3 &T_c_w){
    return T_c_w.inverse()*pose_inv_*p_c;
}

//相机to像素
Vec2 Camera::camera2pixel(const Vec3 &p_c){
    return Vec2(
        fx_*p_c(0,0)/p_c(2,0)+cx_,      // u = fx * X / Z+ cx
        fy_*p_c(1,0)/p_c(2,0)+cy_       // v = fy * Y / Z+ cy
    );
}

//像素to相机
Vec3 Camera::pixel2camera(const Vec2 &p_p, double depth ){
    return Vec3(
        (p_p(0,0)-cx_)*depth / fx_,    //  X=(u-cx)*z / fx
        (p_p(1,0)-cy_)*depth / fy_,     // Y=(v-cy)*z/ fy
        depth
    );
}

// 像素to世界
Vec3 Camera::pixel2world(const Vec2 &p_p,const SE3 &T_c_w, double depth){
    return camera2world(pixel2camera(p_p , depth),T_c_w);//像素to相机，相机to世界
}

//世界to像素
Vec2 Camera::world2pixel(const Vec3 &p_w,const SE3 &T_c_w){
    return camera2pixel(world2camera(p_w , T_c_w)); //世界to相机，相机to像素
}


}//namespace MYSLAM