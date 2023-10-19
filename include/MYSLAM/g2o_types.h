#ifndef MYSLAM_G2O_TYPES_H
#define MYSLAM_G2O_TYPES_H

#include "MYSLAM/common_include.h"

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

namespace MYSLAM {
// 定义g2o中的顶点和边

// 位姿顶点
class VertexPose : public g2o::BaseVertex<6,SE3>{//优化量的参数模板
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

// 重置，设定被优化变量的原始值
        virtual  void setToOriginImpl() override 
        {
            _estimate =SE3();
        }  

// 更新
        virtual void oplusImpl(const double * update) override
        {
            Vec6 update_eigen;
            update_eigen <<  update[0], update[1], update[2], update[3], update[4], update[5];
            _estimate= SE3::exp(update_eigen) * _estimate; // 左乘更新 SE3 - 旋转矩阵R
        }
// 存盘、读盘
    virtual bool read(std::istream &in) override { return true; }   //存盘
    virtual bool write(std::ostream &out) const override { return true; } //读盘
};

//地图顶点
class VertexXYZ : public g2o::BaseVertex<3,Vec3>{//优化量的参数模板
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

// 重置，设定被优化变量的原始值
        virtual  void setToOriginImpl() override 
        {
            _estimate =Vec3::Zero();
        }  

// 更新
        virtual void oplusImpl(const double *update) override
        {
            _estimate[0] += update[0];
            _estimate[1] += update[1];
            _estimate[2] += update[2];
        }
// 存盘、读盘
    virtual bool read(std::istream &in) override { return true; }   //存盘
    virtual bool write(std::ostream &out) const override { return true; } //读盘
};

// 仅估计位姿的一元边
class EdgeProjectionPoseOnly : public g2o::BaseUnaryEdge<2,Vec2,VertexPose>{
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

// 构造函数
    EdgeProjectionPoseOnly(const Vec3 &pos, const Mat33 &K ): _pos3d(pos), _K(K) {}

// 误差计算函数
    virtual void computeError() override{
        const VertexPose *v = static_cast<VertexPose*>(_vertices[0]);
        SE3 T = v->estimate();
        Vec3 pos_pixel= _K * (T*_pos3d);    //将3D世界坐标转为相机像素坐标
        pos_pixel /= pos_pixel[2];
        _error=_measurement-pos_pixel.head<2>();
    }

    virtual void linearizeOplus() override {
        const VertexPose *v = static_cast<VertexPose *>(_vertices[0]);
        SE3 T = v->estimate();
        Vec3 pos_cam = T * _pos3d;   //相机坐标系下空间点的坐标=相机位姿 * 空间点的坐标
        double fx = _K(0, 0); 
        double fy = _K(1, 1);
        double X = pos_cam[0];
        double Y = pos_cam[1];
        double Z = pos_cam[2];
        double Zinv = 1.0 / (Z + 1e-18);
        double Zinv2 = Zinv * Zinv;
        //2*6的雅克比矩阵
        _jacobianOplusXi << -fx * Zinv, 0, fx * X * Zinv2, fx * X * Y * Zinv2,
            -fx - fx * X * X * Zinv2, fx * Y * Zinv, 0, -fy * Zinv,
            fy * Y * Zinv2, fy + fy * Y * Y * Zinv2, -fy * X * Y * Zinv2,
            -fy * X * Zinv;
    }
    // 读操作
    virtual bool read(std::istream &in) override { return true; }
    // 写操作
    virtual bool write(std::ostream &out) const override { return true; }

    private:
        Vec3 _pos3d;
        Mat33 _K;
};

//包含地图的二元边
class EdgeProjection : public g2o::BaseBinaryEdge<2,Vec2,VertexPose,VertexXYZ>{
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

// 构造函数,构造时传入相机内外参
    EdgeProjection( const Mat33 &K ,const SE3 &cam_ext):  _K(K) , _cam_ext(cam_ext){}

    // 计算雅克比矩阵 
    virtual void computeError() override {
        const VertexPose *v0 = static_cast<VertexPose *>(_vertices[0]);
        const VertexXYZ *v1 = static_cast<VertexXYZ *>(_vertices[1]);
        SE3 T = v0->estimate();
        Vec3 pos_pixel = _K * (_cam_ext * (T * v1->estimate()));   //估计值:T*p,得到相机坐标系下坐标，然后在利用camera2pixel()函数得到像素坐标。
        pos_pixel /= pos_pixel[2];
        _error = _measurement - pos_pixel.head<2>();
    }

    // 计算雅克比矩阵 
    virtual void linearizeOplus() override{
        const VertexPose *v0=static_cast<VertexPose*>(_vertices[0]);
        const VertexXYZ *v1 = static_cast<VertexXYZ *>(_vertices[1]);
        SE3 T = v0->estimate();
        Vec3 pw=v1->estimate();
        Vec3 pos_cam=_cam_ext*T*pw;

        double fx = _K(0, 0);
        double fy = _K(1, 1);
        double X = pos_cam[0];
        double Y = pos_cam[1];
        double Z = pos_cam[2];
        double Zinv = 1.0 / (Z + 1e-18);
        double Zinv2 = Zinv * Zinv;
        //2*6的雅克比矩阵
        _jacobianOplusXi << -fx * Zinv, 0, fx * X * Zinv2, fx * X * Y * Zinv2,
            -fx - fx * X * X * Zinv2, fx * Y * Zinv, 0, -fy * Zinv,
            fy * Y * Zinv2, fy + fy * Y * Y * Zinv2, -fy * X * Y * Zinv2,
            -fy * X * Zinv;
        //P187 (7.48) 的雅克比矩阵
        _jacobianOplusXj = _jacobianOplusXi.block<2, 3>(0, 0) *
                           _cam_ext.rotationMatrix() * T.rotationMatrix();
    }
        // 读操作
    virtual bool read(std::istream &in) override { return true; }
    // 写操作
    virtual bool write(std::ostream &out) const override { return true; }

    private:
        SE3 _cam_ext;
        Mat33 _K;
};


}  // namespace MYSLAM

#endif  // MYSLAM_G2O_TYPES_H
