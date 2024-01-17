#ifndef MYSLAM_ALGORITHM_H
#define MYSLAM_ALGORITHM_H

// algorithms used in myslam
#include "MYSLAM/common_include.h"

namespace MYSLAM{

// 通过构建线性方程，然后SVD分解来计算特征点的位置。
/**
 * linear triangulation with SVD
 * @param poses     poses,
 * @param points    points in normalized plane
 * @param pt_world  triangulated point in the world
 * @return true if success
 */

inline bool triangulation(const std::vector<SE3> &poses,
                   const std::vector<Vec3> points, Vec3 &pt_world) {
    MatXX A(2 * poses.size(), 4);
    VecX b(2 * poses.size());
    b.setZero();
    for (size_t i = 0; i < poses.size(); ++i) {
        Mat34 m = poses[i].matrix3x4();
        A.block<1, 4>(2 * i, 0) = points[i][0] * m.row(2) - m.row(0);
        A.block<1, 4>(2 * i + 1, 0) = points[i][1] * m.row(2) - m.row(1);
    }
    // Eigen::ComputeThinU和Eigen::ComputeThinV是两个参数，用于指定只计算矩阵的前k个奇异向量，其中k是矩阵的秩。这样可以加快计算速度并减小内存占用
    auto svd = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
    /*
    取SVD分解得到v矩阵的最有一列作为解：svd.matrixV().col(3)
    深度值是svd.matrixV()(3, 3)
    head<3>() 是取前三个值
    */
    pt_world = (svd.matrixV().col(3) / svd.matrixV()(3, 3)).head<3>();

    if (svd.singularValues()[3] / svd.singularValues()[2] < 1e-2) {
        return true;
    }
    return false;// 解质量不好，放弃
}

inline Vec2 toVec2(const cv::Point2f p) { 
    return Vec2(p.x, p.y); 
}


}// namespace MYSLAM

#endif  // MYSLAM_ALGORITHM_H
