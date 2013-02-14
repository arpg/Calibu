#ifndef PNP_H
#define PNP_H

#include <vector>
#include <sophus/se3.hpp>
#include "camera.h"

Sophus::SE3d FindPose(
    const LinearCamera& cam,
    const std::vector<Eigen::Vector3d >& pts3d,
    const std::vector<Eigen::Vector2d >& pts2d,
    std::vector<int>& map2d_3d,
    double robust_inlier_tol,
    size_t robust_iterations
);

void PoseFromPointsLeastSq(
    const LinearCamera& cam,
    const std::vector<Eigen::Vector3d >& pts3d,
    const std::vector<Eigen::Vector2d >& pts2d,
    const std::vector<int>& map2d_3d,
    Sophus::SE3d& T_cw,
    bool use_guess = false
    );

double ReprojectionErrorRMS(
    const AbstractCamera& cam,
    const Sophus::SE3d& T_cw,
    const std::vector<Eigen::Vector3d >& pts3d,
    const std::vector<Eigen::Vector2d >& pts2d,
    const std::vector<int>& map2d_3d
    );

#endif // PNP_H
