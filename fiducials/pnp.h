#include <vector>
#include <sophus/se3.hpp>
#include "camera.h"

std::vector<int> PosePnPRansac(
    const LinearCamera& cam,
    const std::vector<Eigen::Vector2d> & img_pts,
    const std::vector<Eigen::Vector3d> & ideal_pts,
    const std::vector<int> & candidate_map,
    int robust_3pt_its,
    float robust_3pt_tol,
    Sophus::SE3d * T
);

int CountInliers(const std::vector<int> & conics_target_map);

double ReprojectionErrorRMS(
    const AbstractCamera& cam,
    const Sophus::SE3d& T_cw,
    const std::vector<Eigen::Vector3d>& pts3d,
    const std::vector<Eigen::Vector2d>& pts2d,
    const std::vector<int>& map2d_3d
);
