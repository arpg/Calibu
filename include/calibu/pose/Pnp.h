/*
   This file is part of the Calibu Project.
   https://github.com/gwu-robotics/Calibu

   Copyright (C) 2013 George Washington University,
                      Hauke Strasdat,
                      Steven Lovegrove

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

   http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
 */

#pragma once

#include <vector>
#include <sophus/se3.hpp>
#include <calibu/cam/camera_crtp.h>

namespace calibu {

  template<typename Scalar>
    std::vector<int> PosePnPRansac(
        const CameraInterface<double>& cam,
        const std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > & img_pts,
        const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > & ideal_pts,
        const std::vector<int> & candidate_map,
        int robust_3pt_its,
        float robust_3pt_tol,
        Sophus::SE3d * T
        );

  template<typename Scalar>
    double ReprojectionErrorRMS(
        const CameraInterface<double>& cam,
        const Sophus::SE3d& T_cw,
        const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& pts3d,
        const std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> >& pts2d,
        const std::vector<int>& map2d_3d
        );

  int CountInliers(const std::vector<int> & conics_target_map);

}
