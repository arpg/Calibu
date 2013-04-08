/* This file is part of the calibu Project.
 * https://github.com/stevenlovegrove/calibu
 *
 * Copyright (C) 2010  Steven Lovegrove, Richard Newcombe, Hauke Strasdat
 *                     Imperial College London
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

#pragma once

#include <vector>
#include <sophus/se3.hpp>
#include <calibu/cam/CameraModelBase.h>

namespace calibu {

std::vector<int> PosePnPRansac(
    const CameraModelBase& cam,
    const std::vector<Eigen::Vector2d> & img_pts,
    const std::vector<Eigen::Vector3d> & ideal_pts,
    const std::vector<int> & candidate_map,
    int robust_3pt_its,
    float robust_3pt_tol,
    Sophus::SE3d * T
);

int CountInliers(const std::vector<int> & conics_target_map);

double ReprojectionErrorRMS(
    const CameraModelBase& cam,
    const Sophus::SE3d& T_cw,
    const std::vector<Eigen::Vector3d>& pts3d,
    const std::vector<Eigen::Vector2d>& pts2d,
    const std::vector<int>& map2d_3d
);

}
