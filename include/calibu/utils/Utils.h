/*
   This file is part of the Calibu Project.
   https://github.com/gwu-robotics/Calibu

   Copyright (C) 2013 George Washington University,
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

#include <calibu/Platform.h>

#include <vector>
#include <Eigen/Dense>
#include <sophus/se3.hpp>

namespace calibu {

template<typename Derived>
bool is_nan(const Eigen::MatrixBase<Derived>& x)
{
    return !(x.array() == x.array()).all();
}

template<typename Derived>
bool is_finite(const Eigen::MatrixBase<Derived>& x)
{
    return !is_nan( (x.array() - x.array()).matrix() );
}

CALIBU_EXPORT
Eigen::Matrix3d EstimateH_ba(
        const std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> >& a,
        const std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> >& b
        );

inline Eigen::Matrix3d SkewSym( const Eigen::Vector3d& A)
{
    Eigen::Matrix3d R;
    R <<
         0, -A[2], A[1],
            A[2], 0, -A[0],
            -A[1], A[0], 0;
    return R;
}

inline Eigen::Matrix4d SymmetryTransform( const Eigen::Vector4d& N )
{
    // Compute Symmetry transformation S in ss(3) induced by plane N
    // "The top-left 3 × 3 sub-matrix of any element in ss(3) is always a House-holder matrix"
    // therefore S in ss(3) is involutionary: S^{-1} = S
    // T = S1.S2 where S1,S2 in ss(3) and T in SE(3)

    Eigen::Matrix4d S = Eigen::Matrix4d::Identity();
    const Eigen::Vector3d n = N.head<3>();
    const double d = -N[3];
    S.block<3,3>(0,0) = (Eigen::Matrix3d::Identity()) - 2 * n * n.transpose();
    S.block<3,1>(0,3) = 2 * d * n;
    return S;
}

}
