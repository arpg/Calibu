/* This file is part of the calibu Project.
 * https://github.com/stevenlovegrove/calibu
 *
 * Copyright (C) 2011  Steven Lovegrove
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

Eigen::Matrix3d EstimateH_ba(
        const std::vector<Eigen::Vector2d >& a,
        const std::vector<Eigen::Vector2d >& b
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
