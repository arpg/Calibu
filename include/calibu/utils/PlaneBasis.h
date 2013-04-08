/* This file is part of the calibu Project.
 * https://github.com/stevenlovegrove/calibu
 *
 * Copyright (C) 2010  Steven Lovegrove
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

#include <sophus/se3.hpp>

namespace calibu
{

/// Adapted from TooN so3.h:
/// creates an SO3 as a rotation that takes Vector a into the direction of Vector b
/// with the rotation axis along a ^ b. If |a ^ b| == 0, it creates the identity rotation.
/// An assertion will fail if Vector a and Vector b are in exactly opposite directions.
/// @param a source Vector
/// @param b target Vector
inline Sophus::SO3d Rotation(const Eigen::Vector3d& a, const Eigen::Vector3d& b)
{
    Eigen::Vector3d n = a.cross(b);
    
    if(n.squaredNorm() == 0) {
        //check that the vectors are in the same direction if cross product is 0. If not,
        //this means that the rotation is 180 degrees, which leads to an ambiguity in the rotation axis.
        //        assert(a.dot(b)>=0);
        return Sophus::SO3d();
    }
    
    n.normalize();
    Eigen::Matrix3d R1;
    R1.col(0) = a.normalized();
    R1.col(1) = n;
    R1.col(2) = n.cross(R1.col(0));
    
    Eigen::Matrix3d M;
    M.col(0) = b.normalized();
    M.col(1) = n;
    M.col(2) = n.cross(M.col(0));
    M = M * R1.transpose();
    
    return Sophus::SO3d(M);
}

inline Sophus::SE3d PlaneBasis_wp(const Eigen::Vector3d& nd_w)
{
    const double d = 1.0 / nd_w.norm();
    const Eigen::Vector3d n = d * nd_w;
    const Sophus::SO3d R_wn = Rotation(Eigen::Vector3d(0,0,-1),n);
    return Sophus::SE3d(R_wn, -d*n);
}

}
