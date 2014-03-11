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
