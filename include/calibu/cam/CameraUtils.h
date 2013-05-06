/* 
   This file is part of the Calibu Project.
   https://robotics.gwu.edu/git/calibu

   Copyright (C) 2013 George Washington University
                      Steven Lovegrove,

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

#include <Eigen/Eigen>

namespace calibu
{

//////////////////////////////////////////////////////////////////////////////
// Projection / Unprojection utilities
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
/// Unproject returns .
template<typename T> inline
Eigen::Matrix<T,3,1> Unproject(
        const Eigen::Matrix<T,2,1>& P //< Input:
        )
{
    Eigen::Matrix<T,3,1> ret;
    ret.template head<2>() = P;
    ret[2] = (T)1.0;
    return ret;
}

//////////////////////////////////////////////////////////////////////////////
/// Unproject 
template<typename T> inline
Eigen::Matrix<T,4,1> Unproject(
        const Eigen::Matrix<T,3,1>& P //< Input:
        )
{
    Eigen::Matrix<T,4,1> ret;
    ret.template head<3>() = P;
    ret[3] = (T)1.0;
    return ret;
}

//////////////////////////////////////////////////////////////////////////////
/// This function Projects from 3D into an image, returning a 2x1 image point.
template<typename T> inline
Eigen::Matrix<T,2,1> Project(const Eigen::Matrix<T,3,1>& P)
{
    return Eigen::Matrix<T,2,1>(P(0)/P(2), P(1)/P(2));
}

//////////////////////////////////////////////////////////////////////////////
/// This function Project from homogeneous 3D into an image, returning a 3x1
//  image point.
template<typename T> inline
Eigen::Matrix<T,3,1> Project(
        const Eigen::Matrix<T,4,1>& P //< Input:
        )
{
    return Eigen::Matrix<T,3,1>(P(0)/P(3), P(1)/P(3), P(2)/P(3));
}

//////////////////////////////////////////////////////////////////////////////
/// dNorm_dx returns ... TODO
inline
Eigen::Matrix<double,1,2> dNorm_dx(
        const Eigen::Vector2d& x //< Input:
        )
{
    const double normx = x.norm();
    return Eigen::Matrix<double,1,2>(x(0)/normx, x(1)/normx);
}

}
