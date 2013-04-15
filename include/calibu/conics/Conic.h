/* 
   This file is part of the Calibu Project.
   https://robotics.gwu.edu/git/calibu

   Copyright (C) 2013 George Washington University

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
#include <array>

#include <Eigen/Dense>

#include <calibu/utils/Rectangle.h>
#include <calibu/cam/CameraModel.h>

namespace calibu {

struct Conic
{
    IRectangle bbox;
    
    // quadratic form: x'*C*x = 0 with x = (x1,x2,1) are points on the ellipse
    Eigen::Matrix3d C;      
    
    // l:=C*x is tangent line throught the point x. The dual of C is adj(C).
    // For lines through C it holds: l'*adj(C)*l = 0.
    // If C has full rank it holds up to scale: adj(C) = C^{-1}
    Eigen::Matrix3d Dual;
    
    // center (c1,c2)
    Eigen::Vector2d center; 
};

double Distance( const Conic& c1, const Conic& c2, double circle_radius );

std::array<std::pair<Eigen::Vector3d,Eigen::Matrix3d >, 2 > PlaneFromConic(
        const Conic& c, double plane_circle_radius, const Eigen::Matrix3d& K
        );

std::pair<Eigen::Vector3d,Eigen::Matrix3d > PlaneFromConics(
        const std::vector<Conic>& conics, double plane_circle_radius,
        const Eigen::Matrix3d& K, double inlier_threshold
        );

Conic UnmapConic( const Conic& c, const CameraModelInterface& cam );

}
