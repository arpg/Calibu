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
#include <array>

#include <Eigen/Dense>

#include <calibu/utils/Rectangle.h>
#include <calibu/cam/CameraModelBase.h>

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

Conic UnmapConic( const Conic& c, const CameraModelBase& cam );

}
