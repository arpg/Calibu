/* This file is part of the calibu Project.
 * https://github.com/stevenlovegrove/calibu
 *
 * Copyright (C) 2013  Steven Lovegrove, 
 *                     George Washington University
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

#include <calibu/cam/CameraModelBase.h>
#include <calibu/image/ImageProcessing.h>
#include <calibu/conics/Conic.h>

namespace calibu
{

class TargetInterface
{
public:
    
    // Find target given conic observations
    // Returns true on success, false on failure
    
    // Assume approximately known camera and pose
    virtual bool FindTarget(
            const Sophus::SE3d& T_cw,            
            const CameraModelBase& cam,
            const ImageProcessing& images,
            const std::vector<Conic>& conics,
            std::vector<int>& conics_target_map
            ) = 0;
    
    // Assume approximately known camera
    virtual bool FindTarget(
            const CameraModelBase& cam,
            const ImageProcessing& images,
            const std::vector<Conic>& conics,
            std::vector<int>& conics_target_map
            ) = 0;
    
    // Only observations known
    virtual bool FindTarget(
            const ImageProcessing& images,
            const std::vector<Conic>& conics,
            std::vector<int>& conics_target_map
            ) = 0;
    
    // Return circle radius
    virtual double CircleRadius() const = 0;
    
    // Return canonical set of known 2D/3D points.
    virtual const std::vector<Eigen::Vector2d>& Circles2D() const = 0;
    virtual const std::vector<Eigen::Vector3d>& Circles3D() const = 0;
};

}
