/* 
   This file is part of the Calibu Project.
   https://robotics.gwu.edu/git/calibu

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

#include <vector>
#include <Eigen/Dense>
#include <sophus/se3.hpp>

#include <calibu/cam/CameraModel.h>
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
            const CameraModelInterface& cam,
            const ImageProcessing& images,
            const std::vector<Conic>& conics,
            std::vector<int>& conics_target_map
            ) = 0;
    
    // Assume approximately known camera
    virtual bool FindTarget(
            const CameraModelInterface& cam,
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
    virtual const std::vector<Eigen::Vector3d>& Code3D() const = 0;
};

}
