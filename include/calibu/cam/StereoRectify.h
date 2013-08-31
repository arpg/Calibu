/* 
   This file is part of the Calibu Project.
   https://github.com/gwu-robotics/Calibu
   
   Copyright (C) 2013 George Washington University,
                      Steven Lovegrove,
                      Gabe Sibley
                      
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

#include <sophus/se3.hpp>
#include <calibu/cam/CameraModel.h>
#include <calibu/cam/Rectify.h>

namespace calibu
{

/// Create left and right camera lookup tables from left and right camera models,
/// and output their new intrinsics and extrinsics.
/// Returns: New camera intrinsics shared by both cameras
/// T_nr_nl: New scanline rectified extrinsics considering rotation applied in lookup tables.
calibu::CameraModelT<Pinhole> CreateScanlineRectifiedLookupAndCameras(
        const Sophus::SE3d T_rl,
        const calibu::CameraModelInterface& cam_left,
        const calibu::CameraModelInterface& cam_right,
        Sophus::SE3d& T_nr_nl,
        LookupTable& left_lut,
        LookupTable& right_lut
        );

}
