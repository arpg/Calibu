#if 0

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

#include <calibu/cam/Rectify.h>
#include <calibu/cam/StereoRectify.h>
#include <calibu/cam/camera_crtp.h>
#include <calibu/utils/Range.h>

namespace calibu
{

calibu::CameraInterface<4> CreateScanlineRectifiedLookupAndCameras(
        const Sophus::SE3d& T_rl,
        const calibu::CameraInterface& cam_left,
        const calibu::CameraInterface& cam_right,
        Sophus::SE3d& T_nr_nl,        
        LookupTable& left_lut,
        LookupTable& right_lut
        )
{
    const Sophus::SO3d R_rl = T_rl.so3();
    const Sophus::SO3d R_lr = R_rl.inverse();
    const Eigen::Vector3d l_r = T_rl.translation();
    const Eigen::Vector3d r_l = - (R_lr * l_r);
    
    // Current up vector for each camera (in left FoR)
    const Eigen::Vector3d lup_l = Eigen::Vector3d(0,-1,0);
    const Eigen::Vector3d rup_l = R_lr * Eigen::Vector3d(0,-1,0);
    
    // Hypothetical fwd vector for each camera, perpendicular to baseline (in left FoR)
    const Eigen::Vector3d lfwd_l = (lup_l.cross(r_l)).normalized();
    const Eigen::Vector3d rfwd_l = (rup_l.cross(r_l)).normalized();
    
    // New fwd is average of left / right hypothetical baselines (also perpendicular to baseline)
    const Eigen::Vector3d avgfwd_l = lfwd_l + rfwd_l;
    
    // Define new basis (in left FoR);
    const Eigen::Vector3d x_l = r_l.normalized();
    const Eigen::Vector3d z_l = avgfwd_l.normalized();
    const Eigen::Vector3d y_l = z_l.cross(x_l).normalized();
    
    // New orientation for both left and right cameras (expressed relative to original left)
    Eigen::Matrix3d Rnl_l;
    Rnl_l << x_l, y_l, z_l;
    
    // By definition, the right camera now lies exactly on the x-axis with the same orientation
    // as the left camera.
    T_nr_nl = Sophus::SE3d(Eigen::Matrix3d::Identity(), Eigen::Vector3d(-r_l.norm(),0,0) );

    // Work out parameters of new linear camera
    const Range range_width = //Intersection(
                MinMaxRotatedCol(cam_left, Rnl_l);
//                MinMaxRotatedCol(cam_right, Rnl_l)
//                );
    const Range range_height = //Intersection(
                MinMaxRotatedRow(cam_left, Rnl_l);
//                MinMaxRotatedRow(cam_right, Rnl_l)
//                );

    // We want to map range width/height to image via K.    
    const double fu = (cam_left.Width()-1) / range_width.Size();
    const double fv = (cam_left.Height()-1) / range_height.Size();
    const double u0 = -fu * range_width.minr;
    const double v0 = -fv * range_height.minr;
    
    // Setup new camera
    calibu::CameraInterface<4> new_cam(cam_left.Width(),cam_left.Height());
    new_cam.Params() << fu, fv, u0, v0;
 
    // Homographies which should be applied to left and right images to scan-line rectify them
    const Eigen::Matrix3d Rl_nlKlinv = Rnl_l.transpose() * new_cam.Kinv();
    const Eigen::Matrix3d Rr_nrKlinv = R_lr.inverse().matrix() * Rnl_l.transpose() * new_cam.Kinv();
    
    CreateLookupTable(cam_left, Rl_nlKlinv, left_lut );
    CreateLookupTable(cam_right, Rr_nrKlinv, right_lut );     
    return new_cam;
}

}

#endif
