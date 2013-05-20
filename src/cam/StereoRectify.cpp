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

#include <calibu/cam/StereoRectify.h>

namespace calibu
{

void CreateLookupTable(
        const calibu::CameraModelInterface& cam_from,
        const Eigen::Matrix3d R_onKinv,
        Eigen::Matrix<Eigen::Vector2f, Eigen::Dynamic, Eigen::Dynamic>& lookup_warp
        )
{
    for(size_t r = 0; r < cam_from.Height(); ++r) {
        for(size_t c = 0; c < cam_from.Width(); ++c) {
            // Remap
            const Eigen::Vector3d p_o = R_onKinv * Eigen::Vector3d(c,r,1);
            Eigen::Vector2d p_warped = cam_from.Map(calibu::Project(p_o));
            
            // Clamp to valid image coords
            p_warped[0] = std::min(std::max(0.0, p_warped[0]), cam_from.Width() - 1.0 );
            p_warped[1] = std::min(std::max(0.0, p_warped[1]), cam_from.Height() - 1.0 );
            
            lookup_warp(r,c) = p_warped.cast<float>();
        }
    }
}

calibu::CameraModelT<Pinhole> CreateScanlineRectifiedLookupAndCameras(
        const Sophus::SE3d T_rl,
        const calibu::CameraModelInterface& cam_left,
        const calibu::CameraModelInterface& cam_right,
        Sophus::SE3d& T_nr_nl,        
        Eigen::Matrix<Eigen::Vector2f, Eigen::Dynamic, Eigen::Dynamic>& dlookup_left,
        Eigen::Matrix<Eigen::Vector2f, Eigen::Dynamic, Eigen::Dynamic>& dlookup_right
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
    Eigen::Matrix3d mRl_nl;
    mRl_nl << x_l, y_l, z_l;
    
    // By definition, the right camera now lies exactly on the x-axis with the same orientation
    // as the left camera.
    T_nr_nl = Sophus::SE3d(Eigen::Matrix3d::Identity(), Eigen::Vector3d(-r_l.norm(),0,0) );

    // Setup new camera
    const Eigen::Matrix3d Kl = cam_left.K();
    calibu::CameraModelT<Pinhole> new_cam(cam_left.Width(),cam_left.Height());
    new_cam.Params() << Kl(0,0), Kl(1,1), Kl(0,2), Kl(1,2);    
    
    // Homographies which should be applied to left and right images to scan-line rectify them
    const Eigen::Matrix3d Rl_nlKlinv = mRl_nl.transpose() * new_cam.Kinv();
    const Eigen::Matrix3d Rr_nrKlinv = (mRl_nl * R_lr.matrix()).transpose() * new_cam.Kinv();
    
    CreateLookupTable(cam_left, Rl_nlKlinv, dlookup_left);
    CreateLookupTable(cam_right, Rr_nrKlinv, dlookup_right);
    
    return new_cam;
}

}
