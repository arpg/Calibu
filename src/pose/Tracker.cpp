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

#include <calibu/pose/Tracker.h>
#include <calibu/pose/Pnp.h>
#include <calibu/image/ImageProcessing.h>
#include <calibu/cam/CameraModel.h>

#include <iostream>

using namespace std;
using namespace Eigen;

namespace calibu {

Tracker::Tracker(TargetInterface& target, int w, int h)
    : target(target), imgs(w,h),
      last_good(0), good_frames(0)
{
    
}

bool Tracker::ProcessFrame(
        CameraModel& cam, unsigned char* I, size_t pitch
        )
{
    double rms = 0;
    
    imgs.Process(I, pitch );
    conic_finder.Find(imgs);

    const std::vector<Conic>& conics = conic_finder.Conics();

    // Generate map and point structures
    conics_target_map.clear();
    conics_target_map.resize(conics.size(),-1);
    vector<Vector2d > ellipses;
    for( size_t i=0; i < conics.size(); ++i ) {
        ellipses.push_back(Vector2d(conics[i].center.x(),conics[i].center.y()));
    }

    // Undistort Conics
    vector<Conic> conics_camframe;
    for( unsigned int i=0; i<conics.size(); ++i ) {
        conics_camframe.push_back(UnmapConic(conics[i],cam));
    }
 
    // Find target given (approximately) undistorted conics
    const static CameraModelSpecialization<ProjectionLinearId> idcam;
 
    target.FindTarget( idcam, imgs, conics_camframe, conics_target_map );
    conics_candidate_map_first_pass = conics_target_map;
    int inliers = CountInliers(conics_candidate_map_first_pass);
    if (inliers<params.inlier_num_required) {
        return false;
    }
 
    conics_target_map = PosePnPRansac( cam, ellipses, target.Circles3D(),
            conics_candidate_map_first_pass, params.robust_3pt_its,
            params.robust_3pt_inlier_tol, &T_hw );
 
    rms = ReprojectionErrorRMS(cam, T_hw, target.Circles3D(), ellipses,
                               conics_target_map);
    target.FindTarget( T_hw, cam, imgs, conics, conics_target_map);
    
    conics_candidate_map_second_pass = conics_target_map;
    
    inliers = CountInliers(conics_candidate_map_second_pass);
    
    if (inliers<params.inlier_num_required){
        return false;
    }
 
    conics_target_map = PosePnPRansac( cam, ellipses, target.Circles3D(),
            conics_candidate_map_second_pass, params.robust_3pt_its,
            params.robust_3pt_inlier_tol, &T_hw );
 
    rms = ReprojectionErrorRMS(cam, T_hw, target.Circles3D(), ellipses,
                               conics_target_map);
    
    inliers = CountInliers(conics_target_map);
    
    if( isfinite((double)rms) && rms < params.max_rms
            &&  inliers>=params.inlier_num_required) {
        T_gw = T_hw;
        return true;
    }

    return false;
}

}
