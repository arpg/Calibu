/* 
   This file is part of the Calibu Project.
   https://robotics.gwu.edu/git/calibu

   Copyright (C) 2013 George Washington University,
                      Steven Lovegrove,
                      Hauke Strasdat

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

#include <memory>
#include <sophus/se3.hpp>
#include <ctime>

#include <calibu/target/Target.h>
#include <calibu/conics/ConicFinder.h>
#include <calibu/cam/CameraModel.h>

namespace calibu {

struct ParamsTracker
{
    ParamsTracker () :
        robust_3pt_inlier_tol(1.5),
        robust_3pt_its(100),
        inlier_num_required(10),
        max_rms(3.0) {}
    
    double robust_3pt_inlier_tol;
    int robust_3pt_its;
    int inlier_num_required;
    double max_rms;
};


class Tracker
{
public:
    Tracker(TargetInterface& target, int w, int h);
    
    bool ProcessFrame( CameraModelInterface& cam, unsigned char *I, size_t pitch );
    
    const TargetInterface& Target() const {
        return target;
    }
    
    const ConicFinder& GetConicFinder() const {
        return conic_finder;
    }
    
    const ImageProcessing& Images() const {
        return imgs;
    }
    
    const std::vector<int>& ConicsTargetMap() const{
        return conics_target_map;
    }
    
    const Sophus::SE3d& PoseT_gw() const
    {
        return T_gw;
    }
    
protected:
    // Target
    TargetInterface& target;
    ImageProcessing imgs;
    ConicFinder conic_finder;
    
    // Hypothesis conics
    std::vector<int> conics_target_map;
    std::vector<int> conics_candidate_map_first_pass;
    std::vector<int> conics_candidate_map_second_pass;
    
    // Last good pose
    Sophus::SE3d T_gw;
    std::clock_t last_good;
    int good_frames;
    
    // Pose hypothesis
    Sophus::SE3d T_hw;
    
    ParamsTracker params;
};

}
