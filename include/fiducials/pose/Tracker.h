/* This file is part of the fiducials Project.
 * https://github.com/stevenlovegrove/fiducials
 *
 * Copyright (C) 2012-2013 Steven Lovegrove
 * Copyright (C) 2013      Hauke Strasdat
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

#include <memory>
#include <sophus/se3.hpp>
#include <ctime>

#include <fiducials/target/Target.h>
#include <fiducials/conics/ConicFinder.h>

namespace fiducials {

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

    bool ProcessFrame(CameraModelBase& cam, unsigned char *I, size_t pitch);

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
    // Fiducial Target
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
