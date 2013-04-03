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

#include "target.h"
#include "label.h"
#include "conics.h"

namespace fiducials {

struct TrackerParams
{
  TrackerParams () :
    at_threshold(0.7),
    at_window_ratio(3),
    conic_min_area(25),
    conic_max_area(4E4),
    conic_min_density(0.4),
    conic_min_aspect(0.1),
    robust_3pt_inlier_tol(1.5),
    robust_3pt_its(100),
    inlier_num_required(10),
    max_rms(3.0) {}

  float at_threshold;
  int at_window_ratio;
  float conic_min_area;
  float conic_max_area;
  float conic_min_density;
  float conic_min_aspect;
  double robust_3pt_inlier_tol;
  int robust_3pt_its;
  int inlier_num_required;
  double max_rms;
};


class Tracker
{
public:
    Tracker(const TargetInterface& target, int w, int h);

    bool ProcessFrame(LinearCamera& cam, unsigned char *I);
    bool ProcessFrame(LinearCamera& cam, unsigned char* I, boost::array<float, 2>* dI, float* intI);

    const TargetInterface& Target() const {
        return target;
    }
    
    const std::vector<Conic>& Conics() const{
        return conics;
    }
    
    const std::vector<int>& ConicsTargetMap() const{
        return conics_target_map;
    }
    
    const unsigned char* ImageThresholded() const {
        return tI.get();
    }
    
    const Sophus::SE3d& PoseT_gw() const
    {
        return T_gw;
    }
    
protected:
    // Fiducial Target
    const TargetInterface& target;

    // Images
    int w, h;
    std::auto_ptr<float> intI;
    std::auto_ptr<boost::array<float,2> > dI;
    std::auto_ptr<short> lI;
    std::auto_ptr<unsigned char> tI;

    // Hypothesis conics
    std::vector<PixelClass> candidates;
    std::vector<Conic> conics;
    std::vector<int> conics_target_map;
    std::vector<int> conics_candidate_map_first_pass;
    std::vector<int> conics_candidate_map_second_pass;

    // Last good pose
    Sophus::SE3d T_gw;
    std::clock_t last_good;
    int good_frames;

    // Pose hypothesis
    Sophus::SE3d T_hw;
    
    TrackerParams params;
};

}
