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

#include "tracker.h"

#include <iostream>
#include "adaptive_threshold.h"
#include "gradient.h"
#include "integral_image.h"
#include "label.h"
#include "find_conics.h"
#include "pnp.h"

using namespace std;
using namespace Eigen;

namespace fiducials {

Tracker::Tracker(int w, int h)
    : w(w), h(w),
      intI(new float[w*h]), dI(new boost::array<float,2>[w*h]), lI(new short[w*h]), tI(new unsigned char[w*h]),
    last_good(0), good_frames(0)
{

}

bool Tracker::ProcessFrame(const TrackerParams & params,
                           LinearCamera& cam,
                           unsigned char* I)
{
    gradient<>(cam.width(), cam.height(), I, dI.get() );
    integral_image(cam.width(), cam.height(), I, intI.get() );
    return ProcessFrame(params, cam, I, dI.get(), intI.get());
}

bool Tracker::ProcessFrame(const TrackerParams & params,
                           LinearCamera& cam,
                           unsigned char* I,
                           boost::array<float,2>* dI,
                           float* intI)
{
  const int w = cam.width();
  const int h = cam.height();

  double rms = 0;

  // Threshold and label image
  AdaptiveThreshold(w, h, I, intI, tI.get(), params.at_threshold,
                    w/params.at_window_ratio,
                    (unsigned char)0, (unsigned char)255);

  vector<PixelClass> labels;
  Label(w,h,tI.get(),lI.get(),labels);

  // Find candidate regions for conics
  candidates.clear();
  FindCandidateConicsFromLabels(w, h, labels, candidates,
                                params.conic_min_area, params.conic_max_area,
                                params.conic_min_density,
                                params.conic_min_aspect);
 // Find conic parameters
  conics.clear();
  FindConics(w,h,candidates,dI,conics );

  // Generate map and point structures
  conics_target_map.clear();
  conics_target_map.resize(conics.size(),-1);
  vector<Vector2d > ellipses;
  for( size_t i=0; i < conics.size(); ++i )
  {
    ellipses.push_back(Vector2d(conics[i].center.x(),conics[i].center.y()));
  }

  // Undistort Conics
  vector<Conic> conics_camframe;
  for( unsigned int i=0; i<conics.size(); ++i )
  {
    conics_camframe.push_back(UnmapConic(conics[i],cam));
  }

  // Find target given (approximately) undistorted conics
  const static LinearCamera idcam(-1,-1,1,1,0,0);
  target.FindTarget(idcam, conics_camframe, conics_target_map);
  conics_candidate_map_first_pass = conics_target_map;
  int inliers = CountInliers(conics_candidate_map_first_pass);
  if (inliers<params.inlier_num_required)
    return false;

  conics_target_map = PosePnPRansac(cam, ellipses, target.Circles3D(), conics_candidate_map_first_pass, params.robust_3pt_its, params.robust_3pt_inlier_tol, &T_hw);

  rms = ReprojectionErrorRMS(cam, T_hw, target.Circles3D(), ellipses,
                             conics_target_map);
  target.FindTarget(T_hw, cam, conics, conics_target_map);

  conics_candidate_map_second_pass = conics_target_map;

  inliers = CountInliers(conics_candidate_map_second_pass);

  if (inliers<params.inlier_num_required)
    return false;

  conics_target_map = PosePnPRansac(cam, ellipses, target.Circles3D(), conics_candidate_map_second_pass, params.robust_3pt_its, params.robust_3pt_inlier_tol, &T_hw);

  rms = ReprojectionErrorRMS(cam, T_hw, target.Circles3D(), ellipses,
                             conics_target_map);


  inliers = CountInliers(conics_target_map);

  if( isfinite((double)rms) && rms < params.max_rms &&  inliers>=params.inlier_num_required)
  {
    T_gw = T_hw;
    return true;
  }
  return false;
}

int Tracker::NumVisibleFeatures() const
{
    int seen = 0;
    for(size_t i=0; i< conics_target_map.size(); ++i )
    {
        if(conics_target_map[i] >= 0 ) {
            seen++;
        }
    }
    return seen;
}


Eigen::Matrix<double,3,Eigen::Dynamic> Tracker::TargetPattern3D() const
{
    Eigen::MatrixXd pattern3d = Eigen::MatrixXd(3, target.Circles2D().size() );
    for(size_t i=0; i < target.Circles2D().size(); ++i )
    {
        pattern3d.col(i) = target.Circles3D()[i];
    }
    return pattern3d;
}

Eigen::Matrix<double,2,Eigen::Dynamic> Tracker::TargetPatternObservations() const
{
    // Reverse map (target -> conic from conic -> target)
    vector<int> target_conics_map(target.Circles2D().size(), -1);
    for( size_t i=0; i < conics_target_map.size(); ++i ) {
        if(conics_target_map[i] >= 0) {
            target_conics_map[conics_target_map[i]] = i;
        }
    }

    // Generate list of visible indices
    int unseen = 0;
    std::vector<short int> visibleCircles;
    for( size_t i=0; i < target_conics_map.size(); ++i ) {
        if( target_conics_map[i] != -1 ) {
            visibleCircles.push_back(i);
        }else{
            ++unseen;
        }
    }

    Eigen::Matrix<double,2,Eigen::Dynamic> obs = Eigen::MatrixXd(2, target.Circles2D().size() );
    for(size_t i=0; i < target.Circles2D().size(); ++i ) {
        const int c = target_conics_map[i];
        if(c >= 0 ) {
            Eigen::Vector2d circ = conics[c].center;
            obs.col(i) <<  circ[0] , circ[1];
        }else{
            obs.col(i) << NAN, NAN;
        }
    }
    return obs;
}

}
