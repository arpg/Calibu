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

#include <fiducials/pose/Tracker.h>
#include <fiducials/pose/Pnp.h>
#include <fiducials/image/ImageProcessing.h>
#include <fiducials/cam/CameraModel.h>

#include <iostream>

using namespace std;
using namespace Eigen;

namespace fiducials {

Tracker::Tracker(const TargetInterface& target, int w, int h)
    : target(target), imgs(w,h),
    last_good(0), good_frames(0)
{

}

bool Tracker::ProcessFrame(
    CameraModelBase& cam, unsigned char* I, size_t pitch
) {
  double rms = 0;
  
  imgs.Process(I, pitch );
  conic_finder.Find(imgs);

  const std::vector<Conic>& conics = conic_finder.Conics();

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
  const static CameraModel<ProjectionLinearId> idcam;
  
  target.FindTarget(idcam, imgs, conics_camframe, conics_target_map);
  conics_candidate_map_first_pass = conics_target_map;
  int inliers = CountInliers(conics_candidate_map_first_pass);
  if (inliers<params.inlier_num_required)
    return false;

  conics_target_map = PosePnPRansac(cam, ellipses, target.Circles3D(), conics_candidate_map_first_pass, params.robust_3pt_its, params.robust_3pt_inlier_tol, &T_hw);

  rms = ReprojectionErrorRMS(cam, T_hw, target.Circles3D(), ellipses,
                             conics_target_map);
  target.FindTarget(T_hw, cam, imgs, conics, conics_target_map);

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

}
