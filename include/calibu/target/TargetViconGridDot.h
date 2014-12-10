/*
   This file is part of the Calibu Project.
   https://github.com/arpg/Calibu

   Copyright (C) 2013 University of Colorado Boulder
                      Steven Lovegrove,
                      Dorian Galvez-Lopez

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

#include <calibu/target/TargetGridDot.h>

#include <vector>
#include <opencv2/opencv.hpp>

namespace calibu {

/** Note:
 *
 * This class does not fully meet the requirements of class TargetInterface,
 * so it can't be used as the rest of Target classes. This was decided
 * this way to minimize the impact in the code of the applications already using
 * TargetInterface, such as vicalib.
 *
 * This is the behavior of the main functions of TargetViconGridDot:
 *
 * - FindTarget: finds calibu target only
 * - FindViconTarget: finds vicon target after finding the calibu target
 * - Circles2D: returns calibu points and vicon points
 * - Circles3D: returns calibu points only
 * - ViconCircles3D: returns vicon 3D points
 * - Code3D: returns data related to calibu points only
 * - Map: returns a map of calibu points and vicon points
 * - LineGroups: as in TargetGridDot
 * - GetBinaryPattern: as in TargetGridDot
 */

CALIBU_EXPORT
class TargetViconGridDot: public TargetGridDot
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  // How Vicon markers are arranged around the target grid dot
  enum ViconLayout {
    EShapeAlignRight,   // E shape, markers on the right aligned with grid
    EShapeAlignBottom   // E shape, markers at the bottom aligned with grid
  };

public:
  TargetViconGridDot(TargetGridDot grid, ViconLayout layout);

  bool HasViconMarkers() const override {
    return true;
  }

  bool FindViconTarget(
      const std::vector<Conic, Eigen::aligned_allocator<Conic> >& grid_conics,
      const std::vector<int>& grid_conics_map,
      const std::vector<Conic, Eigen::aligned_allocator<Conic> >& new_conics,
      std::vector<int>& vicon_map) override;

  void SaveSVG(std::string filename, double rad0, double rad1) const override;

private:

  // Position of a Vicon marker relative to the target grid dot
  // e.g.: {0,1} means the position of the grid dot of col 0 and row 1
  std::vector<cv::Point2f> m_markers;
};

} // namespace calibu
