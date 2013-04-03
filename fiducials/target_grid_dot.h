/* This file is part of the fiducials Project.
 * https://github.com/stevenlovegrove/fiducials
 *
 * Copyright (C) 2013  Steven Lovegrove, Nima Keivan
 *                     George Washington University
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

#include "target.h"
#include "line_group.h"

namespace fiducials {

struct Dist { size_t i; double dist; };
inline bool operator<(const Dist& lhs, const Dist& rhs) { return lhs.dist < rhs.dist; }

struct ParamsGridDot
{
    ParamsGridDot() :
        thresh_dist(0.13),
        thresh_area(0.017),
        cross_area_threshold(2.0),
        cross_max_area_threshold(8.9),
        cross_radius_ratio(0.058),
        cross_line_ratio(0.036)
    {}
    
    double thresh_dist;
    double thresh_area;
    double cross_area_threshold;
    double cross_max_area_threshold;
    double cross_radius_ratio;
    double cross_line_ratio;
};

class TargetGridDot
    : public TargetInterface
{
public:
    TargetGridDot(double grid_spacing, Eigen::Vector2i grid_size, Eigen::Vector2i grid_center);
    
    ////////////////////////////////////////////////////////////////////////////
  
    bool FindTarget(
      const Sophus::SE3d& T_cw,
      const LinearCamera& cam,
      const ImageProcessing& images,
      std::vector<Conic>& conics,
      std::vector<int>& ellipse_target_map
    ) const;
  
    bool FindTarget(
      const LinearCamera& cam,
      const ImageProcessing& images,
      std::vector<Conic>& conics,
      std::vector<int>& ellipse_target_map
    ) const;
  
    bool FindTarget(
      const ImageProcessing& images,
      std::vector<Conic>& conics,
      std::vector<int>& ellipse_target_map
    ) const;
    
    ////////////////////////////////////////////////////////////////////////////
  
    const std::vector<Eigen::Vector2d >& Circles2D() const {
        return tpts2d;
    }
    
    const std::vector<Eigen::Vector3d >& Circles3D() const {
        return tpts3d;
    }
    
protected:
    void Clear() const;
    bool Find(std::vector<Eigen::Vector2d>& pts, double thresh_dist, double thresh_area ) const;
    void PropagateGrid(const std::vector<Eigen::Vector2d>& pts,const int idxCross) const;
    
    std::vector<Eigen::Vector2d > tpts2d;
    std::vector<Eigen::Vector3d > tpts3d;
    
    double grid_spacing;
    Eigen::Vector2i grid_size;
    Eigen::Vector2i grid_center;
    
    ParamsGridDot params;
    
    mutable double k[2];
    mutable std::vector<std::vector<Dist> > pts_distance;
    mutable std::vector<std::vector<Opposite> > pts_neighbours;
    mutable std::list<LineGroup> line_groups;
    mutable std::vector<Eigen::Vector2i> grid;
};

}
