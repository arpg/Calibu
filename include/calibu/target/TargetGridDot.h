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

#pragma once

#include <array>
#include <map>
#include <calibu/target/Target.h>
#include <calibu/target/LineGroup.h>

namespace std {
template<> struct less<Eigen::Vector2i> {
    bool operator() (const Eigen::Vector2i& lhs, const Eigen::Vector2i& rhs) {
        return (lhs[0] < rhs[0]) || (lhs[0]==rhs[0] && lhs[1] < rhs[1]);
    }
};
}

namespace calibu {

struct Dist { Vertex* v; double dist; };
inline bool operator<(const Dist& lhs, const Dist& rhs) { return lhs.dist < rhs.dist; }

struct ParamsGridDot
{
    ParamsGridDot() :
        max_line_dist_ratio(0.3),
        max_norm_triple_area(0.03),
        min_cross_area(1.5),
        max_cross_area(9.0),
        cross_radius_ratio(0.058),
        cross_line_ratio(0.036)
    {}
    
    double max_line_dist_ratio;
    double max_norm_triple_area;
    double min_cross_area;
    double max_cross_area;
    double cross_radius_ratio;
    double cross_line_ratio;
};

class TargetGridDot
        : public TargetInterface
{
public:
    TargetGridDot(double grid_spacing, Eigen::Vector2i grid_size, Eigen::Vector2i grid_center, uint32_t seed = 71);
    
    ////////////////////////////////////////////////////////////////////////////
    
    bool FindTarget(
            const Sophus::SE3d& T_cw,
            const CameraModelBase& cam,
            const ImageProcessing& images,
            const std::vector<Conic>& conics,
            std::vector<int>& ellipse_target_map
            );
    
    bool FindTarget(
            const CameraModelBase& cam,
            const ImageProcessing& images,
            const std::vector<Conic>& conics,
            std::vector<int>& ellipse_target_map
            );
    
    bool FindTarget(
            const ImageProcessing& images,
            const std::vector<Conic>& conics,
            std::vector<int>& ellipse_target_map
            );
    
    ////////////////////////////////////////////////////////////////////////////

    inline double CircleRadius() const {
        // TODO: Load this from eps or something.
        return grid_spacing / 10.0;
    }
    
    inline const std::vector<Eigen::Vector2d >& Circles2D() const {
        return tpts2d;
    }
    
    inline const std::vector<Eigen::Vector3d >& Circles3D() const {
        return tpts3d;
    }
    
    ////////////////////////////////////////////////////////////////////////////
    
    const std::vector<Vertex>& Map() const {
        return vs;
    }
    
    const std::list<LineGroup>& LineGroups() const {
        return line_groups;
    }
    
    int CenterId() const {
        return idxCrossConic;
    }
    
protected:
    void Clear();
    void SetGrid(Vertex& v, const Eigen::Vector2i& g);
    bool Find(std::vector<Eigen::Vector2d>& pts, double thresh_dist, double thresh_area ) const;
    void PropagateGrid(const std::vector<Eigen::Vector2d>& pts,const int idxCross) const;
    
    std::vector<Eigen::Vector2d > tpts2d;
    std::vector<Eigen::Vector3d > tpts3d;
    
    double grid_spacing;
    Eigen::Vector2i grid_size;
    Eigen::Vector2i grid_center;
    std::array<Eigen::MatrixXi,4> PG;
    
    ParamsGridDot params;
    
    std::vector<Vertex> vs;
    std::map<Eigen::Vector2i, Vertex*> map_grid_ellipse;
    
    std::list<LineGroup> line_groups;
    int idxCrossConic;
};

}
