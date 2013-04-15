/* This file is part of the calibu Project.
 * https://github.com/stevenlovegrove/calibu
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
        
protected:
    void Clear();
    void SetGrid(Vertex& v, const Eigen::Vector2i& g);
    bool Match(std::map<Eigen::Vector2i, Vertex*>& obs, const std::array<Eigen::MatrixXi,4>& PG);
    
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
};

}
