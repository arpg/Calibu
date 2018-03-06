/*
   This file is part of the Calibu Project.
   https://github.com/arpg/Calibu

   Copyright (C) 2013 George Washington University
                      University of Colorado Boulder
                      Gabe Sibley
                      Christoffer Heckman
                      Nima Keivan
                      Steven Lovegrove

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

#include <calibu/Platform.h>
#include <calibu/target/Target.h>
#include <calibu/target/LineGroup.h>
#include <Eigen/Eigen>
#include <Eigen/StdVector>

namespace std {
template<> struct less<Eigen::Vector2i> {
    bool operator() (const Eigen::Vector2i& lhs, const Eigen::Vector2i& rhs) const {
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

CALIBU_EXPORT
class TargetGridDot
        : public TargetInterface
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    TargetGridDot(double grid_spacing, Eigen::Vector2i grid_size, uint32_t seed = 71);
    TargetGridDot(double grid_spacing, const Eigen::MatrixXi& grid);
    TargetGridDot( const std::string& preset );

    ////////////////////////////////////////////////////////////////////////////

    bool FindTarget(
            const Sophus::SE3d& T_cw,
            const std::shared_ptr<CameraInterface<double>> cam,
            const ImageProcessing& images,
            const std::vector<Conic, Eigen::aligned_allocator<Conic> >& conics,
            std::vector<int>& ellipse_target_map
            );

    bool FindTarget(
            const std::shared_ptr<CameraInterface<double>> cam,
            const ImageProcessing& images,
            const std::vector<Conic, Eigen::aligned_allocator<Conic> >& conics,
            std::vector<int>& ellipse_target_map
            );

    bool FindTarget(
            const ImageProcessing& images,
            const std::vector<Conic, Eigen::aligned_allocator<Conic> >& conics,
            std::vector<int>& ellipse_target_map
            );

    ////////////////////////////////////////////////////////////////////////////

    inline double CircleRadius() const
    {
        // TODO: Load this from eps or something.
        return grid_spacing_ / 10.0;
    }

    inline double CircleRadius(uint32_t circle_index) const
    {
        // TODO: Load this from eps or something.
        return grid_spacing_ / 10.0 *
            (tpts2d_radius[circle_index] == 1 ? 2.0 : 0.5);
    }

    inline const std::vector<Eigen::Vector2d,
                             Eigen::aligned_allocator<Eigen::Vector2d> >& Circles2D() const
    {
        return tpts2d;
    }

    inline const std::vector<Eigen::Vector3d,
                      Eigen::aligned_allocator<Eigen::Vector3d> >& Circles3D() const {
        return tpts3d;
    }

    inline const std::vector<Eigen::Vector3d,
                             Eigen::aligned_allocator<Eigen::Vector3d> >& Code3D() const {
        return codepts3d;
    }

    ////////////////////////////////////////////////////////////////////////////

  const std::vector<Vertex,
                    Eigen::aligned_allocator<Vertex> >& Map() const {
        return vs_;
    }

    const std::list<LineGroup>& LineGroups() const {
        return line_groups_;
    }

    Eigen::MatrixXi GetBinaryPattern( unsigned int idx = 0 ) const
    {
        return PG_[idx];
    }

    void SaveEPS(
            std::string filename,
            const Eigen::Vector2d& offset,
            double rad0,
            double rad1,
            double pts_per_unit,
            unsigned char id = 0
            ) const;

    void SaveSVG(
            std::string filename,
            double rad0,
            double rad1
            ) const;

 protected:
    void Init();
    void Clear();
    void SetGrid(Vertex& v, const Eigen::Vector2i& g);
  bool Match(std::map<Eigen::Vector2i, Vertex *, std::less<Eigen::Vector2i>, Eigen::aligned_allocator<std::pair<const Eigen::Vector2i, Vertex *> > > &obs,
             const std::array<Eigen::MatrixXi,4>& PG);

    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > tpts2d;
    std::vector<double> tpts2d_radius;
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > tpts3d;
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > codepts3d;

    double grid_spacing_;
    Eigen::Vector2i grid_size_;
    std::array<Eigen::MatrixXi,4> PG_;

    ParamsGridDot params_;

  std::vector<Vertex, Eigen::aligned_allocator<Vertex> > vs_;
    std::map<Eigen::Vector2i, Vertex*,
             std::less<Eigen::Vector2i>,
             Eigen::aligned_allocator<
               std::pair<const Eigen::Vector2i, Vertex*> > > map_grid_ellipse_;

    std::list<LineGroup> line_groups_;
};

}
