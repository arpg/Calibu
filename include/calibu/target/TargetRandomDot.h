/*
   This file is part of the Calibu Project.
   https://github.com/gwu-robotics/Calibu

   Copyright (C) 2013 George Washington University,
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

#include <Eigen/Dense>
#include <sophus/se3.hpp>

#include <calibu/Platform.h>
#include <calibu/target/Target.h>

namespace calibu {

struct ParamsRandomDot
{
    ParamsRandomDot()
        : match_neighbours(10),
          ransac_its(100),
          ransac_min_pts(5),
          ransac_max_inlier_err_mm(15),
          plane_inlier_thresh(1.5)
    {
    }

    int match_neighbours;
    int ransac_its;
    int ransac_min_pts;
    float ransac_max_inlier_err_mm;
    float plane_inlier_thresh;
};

CALIBU_EXPORT
class TargetRandomDot
        : public TargetInterface
{
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    TargetRandomDot();

    ////////////////////////////////////////////////////////////////////////////
    void SetSeed(int s );
    void GenerateCircular(unsigned int max_circles, double radius, double min_distance, double border, const Eigen::Vector2d& size );
    void GenerateEmptyCircle(unsigned int max_circles, double radius, double min_distance, double border, double clear_radius, const Eigen::Vector2d& size );
    void GenerateRandom(unsigned int max_circles, double radius, double min_distance, double border, const Eigen::Vector2d& size );
    void LoadPattern( std::string filename, double radius, double scale = 1.0 );
    bool LoadEPS( std::string filename, float points_per_unit =1 );
    void SaveEPS( std::string filename, float points_per_unit =1 );
    void SaveRotatedEPS( std::string filename, float points_per_unit=1);

    ////////////////////////////////////////////////////////////////////////////

    template<typename Scalar=double>
    bool FindTarget(
            const Sophus::SE3d& T_cw,
            const CameraInterface<Scalar>& cam,
            const ImageProcessing& images,
            const std::vector<Conic, Eigen::aligned_allocator<Conic> >& conics,
            std::vector<int>& ellipse_target_map
            );

    template<typename Scalar=double>
    bool FindTarget(
            const CameraInterface<Scalar>& cam,
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

    double CircleRadius() const {
        return radius;
    }

    inline double CircleRadius(uint32_t circle_index) const
    {
        return CircleRadius();
    }

  const std::vector<Eigen::Vector2d,
                    Eigen::aligned_allocator<Eigen::Vector2d> >& Circles2D() const {
        return tpts;
    }

  const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& Circles3D() const {
        return tpts3d;
    }

  const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& Code3D() const {
        return codepts3d;
    }

    ////////////////////////////////////////////////////////////////////////////

    inline Eigen::Vector2d Size() const {
        return size;
    }

protected:
    void Clear();
    void InitializeFrom2DPts();
    void Match(
            const Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>& sorted_measurement_distance_matrix,
            std::vector<int>& measurement_label, int match_neighbours
            ) const;
  void Match(const std::vector<Eigen::Vector2d,
             Eigen::aligned_allocator<Eigen::Vector2d> >& measurement,
             std::vector<int>& measurement_label, int match_neighbours  ) const;

    unsigned int seed;
    Eigen::Vector2d size;
    double radius;
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > tpts;
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > tpts_reflected;
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > tpts3d;
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > codepts3d;
    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>* dt;

    ParamsRandomDot params;
};

}
