/* This file is part of the calibu Project.
 * https://github.com/stevenlovegrove/calibu
 *
 * Copyright (C) 2010  Steven Lovegrove, Richard Newcombe, Hauke Strasdat
 *                     Imperial College London
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

#include <Eigen/Dense>
#include <sophus/se3.hpp>
#include "target.h"

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

class TargetRandomDot
        : public TargetInterface
{
public:
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
    
    const std::vector<Eigen::Vector2d >& Circles2D() const {
        return tpts;
    }
    
    const std::vector<Eigen::Vector3d >& Circles3D() const {
        return tpts3d;
    }
    
    ////////////////////////////////////////////////////////////////////////////
    
    inline Eigen::Vector2d Size() const {
        return size;
    }
    
    inline double Radius() const {
        return radius;
    }
    
protected:
    void Clear();
    void InitialiseFrom2DPts();
    void Match(
            const Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>& sorted_measurement_distance_matrix,
            std::vector<int>& measurement_label, int match_neighbours
            ) const;
    void Match( const std::vector<Eigen::Vector2d >& measurement, std::vector<int>& measurement_label, int match_neighbours  ) const;
    
    unsigned int seed;
    Eigen::Vector2d size;
    double radius;
    std::vector<Eigen::Vector2d > tpts;
    std::vector<Eigen::Vector2d > tpts_reflected;
    std::vector<Eigen::Vector3d > tpts3d;
    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>* dt;
    
    ParamsRandomDot params;
};

}
