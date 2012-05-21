/* This file is part of the fiducials Project.
 * https://github.com/stevenlovegrove/fiducials
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

#ifndef TARGET_H
#define TARGET_H

#include <algorithm>

#include <Eigen/Dense>
#include <sophus/se3.h>

#include "conics.h"
#include "camera.h"

class Target
{
public:
  Target();

  void SetSeed(int s );
  void GenerateCircular(unsigned int max_circles, double radius, double min_distance, double border, const Eigen::Vector2d& size );
  void GenerateEmptyCircle(unsigned int max_circles, double radius, double min_distance, double border, double clear_radius, const Eigen::Vector2d& size );
  void GenerateRandom(unsigned int max_circles, double radius, double min_distance, double border, const Eigen::Vector2d& size );
  void LoadPattern( std::string filename, double radius, double scale = 1.0 );
  void SaveEPS( std::string filename );
  void SaveRotatedEPS( std::string filename);

  Eigen::Vector2d Size() const;

  double Radius() const;

  const std::vector<Eigen::Vector2d >& circles() const;
  const std::vector<Eigen::Vector3d >& circles3D() const;

  void FindTarget(
    const Sophus::SE3& T_cw,
    const AbstractCamera& cam,
    std::vector<Conic>& conics,
    std::vector<int>& ellipse_target_map
  );

  void FindTarget(
    const LinearCamera& cam,
    std::vector<Conic>& conics,
    std::vector<int>& ellipse_target_map,
    int match_neighbours = 10,
    int ransac_iterations = 100,
    int ransac_min_ellipses = 10,
    double ransac_max_fit_error = 2000,
    double plane_inlier_threshold = 1.5,
    bool use_mirror = false
  );

protected:
  void Clear();
  void InitialiseFrom2DPts();
  void Match(
    const Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>& sorted_measurement_distance_matrix,
    std::vector<int>& measurement_label, int match_neighbours
  );
  void Match( const std::vector<Eigen::Vector2d >& measurement, std::vector<int>& measurement_label, int match_neighbours  );

  unsigned int seed;
  Eigen::Vector2d size;
  double radius;
  std::vector<Eigen::Vector2d > tpts;
  std::vector<Eigen::Vector2d > tpts_reflected;
  std::vector<Eigen::Vector3d > tpts3d;
  Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>* dt;
};

// Utilities

Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>
  DistanceMatrix(const std::vector<Eigen::Vector2d >& pts );

template<int R,int C,typename P>
void SortRows(Eigen::Matrix<P,R,C>& M);

// Inlines

inline Eigen::Vector2d Target::Size() const
{
  return size;
}

inline const std::vector<Eigen::Vector2d >& Target::circles() const
{
  return tpts;
}

inline const std::vector<Eigen::Vector3d >& Target::circles3D() const
{
  return tpts3d;
}

inline double Target::Radius() const
{
  return radius;
}

template<int R,int C,typename P>
void SortRows(Eigen::Matrix<P,R,C,Eigen::RowMajor>& M)
{
  for( int r=0; r < M.rows(); ++r )
    std::sort(&(M(r,0)), &(M(r,M.cols()-1))+1);
}

#endif // TARGET_H
