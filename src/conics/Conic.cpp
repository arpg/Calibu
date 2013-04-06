/* This file is part of the fiducials Project.
 * https://github.com/stevenlovegrove/fiducials
 *
 * Copyright (c) 2011 Steven Lovegrove
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

#include <fiducials/conics/Conic.h>
#include <fiducials/utils/Utils.h>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

namespace fiducials {

double Distance( const Conic& c1, const Conic& c2, double circle_radius )
{
  const Matrix3d Q = c1.Dual * c2.C;
  //  const double dsq = 3 - trace(Q)* pow(determinant(Q),-1.0/3.0);
  const double dsq = 3 - Q.trace()* 1.0 / cbrt(Q.determinant());
  return sqrt(dsq) * circle_radius;
}

Matrix3d RotY(double theta)
{
  Matrix3d R;
  R <<
    cos(theta),0,sin(theta),
    0,1,0,
    -sin(theta),0,cos(theta);
  return R;
}

double Cost( const pair<Vector3d,Matrix3d >& plane, const Conic& conic, const Matrix3d& K )
{
  const Matrix3d Cn = K.transpose() * conic.C *K;
  const Matrix3d& R = plane.second;
  const Matrix3d C_ = R.transpose() * Cn * R;
  const double a = C_(0,0);
//  const double b = C_[0][1] / 2.0;
  const double c = C_(1,1);
  const double amc = abs(a - c);
  return amc / max(abs(a),abs(c));;// + b*b;
}

double Cost( const pair<Vector3d,Matrix3d >& plane, const vector<Conic>& conics, const Matrix3d& K, double threshold )
{
  int n = 0;
  double sum = 0;
  for( unsigned int k=0; k < conics.size(); ++ k )
  {
    const double cost = Cost(plane,conics[k],K);
    if( cost < threshold )
    {
      ++n;
      sum += cost;
    }
  }
  return sum / n;
}

std::array<pair<Vector3d,Matrix3d >, 2 > PlaneFromConic(const Conic& c, double plane_circle_radius, const Matrix3d& K )
{
  const Matrix3d Cn = K.transpose() * c.C *K;
  Eigen::JacobiSVD<Matrix3d> svd(Cn, ComputeFullU );
  const Matrix3d R1 = svd.matrixU();
  const Vector3d l = svd.singularValues();
  const double t = atan( sqrt( (l[1]-l[0]) / (l[2]-l[1]) ) );

  std::array<pair<Vector3d,Matrix3d >, 2> r;

  for( int i=0; i < 2; ++i )
  {
    const double theta = (i*2-1)*t;
    const Matrix3d R2 = RotY(theta);
    const Matrix3d R = R1 * R2;
//    const Matrix3d C__ = R.T() * Cn * R;
    const Vector3d n = R * Vector3d(0,0,-1);
    const double d = sqrt( (l[1]*l[1])/(l[0]*l[2]) ) * plane_circle_radius;

    r[i].first = n.normalized() / d;
    r[i].second = R;
  }

  return r;
}

pair<Vector3d,Matrix3d > PlaneFromConics( const vector<Conic>& conics, double plane_circle_radius, const Matrix3d& K, double inlier_threshold)
{
  double best_score = numeric_limits<double>::max();
  pair<Vector3d,Matrix3d > best;

  // Find transformation with lowest score over all conics
  for( unsigned int i=0; i < conics.size(); ++i )
  {
    const std::array<pair<Vector3d,Matrix3d >, 2> nds = PlaneFromConic(conics[i],plane_circle_radius,K );
    for( int j=0; j<2; ++j )
    {
      const double cost = Cost(nds[j],conics,K,inlier_threshold);
      if( cost < best_score )
      {
        best_score = cost;
        best = nds[j];
      }
    }
  }

//  // Given best transformation, perform outlier rejection
//  for( int i = conics.size()-1; i >= 0; --i )
//  {
//    const double cost = Cost( best, conics[i],K);
//    if( cost > inlier_threshold )
//      conics.erase(conics.begin()+i);
//  }

  return best;
}

Conic UnmapConic( const Conic& c, const CameraModelBase& cam )
{
  std::vector<Eigen::Vector2d > d;
  std::vector<Eigen::Vector2d > u;

  d.push_back(c.center);
  d.push_back(Eigen::Vector2d(c.bbox.x1,c.bbox.y1));
  d.push_back(Eigen::Vector2d(c.bbox.x1,c.bbox.y2));
  d.push_back(Eigen::Vector2d(c.bbox.x2,c.bbox.y1));
  d.push_back(Eigen::Vector2d(c.bbox.x2,c.bbox.y2));

  for( int i=0; i<5; ++i )
    u.push_back( cam.Unmap(d[i]) );

  // Distortion locally estimated by homography
  const Matrix3d H_du = EstimateH_ba(u,d);

  Conic ret;
//  ret.bbox = c.bbox;
  ret.C = H_du.transpose() * c.C * H_du;
  ret.Dual = ret.C.inverse();
  ret.center = u[0];
  return ret;
}

}
