/**
 * @author  Steven Lovegrove
 *
 * Copyright (C) 2010  Steven Lovegrove
 *                     Imperial College London
 */

#include "conics.h"

#include <TooN/determinant.h>
#include <TooN/LU.h>
#include <TooN/SVD.h>

#include "utils.h"

using namespace std;
using namespace TooN;

double Distance( const Conic& c1, const Conic& c2, double circle_radius )
{
  const Matrix<3,3> Q = c1.Dual * c2.C;
  //  const double dsq = 3 - trace(Q)* pow(determinant(Q),-1.0/3.0);
  const double dsq = 3 - trace(Q)* 1.0 / cbrt(determinant(Q));
  return sqrt(dsq) * circle_radius;
}

Matrix<3,3> RotY(double theta)
{
  return Data(
    cos(theta),0,sin(theta),
    0,1,0,
    -sin(theta),0,cos(theta)
  );
}

double Cost( const pair<Vector<3>,Matrix<3,3> >& plane, const Conic& conic, const Matrix<3,3>& K )
{
  const Matrix<3,3> Cn = K.T() * conic.C *K;
  const Matrix<3,3>& R = plane.second;
  const Matrix<3,3> C_ = R.T() * Cn * R;
  const double a = C_[0][0];
//  const double b = C_[0][1] / 2.0;
  const double c = C_[1][1];
  const double amc = abs(a - c);
  return amc / max(abs(a),abs(c));;// + b*b;
}

double Cost( const pair<Vector<3>,Matrix<3,3> >& plane, const vector<Conic>& conics, const Matrix<3,3>& K, double threshold )
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

Vector<2,pair<Vector<3>,Matrix<3,3> > > PlaneFromConic(const Conic& c, double plane_circle_radius, const Matrix<3,3>& K )
{
  const Matrix<3,3> Cn = K.T() * c.C *K;
  SVD<3> svd(Cn);
  const Matrix<3,3> R1 = svd.get_U();
  const Vector<3> l = svd.get_diagonal();
//  const Matrix<3,3> C_ = svd.get_diagonal();
  const double t = atan( sqrt( (l[1]-l[0]) / (l[2]-l[1]) ) );

  Vector<2,pair<Vector<3>,Matrix<3,3> > > r;

  for( int i=0; i < 2; ++i )
  {
    const double theta = (i*2-1)*t;
    const Matrix<3,3> R2 = RotY(theta);
    const Matrix<3,3> R = R1 * R2;
//    const Matrix<3,3> C__ = R.T() * Cn * R;
    const Vector<3> n = R * makeVector(0,0,-1);
    const double d = sqrt( (l[1]*l[1])/(l[0]*l[2]) ) * plane_circle_radius;

    r[i].first = unit(n) / d;
    r[i].second = R;
  }

  return r;
}

pair<Vector<3>,Matrix<3,3> > PlaneFromConics( vector<Conic>& conics, double plane_circle_radius, const Matrix<3,3>& K, double inlier_threshold)
{
  double best_score = numeric_limits<double>::max();
  pair<Vector<3>,Matrix<3,3> > best;

  // Find transformation with lowest score over all conics
  for( unsigned int i=0; i < conics.size(); ++i )
  {
    const Vector<2,pair<Vector<3>,Matrix<3,3> > > nds = PlaneFromConic(conics[i],plane_circle_radius,K );
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

  // Given best transformation, perform outlier rejection
  for( int i = conics.size()-1; i >= 0; --i )
  {
    const double cost = Cost( best, conics[i],K);
    if( cost > inlier_threshold )
      conics.erase(conics.begin()+i);
  }

  return best;
}

Conic UnmapConic( const Conic& c, const AbstractCamera& cam )
{
  std::vector<TooN::Vector<2> > d;
  std::vector<TooN::Vector<2> > u;

  d.push_back(c.center);
  d.push_back(makeVector(c.bbox.x1,c.bbox.y1));
  d.push_back(makeVector(c.bbox.x1,c.bbox.y2));
  d.push_back(makeVector(c.bbox.x2,c.bbox.y1));
  d.push_back(makeVector(c.bbox.x2,c.bbox.y2));

  for( int i=0; i<5; ++i )
    u.push_back( cam.unmap(d[i]) );

  // Distortion locally estimated by homography
  const Matrix<3,3> H_du = EstimateH_ba(u,d);

  Conic ret;
//  ret.bbox = c.bbox;
  ret.C = H_du.T() * c.C * H_du;
  ret.Dual = LU<>(ret.C).get_inverse();
  ret.center = u[0];
  return ret;
}
