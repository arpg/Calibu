/**
 * @author  Steven Lovegrove, Richard Newcombe, Hauke Strasdat
 *
 * Copyright (C) 2010  Steven Lovegrove, Richard Newcombe, Hauke Strasdat
 *                     Imperial College London
 */

#include "target.h"

#include <algorithm>
#include <iostream>
#include <fstream>

#include <TooN/LU.h>

#include "hungarian.h"
#include "conics.h"
#include "ransac.h"

using namespace std;
using namespace TooN;
using namespace CVD;
using namespace RobotVision;

namespace RobotVision
{

Target::Target()
  : seed(0), dt(NULL)
{
  srand(seed);
}

void Target::SetSeed(int s )
{
  seed = s;
  srand(seed);
}

Matrix<> DistanceMatrix(const vector<Vector<2> >& pts )
{
  Matrix<> D(pts.size(),pts.size());
  for( unsigned int j=0; j < pts.size(); ++j )
    for( unsigned int i=0; i < pts.size(); ++i )
      D[j][i] = norm(pts[i] - pts[j]);
  return D;
}

void Target::Clear()
{
  // remove existing
  if( dt ) delete dt;
  tpts.clear();
  tpts3d.clear();
}

void Target::LoadPattern( std::string filename, double radius, double scale )
{
  Clear();

  this->radius = radius * scale;

  size[0] = numeric_limits<double>::min();
  size[1] = numeric_limits<double>::min();

  ifstream f;
  f.open(filename.c_str());
  while( !f.eof() )
  {
    Vector<3> p;
    f >> p[0];
    f >> p[1];
    f >> p[2];
    p *= scale;
    size[0] = max(size[0],p[0]);
    size[1] = max(size[1],p[1]);
    tpts3d.push_back(p);
    tpts.push_back(p.slice<0,2>());
  }

  cout << "Target loaded with " << tpts.size() << " circles." << endl;

  // Construct Distance matrix
  dt = new Matrix<>(DistanceMatrix(tpts));
  SortRows(*dt);
}


void Target::Generate(unsigned int max_circles, double radius, double min_distance, double border, const Vector<2>& size)
{
  Clear();

  const int MAX_FAILS = 1000;

  this->size = size;
  this->radius = radius;

  const double md2 = min_distance * min_distance;

  int fails = 0;

  // Generate circles

  while( tpts.size() < max_circles && fails < MAX_FAILS )
  {
    Vector<2> p = makeVector(
      (radius+border) + (double)rand() * ((double)size[0] - 2*(radius+border)) / (double)RAND_MAX,
      (radius+border) + (double)rand() * ((double)size[1] - 2*(radius+border)) / (double)RAND_MAX
    );
    bool good = true;
    for( unsigned int i=0; i<tpts.size(); ++i )
    {
      if( norm_sq(p - tpts[i]) < md2 )
      {
        good = false;
        break;
      }
    }
    if( good )
    {
      tpts.push_back(p);
      tpts3d.push_back(makeVector(p[0],p[1],0));
      fails = 0;
    }else{
      ++fails;
    }
  }

  cout << "Target generated with " << tpts.size() << " circles." << endl;

  // Construct Distance matrix
  dt = new Matrix<>(DistanceMatrix(tpts));
  SortRows(*dt);
}

void Target::SaveEPS(string filename)
{
  ofstream f;
  f.open(filename.c_str());
  f << "%!PS-Adobe EPSF-3.0" << endl;
  f << "%%Creator: FiducialCalibrationTarget" << endl;
  f << "%%Title: Calibration Target" << endl;
  f << "%%Origin: 0 0" << endl;
  f << "%%BoundingBox: 0 0 " << size[0] << " " << size[1] << endl;
  f << "% seed: " << seed << endl;
  f << "% radius: " << radius << endl;
  f << endl;

  for( unsigned int i=0; i<tpts.size(); ++i )
  {
    Vector<2>& p = tpts[i];
    f << p[0] << " " << size[1] - p[1] << " " << radius << " 0 360 arc closepath" << endl
      << "0.0 setgray fill" << endl
      << endl;
  }

  f.close();
}



struct RansacMatchData
{
  RansacMatchData(const vector<Vector<2> >& m, const vector<Vector<2> >& t, vector<int>& ml)
    :mpts(m),tpts(t),mpts_label(ml)
  {}

  const vector<Vector<2> >& mpts;
  const vector<Vector<2> >& tpts;
  vector<int>& mpts_label;
};

double RansacMatchCostFunction( const SE2<>& T, int i, RansacMatchData* data )
{
  const Vector<2> p = T * data->mpts[i];
  const double d = norm(data->tpts[data->mpts_label[i]] - p);
  return d*d;
}

SE2<> RansacMatchModelFunction( const std::vector<int>& indices, RansacMatchData* data )
{
  vector<const Vector<2>* > mp;
  vector<const Vector<2>* > tp;
  for( unsigned int k=0; k<indices.size(); ++k )
  {
    const int i = indices[k];
    mp.push_back( &data->mpts[i] );
    tp.push_back( &data->tpts[data->mpts_label[i]] );
  }
  return PoseFromCorrespondences(mp,tp);
}

SE2<> RansacMatchCorrespondences(
  const vector<Vector<2> >& measurement,
  const vector<Vector<2> >& target,
  vector<int>& measurement_label,
  int iterations,
  double max_point_fit_error,
  int min_consensus_size
) {
  RansacMatchData rmd(measurement,target,measurement_label);
  Ransac<SE2<>,2,RansacMatchData*> ransac( &RansacMatchModelFunction, &RansacMatchCostFunction, &rmd);

  vector<int> inliers;
  SE2<> T = ransac.Compute(
    measurement.size(),inliers,iterations,
    max_point_fit_error,min_consensus_size
  );

  // Remove outliers from map.
  for( unsigned int i=0; i<measurement.size(); ++i )
  {
    if( find(inliers.begin(),inliers.end(),i) == inliers.end() )
      measurement_label[i] = -1;
  }
  return T;
}

double DescriptorDist( const Vector<>& m, const Vector<>& t, int neighbours )
{
  const int dsize = std::min(m.size(),neighbours);
  return norm( m.slice(0,dsize) - t.slice(0,dsize) );
}

int ClosestPoint( const vector<Vector<2> >& t, const Vector<2>& p )
{
  double best_d = numeric_limits<double>::max();
  int best_i = -1;

  for( unsigned int i=0; i < t.size(); ++i )
  {
    const double d = norm(t[i] - p);
    if( d < best_d )
    {
      best_i = i;
      best_d = d;
    }
  }

  return best_i;
}

void ClosestPoints( const vector<Vector<2> >& a, const vector<Vector<2> >& b, vector<int>& a_map)
{
  for( unsigned i=0; i<a.size(); ++i )
  {
    a_map[i] = ClosestPoint(b,a[i]);
  }
}

void MutualClosest( const vector<Vector<2> >& a, const vector<Vector<2> >& b, vector<int>& a_map)
{
  vector<int> b_map(b.size(),-1);
  ClosestPoints(b,a,b_map);

  for( unsigned i=0; i<a.size(); ++i )
  {
    const int ai_close = ClosestPoint(b,a[i]);
    const int ai_close_close = ai_close >= 0 ? b_map[ai_close] : -1;
    a_map[i] = ai_close_close == (int)i ? ai_close : -1;
  }
}

void Target::Match( const vector<Vector<2> >& measurement, vector<int>& measurement_label, int match_neighbours  )
{
  Matrix<> dm = DistanceMatrix(measurement);
  SortRows(dm);
  Match(dm,measurement_label,match_neighbours);
}

void Target::Match( const TooN::Matrix<>& sorted_measurement_distance_matrix, std::vector<int>& measurement_label, int match_neighbours )
{
  const Matrix<>& dm = sorted_measurement_distance_matrix;
  const size_t msize = dm.num_rows();

  // Create cost matrix and padd with zeroes
  const size_t size = max(msize, tpts.size());
  Matrix<> cost(size,size);
  cost = Zeros;

  // Assign measurement to target association cost
  for( unsigned int j = 0; j < msize; ++j )
    for( unsigned int i = 0; i < tpts.size(); ++i )
      cost[j][i] = DescriptorDist( dm[j], (*dt)[i], match_neighbours );

  double* c_cost[cost.num_rows()];
  for( int i=0; i<cost.num_rows(); ++i )
    c_cost[i] = &cost[i][0];

  hungarian_problem_t hp;
  hungarian_init(&hp,c_cost,cost.num_rows(),cost.num_cols() );
  hungarian_solve(&hp);

  for( unsigned int j=0; j<msize; ++j )
  {
    const int label = hp.row_to_col_map[j];
    measurement_label[j] = label < (int)tpts.size() ? label : -1;
  }

  hungarian_free(&hp);
}

Vector<2> Mean( vector<Vector<2> >& pts )
{
  Vector<2> sum = Zeros;
  for( unsigned int i=0; i<pts.size(); ++i )
  {
    sum = sum + pts[i];
  }
  return sum / pts.size();
}


double RansacHomogCostFunction( const Matrix<3,3>& H_tm, int i, RansacMatchData* data )
{
  const Vector<2>& m = data->mpts[i];
  const Vector<2>& t = data->tpts[data->mpts_label[i]];
  const Vector<2> m_t = dn(H_tm * up(m));
  return norm_sq(m_t - t);
}

Matrix<3,3> RansacHomogModelFunction( const std::vector<int>& indices, RansacMatchData* data )
{
  std::vector<Vector<2> > mpts;
  std::vector<Vector<2> > tpts;

  for( unsigned int i=0; i< indices.size(); ++i )
  {
    const int mi = indices[i];
    const Vector<2>& m = data->mpts[mi];
    const Vector<2>& t = data->tpts[data->mpts_label[mi]];
    mpts.push_back(m);
    tpts.push_back(t);
  }

  return EstimateH_ba(mpts,tpts);
}

void Target::FindTarget(
  const SE3<>& T_cw,
  const AbstractCamera& cam,
  vector<Conic>& conics,
  vector<int>& conics_target_map,
  int match_neighbours,
  int ransac_iterations,
  int ransac_min_ellipses,
  double ransac_max_fit_error,
  double plane_inlier_threshold
) {
  // We have conic centers, and projected centers. Try to match

  const IRectangle img_rect( 0,0,cam.width(),cam.height());

  vector<Vector<2> > vis_t;
  vector<int> vis_t_map;
  for( unsigned int i=0; i < tpts3d.size(); ++i )
  {
    const Vector<2> t = cam.projectmap( T_cw * tpts3d[i] );
    if( img_rect.Contains(t) )
    {
      vis_t.push_back(t);
      vis_t_map.push_back(i);
    }
  }

  vector<Vector<2> > m;
  for( unsigned i=0; i<conics.size(); ++i )
    m.push_back(conics[i].center);

  vector<int> m_map(m.size(),-1);
  MutualClosest(m,vis_t,m_map);

  for(unsigned i=0; i<m.size(); ++i )
    conics_target_map[i] = m_map[i] >= 0 ? vis_t_map[m_map[i]] : -1;
}

void Target::FindTarget(
  const LinearCamera& cam,
  vector<Conic>& conics,
  vector<int>& conics_target_map,
  int match_neighbours,
  int ransac_iterations,
  int ransac_min_ellipses,
  double ransac_max_fit_error,
  double plane_inlier_threshold
) {

//  // Compute distance matrix for observed conics
//  Matrix<> ellipse_dm(conics.size(),conics.size());
//  ellipse_dm = Zeros;
//  for( unsigned int j=0; j<conics.size(); ++j)
//  {
//    for( unsigned int i=0; i < j; ++i)
//    {
//      const double d = Distance(conics[j],conics[i],radius);
//      ellipse_dm[j][i] = d;
//      ellipse_dm[i][j] = d;
//    }
//    if( !isfinite(conics[j].center) )
//      cout << conics[j].C << endl;
//    ellipses.push_back(conics[j].center);
//    ellipse_target_map.push_back(-1);
//  }
//  SortRows(ellipse_dm);
//
//  // Match using Hungarian method
//  Match(ellipse_dm,ellipse_target_map,match_neighbours);
//
//  // RANSAC Consistent Homography
//  RansacMatchData rmd(ellipses,tpts,ellipse_target_map);
//  Ransac<Matrix<3,3>,5,RansacMatchData*> ransac( &RansacHomogModelFunction, &RansacHomogCostFunction, &rmd);
//
//  vector<int> inliers;
//  Matrix<3,3> H_tm = ransac.Compute(
//    ellipses.size(),inliers,ransac_iterations,
//    ransac_max_fit_error,ransac_min_ellipses
//  );
//
//  // strip outliers from map
//  for( unsigned int i=0; i<ellipses.size(); ++i )
//  {
//    if( find(inliers.begin(),inliers.end(),i) == inliers.end() )
//      ellipse_target_map[i] = -1;
//  }


  // Compute metric positions in 2D
  vector<Vector<2> >  mpts;

  pair<Vector<3>,Matrix<3,3> > plane = PlaneFromConics(conics,radius,cam.K(), plane_inlier_threshold);
  const Vector<4> N_w = ToN(plane.first);
  if( !TooN::isfinite(N_w) )
    return;

  for( unsigned int i=0; i< conics.size(); ++i )
  {
    const Vector<2> ud = conics[i].center;
    const Vector<3> p3d = IntersectCamFeaturePlane(ud,cam,SE3<>(),N_w);
    const Vector<2> pnorm = (plane.second.T() * p3d).slice<0,2>();
    mpts.push_back(pnorm);
  }

  if( conics.size() >= 2 )
  {
    Matrix<> ellipse_dm = DistanceMatrix(mpts);
    SortRows(ellipse_dm);

    // Match using Hungarian method
    Match(ellipse_dm,conics_target_map,match_neighbours);

    SE2<> T_tm;

    // Perform RANSAC to remove outliers
    if( ransac_iterations )
    {
      T_tm = RansacMatchCorrespondences(
        mpts,tpts,conics_target_map,ransac_iterations,
        ransac_max_fit_error,ransac_min_ellipses
      );
    }

    // Given inliers and 2D transform, find more correspondences
    for( unsigned int i=0; i< mpts.size(); ++i )
    {
      // For each point without a match
      if( conics_target_map[i] < 0 )
      {
        // find closest point
        const Vector<2> m_t = T_tm * mpts[i];
        const int t = ClosestPoint(tpts, m_t );

        assert( t >= 0 && t < (int)tpts.size() );

        const double d = norm(m_t - tpts[t]);

        // check error is small
        if( d*d < ransac_max_fit_error )
        {
          // check target circle hasn't already been matched
          if( find(conics_target_map.begin(),conics_target_map.end(),t) == conics_target_map.end() )
          {
            conics_target_map[i] = t;
          }
        }
      }
    }
  }

}

}
