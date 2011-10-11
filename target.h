/**
 * @author  Steven Lovegrove, Richard Newcombe, Hauke Strasdat
 *
 * Copyright (C) 2010  Steven Lovegrove, Richard Newcombe, Hauke Strasdat
 *                     Imperial College London
 */

#ifndef TARGET_H
#define TARGET_H

#include <algorithm>

#include <TooN/TooN.h>
#include <TooN/se3.h>

#include "conics.h"
#include "camera.h"

class Target
{
public:
  Target();

  void SetSeed(int s );
  void Generate(unsigned int max_circles, double radius, double min_distance, double border, const TooN::Vector<2>& size );
  void LoadPattern( std::string filename, double radius, double scale = 1.0 );
  void SaveEPS( std::string filename );

  TooN::Vector<2> Size() const;

  double Radius() const;

  const std::vector<TooN::Vector<2> >& circles() const;
  const std::vector<TooN::Vector<3> >& circles3D() const;

  void FindTarget(
    const TooN::SE3<>& T_cw,
    const AbstractCamera& cam,
    std::vector<Conic>& conics,
    std::vector<int>& ellipse_target_map,
    int match_neighbours = 10,
    int ransac_iterations = 100,
    int ransac_min_ellipses = 10,
    double ransac_max_fit_error = 2000,
    double plane_inlier_threshold = 1.5
  );

  void FindTarget(
    const LinearCamera& cam,
    std::vector<Conic>& conics,
    std::vector<int>& ellipse_target_map,
    int match_neighbours = 10,
    int ransac_iterations = 100,
    int ransac_min_ellipses = 10,
    double ransac_max_fit_error = 2000,
    double plane_inlier_threshold = 1.5
  );

protected:
  void Clear();
  void Match( const TooN::Matrix<>& sorted_measurement_distance_matrix, std::vector<int>& measurement_label, int match_neighbours  );
  void Match( const std::vector<TooN::Vector<2> >& measurement, std::vector<int>& measurement_label, int match_neighbours  );

  unsigned int seed;
  TooN::Vector<2> size;
  double radius;
  std::vector<TooN::Vector<2> > tpts;
  std::vector<TooN::Vector<3> > tpts3d;
  TooN::Matrix<>* dt;
};

// Utilities

TooN::Matrix<> DistanceMatrix(const std::vector<TooN::Vector<2> >& pts );

template<int R,int C,typename P>
void SortRows(TooN::Matrix<R,C,P>& M);

// Inlines

inline TooN::Vector<2> Target::Size() const
{
  return size;
}

inline const std::vector<TooN::Vector<2> >& Target::circles() const
{
  return tpts;
}

inline const std::vector<TooN::Vector<3> >& Target::circles3D() const
{
  return tpts3d;
}

inline double Target::Radius() const
{
  return radius;
}

template<int R,int C,typename P>
void SortRows(TooN::Matrix<R,C,P>& M)
{
  for( int r=0; r < M.num_rows(); ++r )
    std::sort(&(M[r][0]), &(M[r][M.num_cols()-1])+1);
}

#endif // TARGET_H
