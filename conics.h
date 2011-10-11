/**
 * @author  Steven Lovegrove
 *
 * Copyright (C) 2010  Steven Lovegrove
 *                     Imperial College London
 */

#ifndef CONICS_H
#define CONICS_H

#include <TooN/TooN.h>

#include "rectangle.h"
#include "camera.h"

struct Conic
{
  IRectangle bbox;
  TooN::Matrix<3,3> C;
  TooN::Matrix<3,3> Dual;
  TooN::Vector<2> center;
};

double Distance( const Conic& c1, const Conic& c2, double circle_radius );

TooN::Vector<2,std::pair<TooN::Vector<3>,TooN::Matrix<3,3> > > PlaneFromConic(
  const Conic& c, double plane_circle_radius, const TooN::Matrix<3,3>& K
);

std::pair<TooN::Vector<3>,TooN::Matrix<3,3> > PlaneFromConics(
  std::vector<Conic>& conics, double plane_circle_radius,
  const TooN::Matrix<3,3>& K, double inlier_threshold
);

Conic UnmapConic( const Conic& c, const AbstractCamera& cam );

#endif // CONICS_H
