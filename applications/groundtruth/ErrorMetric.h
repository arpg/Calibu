#ifndef ERRORMETRIC_H
#define ERRORMETRIC_H

#include <sophus/se3.hpp>
#include <vector>
#include <Eigen/Eigen>

typedef std::vector< Sophus::SE3d > PL;
typedef std::vector< Eigen::Matrix<double, 6, 1> > pl;

class ErrorMetric
{
public:
  ErrorMetric();

  static double RPE(PL Q, PL P, int i, int delta );
  static double RMSE(PL Q, PL P, int delta );
  static double RMSE_ave(PL Q, PL P);
  static double ATE(PL Q, PL P);

  static double RPE(pl Q, pl P, int i, int delta );
  static double RMSE(pl Q, pl P, int delta );
  static double RMSE_ave(pl Q, pl P);
  static double ATE(pl Q, pl P);
  static void   bring_to_frame(pl& q, pl p);

private:
  static void pl_to_PL( pl In, PL& out);
  static void PL_to_pl( PL In, pl& out);

};

#endif // ERRORMETRIC_H
