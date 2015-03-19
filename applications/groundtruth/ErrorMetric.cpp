#include "ErrorMetric.h"
#include <ceres/ceres.h>
#include "math.h"

//  These definitions are available in the paper: "A Benchmark for the
//  Evaluation of RGB-D SLAM Systems" by Sturm et al
template <typename Scalar=double>
struct XformCostFunctor
{
  XformCostFunctor( Sophus::SE3d _qi, Sophus::SE3d _p) : qi(_qi), p(_p) {}

  template< typename T>
  bool operator()( const T* const _s, T* residuals ) const
  {
    Sophus::SE3Group< T > x;
    const Eigen::Map<const Sophus::SE3Group<T> > S(_s);

    x = qi.template cast<T>() * S * p.template cast<T>();

    Eigen::Matrix< T, 6, 1> l = x.log();
    residuals[0] = l(0);
    residuals[1] = l(1);
    residuals[2] = l(2);
    residuals[3] = l(3);
    residuals[4] = l(4);
    residuals[5] = l(5);
    return true;
  }

  Sophus::SE3Group< Scalar > qi, p;
};

ErrorMetric::ErrorMetric()
{
}

double ErrorMetric::RPE(PL Q, PL P, int i, int delta )
{
  if (Q.size() != P.size()) {
    fprintf(stderr, "Differing number of poses for comparison.\n");
    fflush(stderr);
    return -1;
  }
  if (i + delta > Q.size()) {
    return -1;
  }
  Sophus::SE3d q = Q[i].inverse()*Q[i + delta];
  Sophus::SE3d p = P[i].inverse()*P[i + delta];
  Eigen::Vector3d t = (q.inverse()*p).translation();
  return t.squaredNorm();
}

double ErrorMetric::RMSE(PL Q, PL P, int delta )
{
  if (Q.size() != P.size()) {
    fprintf(stderr, "Differing number of poses for comparison.\n");
    fflush(stderr);
    return -1;
  }
  double sum = 0;
  for (int m = 0; m < Q.size() - delta; m++) {
    sum += RPE(Q, P, m, delta);
  }
  return sqrt( sum  / (Q.size() - delta));
}

double ErrorMetric::RMSE_ave(PL Q, PL P)
{
  if (Q.size() != P.size()) {
    fprintf(stderr, "Differing number of poses for comparison.\n");
    fflush(stderr);
    return -1;
  }
  double sum = 0;
  for (int n = 0; n < Q.size(); n++) {
    sum += RMSE(Q, P, n);
  }
  return ( sum  / Q.size());
}

double ErrorMetric::ATE(PL Q, PL P)
{
  Sophus::SE3d S = Q[0]*P[0].inverse();

  ceres::Problem problem;
  ceres::Solver::Options options;
  for (int ii = 0; ii < Q.size(); ii++) {
      ceres::CostFunction* cf =
          new ceres::AutoDiffCostFunction<XformCostFunctor<double>, 6, 1>
          (new XformCostFunctor<double>(Q[ii].inverse(), P[ii]));
            problem.AddResidualBlock( cf, NULL, S.data() );
  }
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  double sum = 0;
  for (int n = 0; n < Q.size(); n++) {
    sum += (Q[n].inverse() * S * P[n]).translation().squaredNorm();
  }
  return sqrt( sum  / Q.size());
}

void ErrorMetric::bring_to_frame(pl& q, pl p)
{
  PL Q, P;
  pl_to_PL(q, Q);
  pl_to_PL(p, P);

  Sophus::SE3d S = Q[0]*P[0].inverse();

  ceres::Problem problem;
  ceres::Solver::Options options;
  for (int ii = 0; ii < Q.size(); ii++) {
      ceres::CostFunction* cf =
          new ceres::AutoDiffCostFunction<XformCostFunctor<double>, 6, 1>
          (new XformCostFunctor<double>(Q[ii].inverse(), P[ii]));
            problem.AddResidualBlock( cf, NULL, S.data() );
  }
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  for (int n = 0; n < Q.size(); n++) {
    Q[n] = S.inverse() * Q[n];
  }
  PL_to_pl(Q, q);
}


void ErrorMetric::pl_to_PL(pl In, PL &out)
{
    out.clear();
    for (Eigen::Vector6d a : In) {
      out.push_back( Sophus::SE3d(_Cart2T(a)));
    }
}

void ErrorMetric::PL_to_pl(PL In, pl &out)
{
  out.clear();
  for (Sophus::SE3d a : In) {
    out.push_back( _T2Cart(a.matrix()));
  }
}

double ErrorMetric::RPE(pl Q, pl P, int i, int delta )
{
  PL q, p;
  pl_to_PL(Q, q);
  pl_to_PL(P, p);
  return RPE(q, p, i, delta);
}

double ErrorMetric::RMSE(pl Q, pl P, int delta )
{
  PL q, p;
  pl_to_PL(Q, q);
  pl_to_PL(P, p);
  return RMSE(q, p, delta);
}

double ErrorMetric::RMSE_ave(pl Q, pl P)
{
  PL q, p;
  pl_to_PL(Q, q);
  pl_to_PL(P, p);
  return RMSE_ave(q, p);
}

double ErrorMetric::ATE(pl Q, pl P)
{
  PL q, p;
  pl_to_PL(Q, q);
  pl_to_PL(P, p);
  return ATE(q, p);
}
