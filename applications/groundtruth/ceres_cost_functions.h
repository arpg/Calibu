#pragma once
#include <calibu/cam/camera_crtp.h>
#include <calibu/cam/camera_models_crtp.h>
#include <calibu/cam/camera_crtp_interop.h>
#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>
#include <ceres/ceres.h>
#include "measurements.h"
#include "math.h"

/////////////////////////////////////////////////////////////////////////
template<typename CameraModel,typename Scalar=double>
struct ProjectionCostFunctor
{
  typedef Eigen::Matrix<Scalar, 2, 1> Vec2t;
  typedef Eigen::Matrix<Scalar, 3, 1> Vec3t;
  typedef Eigen::Matrix<Scalar, 3, 3> Mat3t;

  ProjectionCostFunctor(
      std::shared_ptr< detection > _d,
      const Mat3t& _k,
      Scalar* _params
      ) : d(_d), k(_k), params(_params)
  {
    //    t_vr = (calibu::RdfVision * calibu::RdfRobotics.inverse());
  }

  template<typename T>
    bool operator()(
        const T* const _t_wi,  // world pose of i'th camera
        T* residuals
        ) const 
    {
      CHECK_NOTNULL(_t_wi);
      CHECK_NOTNULL(residuals);

      const Eigen::Map< const Eigen::Matrix<T,6,1> > temp(_t_wi);
      const Sophus::SE3Group<T> t_wi = Sophus::SE3Group<T>( _Cart2T<T>(temp) );

      Eigen::Matrix<T,3,1> pwj = d->tag_data.tl.template cast<T>();
      Eigen::Matrix<T,2,1> zij = d->tag_corners.tl.template cast<T>();
      Eigen::Matrix<T,3,1> pij = t_wi.inverse() * pwj.template cast<T>();
      pij = (k.template cast<T>()) * pij;
      pij[0] /= pij[2];
      pij[1] /= pij[2];
      residuals[0] = (T)(zij(0) - pij[0]);
      residuals[1] = (T)(zij(1) - pij[1]);

      pwj = d->tag_data.tr.template cast<T>();
      zij = d->tag_corners.tr.template cast<T>();
      pij = t_wi.inverse() * pwj.template cast<T>();
      pij = (k.template cast<T>()) * pij;
      pij[0] /= pij[2];
      pij[1] /= pij[2];
      residuals[2] = (T)(zij(0) - pij[0]);
      residuals[3] = (T)(zij(1) - pij[1]);

      pwj = d->tag_data.bl.template cast<T>();
      zij = d->tag_corners.bl.template cast<T>();
      pij = t_wi.inverse() * pwj.template cast<T>();
      pij = (k.template cast<T>()) * pij;
      pij[0] /= pij[2];
      pij[1] /= pij[2];
      residuals[4] = (T)(zij(0) - pij[0]);
      residuals[5] = (T)(zij(1) - pij[1]);

      pwj = d->tag_data.br.template cast<T>();
      zij = d->tag_corners.br.template cast<T>();
      pij = t_wi.inverse() * pwj.template cast<T>();
      pij = (k.template cast<T>()) * pij;
      pij[0] /= pij[2];
      pij[1] /= pij[2];
      residuals[6] = (T)(zij(0) - pij[0]);
      residuals[7] = (T)(zij(1) - pij[1]);

      return true;
    }

  const Mat3t&                      k;
  std::shared_ptr< detection >      d;
  Scalar*                           params;
};


/////////////////////////////////////////////////////////////////////////
template<typename Scalar>
ceres::CostFunction* ProjectionCost(
    const std::shared_ptr< detection > _d,
    const Eigen::Matrix3d& _k,   // K Matrix
    calibu::CameraInterface<Scalar>* _cam
    )
{
  Scalar* _params = _cam->GetParams().data();
  if( dynamic_cast<calibu::LinearCamera<Scalar>*>( _cam ) ){
    typedef calibu::LinearCamera<Scalar> CamT;
    return (new ceres::AutoDiffCostFunction<ProjectionCostFunctor<CamT>,8,6>(
          new ProjectionCostFunctor<CamT>( _d,_k,_params ) ) );
  }
  else if( dynamic_cast<calibu::FovCamera<Scalar>*>( _cam ) ){
    typedef calibu::FovCamera<Scalar> CamT;
    return (new ceres::AutoDiffCostFunction<ProjectionCostFunctor<CamT>,8,6>(
          new ProjectionCostFunctor<CamT>( _d,_k,_params ) ) );
  }
  else if( dynamic_cast<calibu::Poly3Camera<Scalar>*>( _cam ) ){
    typedef calibu::Poly3Camera<Scalar> CamT;
    return (new ceres::AutoDiffCostFunction<ProjectionCostFunctor<CamT>,8,6>(
              new ProjectionCostFunctor<CamT>( _d,_k,_params ) ) );
  }
  return NULL;
}
