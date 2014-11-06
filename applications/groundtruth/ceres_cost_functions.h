#pragma once
#include <calibu/cam/camera_crtp.h>
#include <calibu/cam/camera_models_crtp.h>
#include <calibu/cam/camera_crtp_interop.h>
#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>
#include <ceres/ceres.h>
#include "math.h"

/////////////////////////////////////////////////////////////////////////
template<typename CameraModel,typename Scalar=double>
struct ProjectionCostFunctor
{
  typedef Eigen::Matrix<Scalar, 2, 1> Vec2t;
  typedef Eigen::Matrix<Scalar, 3, 1> Vec3t;
  typedef Eigen::Matrix<Scalar, 3, 3> Mat3t;

  ProjectionCostFunctor(
      const Vec3t& _pwj,  // 3D point j in the world
      const Vec2t& _zij,  // 2D image measurement of j from camera i
      const Mat3t& _k,
      Scalar* _params
      ) : pwj(_pwj), zij(_zij), k(_k), params(_params)
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

      // get point j infront of camera i
      Eigen::Matrix<T,3,1> pij = t_wi.inverse() * pwj.template cast<T>();
      pij = (k.template cast<T>()) * pij;
      T hij[2];

      pij[0] /= pij[2];
      pij[1] /= pij[2];

//      T fac = (T) 1;
//      const T param = (T) params[4];
//      if (param * param > 1e-5) {
//        const T mul2_tanw_by2 = (T)2.0 * tan(param / (T)2.0);
//        T rad = ceres::sqrt(pij[0]*pij[0] + pij[1]*pij[1]);
//        if (rad * rad < 1e-5) {
//          fac = mul2_tanw_by2 / param;
//        }
//        fac = atan(rad * mul2_tanw_by2) / (rad * param);
//      }

//      T pix_k[2];
//      pix_k[0] = fac * params[0] * pij[0] + params[2];
//      pix_k[1] = fac * params[1] * pij[1] + params[3];

      //      r = zij - hij;
//      residuals[0] = (T)(zij(0) - pix_k[0]);
//      residuals[1] = (T)(zij(1) - pix_k[1]);
      residuals[0] = (T)(zij(0) - pij[0]);
      residuals[1] = (T)(zij(1) - pij[1]);
      return true;
    }

  const Eigen::Matrix<Scalar,3,1>&  pwj; // point j in world frame
  const Eigen::Matrix<Scalar,2,1>&  zij; // measurement of i from frame j
  const Mat3t&                      k;
  Scalar*                           params;
};


/////////////////////////////////////////////////////////////////////////
template<typename Scalar>
ceres::CostFunction* ProjectionCost(
    const Eigen::Vector3d& _pwj, // 3D point j in the world
    const Eigen::Vector2d& _zij, // 2D image measurement of j from camrea i
    const Eigen::Matrix3d& _k,   // K Matrix
    calibu::CameraInterface<Scalar>* _cam
    )
{
  Scalar* _params = _cam->GetParams().data();
  if( dynamic_cast<calibu::LinearCamera<Scalar>*>( _cam ) ){
    typedef calibu::LinearCamera<Scalar> CamT;
    return (new ceres::AutoDiffCostFunction<ProjectionCostFunctor<CamT>,2,6>(
          new ProjectionCostFunctor<CamT>( _pwj,_zij,_k,_params ) ) );
  }
  else if( dynamic_cast<calibu::FovCamera<Scalar>*>( _cam ) ){
    typedef calibu::FovCamera<Scalar> CamT;
    return (new ceres::AutoDiffCostFunction<ProjectionCostFunctor<CamT>,2,6>(
          new ProjectionCostFunctor<CamT>( _pwj,_zij,_k,_params ) ) );
  }
  return NULL;
}
