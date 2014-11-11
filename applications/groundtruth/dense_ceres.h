#pragma once
#include <calibu/cam/camera_crtp.h>
#include <calibu/cam/camera_models_crtp.h>
#include <calibu/cam/camera_crtp_interop.h>
#include <sophus/se3.hpp>
#include <ceres/ceres.h>
#include "math.h"
#include "measurements.h"
#include <SceneGraph/SceneGraph.h>
#include <opencv2/opencv.hpp>

template <typename Scalar>
double diff( cv::Mat img_a,
             cv::Mat img_b,
             Scalar p[4][2])
{
  int x1, x2, y1, y2;

  x1 = std::min(p[0][0], std::min(p[1][0], std::min(p[2][0], p[3][0])));
  y1 = std::min(p[0][1], std::min(p[1][1], std::min(p[2][1], p[3][1])));

  x2 = std::max(p[0][0], std::max(p[1][0], std::max(p[2][0], p[3][0])));
  y2 = std::max(p[0][1], std::max(p[1][1], std::max(p[2][1], p[3][1])));

  x1 = std::max(0, x1);
  y1 = std::max(0, y1);
  x2 = std::min(x2, img_a.cols);
  y2 = std::min(y2, img_a.rows);

  cv::Mat mask(img_a.rows, img_a.cols, img_a.type());
  mask = cv::Scalar(0);
  std::vector< std::vector< cv::Point > > pts;
  std::vector< cv::Point > ps;
  ps.push_back( cv::Point(p[0][0], p[0][1]));
  ps.push_back( cv::Point(p[1][0], p[1][1]));
  ps.push_back( cv::Point(p[2][0], p[2][1]));
  ps.push_back( cv::Point(p[3][0], p[3][1]));
  pts.push_back(ps);
  cv::fillPoly(mask, pts, 255);

  float sum = 0;
  int count = 0;
  for (int jj = (int) y1; jj <= (int) y2; jj++) {
    for (int ii = (int) x1; ii <= (int) x2; ii++) {
      if ( mask.at<uchar>(jj, ii) != 0) {
        count++;
        sum += pow(img_a.at<uchar>(jj, ii) - img_b.at<uchar>(jj, ii) , 2);
      }
    }
  }

  sum /= (1.0f*count);

  return sum;
}


template<typename CameraModel,typename Scalar=double>
struct PhotometricCostFunctor
{
  PhotometricCostFunctor(
      std::shared_ptr< detection> _d, // detection
      SceneGraph::GLSimCam* _cam,
      const Eigen::Matrix3d _k,
      double _pts[4][2],
      bool dbug
      ) : det(_d), simcam(_cam), k(_k), debug(dbug)
  {
    for (int x = 0; x < 4; x++) {
      pts[x][0] = _pts[x][0];
      pts[x][1] = _pts[x][1];
    }
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

    if (debug) {
      fprintf(stdout, "Pose : <%f, %f, %f, %f, %f, %f>\n", temp(0),
              temp(1), temp(2), temp(3), temp(4), temp(5) );
      fflush(stdout);
    }


    cv::Mat img(det->image.rows, det->image.cols, det->image.type());
    simcam->SetPoseVision(t_wi.matrix());
    simcam->RenderToTexture();
    simcam->CaptureGrey( img.data );


    residuals[0] = (T)( diff( img, det->image, pts) );
    return true;
  }

  std::shared_ptr< detection >          det;
  SceneGraph::GLSimCam*                 simcam;
  calibu::CameraInterface<Scalar>*      cmod;
  Eigen::Matrix3d                       k;
  double                                pts[4][2];
  bool                                  debug;
};

template<typename Scalar>
ceres::CostFunction* PhotometricCost(
    std::shared_ptr< detection> _d, // detection
    SceneGraph::GLSimCam* _sim_cam,
    Eigen::Matrix3d _k,
    calibu::CameraInterface<Scalar>* _cam,
    double pts[4][2],
    bool debug = false
    )
{
  Scalar* _params = _cam->GetParams().data();
  if( dynamic_cast<calibu::LinearCamera<Scalar>*>( _cam ) ){
    typedef calibu::LinearCamera<Scalar> CamT;
    return (new ceres::NumericDiffCostFunction<PhotometricCostFunctor<CamT>, ceres::CENTRAL, 1,6>(
              new PhotometricCostFunctor<CamT>( _d,_sim_cam,_k, pts, debug ) ) );
  }
  else if( dynamic_cast<calibu::FovCamera<Scalar>*>( _cam ) ){
    typedef calibu::FovCamera<Scalar> CamT;
    return (new ceres::NumericDiffCostFunction<PhotometricCostFunctor<CamT>,ceres::CENTRAL, 1,6>(
              new PhotometricCostFunctor<CamT>( _d,_sim_cam,_k, pts, debug ) ) );
  }
  return NULL;
}


