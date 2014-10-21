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
             std::shared_ptr< detection > d,
//             calibu::CameraInterface<Scalar>* cm,
             const Eigen::Matrix3d &k,
             Sophus::SE3Group<Scalar> t,
             bool debug)
{
  int min, max;
  min = 255;
  max = 0;
  int x1, x2, y1, y2;
  double p[4][2];

  cv::Mat img_b = d->image;

  // These corners should really be determined from the pixel information
  Eigen::Matrix<Scalar, 3, 1> temp;

  temp = (k.template cast<Scalar>()) * (t.inverse() * d->tag_data.tl);
  d->tag_data.tl(0) = temp(0) / temp(2);
  d->tag_data.tl(1) = temp(1) / temp(2);

  temp = (k.template cast<Scalar>()) * (t.inverse() * d->tag_data.tr);
  d->tag_data.tr(0) = temp(0) / temp(2);
  d->tag_data.tr(1) = temp(1) / temp(2);

  temp = (k.template cast<Scalar>()) * (t.inverse() * d->tag_data.bl);
  d->tag_data.bl(0) = temp(0) / temp(2);
  d->tag_data.bl(1) = temp(1) / temp(2);

  temp = (k.template cast<Scalar>()) * (t.inverse() * d->tag_data.br);
  d->tag_data.br(0) = temp(0) / temp(2);
  d->tag_data.br(1) = temp(1) / temp(2);


  p[0][0] = d->tag_corners.tl(0);
  p[0][1] = d->tag_corners.tl(1);

  p[1][0] = d->tag_corners.bl(0);
  p[1][1] = d->tag_corners.bl(1);

  p[2][0] = d->tag_corners.br(0);
  p[2][1] = d->tag_corners.br(1);

  p[3][0] = d->tag_corners.tr(0);
  p[3][1] = d->tag_corners.tr(1);

  x1 = std::min(p[0][0], std::min(p[1][0], std::min(p[2][0], p[3][0])));
  y1 = std::min(p[0][1], std::min(p[1][1], std::min(p[2][1], p[3][1])));

  x2 = std::max(p[0][0], std::max(p[1][0], std::max(p[2][0], p[3][0])));
  y2 = std::max(p[0][1], std::max(p[1][1], std::max(p[2][1], p[3][1])));

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

  cv::Mat temp1 = img_a + img_b;
//  cv::imshow("img a", img_a);
//  cv::imshow("img b", img_b);
  if(debug){
    cv::imshow("sum", temp1);
    cv::waitKey();
  }

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

  sum /= count;

  fprintf(stderr, "Sum = %f\n", sum);
  fflush(stderr);

  return sum;
}


template<typename CameraModel,typename Scalar=double>
struct PhotometricCostFunctor
{
  PhotometricCostFunctor(
      std::shared_ptr< detection> _d, // detection
      SceneGraph::GLSimCam* _cam,
      const Eigen::Matrix3d _k,
      bool dbug
      ) : det(_d), simcam(_cam), k(_k), debug(dbug)
  {
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

      fprintf(stdout, "Pose : <%f, %f, %f, %f, %f, %f>\n", temp(0),
              temp(1), temp(2), temp(3), temp(4), temp(5) );
      fflush(stdout);


      cv::Mat img(det->image.rows, det->image.cols, det->image.type());
      simcam->SetPoseVision(t_wi.matrix());
      simcam->RenderToTexture();
      simcam->DrawCamera();
      simcam->CaptureGrey( img.data );


      residuals[0] = (T)( diff( img, det, k, t_wi, debug) );
      return true;
    }

  std::shared_ptr< detection >          det;
  SceneGraph::GLSimCam*                 simcam;
  calibu::CameraInterface<Scalar>*      cmod;
  Eigen::Matrix3d                       k;
  bool                                  debug;
};

template<typename Scalar>
ceres::CostFunction* PhotometricCost(
    std::shared_ptr< detection> _d, // detection
    SceneGraph::GLSimCam* _sim_cam,
    Eigen::Matrix3d _k,
    calibu::CameraInterface<Scalar>* _cam,
    bool debug = false
    )
{
  Scalar* _params = _cam->GetParams();
  if( dynamic_cast<calibu::LinearCamera<Scalar>*>( _cam ) ){
    typedef calibu::LinearCamera<Scalar> CamT;
    return (new ceres::NumericDiffCostFunction<PhotometricCostFunctor<CamT>, ceres::CENTRAL, 1,6>(
          new PhotometricCostFunctor<CamT>( _d,_sim_cam,_k, debug ) ) );
  }
  else if( dynamic_cast<calibu::FovCamera<Scalar>*>( _cam ) ){
    typedef calibu::FovCamera<Scalar> CamT;
    return (new ceres::NumericDiffCostFunction<PhotometricCostFunctor<CamT>,ceres::CENTRAL, 1,6>(
          new PhotometricCostFunctor<CamT>( _d,_sim_cam,_k, debug ) ) );
  }
  return NULL;
}


