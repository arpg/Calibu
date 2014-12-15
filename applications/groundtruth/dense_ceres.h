#pragma once
//#include <calibu/cam/camera_crtp.h>
//#include <calibu/cam/camera_models_crtp.h>
//#include <calibu/cam/camera_crtp_interop.h>
#include <sophus/se3.hpp>
#include <ceres/ceres.h>
#include "math.h"
#include "measurements.h"
#include <SceneGraph/SceneGraph.h>
#include <opencv2/opencv.hpp>

//double diff( cv::Mat img_a,
//             const std::shared_ptr<detection> d,
//             const Sophus::SE3Group<Scalar> t_wi,
//             const Eigen::Matrix3d k
//             )
//{
//  double p[4][2];
//  cv::Mat img_b = d->image;

//  Eigen::Matrix<Scalar,3,1> pwj;
//  Eigen::Matrix<Scalar,3,1> pij;

//  pwj = (d->tag_data.wb == 0) ? d->tag_data.tl.template cast<Scalar>() :
//                                d->tag_data.tl_wb.template cast<Scalar>();
//  pij = t_wi.inverse() * pwj.template cast<Scalar>();
//  pij = (k.template cast<Scalar>()) * pij;
//  p[0][0] = pij[0] / pij[2];
//  p[0][1] = pij[1] / pij[2];

//  pwj = (d->tag_data.wb == 0) ? d->tag_data.tr.template cast<Scalar>() :
//                                d->tag_data.tr_wb.template cast<Scalar>();
//  pij = t_wi.inverse() * pwj.template cast<Scalar>();
//  pij = (k.template cast<Scalar>()) * pij;
//  p[1][0] = pij[0] / pij[2];
//  p[1][1] = pij[1] / pij[2];

//  pwj = (d->tag_data.wb == 0) ? d->tag_data.br.template cast<Scalar>() :
//                                d->tag_data.br_wb.template cast<Scalar>();
//  pij = t_wi.inverse() * pwj.template cast<Scalar>();
//  pij = (k.template cast<Scalar>()) * pij;
//  p[2][0] = pij[0] / pij[2];
//  p[2][1] = pij[1] / pij[2];

//  pwj = (d->tag_data.wb == 0) ? d->tag_data.bl.template cast<Scalar>() :
//                                d->tag_data.bl_wb.template cast<Scalar>();
//  pij = t_wi.inverse() * pwj.template cast<Scalar>();
//  pij = (k.template cast<Scalar>()) * pij;
//  p[3][0] = pij[0] / pij[2];
//  p[3][1] = pij[1] / pij[2];

//  int x1, x2, y1, y2;

//  x1 = std::min(p[0][0], std::min(p[1][0], std::min(p[2][0], p[3][0])));
//  y1 = std::min(p[0][1], std::min(p[1][1], std::min(p[2][1], p[3][1])));

//  x2 = std::max(p[0][0], std::max(p[1][0], std::max(p[2][0], p[3][0])));
//  y2 = std::max(p[0][1], std::max(p[1][1], std::max(p[2][1], p[3][1])));

//  x1 = std::max(0, x1);
//  y1 = std::max(0, y1);
//  x2 = std::min(x2, img_a.cols);
//  y2 = std::min(y2, img_a.rows);

//  cv::Mat mask(img_a.rows, img_a.cols, img_a.type());
//  mask = cv::Scalar(0);
//  std::vector< std::vector< cv::Point > > pts;
//  std::vector< cv::Point > ps;
//  ps.push_back( cv::Point(p[0][0], p[0][1]));
//  ps.push_back( cv::Point(p[1][0], p[1][1]));
//  ps.push_back( cv::Point(p[2][0], p[2][1]));
//  ps.push_back( cv::Point(p[3][0], p[3][1]));
//  pts.push_back(ps);
//  cv::fillPoly(mask, pts, 255);

//  float sum = 0;
//  int count = 0;
//  for (int jj = (int) y1; jj <= (int) y2; jj++) {
//    for (int ii = (int) x1; ii <= (int) x2; ii++) {
//      if ( mask.at<uchar>(jj, ii) != 0) {
//        count++;
//        sum += pow(img_a.at<uchar>(jj, ii) - img_b.at<uchar>(jj, ii) , 2);
//      }
//    }
//  }

////  sum /= (1.0f*count);

//  return sum;
//}


struct PhotometricCostFunctor
{
  PhotometricCostFunctor(
      std::shared_ptr< detection> _d,
      SceneGraph::GLSimCam* _cam,
      int _level
      ) : det(_d), simcam(_cam), level(_level)
  {
  }

  bool operator()(
      const double* const _t_wi,  // world pose of i'th camera
      double* residuals
      ) const
  {
    const Eigen::Map< const Eigen::Matrix<double,6,1> > temp1(_t_wi);
    Eigen::Vector6d temp(temp1);

    cv::Mat img(det->image.rows, det->image.cols, CV_8UC1);
    simcam->SetPoseVision(_Cart2T(temp));
    simcam->RenderToTexture();
    simcam->CaptureGrey( img.data );

    cv::Mat data;
    det->image.copyTo(data);
    for (int i = 1; i <= level; i++) {
      cv::pyrDown(img, img);
      cv::pyrDown(data, data);
//      cv::imshow("img", img);
//      cv::imshow("data", data);
//      cv::waitKey();
    }

    cv::Mat img_d;
    img.convertTo(img_d, CV_64F);
    cv::Mat data_d;
    data.convertTo(data_d, CV_64F);

    img_d /= 255;
    data_d /= 255;

    cv::Mat sum;
    cv::subtract(data_d, img_d, sum, img);
    sum = cv::abs(sum);
    double s = 0;
    s = (double) cv::sum(sum)[0];
    std::cout<<"Error at "<<level<<" is "<<s<<std::endl;
    residuals[0] = s;
    return true;
  }

  std::shared_ptr< detection >          det;
  SceneGraph::GLSimCam*                 simcam;
  int                                   level;
};

//template<typename Scalar>
//ceres::CostFunction* PhotometricCost(
//    std::shared_ptr< detection> _d,
//    SceneGraph::GLSimCam* _sim_cam,
//    calibu::CameraInterface<Scalar>* _cam
//    )
//{
//  if( dynamic_cast<calibu::LinearCamera<Scalar>*>( _cam ) ){
//    return (new ceres::NumericDiffCostFunction<PhotometricCostFunctor, ceres::CENTRAL, 1,6>(
//              new PhotometricCostFunctor( _d,_sim_cam ) ) );
//  }
//  else if( dynamic_cast<calibu::FovCamera<Scalar>*>( _cam ) ){
//    return (new ceres::NumericDiffCostFunction<PhotometricCostFunctor, ceres::CENTRAL, 1,6>(
//              new PhotometricCostFunctor( _d,_sim_cam ) ) );
//  }
//  return NULL;
//}


