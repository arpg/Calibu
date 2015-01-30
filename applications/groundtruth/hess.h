#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <SceneGraph/SceneGraph.h>
#include <sophus/sophus.hpp>
#include <sophus/se3.hpp>
#include "math.h"
#include "measurements.h"

#define STEP 1e-3

double cost( SceneGraph::GLSimCam* simcam,
             std::shared_ptr< detection > d,
             Eigen::Vector6d pose,
             int level = 0)
{
  cv::Mat img(d->image.rows, d->image.cols, CV_8UC1);

  simcam->SetPoseVision(_Cart2T(pose));
  simcam->RenderToTexture();
  simcam->CaptureGrey( img.data );
  cv::GaussianBlur(img, img, cv::Size(5, 5), 0);

  cv::Mat data;
  d->image.copyTo(data);

  for (int count = 1; count <= level; count++) {
    cv::pyrDown(img, img);
    cv::pyrDown(data, data);
  }

  cv::Mat img_d;
  img.convertTo(img_d, CV_64F);
  cv::Mat data_d;
  data.convertTo(data_d, CV_64F);


  img_d /= 255;
  data_d /= 255;
  cv::Mat sum;
  sum = cv::abs(img_d - data_d);
  return (double) cv::sum(sum)[0];
}

double D1( SceneGraph::GLSimCam* simcam,
           std::shared_ptr< detection > d,
           int a,
           Eigen::Vector6d pose,
           int level,
           float step = STEP )
{
  Eigen::Vector6d pose1 = pose;
  pose1(a) += step;
  double c1 = cost(simcam, d, pose1, level);
  double c2 = cost(simcam, d, pose, level);
  return ((c1 - c2) / step);
}

double D2( SceneGraph::GLSimCam* simcam,
           std::shared_ptr< detection > d,
           int a,
           int b,
           Eigen::Vector6d pose,
           int level,
           float step = STEP )
{
  Eigen::Vector6d pose1 = pose;
  pose1(b) += step;
  double c1 = D1(simcam, d, a, pose1, level);
  double c2 = D1(simcam, d, a, pose, level);
  return ((c1 - c2) / step);
}


void make_hessian( SceneGraph::GLSimCam* simcam,
                   std::shared_ptr< detection > d,
                   int level )
{
  Eigen::Matrix<double, 6, 6> hess;
  Eigen::Matrix<double, 6, 1> update;
  Eigen::Matrix<double, 6, 1> grad;
  int step = 0;

  Eigen::Vector6d pose = d->pose;
  Eigen::Vector6d temp;
  double new_ = cost(simcam, d, pose, level);
  std::cout<<"Initial cost: "<<new_ <<" at level "<<level<<std::endl;
  double last_ = FLT_MAX;
  pose = d->pose;
  while (((new_ < last_)) && (step < 25)) {
    step++;
    std::cout<< "Step: "<<step<<" -- C:"<<new_<<" -- ["<<update.norm()<<"]"<<std::endl;
//    for (int jj = 0; jj < 6; jj++) {
//      for (int ii = 0; ii < 6; ii++) {
//        hess(ii, jj) = D2(simcam, d, ii, jj, pose, level);
//      }
//    }

    for (int ii = 0; ii < 6; ii++) {
      grad(ii) = D1(simcam, d, ii, pose, level);
    }

//    update = hess.inverse() * grad;
    update = grad;

    last_ = new_;
    temp = pose - update;

    new_ = cost(simcam, d, temp, level);
    if (new_ < last_)
      pose = temp;

  }
  d->pose = pose;
}


void fundamentally_essential( SceneGraph::GLSimCam* simcam,
                              std::shared_ptr< detection > d,
                              Eigen::Matrix3d k )
{
  cv::Mat img_scene = d->image;

  cv::Mat img_object(img_scene.rows, img_scene.cols, img_scene.type());
  simcam->SetPoseVision( _Cart2T(d->pose) );
  simcam->RenderToTexture();
  simcam->DrawCamera();
  simcam->CaptureGrey( img_object.data );

  cv::GaussianBlur( img_object, img_object, cv::Size(10, 10), 0);

  cv::Mat K;
  cv::eigen2cv(k, K);

  int minHessian = 200;

  cv::FastAdjuster  detector;

  std::vector<cv::KeyPoint> keypoints_object, keypoints_scene;

  detector.detect( img_object, keypoints_object/*, mask*/ );
  detector.detect( img_scene, keypoints_scene/*, mask*/ );

//  cv::SurfDescriptorExtractor extractor;
  cv::SiftDescriptorExtractor extractor(minHessian);
  cv::Mat descriptors1, descriptors2;
  extractor.compute(img_object, keypoints_object, descriptors1);
  extractor.compute(img_scene, keypoints_scene, descriptors2);

  cv::BFMatcher matcher;
  std::vector<cv::DMatch> matches;
  matcher.match(descriptors1, descriptors2, matches);

  double max_dist = 0; double min_dist = FLT_MAX;

  //-- Quick calculation of max and min distances between keypoints
  for( int i = 0; i < descriptors1.rows; i++ )
  { double dist = matches[i].distance;
    if( dist < min_dist ) min_dist = dist;
    if( dist > max_dist ) max_dist = dist;
  }

  //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
  std::vector< cv::DMatch > good_matches;

  for( int i = 0; i < descriptors1.rows; i++ )
  {
//    if( matches[i].distance < 3*min_dist ){
      good_matches.push_back( matches[i]);
//    }
  }


  cv::Mat img_matches;
  cv::drawMatches(img_object, keypoints_object, img_scene, keypoints_scene, good_matches, img_matches);
  cv::imshow("matches", img_matches);
//  cv::waitKey();


  std::vector< cv::Point2f> to, from;
  for (size_t i = 0; i < good_matches.size(); ++i)
  {
    from.push_back(keypoints_object[matches[i].queryIdx].pt);
    to.push_back(keypoints_scene[matches[i].trainIdx].pt);
  }

  cv::Mat fm = cv::findFundamentalMat( from, to);

  Eigen::Matrix3d F;
  cv::cv2eigen(fm, F);
  Eigen::Matrix3d E = k.transpose() * F * k;
  cv::SVD svd(K.t() * fm * K);
  Eigen::Matrix3d W;
  W <<  0,-1,0,   //HZ 9.13
        1,0,0,
        0,0,1;

  cv::Mat u = svd.u;
  cv::Mat vt = svd.vt;
  Eigen::Matrix3d U;
  cv::cv2eigen(u, U);
  Eigen::Matrix3d VT;
  cv::cv2eigen(vt, VT);
  Eigen::Matrix3d R = U * W * VT; //HZ 9.19
  std::cout<<"R: "<<std::endl<<R<<std::endl;
  Eigen::Vector3d t;// = U.col(2); //u3
  t << U(0, 2), U(1, 2), U(2, 2);

  Eigen::Matrix4d P1, P2, P3, P4, P;
  P1 <<    R(0,0),    R(0,1), R(0,2), t(0),
           R(1,0),    R(1,1), R(1,2), t(1),
           R(2,0),    R(2,1), R(2,2), t(2),
                0,         0,      0,    1;

  P2 <<    R(0,0),    R(0,1), R(0,2), -t(0),
           R(1,0),    R(1,1), R(1,2), -t(1),
           R(2,0),    R(2,1), R(2,2), -t(2),
                0,         0,      0,    1;

  R = U * W.transpose() * VT;
  P3 <<    R(0,0),    R(0,1), R(0,2), t(0),
           R(1,0),    R(1,1), R(1,2), t(1),
           R(2,0),    R(2,1), R(2,2), t(2),
                0,         0,      0,    1;

  P4 <<    R(0,0),    R(0,1), R(0,2), -t(0),
           R(1,0),    R(1,1), R(1,2), -t(1),
           R(2,0),    R(2,1), R(2,2), -t(2),
                0,         0,      0,    1;

  int sum = 0;
  Eigen::Matrix4d current = _Cart2T(d->pose);
  simcam->SetPoseVision( current*P1.inverse() );
  simcam->RenderToTexture();
  simcam->DrawCamera();
  simcam->CaptureGrey( img_object.data );
//  cv::imshow("p1", img_object);
  if (cv::sum(img_object)[0] > sum) {
    sum = cv::sum(img_object)[0];
    P = P1;
  }

  simcam->SetPoseVision( current*P2.inverse() );
  simcam->RenderToTexture();
  simcam->DrawCamera();
  simcam->CaptureGrey( img_object.data );
//  cv::imshow("p2", img_object);
  if (cv::sum(img_object)[0] > sum) {
    sum = cv::sum(img_object)[0];
    P = P2;
  }

  simcam->SetPoseVision( current*P3.inverse() );
  simcam->RenderToTexture();
  simcam->DrawCamera();
  simcam->CaptureGrey( img_object.data );
//  cv::imshow("p3", img_object);
  if (cv::sum(img_object)[0] > sum) {
    sum = cv::sum(img_object)[0];
    P = P3;
  }

  simcam->SetPoseVision( current*P4.inverse() );
  simcam->RenderToTexture();
  simcam->DrawCamera();
  simcam->CaptureGrey( img_object.data );
//  cv::imshow("p4", img_object);
  if (cv::sum(img_object)[0] > sum) {
    sum = cv::sum(img_object)[0];
    P = P4;
  }

  std::cout<<"Before: "<<std::endl<<current<<std::endl;
  std::cout<<"After:  "<<std::endl<<current*P.inverse()<<std::endl;
  std::cout<<"After': "<<std::endl<<current*P<<std::endl;

  if (sum != 0) d->pose = _T2Cart( current*P.inverse() );
}
