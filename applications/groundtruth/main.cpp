#include <iostream>
#include <fstream>
#include <vector>
#define __STDC_FORMAT_MACROS
#include <inttypes.h>
#include <stdlib.h>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <calibu/cam/camera_crtp.h>
#include <calibu/cam/camera_models_crtp.h>
#include <calibu/cam/camera_crtp_interop.h>

#include <calibu/cam/rectify_crtp.h>
#include <calibu/pose/Pnp.h>

#include <pangolin/pangolin.h>
#include <SceneGraph/SceneGraph.h>

#include <HAL/Camera/CameraDevice.h>
#include <HAL/Utils/GetPot>

#include <sophus/sophus.hpp>
#include <sophus/se3.hpp>

#include "tags.h"
#include "math.h"

#include "usage.h"

#include <ceres/ceres.h>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include "ceres_cost_functions.h"
#include "dense_ceres.h"
#include "hess.h"
#include "dtrack.h"
#include "ImageAlign/compute_homography.h"

// -cam split:[roi1=0+0+640+480]//proto:[startframe=1500]///Users/faradazerage/Desktop/DataSets/APRIL/Hallway-9-12-13/Twizzler/proto.log -cmod /Users/faradazerage/Desktop/DataSets/APRIL/Hallway-9-12-13/Twizzler/cameras.xml -o outfile.out -map /Users/faradazerage/Desktop/DataSets/APRIL/Hallway-9-12-13/Twizzler/DS20.csv -v -debug -show-ceres

using namespace std;

typedef std::map<int, std::vector< std::shared_ptr< detection > > > DMap;

class GLTag : public SceneGraph::GLObject {

public:
  Eigen::Vector3d tr, tl, br, bl;
  Eigen::Vector3d tr_wb, tl_wb, br_wb, bl_wb;
  unsigned long long data;
  bool px[36];
  bool has_data;
  float color_high;
  float color_low;
  float wb;

  GLTag(){
    has_data = false;
    color_high = 1.0f;
    color_low = 0.0f;
  }

  GLTag( Eigen::Vector3d atr, Eigen::Vector3d atl, Eigen::Vector3d abr, Eigen::Vector3d abl ) : tr(atr), tl(atl), br(abr),  bl(abl)
  {
    has_data = false;
    color_high = 1.0f;
    color_low = 0.0f;
  }

  void CreateData( void )
  {
    if (data == 0)
      return;
    has_data = true;
    unsigned long long value = data;
    bool t_reverse[36];
    int idx = 0;
    for (int count = 0; count < 9; count++) {
      unsigned long long temp = value - ((value >> 4) << 4);
      t_reverse[idx + 0] = temp & 1;
      t_reverse[idx + 1] = temp & 2;
      t_reverse[idx + 2] = temp & 4;
      t_reverse[idx + 3] = temp & 8;
      idx += 4;
      value >>= 4;
    }
    for (int count = 0; count < 36; count++){
      px[count] = t_reverse[35 - count];
    }

  }

  void add_white_border( void )
  {
    Eigen::Vector3d i, j, k;

    i = 0.5*((br - bl) + (tr - tl));
    i /= i.norm();
    j = 0.5*((tl - bl) + (tr - br));
    j /= j.norm();
    k = i.cross(j);

    k = k / k.norm();
    j = k.cross(i);

    tr_wb = tr + wb*i + wb*j;
    tl_wb = tl - wb*i + wb*j;
    br_wb = br + wb*i - wb*j;
    bl_wb = bl - wb*i - wb*j;
  }

  inline void DrawCanonicalObject()
  {
    glPushMatrix();
    glColor4f( 1.0, 1.0, 1.0, 1.0 );
    if (!has_data) {
//      pangolin::glDrawLine(tl[0],tl[1],tl[2],tr[0],tr[1],tr[2]);
//      pangolin::glDrawLine(bl[0],bl[1],bl[2],br[0],br[1],br[2]);
//      pangolin::glDrawLine(tl[0],tl[1],tl[2],bl[0],bl[1],bl[2]);
//      pangolin::glDrawLine(tr[0],tr[1],tr[2],br[0],br[1],br[2]);
    }
    if (has_data) {
      Eigen::Vector3d dx, dy;
      dx = (tl - tr) / 8;
      dy = (tl - bl) / 8;
      glBegin(GL_QUADS);
      if (wb != 0) {
        glColor3f(color_high, color_high, color_high);

        glVertex3d(tl_wb(0), tl_wb(1), tl_wb(2));
        glVertex3d(tl(0), tl(1), tl(2));
        glVertex3d(bl(0), bl(1), bl(2));
        glVertex3d(bl_wb(0), bl_wb(1), bl_wb(2));

        glVertex3d(bl_wb(0), bl_wb(1), bl_wb(2));
        glVertex3d(bl(0), bl(1), bl(2));
        glVertex3d(br(0), br(1), br(2));
        glVertex3d(br_wb(0), br_wb(1), br_wb(2));

        glVertex3d(br_wb(0), br_wb(1), br_wb(2));
        glVertex3d(br(0), br(1), br(2));
        glVertex3d(tr(0), tr(1), tr(2));
        glVertex3d(tr_wb(0), tr_wb(1), tr_wb(2));

        glVertex3d(tr_wb(0), tr_wb(1), tr_wb(2));
        glVertex3d(tr(0), tr(1), tr(2));
        glVertex3d(tl(0), tl(1), tl(2));
        glVertex3d(tl_wb(0), tl_wb(1), tl_wb(2));
      }
      for (int jj = 0; jj < 8; jj++) {
        for (int ii = 0; ii < 8; ii++) {
          if ((ii == 0) || (ii == 7) || (jj == 0) || (jj == 7)) {
            glColor3f(color_low, color_low, color_low);
          }
          else {
            int i = ii - 1;
            int j = jj - 1;
            if (px[i + 6*j]) {
              glColor3f(color_high, color_high, color_high);
            } else {
              glColor3f(color_low, color_low, color_low);
            }
          }
          Eigen::Vector3d o, t, r, f;
          o = tl - ii*dx - jj*dy;
          t = tl - (ii + 1)*dx - jj*dy;
          r = tl - (ii + 1)*dx - (jj + 1)*dy;
          f = tl - (jj + 1)*dy - ii*dx;

          glVertex3d(o(0), o(1), o(2));
          glVertex3d(f(0), f(1), f(2));
          glVertex3d(r(0), r(1), r(2));
          glVertex3d(t(0), t(1), t(2));
        }
      }
      glEnd();
    }

    glPopMatrix();
  }

};


/////////////////////////////////////////////////////////////////////////
void ParseCameraUriOrDieComplaining( const string& s_Uri, hal::Camera& cam )
{
  try{
    cam = hal::Camera( hal::Uri( s_Uri) );
  }
  catch( hal::DeviceException e ){
    printf("Error parsing camera URI: '%s' -- %s\n", s_Uri.c_str(), e.what() );
    printf("Perhaps you meant something like one of these:\n");
    printf("    rectify:[file=cameras.xml]//deinterlace://uvc://\n");
    printf("    file:[loop=1]//~/Data/CityBlock-Noisy/[left*,right*].pgm\n" );
    printf("    trailmix:[narrow=0,depth=0]//file:[startframe=30]//~/Data/stairwell/superframes/[*.pgm]\n");
    exit(-1);
  }
}

/////////////////////////////////////////////////////////////////////////
void ParseSurveyMapFile(
    const string& filename,
    std::map<int,Eigen::Vector3d>& survey_map,
    std::map<int, tag_t>& tags
    )
{
  std::ifstream ifs( filename );
  std::string line;
  fprintf(stderr, "Loading data file %s: \n", filename.c_str());
  std::getline ( ifs, line );
  while( ifs.good() ){
    int uid; // uniquely encodes tag id and landmark id
    double x, y, z;
    sscanf( line.c_str(), "%d, %lf, %lf, %lf", &uid, &x, &y, &z );
    z *= -1;

    survey_map.insert( std::pair<int, Eigen::Vector3d >(uid, Eigen::Vector3d( x, y, z )));

    // first two digits are tag id, second two are landmark id:
    int lmid = uid % 100;
    int tagid = uid / 100;
    if (tags.find(tagid) == tags.end()) {
      tag_t t;
      tags.insert( std::pair< int, tag_t >(tagid, t));
    }
    tags[tagid].AddPoint(lmid, survey_map[uid]);
    std::getline ( ifs, line );
  }
  fflush(stderr);
}

Eigen::Vector2i find_minmax(cv::Mat img, double p[4][2])
{
  int min, max;
  min = 255;
  max = 0;
  int x1, x2, y1, y2;
  x1 = std::min(p[0][0], std::min(p[1][0], std::min(p[2][0], p[3][0])));
  y1 = std::min(p[0][1], std::min(p[1][1], std::min(p[2][1], p[3][1])));

  x2 = std::max(p[0][0], std::max(p[1][0], std::max(p[2][0], p[3][0])));
  y2 = std::max(p[0][1], std::max(p[1][1], std::max(p[2][1], p[3][1])));

  cv::Mat test(img.rows, img.cols, img.type());
  vector< vector< cv::Point > > pts;
  vector< cv::Point > ps;
  ps.push_back( cv::Point(p[0][0], p[0][1]));
  ps.push_back( cv::Point(p[1][0], p[1][1]));
  ps.push_back( cv::Point(p[2][0], p[2][1]));
  ps.push_back( cv::Point(p[3][0], p[3][1]));
  pts.push_back(ps);
  cv::fillPoly(test, pts, 255);

  y1 = std::max(0, y1);
  y2 = std::min(test.rows, y2);
  x1 = std::max(0, x1);
  x2 = std::min(test.cols, x2);

  for (int jj = (int) y1; jj <= (int) y2; jj++) {
    for (int ii = (int) x1; ii <= (int) x2; ii++) {
      if ( test.at<uchar>(jj, ii) != 0) {
        if (img.at<uchar>(jj, ii) < min)
          min = img.at<uchar>(jj, ii);
        if (img.at<uchar>(jj, ii) > max)
          max = img.at<uchar>(jj, ii);
      }
    }
  }

  return Eigen::Vector2i(min, max);
}

Eigen::Matrix4d cameraPoseFromHomography(cv::Mat H, Eigen::Matrix3d K)
{
  cv::Mat pose;
  cv::Mat k;
  Eigen::Matrix3f temp = K.cast<float>();
  cv::eigen2cv(temp, k);
  //  H = k.inv()*H;
  pose = cv::Mat::eye(3, 4, CV_32F);      // 3x4 matrix, the camera pose
  float norm1 = (float)norm(H.col(0));
  float norm2 = (float)norm(H.col(1));
  float tnorm = (norm1 + norm2) / 2.0f; // Normalization value

  cv::Mat p1 = H.col(0);       // Pointer to first column of H
  cv::Mat p2 = pose.col(0);    // Pointer to first column of pose (empty)

  cv::normalize(p1, p2);   // Normalize the rotation, and copies the column to pose

  p1 = H.col(1);           // Pointer to second column of H
  p2 = pose.col(1);        // Pointer to second column of pose (empty)

  cv::normalize(p1, p2);   // Normalize the rotation and copies the column to pose

  p1 = pose.col(0);
  p2 = pose.col(1);

  cv::Mat p3 = p1.cross(p2);   // Computes the cross-product of p1 and p2
  cv::Mat c2 = pose.col(2);    // Pointer to third column of pose
  p3.copyTo(c2);               // Third column is the crossproduct of columns one and two

  pose.col(3) = H.col(2) / tnorm;  //vector t [R|t] is the last column of pose
  std::cout<< H << std::endl;
  std::cout<< pose << std::endl;
  Eigen::Matrix4d toRet;
  toRet << pose.at<float>(0, 0), pose.at<float>(0, 1), pose.at<float>(0, 2), pose.at<float>(0, 3),
      pose.at<float>(1, 0), pose.at<float>(1, 1), pose.at<float>(1, 2), pose.at<float>(1, 3),
      pose.at<float>(2, 0), pose.at<float>(2, 1), pose.at<float>(2, 2), pose.at<float>(2, 3),
      0,          0,          0,          1;
  return toRet;
}

void threshold( cv::Mat& img, float low, float high )
{
  float ave = 0.5*(high + low);
  img = img > ave;
}

cv::Mat get_mask( std::shared_ptr< detection> d,
                  Sophus::SE3d t,
                  Eigen::Matrix3d k,
                  bool from_ts = true)
{
  double p[4][2];

  Eigen::Vector3d temp;

  if (from_ts) {
    temp = k * (t.inverse() * d->tag_data.tl);
    d->tag_data.tl(0) = temp(0) / temp(2);
    d->tag_data.tl(1) = temp(1) / temp(2);

    temp = k * (t.inverse() * d->tag_data.tr);
    d->tag_data.tr(0) = temp(0) / temp(2);
    d->tag_data.tr(1) = temp(1) / temp(2);

    temp = k * (t.inverse() * d->tag_data.bl);
    d->tag_data.bl(0) = temp(0) / temp(2);
    d->tag_data.bl(1) = temp(1) / temp(2);

    temp = k * (t.inverse() * d->tag_data.br);
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
  } else {
    p[0][0] = d->tag_corners.tl(0);
    p[0][1] = d->tag_corners.tl(1);

    p[1][0] = d->tag_corners.bl(0);
    p[1][1] = d->tag_corners.bl(1);

    p[2][0] = d->tag_corners.br(0);
    p[2][1] = d->tag_corners.br(1);

    p[3][0] = d->tag_corners.tr(0);
    p[3][1] = d->tag_corners.tr(1);
  }

  cv::Mat mask(d->image.rows, d->image.cols, d->image.type());
  mask = cv::Scalar(0);
  std::vector< std::vector< cv::Point > > pts;
  std::vector< cv::Point > ps;
  ps.push_back( cv::Point(p[0][0], p[0][1]));
  ps.push_back( cv::Point(p[1][0], p[1][1]));
  ps.push_back( cv::Point(p[2][0], p[2][1]));
  ps.push_back( cv::Point(p[3][0], p[3][1]));
  pts.push_back(ps);
  cv::fillPoly(mask, pts, 255);
  return mask;
}

void normalize(cv::Mat& im)
{
  double min, max;
  cv::minMaxLoc(im, &min, &max);
  im = (im - min) / (max - min);
}

Eigen::Vector6d dtrack_update( std::vector< std::shared_ptr< detection > > &ds,
                               DTrack* dtrack,
                               SceneGraph::GLSimCam* sim_cam,
                               SceneGraph::GLSimCam* depth_cam,
                               Eigen::Matrix3d k )
{
  cv::Mat rect;
  std::shared_ptr< detection > d = ds[0];
  d->image.copyTo(rect);
  cv::Mat dtrackWeights(rect.rows, rect.cols, CV_32FC1);
  cv::Mat temp(rect.rows, rect.cols, rect.type());
  cv::Mat synthetic(rect.rows, rect.cols, CV_32FC1);
  cv::Mat depth(rect.rows, rect.cols, CV_32FC1);

  sim_cam->SetPoseVision( _Cart2T(d->pose) );
  sim_cam->RenderToTexture();
  sim_cam->DrawCamera();
  sim_cam->CaptureGrey( temp.data );
  temp.convertTo(synthetic, CV_32FC1);

  depth_cam->SetPoseVision( _Cart2T(d->pose) );
  depth_cam->RenderToTexture();
  depth_cam->DrawCamera();
  depth_cam->CaptureDepth( depth.data );

  Sophus::SE3d t_sl(Eigen::Matrix4d::Identity());
  Sophus::SE3d t_ws(_Cart2T(d->pose));

  cv::Mat all_masks(d->image.rows, d->image.cols, CV_32FC1);
  all_masks = cv::Scalar(0.0f);
  for (int count = 0; count < ds.size(); count++) {
    d = ds[count];
    cv::Mat mask = get_mask( d, t_ws, k, false);
    mask.convertTo(mask, CV_32FC1);
    mask /= 255.0f;
    all_masks = all_masks + mask;
  }

  d = ds[0];
  synthetic /= 255.0f;
  d->image.convertTo(temp, CV_32FC1);
  temp /= 255.0f;
  for (int jj = 0; jj < temp.rows; jj++) {
    for (int ii = 0; ii < temp.cols; ii++) {
      if (all_masks.at<float>(jj, ii) == 0) {
        temp.at<float>(jj, ii) = 0;
        depth.at<float>(jj, ii) = 0;
        synthetic.at<float>(jj, ii) = 0;
      }
    }
  }

  return _T2Cart((t_ws * t_sl).matrix());
}

void homography_minimization( std::shared_ptr< detection > d,
                              SceneGraph::GLSimCam* simcam,
                              Eigen::Matrix3d k )
{
  cv::Mat img_scene = d->image;

  cv::Mat img_object(img_scene.rows, img_scene.cols, img_scene.type());
  simcam->SetPoseVision( _Cart2T(d->pose) );
  simcam->RenderToTexture();
  simcam->DrawCamera();
  simcam->CaptureGrey( img_object.data );


  int minHessian = 400;

  cv::SurfFeatureDetector detector( minHessian );

  std::vector<cv::KeyPoint> keypoints_object, keypoints_scene;

  double p[4][2];

  // These corners should really be determined from the pixel information
  Eigen::Vector3d temp;
  Sophus::SE3d t( _Cart2T(d->pose) );

  temp = k * (t.inverse() * d->tag_data.tl);
  d->tag_data.tl(0) = temp(0) / temp(2);
  d->tag_data.tl(1) = temp(1) / temp(2);

  temp = k * (t.inverse() * d->tag_data.tr);
  d->tag_data.tr(0) = temp(0) / temp(2);
  d->tag_data.tr(1) = temp(1) / temp(2);

  temp = k * (t.inverse() * d->tag_data.bl);
  d->tag_data.bl(0) = temp(0) / temp(2);
  d->tag_data.bl(1) = temp(1) / temp(2);

  temp = k * (t.inverse() * d->tag_data.br);
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

  cv::Mat mask(img_scene.rows, img_scene.cols, img_scene.type());
  mask = cv::Scalar(0);
  std::vector< std::vector< cv::Point > > pts;
  std::vector< cv::Point > ps;
  ps.push_back( cv::Point(p[0][0], p[0][1]));
  ps.push_back( cv::Point(p[1][0], p[1][1]));
  ps.push_back( cv::Point(p[2][0], p[2][1]));
  ps.push_back( cv::Point(p[3][0], p[3][1]));
  pts.push_back(ps);
  cv::fillPoly(mask, pts, 255);

  //  threshold( img_object, d->tag_data.color_low, d->tag_data.color_high);
  //  threshold( img_scene, d->tag_data.color_low, d->tag_data.color_high);

  detector.detect( img_object, keypoints_object, mask );
  detector.detect( img_scene, keypoints_scene, mask );

  //-- Step 2: Calculate descriptors (feature vectors)
  cv::SurfDescriptorExtractor extractor;

  cv::Mat descriptors_object, descriptors_scene;

  extractor.compute( img_object, keypoints_object, descriptors_object );
  extractor.compute( img_scene, keypoints_scene, descriptors_scene );

  //-- Step 3: Matching descriptor vectors using FLANN matcher
  cv::FlannBasedMatcher matcher;
  std::vector< cv::DMatch > matches;
  matcher.match( descriptors_object, descriptors_scene, matches );

  double max_dist = 0; double min_dist = 10;

  //-- Quick calculation of max and min distances between keypoints
  for( int i = 0; i < descriptors_object.rows; i++ )
  { double dist = matches[i].distance;
    if( dist < min_dist ) min_dist = dist;
    if( dist > max_dist ) max_dist = dist;
  }

  //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
  std::vector< cv::DMatch > good_matches;

  for( int i = 0; i < descriptors_object.rows; i++ )
  {
    if( matches[i].distance < 3*min_dist ){
      good_matches.push_back( matches[i]);
    }
  }

  cv::Mat img_matches;
  cv::drawMatches( img_object, keypoints_object, img_scene/* + img_object*/, keypoints_scene,
                   good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
                   vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

  cv::imshow("matches", img_matches);

  //-- Localize the object
  std::vector<cv::Point2f> obj;
  std::vector<cv::Point2f> scene;

  for( int i = 0; i < good_matches.size(); i++ )
  {
    //-- Get the keypoints from the good matches
    obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
    scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
  }

  cv::Mat H = cv::findHomography( obj, scene, CV_RANSAC );


  //  Eigen::Matrix4d h = cameraPoseFromHomography( H );
  //  d->pose = _T2Cart( _Cart2T(d->pose) * h.inverse() );
}

void sparse_frame_optimize( std::vector<std::shared_ptr < detection > > ds,
                            Eigen::Matrix3d K,
                            calibu::CameraInterface<double> *cmod )
{
  ceres::Solver::Options options;
  options.trust_region_strategy_type = ceres::DOGLEG;
  options.num_threads = 1;
  options.max_num_iterations = 50;
  options.minimizer_progress_to_stdout = false;
  double x[6];
  ceres::Problem problem;
//  problem.AddParameterBlock(x, 6);
  for (int count = 0; count < 6; count++)
    x[count] = ds[0]->pose.data()[count];
  for( int count = 0; count < ds.size(); count++) {
    std::shared_ptr< detection > d = ds[count];
    ceres::CostFunction* cost_function = ProjectionCost( d, K, cmod);
    problem.AddResidualBlock( cost_function, NULL, x);
  }
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  for (int count = 0; count < 6; count++)
    ds[0]->pose.data()[count] = x[count];

//  ceres::Covariance::Options cov_options;
//  cov_options.apply_loss_function = false;
//  ceres::Covariance cov(cov_options);
//  std::vector<std::pair<const double*, const double*> > covariance_blocks;

//  covariance_blocks.push_back(std::make_pair(x, x));
//  if (cov.Compute( covariance_blocks, &problem )) {

//    double covariance_xx[36];

//    cov.GetCovarianceBlock(x, x, covariance_xx);

//    Eigen::Map< const Eigen::Matrix<double,6,6> > covariance(covariance_xx);
//    ds[0]->covariance = covariance.determinant();
//  } else {
//    ds[0]->covariance = FLT_MAX;
//  }
}

void sparse_optimize( DMap detections,
                      Eigen::Matrix3d K,
                      calibu::CameraInterface<double> *cmod )
{
  for (DMap::iterator it = detections.begin(); it != detections.end(); it++) {
    sparse_frame_optimize(it->second, K, cmod);
  }
}

void dense_frame_optimize( std::vector<std::shared_ptr < detection > > dets,
                           SceneGraph::GLSimCam* sim_cam,
                           Eigen::Matrix3d k,
                           int level = 5)
{
  for (int l = level; l >= 0; l--) {
    fprintf(stdout, "Level = %d\n", l);
    fflush(stdout);
    //    ceres::Solver::Options options;
    //    options.linear_solver_type = ceres::DENSE_QR;

    //    ceres::Problem problem;
    //    for( int count = 0; count < dets.size(); count++) {
    //      std::shared_ptr< detection > d = dets[0];
    //      ceres::CostFunction* dense_cost
    //          = new ceres::NumericDiffCostFunction<PhotometricCostFunctor, ceres::CENTRAL, 1, 6> (
    //            new PhotometricCostFunctor( d, sim_cam, l)
    //            );

    //      problem.AddResidualBlock( dense_cost, NULL, dets[0]->pose.data());
    //    }
    //    ceres::Solver::Summary summary;
    //    ceres::Solve(options, &problem, &summary);
    //  }
    //  for (int i = 0; i < 6; i++) dets[0]->pose.data()[i] = x[i];

    //  ceres::Covariance::Options cov_options;
    //  cov_options.apply_loss_function = false;
    //  ceres::Covariance cov(cov_options);
    //  std::vector<std::pair<const double*, const double*> > covariance_blocks;
    //  covariance_blocks.push_back(std::make_pair(x, x));

    //  if (cov.Compute(covariance_blocks, &problem)) {

    //    double covariance_xx[36];
    //    cov.GetCovarianceBlock(x, x, covariance_xx);
    //    Eigen::Map< const Eigen::Matrix<double,6,6> > covariance(covariance_xx);
    //    dets[0]->covariance = covariance.determinant();
    //  } else {
    //    dets[0]->covariance = FLT_MAX;
    //  }
    make_hessian(sim_cam, dets[0], l);
  }
  //  fundamentally_essential( sim_cam, dets[0], k);
}

void dense_optimize( DMap detections,
                     SceneGraph::GLSimCam* sim_cam,
                     Eigen::Matrix3d K)
{
  for (DMap::iterator it = detections.begin(); it != detections.end(); it++) {
    dense_frame_optimize(it->second, sim_cam, K, 2);
  }
}

void update_objects( DMap detections, std::vector< Eigen::Vector6d > &camPoses,
                     std::vector< SceneGraph::GLAxis > &campose )
{
  int count = 0;
  for(DMap::iterator it = detections.begin();
      it != detections.end(); it++, count++) {
    camPoses[count] = it->second[0]->pose;
    campose[count].SetPose(it->second[0]->pose);
  }
}

cv::Mat hat(cv::Mat in)
{
  cv::Mat toRet(3, 3, CV_32F);
  toRet.at<float>(0, 0) = 0;
  toRet.at<float>(0, 1) = -in.at<float>(0, 2);
  toRet.at<float>(0, 2) = in.at<float>(0, 1);
  toRet.at<float>(1, 0) = in.at<float>(0, 2);
  toRet.at<float>(1, 1) = 0;
  toRet.at<float>(1, 2) = -in.at<float>(0, 0);
  toRet.at<float>(2, 0) = -in.at<float>(0, 1);
  toRet.at<float>(2, 1) = in.at<float>(0, 0);
  toRet.at<float>(2, 2) = 0;
  return toRet;
}

struct DecomposeCostFunctor{
  DecomposeCostFunctor( std::shared_ptr< detection > _d,
                        Eigen::Matrix3d _k,
                        Eigen::Matrix4d _T
                        ) :
    d(_d), K(_k), T(_T)
  {
  }

  bool operator()(const double* const x, double* residual) const
  {
    Eigen::Matrix4d temp = T;
    temp(0, 3) /= x[0];
    temp(1, 3) /= x[0];
    temp(2, 3) /= x[0];
    temp = _Cart2T(d->pose) * temp;
    Eigen::Vector4d pt_homo;
    Eigen::Vector3d pt;
    Eigen::Vector2d px;

    // TL
    pt = d->tag_data.tl;
    px = d->tag_corners.tl;
    pt_homo << pt(0), pt(1), pt(2), 1;
    pt_homo = temp*pt_homo;
    pt << pt_homo(0) / pt_homo(3), pt_homo(1) / pt_homo(3), pt_homo(2) / pt_homo(3);
    pt = K*pt;
    pt(0) /= pt(2);
    pt(1) /= pt(2);
    residual[0] = px(0) - pt(0);
    residual[1] = px(1) - pt(1);


    // TR
    pt = d->tag_data.tr;
    px = d->tag_corners.tr;
    pt_homo << pt(0), pt(1), pt(2), 1;
    pt_homo = temp*pt_homo;
    pt << pt_homo(0) / pt_homo(3), pt_homo(1) / pt_homo(3), pt_homo(2) / pt_homo(3);
    pt = K*pt;
    pt(0) /= pt(2);
    pt(1) /= pt(2);
    residual[2] = px(0) - pt(0);
    residual[3] = px(1) - pt(1);

    // BL
    pt = d->tag_data.bl;
    px = d->tag_corners.bl;
    pt_homo << pt(0), pt(1), pt(2), 1;
    pt_homo = temp*pt_homo;
    pt << pt_homo(0) / pt_homo(3), pt_homo(1) / pt_homo(3), pt_homo(2) / pt_homo(3);
    pt = K*pt;
    pt(0) /= pt(2);
    pt(1) /= pt(2);
    residual[4] = px(0) - pt(0);
    residual[5] = px(1) - pt(1);

    // BR
    pt = d->tag_data.br;
    px = d->tag_corners.br;
    pt_homo << pt(0), pt(1), pt(2), 1;
    pt_homo = temp*pt_homo;
    pt << pt_homo(0) / pt_homo(3), pt_homo(1) / pt_homo(3), pt_homo(2) / pt_homo(3);
    pt = K*pt;
    pt(0) /= pt(2);
    pt(1) /= pt(2);
    residual[6] = px(0) - pt(0);
    residual[7] = px(1) - pt(1);

    return true;
  }

private:
  std::shared_ptr< detection > d;
  Eigen::Matrix3d K;
  Eigen::Matrix4d T;
};


Eigen::Matrix4d pfromh( std::shared_ptr< detection > d,
                        SceneGraph::GLSimCam* simcam,
                        Eigen::Matrix3d K,
                        cv::Mat H )
// Pose estimation from homography: http://vision.ucla.edu//MASKS/MASKS-ch5.pdf
{
  cv::Mat k;
  Eigen::Matrix3f ke = K.cast<float>();
  cv::eigen2cv(ke, k);
  H = k.inv() * H.inv();
  cv::Mat R1, R2, R3, R4;
  cv::Mat T1, T2, T3, T4;
  cv::Mat U, S, Vt;
  cv::SVD::compute(H, S, U, Vt);

  H = H / S.at<float>(0, 1);

  cv::Mat A = H.t()*H;
  cv::SVD::compute(A, S, U, Vt);

  float s1, s2, s3;
  if (cv::determinant(U) == -1) {
    U = U*-1;
  }

  s1 = S.at<float>(0, 0);
  s2 = S.at<float>(0, 1);
  s3 = S.at<float>(0, 2);

  cv::Mat v1, v2, v3;
  v1 = U.col(0);
  v2 = U.col(1);
  v3 = U.col(2);

  cv::Mat u1, u2;
  u1 = (sqrt(1 - s3)*v1 + sqrt(s1 - 1)*v3) / (sqrt(s1 - s3));
  u2 = (sqrt(1 - s3)*v1 - sqrt(s1 - 1)*v3) / (sqrt(s1 - s3));

  cv::Mat U1, U2;
  cv::hconcat(v2, u1, U1);
  cv::hconcat(U1, hat(v2.t())*u1, U1);

  cv::hconcat(v2, u2, U2);
  cv::hconcat(U2, hat(v2.t())*u2, U2);

  cv::Mat W1, W2;
  cv::hconcat(H*v2, H*u1, W1);
  cv::hconcat(W1, hat((H*v2).t())*H*u1, W1);

  cv::hconcat(H*v2, H*u2, W2);
  cv::hconcat(W2, hat((H*v2).t())*H*u2, W2);

  R1 = W1*U1.t();
  cv::Mat N1 = hat(v2.t())*u1;
  T1 = (H - R1)*N1;

  R2 = W2*U2.t();
  cv::Mat N2 = hat(v2.t())*u2;
  T2 = (H - R2)*N2;

  R3 = R1;
  T3 = -T1;
  R4 = R2;
  T4 = -T2;

  cv::Mat R1_good, R2_good, T1_good, T2_good;

  if (N1.at<float>(3, 0) > 0) {
    R1_good = R1;
    T1_good = T1;
  } else {
    R1_good = R3;
    T1_good = T3;
  }
  if (N2.at<float>(3, 0) > 0) {
    R2_good = R2;
    T2_good = T2;
  } else {
    R2_good = R4;
    T2_good = T4;
  }

  Eigen::Vector3d t(0, 0, 0);
  Eigen::Matrix3d r1, r2;
  cv::cv2eigen(R1_good, r1);
  Eigen::Matrix4d T_1;
  T_1 << r1(0, 0), r1(0, 1), r1(0, 2), t(0),
      r1(1, 0), r1(1, 1), r1(1, 2), t(1),
      r1(2, 0), r1(2, 1), r1(2, 2), t(2),
      0,        0,        0,    1;
  cv::cv2eigen(R2_good, r2);
  Eigen::Matrix4d T_2;

  T_2 << r2(0, 0), r2(0, 1), r2(0, 2), t(0),
      r2(1, 0), r2(1, 1), r2(1, 2), t(1),
      r2(2, 0), r2(2, 1), r2(2, 2), t(2),
      0,        0,        0,    1;

  Eigen::Vector6d p1 = _T2Cart( _Cart2T(d->pose) * T_1 );
  Eigen::Vector6d p2 = _T2Cart( _Cart2T(d->pose) * T_2 );

  Eigen::Matrix4d T;
  if (cost(simcam, d, p1, 0) < cost(simcam, d, p2, 0)) {
    T_1(0, 3) = T1_good.at<float>(0, 0);
    T_1(1, 3) = T1_good.at<float>(1, 0);
    T_1(2, 3) = T1_good.at<float>(2, 0);

    T = T_1;
  } else {
    T_2(0, 3) = T2_good.at<float>(0, 0);
    T_2(1, 3) = T2_good.at<float>(1, 0);
    T_2(2, 3) = T2_good.at<float>(2, 0);
    T = T_2;
  }

//  T = T.inverse();
  T = calibu::ToCoordinateConvention(Sophus::SE3d(T), calibu::RdfVision).matrix();
//  if (cost(simcam, d, _T2Cart(T), 0) > cost(simcam, d, _T2Cart(-T), 0)) {
  if (T(3, 3) < 0) {
    T = -T;
  }

  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = true;

  ceres::Problem problem;
  double x = sqrt(T(0, 3)*T(0, 3) + T(1, 3)*T(1, 3) + T(2, 3)*T(2, 3));
  std::cout << "x = "<< x << std::endl;

  ceres::CostFunction* cost_func
      = new ceres::NumericDiffCostFunction<DecomposeCostFunctor, ceres::CENTRAL, 8, 1> (
        new DecomposeCostFunctor(d, K, T)
        );
  problem.AddResidualBlock( cost_func, NULL, &x);

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  T(0, 3) /= x;
  T(1, 3) /= x;
  T(2, 3) /= x;

//  std::cout << "T = "<< T << std::endl;
//  std::cout << "x = "<< x << std::endl;

  return T;
}


Eigen::Vector3d homography_shift( std::shared_ptr< detection > d,
                       SceneGraph::GLSimCam* simcam,
                       Eigen::Matrix3d K,
                       SceneGraph::GLSimCam* depth_cam = NULL )
{
  cv::Mat captured = d->image;
  cv::Mat synth(captured.rows, captured.cols, captured.type());
  simcam->SetPoseVision( _Cart2T(d->pose) );
  simcam->RenderToTexture();
  simcam->DrawCamera();
  simcam->CaptureGrey( synth.data );

  cv::Mat H = compute_homography_(captured, synth);


//  Eigen::Matrix4d h = cameraPoseFromHomography( H, K );
  Eigen::Matrix4d h = pfromh(d, simcam, K, H.inv());
  Eigen::Vector3d toRet(h(0, 3), h(1, 3), h(2, 3));
  h(0, 3) = 0;
  h(1, 3) = 0;
  h(2, 3) = 0;
  d->pose = _T2Cart( _Cart2T(d->pose) * h );
  return toRet;
}

void show_homography( std::shared_ptr< detection > d,
                      SceneGraph::GLSimCam* simcam,
                      Eigen::Matrix3d K )
{
  cv::Mat captured = d->image;
  cv::Mat synth(captured.rows, captured.cols, captured.type());
  simcam->SetPoseVision( _Cart2T(d->pose) );
  simcam->RenderToTexture();
  simcam->DrawCamera();
  simcam->CaptureGrey( synth.data );

  cv::Mat H = compute_homography_(captured, synth);

  cv::Mat result;
  cv::warpPerspective(synth, result, H.inv(), cv::Size(captured.cols,captured.rows));
  cv::imshow( "Result", captured - result);
  cv::waitKey();
}

float delta( Eigen::Vector6d a, Eigen::Vector6d b )
{
  float sum = 0;
  for (int j = 0; j < 6; j++) {
    sum += pow(a(j) - b(j), 2);
  }
  return sqrt(sum);
}

void pose_shift( std::shared_ptr< detection > d,
                 SceneGraph::GLSimCam* simcam,
                 Eigen::Matrix3d K,
                 SceneGraph::GLSimCam* depth_cam )
{
  float c_new = cost(simcam, d, d->pose);
  float c_old = FLT_MAX;

  cv::Mat captured = d->image;
  cv::Mat synth(captured.rows, captured.cols, captured.type());
  c_new = 20;
  while (c_new > 0.1) {
    simcam->SetPoseVision( _Cart2T(d->pose) );
    simcam->RenderToTexture();
    simcam->CaptureGrey( synth.data );

    cv::Mat depth(captured.rows, captured.cols, CV_32FC1);
    depth_cam->SetPoseVision( _Cart2T(d->pose) );
    depth_cam->RenderToTexture();
    depth_cam->CaptureDepth( depth.data );

    Eigen::Matrix4d h = estimate_pose_(captured, synth, depth, K);
    c_old = c_new;
    c_new = delta(d->pose, _T2Cart( _Cart2T(d->pose) * h ));
    d->pose = _T2Cart( _Cart2T(d->pose) * h );
//    c_new = cost(simcam, d, d->pose);
//    std::cout<<c_new <<std::endl;
  }
//  std::cout <<"Stopped."<<std::endl;
}

void pose_shift2( std::shared_ptr< detection > d,
                 SceneGraph::GLSimCam* simcam,
                 Eigen::Matrix3d K,
                 SceneGraph::GLSimCam* depth_cam )
{
  cv::Mat captured = d->image;
  cv::Mat synth(captured.rows, captured.cols, captured.type());
  simcam->SetPoseVision( _Cart2T(d->pose) );
  simcam->RenderToTexture();
  simcam->CaptureGrey( synth.data );

  cv::Mat depth(captured.rows, captured.cols, CV_32FC1);
  depth_cam->SetPoseVision( _Cart2T(d->pose) );
  depth_cam->RenderToTexture();
  depth_cam->CaptureDepth( depth.data );

  cv::SurfFeatureDetector detector;
  std::vector<cv::KeyPoint> keypoints1, keypoints2;
  detector.detect(captured, keypoints1);
  detector.detect(synth, keypoints2);

  // computing descriptors
  cv::SurfDescriptorExtractor extractor;
  cv::Mat descriptors1, descriptors2;
  extractor.compute(captured, keypoints1, descriptors1);
  extractor.compute(synth, keypoints2, descriptors2);

  // matching descriptors
  cv::BFMatcher matcher;
  std::vector<cv::DMatch> matches;
  matcher.match(descriptors1, descriptors2, matches);

//  cv::Mat img_matches;
//  cv::drawMatches(captured, keypoints1, synth, keypoints2, matches, img_matches);
//  cv::imshow("matches", img_matches);
//  cv::waitKey(0);

  std::vector<cv::Point3f> cv_obj;
  std::vector<cv::Point2f> cv_img;

  cv::Mat cv_K(3,3,CV_64F);
  cv::eigen2cv(K, cv_K);

  for (int ii = 0; ii < matches.size(); ii++) {

    Eigen::Vector3d pt3d;
    cv::Point2f px2 = keypoints2[matches[ii].trainIdx].pt;
    double d = depth.at<float>(px2.y, px2.x);
    pt3d(0) = d*px2.x;
    pt3d(1) = d*px2.y;
    pt3d(2) = d;
    pt3d = K*pt3d;

    cv::Point3f pt;
    pt.x = pt3d(0);
    pt.y = pt3d(1);
    pt.z = pt3d(2);pt3d(0);

    cv::Point2f px = keypoints1[matches[ii].queryIdx].pt;

    cv_obj.push_back( pt );
    cv_img.push_back( px );
  }

  cv::Mat cv_coeff;
  cv::Mat cv_rot(3,1,CV_64F);
  cv::Mat cv_trans(3,1,CV_64F);

  cv::solvePnP( cv_obj, cv_img, cv_K, cv_coeff, cv_rot, cv_trans, false );

  cv::Mat R;
  cv::Rodrigues(cv_rot, R);
  R = R.t();
  cv_trans = -R * cv_trans;

  cv::Mat T(4, 4, R.type());
  T( cv::Range(0,3), cv::Range(0,3) ) = R * 1; // copies R into T
  T( cv::Range(0,3), cv::Range(3,4) ) = cv_trans * 1; // copies tvec into T
  // fill the last row of T (NOTE: depending on your types, use float or double)
  double *p = T.ptr<double>(3);
  p[0] = p[1] = p[2] = 0; p[3] = 1;

  Eigen::Matrix4d temp;
  temp << T.at<double>(0, 0), T.at<double>(0, 1), T.at<double>(0, 2), T.at<double>(0, 3),
          T.at<double>(1, 0), T.at<double>(1, 1), T.at<double>(1, 2), T.at<double>(1, 3),
          T.at<double>(2, 0), T.at<double>(2, 1), T.at<double>(2, 2), T.at<double>(2, 3),
                           0,                  0,                  0,                  1;

  std::cout << T << std::endl;

//  d->pose = _T2Cart( _Cart2T(d->pose) * h );
}


void sift_optimize( DMap detections,
                     SceneGraph::GLSimCam* sim_cam,
                     SceneGraph::GLSimCam* depth_cam,
                     Eigen::Matrix3d K)
{
  int count = 0;
  for (DMap::iterator it = detections.begin(); it != detections.end(); it++, count++) {
    std::cout << count << std::endl;
    pose_shift(it->second[0], sim_cam, K, depth_cam);
  }
}


/////////////////////////////////////////////////////////////////////////
int main( int argc, char** argv )
{
  if( argc <= 2 ){
    puts(USAGE);
    return -1;
  }

  hal::Camera cam;
  GetPot cl(argc, argv);
  ParseCameraUriOrDieComplaining( cl.follow("", "-cam"), cam );

  std::map<int,Eigen::Vector3d> survey_map;
  std::map<int, tag_t>  tags;
  ParseSurveyMapFile( cl.follow("", "-map"), survey_map, tags );
  fprintf(stdout, "Finished parsing survey map\n");
  if (cl.search("-wb")) {
    for(auto& kv : tags)
      kv.second.add_white_border(atof(cl.follow("0", "-wb").c_str()));
  }

  calibu::Rig<double> rig;
  std::string rig_name_;
  if (!cl.search("-cmod")) {
    rig_name_ = cam.GetDeviceProperty(hal::DeviceDirectory);
    rig_name_ += std::string("/cameras.xml");
  } else {
    rig_name_ = cl.follow("cameras.xml", "-cmod");
  }
  calibu::LoadRig( rig_name_, &rig );
  calibu::CameraInterface<double> *cmod = rig.cameras_[0];
  Eigen::Matrix3d K;
  double* params = cmod->GetParams().data();
  K << params[0], 0, params[2], 0, params[1], params[3], 0, 0, 1;
  std::cout<< K << std::endl;

  calibu::LookupTable lut;
  calibu::CreateLookupTable( *cmod, lut );
  fprintf(stdout, "%d, %d ?= %d, %d\n", cmod->Width(), cmod->Height(), cam.Width(), cam.Height());
  assert( cmod->Width() == cam.Width() && cmod->Height() == cam.Height() );
  cv::Mat rect( cam.Height(), cam.Width(), CV_8UC1 ); // rectified image

  TagDetector td;

  std::shared_ptr<pb::ImageArray> vImages = pb::ImageArray::Create();
  std::map< int, std::vector< std::shared_ptr< detection > > > detections;

  int count = 0;

  bool capture = false;
  bool start = true;
  cv::Mat last_image;
  while( start || capture && (count < 116)){
    capture = cam.Capture( *vImages );
    count++;
    if (start) {
      start = false;
    } else {
      if (cv::sum(vImages->at(0)->Mat() - last_image) == cv::Scalar(0))
        break;
    }
    vImages->at(0)->Mat().copyTo(last_image);

    // 1) Capture and rectify
    calibu::Rectify( lut, vImages->at(0)->Mat().data, rect.data, rect.cols, rect.rows );

    // 2) Run tag detector and get tag corners
    std::vector<april_tag_detection_t> vDetections;
    td.Detect( rect, vDetections );
    if( vDetections.empty() ){
      fprintf(stderr, "No detections at frame %d\n", count);
      fflush(stderr);
      continue;
    }

    // 3) For all tags detected in a frame, add a detection to taht frame
    std::vector< std::shared_ptr< detection > > ds;
    for( size_t ii = 0; ii < vDetections.size(); ii++ ){
      std::shared_ptr< detection > d(new detection);

      // Where are the actual (total station measured) locations of the corners?
      int t_id = vDetections[ii].id;

      d->tag_data = tags[t_id];

      Eigen::Vector3d pts_3d[4];
      pts_3d[0] = tags[t_id].tl;
      pts_3d[1] = tags[t_id].tr;
      pts_3d[2] = tags[t_id].br;
      pts_3d[3] = tags[t_id].bl;

      april_tag_detection_t* p = &vDetections[ii];
      tags[t_id].data = td.tf_->codes[p->id];
      Eigen::Vector2i min_max = find_minmax(rect, vDetections[ii].p);
      tags[t_id].color_low  = min_max(0);
      d->tag_data.color_low = min_max(0);
      tags[t_id].color_high = min_max(1);
      d->tag_data.color_high = min_max(1);

      Eigen::Vector6d T_tc = CalcPose( vDetections[ii].p, pts_3d, K, params[4] );
      rect.copyTo(d->image);
      d->pose = T_tc;

      // add 4 measurements for this tag's corners
      d->tag_corners.tl = Eigen::Vector2d(p->p[0][0], p->p[0][1]);
      d->tag_corners.tr = Eigen::Vector2d(p->p[1][0], p->p[1][1]);
      d->tag_corners.br = Eigen::Vector2d(p->p[2][0], p->p[2][1]);
      d->tag_corners.bl = Eigen::Vector2d(p->p[3][0], p->p[3][1]);

      if (cl.search("-debug")) {
        cv::Mat out;
        cv::cvtColor(rect, out, CV_GRAY2RGB);
        cv::circle(out, cv::Point2d(p->p[0][0], p->p[0][1]), 3, cv::Scalar(255, 0, 255));
        cv::circle(out, cv::Point2d(p->p[1][0], p->p[1][1]), 3, cv::Scalar(0, 255, 0));
        cv::circle(out, cv::Point2d(p->p[2][0], p->p[2][1]), 3, cv::Scalar(0, 0, 255));
        cv::circle(out, cv::Point2d(p->p[3][0], p->p[3][1]), 3, cv::Scalar(255, 255, 255));
        cv::imshow("Detected Corners", out);
        cv::waitKey();
      }

      d->tag_id = vDetections[ii].id;
      ds.push_back(d);

    }

    detections.insert( std::pair<int, std::vector< std::shared_ptr< detection > > >(count, ds) );
  }

  fprintf(stdout, "Finished parsing file\n");
  fflush(stdout);

  // Setup OpenGL Display (based on GLUT)
  pangolin::CreateWindowAndBind("Visualizer");
  glewInit();
  SceneGraph::GLSceneGraph::ApplyPreferredGlSettings();
  glClearColor(0.0f,0.0f,0.0f,1.0f);


  glPixelStorei(GL_PACK_ALIGNMENT,1);
  glPixelStorei(GL_UNPACK_ALIGNMENT,1);
  glEnable(GL_CULL_FACE);

  pangolin::View& container = pangolin::DisplayBase();

  pangolin::View view3d;
  const double far = 1000;
  const double near = 1E-3;
  SceneGraph::GLSceneGraph glGraph;
  SceneGraph::GLGrid grid;
  glGraph.AddChild( &grid );
  pangolin::OpenGlRenderState stacks3d(
        pangolin::ProjectionMatrix(640,480,420,420,320,240,near,far),
        pangolin::ModelViewLookAt(-10, 0, -5, 0, 0, 0, pangolin::AxisNegZ)
        );

  view3d.SetBounds(0, 0.5, 0, 0.5)
      .SetHandler(new SceneGraph::HandlerSceneGraph(glGraph,stacks3d, pangolin::AxisNegZ))
      .SetDrawFunction(SceneGraph::ActivateDrawFunctor(glGraph, stacks3d));
  container.AddDisplay(view3d);

  SceneGraph::ImageView sim_image;
  sim_image.SetBounds(0, 0.5, 0.5, 1);
  container.AddDisplay(sim_image);

  SceneGraph::ImageView live_image;
  live_image.SetBounds(0.5, 1, 0.5, 1);
  container.AddDisplay(live_image);

  SceneGraph::ImageView diff_image;
  diff_image.SetBounds(0.5, 1, 0, 0.5);
  container.AddDisplay(diff_image);


  std::vector< Eigen::Vector6d > camPoses;
  std::vector< GLTag > glTags;
  std::vector< SceneGraph::GLAxis > glTagPoses;

  glTags.resize( tags.size() );
  glTagPoses.resize( tags.size() );
  count = 0;
  for (std::map<int, tag_t>::iterator it = tags.begin(); it != tags.end(); it++, count++) {
    glTags[count].tr = tags[it->first].tr;
    glTags[count].br = tags[it->first].br;
    glTags[count].tl = tags[it->first].tl;
    glTags[count].bl = tags[it->first].bl;
    glTags[count].color_high = tags[it->first].color_high / 255.0f;
    glTags[count].color_low  = tags[it->first].color_low / 255.0f;
    glTags[count].data = tags[it->first].data;
    glTags[count].CreateData();
    glTags[count].wb = tags[it->first].wb;
    glTags[count].add_white_border();
    glGraph.AddChild( &glTags[count] );

    glTagPoses[count].SetPose( tags[it->first].pose );
    glTagPoses[count].SetScale(0.5);
    glGraph.AddChild( &glTagPoses[count] );
  }

  SceneGraph::GLSimCam sim_cam;
  sim_cam.Init( &glGraph, Eigen::Matrix4d::Identity(), K,
                cam.Width(), cam.Height(), SceneGraph::eSimCamLuminance || SceneGraph::eSimCamDepth, 0.01 );
  SceneGraph::GLSimCam depth_cam;
  depth_cam.Init( &glGraph, Eigen::Matrix4d::Identity(), K,
                  cam.Width(), cam.Height(), SceneGraph::eSimCamDepth, 0.01 );

  //  DTrack dtrack;
  //  dtrack.Init();
  //  calibu::CameraRig old_rig = calibu::ReadXmlRig(rig_name_);
  //  dtrack.SetParams(old_rig.cameras[0].camera, old_rig.cameras[0].camera,
  //      old_rig.cameras[0].camera, Sophus::SE3d());

  std::vector< Eigen::Vector6d > poses;
  for( std::map<int, std::vector< std::shared_ptr< detection > > >::iterator it = detections.begin();
       it != detections.end(); it++){
    //    poses.push_back(dtrack_update(detections[it->first], &dtrack, &sim_cam, &depth_cam, K));
    poses.push_back(it->second[0]->pose);
  }

  if (cl.search("-o")) {
    std::cerr << cl.follow("", "-o") << std::endl;
    FILE* ofile = fopen(cl.follow("", "-o").c_str(), "w");
    for( std::map<int, std::vector< std::shared_ptr< detection > > >::iterator it = detections.begin();
         it != detections.end(); it++){
      Eigen::Vector6d poze = it->second[0]->pose;
      fprintf(ofile, "%d\t%f\t%f\t%f\t%f\t%f\t%f\n", it->first, poze(0),
              poze(1),
              poze(2),
              poze(3),
              poze(4),
              poze(5));
    }
    fclose(ofile);
  }

  if (!cl.search("-v")) {
    return 0;
  }

  std::vector< SceneGraph::GLAxis > campose;
  campose.resize(detections.size());
  camPoses.resize(detections.size());
  count = 0;
  for(DMap::iterator it = detections.begin();
      it != detections.end(); it++, count++) {
    camPoses[count] = it->second[0]->pose;
    campose[count].SetPose(it->second[0]->pose);
    campose[count].SetScale(0.1);
    glGraph.AddChild( &campose[count]);
  }

  bool bStep = false;
  unsigned long nFrame=0;
  int pose_number = 0;
  DMap::iterator it;
  it = detections.begin();
  Eigen::Vector3d del;
  float dx = 1e-3;

  pangolin::RegisterKeyPressCallback(pangolin::PANGO_SPECIAL + pangolin::PANGO_KEY_RIGHT, [&](){bStep=true; pose_number++;} );
  pangolin::RegisterKeyPressCallback(pangolin::PANGO_SPECIAL + pangolin::PANGO_KEY_LEFT, [&](){bStep=true; pose_number--;} );
  pangolin::RegisterKeyPressCallback(pangolin::PANGO_SPECIAL + pangolin::PANGO_KEY_UP,
                                     [&](){ std::shared_ptr< detection > d = it->second[0];
                                            d->pose(0) += dx*del(0);
                                            d->pose(1) += dx*del(1);
                                            d->pose(2) += dx*del(2);
                                            update_objects(detections,
                                                           camPoses,
                                                           campose);});

  pangolin::RegisterKeyPressCallback(pangolin::PANGO_SPECIAL + pangolin::PANGO_KEY_DOWN,
                                     [&](){ std::shared_ptr< detection > d = it->second[0];
                                            d->pose(0) -= dx*del(0);
                                            d->pose(1) -= dx*del(1);
                                            d->pose(2) -= dx*del(2);
                                            update_objects(detections,
                                                           camPoses,
                                                           campose);});
  //  pangolin::RegisterKeyPressCallback('h', [&](){ homography_minimization(it->second[0], &sim_cam, K);});
  pangolin::RegisterKeyPressCallback('h', [&](){ del = homography_shift(it->second[0], &sim_cam, K);
    update_objects(detections,
                   camPoses,
                   campose);});
  pangolin::RegisterKeyPressCallback('p', [&](){ pose_shift(it->second[0], &sim_cam, K, &depth_cam);
    update_objects(detections,
                   camPoses,
                   campose);});
  pangolin::RegisterKeyPressCallback('/', [&](){ show_homography(it->second[0], &sim_cam, K);});

  pangolin::RegisterKeyPressCallback('s', [&](){ sparse_optimize(detections, K, cmod);
    update_objects(detections,
                   camPoses,
                   campose);} );
  pangolin::RegisterKeyPressCallback('d', [&](){ sift_optimize(detections, &sim_cam, &depth_cam, K);
    update_objects(detections,
                   camPoses,
                   campose);} );
  pangolin::RegisterKeyPressCallback('w', [&](){ sparse_frame_optimize(it->second, K, cmod);
    update_objects(detections,
                   camPoses,
                   campose);} );
  pangolin::RegisterKeyPressCallback('e', [&](){ dense_frame_optimize(it->second, &sim_cam, K);
    update_objects(detections,
                   camPoses,
                   campose);
    std::cout<<"Dense optimizing this frame . . . "<<std::endl;} );
  cv::Mat synth(cmod->Height(), cmod->Width(), CV_8UC1);
  cv::Mat diff(cmod->Height(), cmod->Width(), CV_8UC1);
  cv::Mat temp;
  pangolin::RegisterKeyPressCallback('o', [&]{
    cv::imwrite("synthetic.jpg", synth);
    cv::imwrite("captured.jpg", temp);
  });

  for(; !pangolin::ShouldQuit(); nFrame++)
  {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glColor4f(1.0f,1.0f,1.0f,1.0f);

    view3d.Activate(stacks3d);
    sim_cam.DrawCamera();

    glColor4f(1.0, 0, 0, 1);
    for (int ii = 1; ii < camPoses.size(); ii++) {
      pangolin::glDrawLine(camPoses[ii - 1][0], camPoses[ii - 1][1], camPoses[ii - 1][2],
          camPoses[ii][0], camPoses[ii][1], camPoses[ii][2]);
    }

    if (bStep) {
      pose_number = std::min((int) camPoses.size() - 1, pose_number);
      pose_number = std::max(0, pose_number);
      bStep = false;
      it = detections.begin();
      std::advance(it, pose_number);
    }

    sim_cam.SetPoseVision(_Cart2T(camPoses[pose_number]));
    sim_cam.RenderToTexture();
    sim_cam.CaptureGrey( synth.data );
    cv::GaussianBlur(synth, synth, cv::Size(5, 5), 0);
    //    threshold(synth, it->second[0]->tag_data.color_low, it->second[0]->tag_data.color_high);
    sim_image.SetImage( synth.data, cmod->Width(), cmod->Height(), GL_RGB, GL_LUMINANCE, GL_UNSIGNED_BYTE);

    it->second[0]->image.copyTo(temp);
    //    threshold(temp, it->second[0]->tag_data.color_low, it->second[0]->tag_data.color_high);
    live_image.SetImage( temp.data, cmod->Width(), cmod->Height(), GL_RGB, GL_LUMINANCE, GL_UNSIGNED_BYTE);
    live_image.Activate();
    diff.release();
    //    cv::subtract(synth, temp, synth, diff/*, synth*/);
    diff = cv::abs(synth - temp);
    diff_image.SetImage( diff.data, cmod->Width(), cmod->Height(), GL_RGB, GL_LUMINANCE, GL_UNSIGNED_BYTE, true);

    glColor4f(1, 1, 1, 1);

    pangolin::FinishFrame();

  }

  return 0;
}
