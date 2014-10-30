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
#include "dtrack.h"

// -cam split:[roi1=0+0+640+480]//proto:[startframe=1500]///Users/faradazerage/Desktop/DataSets/APRIL/Hallway-9-12-13/Twizzler/proto.log -cmod /Users/faradazerage/Desktop/DataSets/APRIL/Hallway-9-12-13/Twizzler/cameras.xml -o outfile.out -map /Users/faradazerage/Desktop/DataSets/APRIL/Hallway-9-12-13/Twizzler/DS20.csv -v -debug -show-ceres

using namespace std;

class GLTag : public SceneGraph::GLObject {

public:
  Eigen::Vector3d tr, tl, br, bl;
  unsigned long long data;
  bool px[36];
  bool has_data;
  float color_high;
  float color_low;

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

    //    //  The points captured include a black border.  Remove it.
    //    Eigen::Vector3d dx, dy;
    //    dx = (tl - tr) / 8;
    //    dy = (tl - bl) / 8;
    //    tl = tl - dx - dy;
    //    tr = tr + dx - dy;
    //    bl = bl - dx + dy;
    //    br = br + dx + dy;

  }

  inline void DrawCanonicalObject()
  {
    glPushMatrix();
    glColor4f( 1.0, 1.0, 1.0, 1.0 );
    //    if (!has_data) {
    //      pangolin::glDrawLine(tl[0],tl[1],tl[2],tr[0],tr[1],tr[2]);
    //      pangolin::glDrawLine(bl[0],bl[1],bl[2],br[0],br[1],br[2]);
    //      pangolin::glDrawLine(tl[0],tl[1],tl[2],bl[0],bl[1],bl[2]);
    //      pangolin::glDrawLine(tr[0],tr[1],tr[2],br[0],br[1],br[2]);
    //    } else {
    if (has_data) {
      Eigen::Vector3d dx, dy;
      dx = (tl - tr) / 8;
      dy = (tl - bl) / 8;
      glBegin(GL_QUADS);
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

    // first two digits are tag id, secnod two are landmark id:
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

Eigen::Matrix4d cameraPoseFromHomography(const cv::Mat& H)
{
  cv::Mat pose;
  pose = cv::Mat::eye(3, 4, CV_32FC1);      // 3x4 matrix, the camera pose
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
  p3.copyTo(c2);       // Third column is the crossproduct of columns one and two

  pose.col(3) = H.col(2) / tnorm;  //vector t [R|t] is the last column of pose
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

  // These corners should really be determined from the pixel information
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

  cv::Mat original;
  temp.copyTo(original);

//  cv::imshow("Before", synthetic + original);

  dtrack->SetKeyframe(synthetic, depth);
  dtrack->Estimate(temp, t_sl, dtrackWeights);

  sim_cam->SetPoseVision( (t_ws * t_sl).matrix() );
  sim_cam->RenderToTexture();
  sim_cam->DrawCamera();
  sim_cam->CaptureGrey( temp.data );
  temp.convertTo(synthetic, CV_32FC1);

//  cv::imshow("After", synthetic + original);
//  cv::waitKey();

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

  threshold( img_object, d->tag_data.color_low, d->tag_data.color_high);
  threshold( img_scene, d->tag_data.color_low, d->tag_data.color_high);

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
  cv::waitKey();

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

  Eigen::Matrix4d h = cameraPoseFromHomography( H );
  std::cout << h << std::endl;
  //  d->pose = _T2Cart( _Cart2T(d->pose) * h.inverse() );
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

  std::vector<cv::Mat> vImages;
  std::map< int, std::vector< std::shared_ptr< detection > > > detections;

  int count = 0;
  while( cam.Capture( vImages ) && (count < 40)){

    count++;
    // 1) Capture and rectify
    calibu::Rectify( lut, vImages[0].data, rect.data, rect.cols, rect.rows );

    // 2) Run tag detector and get tag corners
    std::vector<april_tag_detection_t> vDetections;
    td.Detect( rect, vDetections );
    if( vDetections.empty() ){
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
      pts_3d[0] = tags[t_id].tl; //survey_map[t_id*100];
      pts_3d[1] = tags[t_id].bl; //survey_map[t_id*100 + 1];
      pts_3d[2] = tags[t_id].br; //survey_map[t_id*100 + 2];
      pts_3d[3] = tags[t_id].tr; //survey_map[t_id*100 + 3];

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
        rect.copyTo(out);
        cv::circle(out, cv::Point2d(p->p[0][0], p->p[0][1]), 3, 'r');
        cv::circle(out, cv::Point2d(p->p[1][0], p->p[1][1]), 3, 'r');
        cv::circle(out, cv::Point2d(p->p[2][0], p->p[2][1]), 3, 'r');
        cv::circle(out, cv::Point2d(p->p[3][0], p->p[3][1]), 3, 'r');
        cv::imshow("Detected Corners", out);
        cv::waitKey();
      }

      d->tag_id = vDetections[ii].id;
      ds.push_back(d);

    }
    detections.insert( std::pair<int, std::vector< std::shared_ptr< detection > > >(count, ds) );
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.num_threads = 2;
  options.max_num_iterations = 100;
  options.minimizer_progress_to_stdout = false;
  for( std::map<int, std::vector< std::shared_ptr< detection > > >::iterator it = detections.begin();
       it != detections.end(); it++){
    if (it->second.size() > 0) {
      ceres::Problem problem;
      for (int i = 0; i < it->second.size(); i++) {
        std::shared_ptr< detection > d = it->second[i];
        ceres::CostFunction* cost_function_tl = ProjectionCost( d->tag_data.tl, d->tag_corners.tl, K, cmod);
        ceres::CostFunction* cost_function_tr = ProjectionCost( d->tag_data.tr, d->tag_corners.tr, K, cmod);
        ceres::CostFunction* cost_function_bl = ProjectionCost( d->tag_data.bl, d->tag_corners.bl, K, cmod);
        ceres::CostFunction* cost_function_br = ProjectionCost( d->tag_data.br, d->tag_corners.br, K, cmod);
        problem.AddResidualBlock( cost_function_tl, NULL, (it->second[i]->pose.data()));
        problem.AddResidualBlock( cost_function_tr, NULL, (it->second[i]->pose.data()));
        problem.AddResidualBlock( cost_function_bl, NULL, (it->second[i]->pose.data()));
        problem.AddResidualBlock( cost_function_br, NULL, (it->second[i]->pose.data()));
      }

      ceres::Solver::Summary summary;
      ceres::Solve(options, &problem, &summary);
    }
  }

  // Setup OpenGL Display (based on GLUT)
  pangolin::CreateWindowAndBind("Visualizer");
  glewInit();
  SceneGraph::GLSceneGraph::ApplyPreferredGlSettings();
  glClearColor(0.0f,0.0f,0.0f,1.0f);


  glPixelStorei(GL_PACK_ALIGNMENT,1);
  glPixelStorei(GL_UNPACK_ALIGNMENT,1);
  glDisable(GL_CULL_FACE);

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
    //    tags[it->first].remove_border();
    glTags[count].tr = tags[it->first].tr;
    glTags[count].br = tags[it->first].br;
    glTags[count].tl = tags[it->first].tl;
    glTags[count].bl = tags[it->first].bl;
    glTags[count].color_high = tags[it->first].color_high / 255.0f;
    glTags[count].color_low  = tags[it->first].color_low / 255.0f;
    glTags[count].data = tags[it->first].data;
    glTags[count].CreateData();
    glGraph.AddChild( &glTags[count] );

    glTagPoses[count].SetPose( tags[it->first].pose );
    glTagPoses[count].SetScale(0.5);
    glGraph.AddChild( &glTagPoses[count] );
  }

  SceneGraph::GLSimCam sim_cam;
  sim_cam.Init( &glGraph, Eigen::Matrix4d::Identity(), K,
                cam.Width(), cam.Height(), SceneGraph::eSimCamLuminance );
  SceneGraph::GLSimCam depth_cam;
  depth_cam.Init( &glGraph, Eigen::Matrix4d::Identity(), K,
                  cam.Width(), cam.Height(), SceneGraph::eSimCamDepth );

  ceres::Solver::Options options2;
  //    options2.minimizer_type = ceres::LINE_SEARCH;
  options2.linear_solver_type = ceres::DENSE_SCHUR;
  //  options2.linear_solver_type = ceres::CGNR;
  options2.max_num_iterations = 100;

  DTrack dtrack;
  dtrack.Init();
  calibu::CameraRig old_rig = calibu::ReadXmlRig(rig_name_);
  dtrack.SetParams(old_rig.cameras[0].camera, old_rig.cameras[0].camera,
      old_rig.cameras[0].camera, Sophus::SE3d());

  std::vector< Eigen::Vector6d > poses;

  for( std::map<int, std::vector< std::shared_ptr< detection > > >::iterator it = detections.begin();
       it != detections.end(); it++){
//      poses.push_back(dtrack_update(detections[it->first], &dtrack, &sim_cam, &depth_cam, K));
    poses.push_back( detections[it->first][0]->pose );
    }

  if (cl.search("-o")) {
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

  Eigen::Vector6d p;

  if (cl.search("-capture")) {

    std::shared_ptr< detection > d = detections.begin()->second[0];

//    Eigen::Matrix4d T = _Cart2T<double>(d->pose);
//    std::cout<<T<<std::endl;

    FILE* dfile = fopen("data.out", "w");
    fprintf(dfile, "%f\t%f\t%f\t%f\t%f\t%f\n", d->pose(0), d->pose(1), d->pose(2),
            d->pose(3), d->pose(4), d->pose(5));
    p = d->pose;
    fprintf(dfile, "%f\t%f\t%f\n", d->tag_data.tl(0), d->tag_data.tl(1), d->tag_data.tl(2));
    fprintf(dfile, "%f\t%f\t%f\n", d->tag_data.bl(0), d->tag_data.bl(1), d->tag_data.bl(2));
    fprintf(dfile, "%f\t%f\t%f\n", d->tag_data.br(0), d->tag_data.br(1), d->tag_data.br(2));
    fprintf(dfile, "%f\t%f\t%f\n", d->tag_data.tr(0), d->tag_data.tr(1), d->tag_data.tr(2));
    fprintf(dfile, "%f\t%f\n", d->tag_corners.tl(0), d->tag_corners.tl(1));
    fprintf(dfile, "%f\t%f\n", d->tag_corners.bl(0), d->tag_corners.bl(1));
    fprintf(dfile, "%f\t%f\n", d->tag_corners.br(0), d->tag_corners.br(1));
    fprintf(dfile, "%f\t%f\n", d->tag_corners.tr(0), d->tag_corners.tr(1));
    fprintf(dfile, "%d\t%d\n", d->tag_data.color_low, d->tag_data.color_high);

    unsigned long long value = tags[d->tag_id].data;
    bool t_reverse[36];
    bool t[36];
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
      t[count] = t_reverse[35 - count];
    }

    for (int jj = 0; jj < 6; jj++){
      for (int ii = 0; ii < 6; ii++) {
        if (t[ii + 6*jj])
          fprintf(dfile, "1\t");
        else
          fprintf(dfile, "0\t");
      }
      fprintf(dfile, "\n");
    }
    fclose(dfile);

    cv::imwrite("image.jpg", d->image);

    cv::Mat img(rect.rows, rect.cols, rect.type());
    sim_cam.SetPoseVision( _Cart2T(d->pose) );
    sim_cam.RenderToTexture();
    sim_cam.DrawCamera();
    sim_cam.CaptureGrey( img.data );
    cv::imwrite("synthetic.jpg", img);
  }

  std::vector< SceneGraph::GLAxis > campose;
  campose.resize(detections.size());
  camPoses.resize(detections.size());
  count = 0;
  for(std::map<int, std::vector< std::shared_ptr< detection > > >::iterator it = detections.begin();
      it != detections.end(); it++, count++) {
    camPoses[count] = it->second[0]->pose;
    campose[count].SetPose(it->second[0]->pose);
    campose[count].SetScale(0.1);
    glGraph.AddChild( &campose[count]);
  }

  SceneGraph::GLWireSphere sphere(0.1);
  sphere.SetPose( p );
  glGraph.AddChild( &sphere );

  bool bRun = false;
  bool bStep = false;
  unsigned long nFrame=0;
  int pose_number = 0;
  pangolin::RegisterKeyPressCallback(pangolin::PANGO_SPECIAL + pangolin::PANGO_KEY_RIGHT, [&](){bStep=true; pose_number++;} );
  pangolin::RegisterKeyPressCallback(pangolin::PANGO_SPECIAL + pangolin::PANGO_KEY_LEFT, [&](){bStep=true; pose_number--;} );
  pangolin::RegisterKeyPressCallback(' ', [&](){bRun = !bRun;} );


  cv::Mat synth(cmod->Height(), cmod->Width(), CV_8UC1);
  cv::Mat diff(cmod->Height(), cmod->Width(), CV_8UC1);

  std::map<int, std::vector< std::shared_ptr< detection > > >::iterator it;
  it = detections.begin();

  for(; !pangolin::ShouldQuit(); nFrame++)
  {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glColor4f(1.0f,1.0f,1.0f,1.0f);

    view3d.Activate(stacks3d);

    glColor4f(1.0, 0, 0, 1);
    for (int ii = 1; ii < camPoses.size(); ii++) {
      pangolin::glDrawLine(camPoses[ii - 1][0], camPoses[ii - 1][1], camPoses[ii - 1][2],
          camPoses[ii][0], camPoses[ii][1], camPoses[ii][2]);
    }

    if (bStep) {
      pose_number = std::min((int) camPoses.size() - 1, pose_number);
      pose_number = std::max(0, pose_number);
      sphere.SetPose(camPoses[pose_number]);
      sim_cam.SetPoseVision(_Cart2T(camPoses[pose_number]));
      sim_cam.RenderToTexture();
      sim_cam.CaptureGrey( synth.data );
      bStep = false;
      it = detections.begin();
      std::advance(it, pose_number);
    }

    sim_image.SetImage( synth.data, cmod->Width(), cmod->Height(), GL_RGB, GL_LUMINANCE, GL_UNSIGNED_BYTE);
    live_image.SetImage( it->second[0]->image.data, cmod->Width(), cmod->Height(), GL_RGB, GL_LUMINANCE, GL_UNSIGNED_BYTE);
    diff = synth + it->second[0]->image;
    diff_image.SetImage( diff.data, cmod->Width(), cmod->Height(), GL_RGB, GL_LUMINANCE, GL_UNSIGNED_BYTE, true);

    glColor4f(1, 1, 1, 1);

    pangolin::FinishFrame();

  }

  return 0;
}
