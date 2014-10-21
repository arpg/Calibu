#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>
#include <fstream>
#include <vector>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <calibu/cam/camera_crtp.h>
#include <calibu/cam/camera_models_crtp.h>
#include <calibu/cam/camera_crtp_interop.h>

#include <calibu/cam/rectify_crtp.h>
#include <calibu/pose/Pnp.h>

#include <HAL/Camera/CameraDevice.h>
#include <HAL/Utils/GetPot>

#include <sophus/sophus.hpp>
#include <sophus/se3.hpp>

#include "tags.h"

namespace Eigen
{
typedef Matrix<double,6,1>  Vector6d;
}

using namespace std;

DEFINE_string(cam,
              "-cam",
              "hal camera specifier.");

DEFINE_string(cmod,
              "-cmod",
              "calibu camera model xml file.");

DEFINE_string( map,
               "-map",
               "Survey map file.");

DEFINE_string(o,
              "-o",
              "Output file name.");


#define USAGE\
  "\nUsage: findtargets -cam <image source> [-cmod <camera file>]  [-o <output file>]\n"\
  "\n"\
  "Example image source: proto:///Users/joe_shmoe/Data/DataLog.log\n"\
  "Example camera file: cameras.xml\n"\
  "\n"\
  "This program will process images with visible AR tags and produce\n"\
  "a text file with the following format:\n"\
  "-----------------------------------------------------\n"\
  " m, the number of poses that see any landmarks\n"\
  " n, the number of unique landmarks seen\n"\
  " k, number of measurements\n"\
  " pose_id, landmark_id, u, v\n"\
  "   .\n"\
  "   .\n"\
  "   .\n"\
  " pose_id, landmark_id, u, v\n"\
  " pose_1\n"\
  " pose_2\n"\
  "   .\n"\
  "   .\n"\
  "   .\n"\
  " pose_m\n"\
  " landmark_1\n"\
  " landmark_2\n"\
  "   .\n"\
  "   .\n"\
  "   .\n"\
  " landmark_n\n"\
  "-----------------------------------------------------\n"\
  "This file is suitable as an initialization for ground-truth pose\n"\
  "estimation. NB:\n"\
  "  each pose is a 1x6 vector:  x,y,z,p,q,r\n"\
  "  each landmark is 1x4 vector:  id,x,y,z\n"\
  "\n\n"

/////////////////////////////////////////////////////////////////////////
struct measurement_t
{
  int pose_idx;
  int lm_idx;
  double u;
  double v;
};

inline Eigen::Matrix4d _Cart2T(
                              double x,
                              double y,
                              double z,
                              double r,
                              double p,
                              double q
                              )
{
    Eigen::Matrix4d T;
    // psi = roll, th = pitch, phi = yaw
    double cq, cp, cr, sq, sp, sr;
    cr = cos( r );
    cp = cos( p );
    cq = cos( q );

    sr = sin( r );
    sp = sin( p );
    sq = sin( q );

    T(0,0) = cp*cq;
    T(0,1) = -cr*sq+sr*sp*cq;
    T(0,2) = sr*sq+cr*sp*cq;

    T(1,0) = cp*sq;
    T(1,1) = cr*cq+sr*sp*sq;
    T(1,2) = -sr*cq+cr*sp*sq;

    T(2,0) = -sp;
    T(2,1) = sr*cp;
    T(2,2) = cr*cp;

    T(0,3) = x;
    T(1,3) = y;
    T(2,3) = z;
    T.row(3) = Eigen::Vector4d( 0.0, 0.0, 0.0, 1.0 );
    return T;
}

inline Eigen::Matrix4d _Cart2T( Eigen::Matrix<double,6,1> x)
{
    return _Cart2T(x(0),x(1),x(2),x(3),x(4),x(5));
}

inline Eigen::Vector3d _R2Cart(
        const Eigen::Matrix3d& R
        )
{
    Eigen::Vector3d rpq;
    // roll
    rpq[0] = atan2( R(2,1), R(2,2) );

    // pitch
    double det = -R(2,0) * R(2,0) + 1.0;
    if (det <= 0) {
        if (R(2,0) > 0){
            rpq[1] = -M_PI / 2.0;
        }
        else{
            rpq[1] = M_PI / 2.0;
        }
    }
    else{
        rpq[1] = -asin(R(2,0));
    }

    // yaw
    rpq[2] = atan2(R(1,0), R(0,0));

    return rpq;
}

inline Eigen::Matrix<double,6,1> _T2Cart(
        const Eigen::Matrix4d& T
        )
{
    Eigen::Matrix<double,6,1> Cart;
    Eigen::Vector3d rpq = _R2Cart( T.block<3,3>(0,0) );
    Cart[0] = T(0,3);
    Cart[1] = T(1,3);
    Cart[2] = T(2,3);
    Cart[3] = rpq[0];
    Cart[4] = rpq[1];
    Cart[5] = rpq[2];

    return Cart;
}


/////////////////////////////////////////////////////////////////////////
struct tag_t
{
  Eigen::Vector3d tr, tl, br, bl;
  Eigen::Vector6d pose;

  tag_t() {
    tr << 0, 0, 0;
    tl << 0, 0, 0;
    br << 0, 0, 0;
    bl << 0, 0, 0;
  }

  bool CheckZero( Eigen::Vector3d t )
  {
    return (t.norm() == 0);
  }

  void CalcTagPose( void )
  {
    if (!CheckZero(tr) && !CheckZero(tl) && !CheckZero(br) && !CheckZero(bl)) {
      Eigen::Vector3d i, j, k, ave;
      double p, q, r;

      i = 0.5*((br - bl) + (tr - tl));
      i /= i.norm();
      j = 0.5*((tl - bl) + (tr - br));
      j /= j.norm();
      k = i.cross(j);

      k = k / k.norm();
      j = k.cross(i);

      ave = 0.25*(tr + tl + br + bl);

      Eigen::Matrix4d tran;

      tran << i(0), j(0), k(0),  ave(0),
              i(1), j(1), k(1),  ave(1),
              i(2), j(2), k(2),  ave(2),
                 0,    0,    0,       1;

      pose = _T2Cart(tran);
    }
  }

  void AddPoint(int lm_id, Eigen::Vector3d pt)
  {
    switch(lm_id) {
    case 0: tl = pt; break;
    case 1: bl = pt; break;
    case 2: br = pt; break;
    case 3: tr = pt; break;
    }
    CalcTagPose();
  }
};

/////////////////////////////////////////////////////////////////////////
void ParseCameraUriOrDieComplaining( const std::string& sUri, hal::Camera& cam )
{
  try{
    cam = hal::Camera( hal::Uri(sUri) );
  }
  catch( hal::DeviceException e ){
    printf("Error parsing camera URI: '%s' -- %s\n", sUri.c_str(), e.what() );
    printf("Perhaps you meant something like one of these:\n");
    printf("    rectify:[file=cameras.xml]//deinterlace://uvc://\n");
    printf("    file:[loop=1]//~/Data/CityBlock-Noisy/[left*,right*].pgm\n" );
    printf("    trailmix:[narrow=0,depth=0]//file:[startframe=30]//~/Data/stairwell/superframes/[*.pgm]\n");
    exit(-1);
  }
}

/////////////////////////////////////////////////////////////////////////
void ParseSurveyMapFile( 
    const std::string& filename,
    std::map<int,Eigen::Vector3d>& survey_map,
    std::map<int, tag_t>& tags
    )
{
  std::ifstream ifs( filename );
  std::string line;
  while( ifs.good() ){
    std::getline ( ifs, line );
    int uid; // uniquely encodes tag id and landmark id
    double x, y, z;
    sscanf( line.c_str(), "%d, %lf, %lf, %lf", &uid, &x, &y, &z );
//    x *=  0.0254;
//    y *=  0.0254;
//    z *= -0.0254;
    z *= -1;

    survey_map[uid] = Eigen::Vector3d( x, y, z );

    // first two digits are tag id, secnod two are landmark id:
    int lmid = uid % 100; //(uid%10) + ((uid-uid%10) % 100);
    int tagid = uid / 100;
    tags[tagid].AddPoint(lmid, survey_map[uid]);
  }
}

/////////////////////////////////////////////////////////////////////////
void Warp_Pts(double &u, double &v, double w)
{
  double ru = sqrtf(u*u + v*v);
  double rd = (1 / w) * atan(2 * ru * tan( w / 2));
  double factor = rd / ru;
  u *= factor;
  v *= factor;
}

/////////////////////////////////////////////////////////////////////////
Eigen::Vector6d CalcPose(
    double pts_2d[4][2],
const Eigen::Vector3d pts_3d[4],
const Eigen::Matrix3d& K,
const double w
)
{
  std::vector<cv::Point3f> cv_obj;
  std::vector<cv::Point2f> cv_img;

//  for (int i = 0; i < 4; i++)
//    Warp_Pts(pts_2d[i][0], pts_2d[i][1], w);

//  fprintf(stdout, "Using these points: \n");
//  for (int c = 0; c < 4; c++)
//  {
//    fprintf(stdout, "(%f, %f) -> <%f, %f, %f>\n", pts_2d[c][0],pts_2d[c][1], pts_3d[c][0], pts_3d[c][1], pts_3d[c][2]);
//  }

  cv_obj.push_back( cv::Point3f(pts_3d[0][0],pts_3d[0][1],pts_3d[0][2]) );
  cv_obj.push_back( cv::Point3f(pts_3d[3][0],pts_3d[3][1],pts_3d[3][2]) );
  cv_obj.push_back( cv::Point3f(pts_3d[2][0],pts_3d[2][1],pts_3d[2][2]) );
  cv_obj.push_back( cv::Point3f(pts_3d[1][0],pts_3d[1][1],pts_3d[1][2]) );

  cv_img.push_back( cv::Point2f(pts_2d[0][0],pts_2d[0][1]) );
  cv_img.push_back( cv::Point2f(pts_2d[1][0],pts_2d[1][1]) );
  cv_img.push_back( cv::Point2f(pts_2d[2][0],pts_2d[2][1]) );
  cv_img.push_back( cv::Point2f(pts_2d[3][0],pts_2d[3][1]) );

  cv::Mat cv_K(3,3,CV_64F);
  cv_K.at<double>(0,0) = K(0,0);
  cv_K.at<double>(1,0) = K(1,0);
  cv_K.at<double>(2,0) = K(2,0);
  cv_K.at<double>(0,1) = K(0,1);
  cv_K.at<double>(1,1) = K(1,1);
  cv_K.at<double>(2,1) = K(2,1);
  cv_K.at<double>(0,2) = K(0,2);
  cv_K.at<double>(1,2) = K(1,2);
  cv_K.at<double>(2,2) = K(2,2);

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

//  Eigen::Map<Eigen::Matrix4d> temp( T.data );
//  return _T2Cart(temp);


  Eigen::Matrix4d temp;
  temp << T.at<double>(0, 0), T.at<double>(0, 1), T.at<double>(0, 2), T.at<double>(0, 3),
          T.at<double>(1, 0), T.at<double>(1, 1), T.at<double>(1, 2), T.at<double>(1, 3),
          T.at<double>(2, 0), T.at<double>(2, 1), T.at<double>(2, 2), T.at<double>(2, 3),
                           0,                  0,                  0,                  1;

  Eigen::Vector6d pose = _T2Cart( temp );
//  pose[0] = cv_trans.at<double>(0);
//  pose[1] = cv_trans.at<double>(1);
//  pose[2] = cv_trans.at<double>(2);
//  pose[3] = cv_rot.at<double>(0);
//  pose[4] = cv_rot.at<double>(1);
//  pose[5] = cv_rot.at<double>(2);

//  fprintf(stdout, "I get this pose: [%f, %f, %f, %f, %f, %f]\n", pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
//  fflush(stdout);

  return pose;
}

/////////////////////////////////////////////////////////////////////////
int main( int argc, char** argv )
{
  if( argc <= 2 ){
    puts(USAGE);
    return -1;
  }
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  hal::Camera cam;
  ParseCameraUriOrDieComplaining( FLAGS_cam, cam );

  std::map<int,Eigen::Vector3d> survey_map;
  std::map<int, tag_t>  tags;
  ParseSurveyMapFile( FLAGS_map, survey_map, tags );
  for (std::map< int, tag_t >::iterator it = tags.begin(); it != tags.end(); it++) {
    fprintf(stdout, "Tag Id = %d Pose = <%f, %f, %f, %f, %f, %f>\n", it->first,
            it->second.pose(0), it->second.pose(1), it->second.pose(2),
            it->second.pose(3), it->second.pose(4), it->second.pose(5));
  }
  fprintf(stdout, "Finished parsing survey map\n");

  // TODO condense this LoadRig into LoadCamera
  calibu::Rig<double> rig;
  if (FLAGS_cmod.compare(std::string("-cmod")) == 0) {
    std::string rig_name_( cam.GetDeviceProperty(hal::DeviceDirectory) );
    rig_name_ += std::string("/cameras.xml");
    calibu::LoadRig( rig_name_, &rig );
  } else {
    calibu::LoadRig( FLAGS_cmod, &rig );
  }
  calibu::CameraInterface<double> *cmod = rig.cameras_[0];
  Eigen::Matrix3d K;
  double* p = cmod->GetParams();
  K << p[0], 0, p[2], 0, p[1], p[3], 0, 0, 1;

  // TODO add AttachLUT funtionality to camera models?
  calibu::LookupTable lut;
  calibu::CreateLookupTable( *cmod, lut );
  fprintf(stdout, "%d, %d ?= %d, %d\n", cmod->Width(), cmod->Height(), cam.Width(), cam.Height());
  assert( cmod->Width() == cam.Width() && cmod->Height() == cam.Height() );
  cv::Mat rect( cam.Height(), cam.Width(), CV_8UC1 ); // rectified image
  cv::Mat rgb;

  // get a tag detector
  TagDetector td;

  cv::namedWindow( "Tag Viewer", CV_WINDOW_AUTOSIZE );

  std::vector<cv::Mat> vImages;

  int local_pose_id = 0;
  int local_landmark_id = 0;
  std::map<int,int> local_to_survey; // map from unique id to local 0-based id
  std::map<int,int> survey_to_local; // map from unique id to local 0-based id
  std::map<int,bool> tag_seen; // tags_seen[id] is true if tag id has been seen
  std::vector<measurement_t> measurements;
  std::vector<Eigen::Vector6d> poses;

  FILE* file;
  bool view_ = true;
  if (FLAGS_o.compare("-o") == 0) {
    file = stdout;
  } else {
    file = fopen(FLAGS_o.c_str(), "w");
    view_ = false;
  }


  int count = 0;
  while( cam.Capture( vImages ) && (count < 40)){

    count++;
    fprintf(stdout, "At frame: %d\n", count);

    // 1) Capture and rectify
    calibu::Rectify( lut, vImages[0].data, rect.data, rect.cols, rect.rows );
    cv::cvtColor( rect, rgb, CV_GRAY2RGB );

    // 2) Run tag detector and get tag corners
    std::vector<april_tag_detection_t> vDetections;
    td.Detect( rect, vDetections );
    if( vDetections.empty() ){
      fprintf(stderr, "No tags detected at this frame.\n");
      fflush(stderr);
      continue;
    }

    // 3) estimate rough pose from this first seen target
    Eigen::Vector3d pts_3d[4];
    for( int cidx = 0; cidx < 4; cidx++ ){
      pts_3d[cidx] = survey_map[ vDetections[0].id*100+cidx ];
//      fprintf(stdout, "%f, %f, %f\n", pts_3d[cidx][0], pts_3d[cidx][1], pts_3d[cidx][2]);
    }

//    for(int i = 0; i < 4; i++)
//    fprintf(stdout, "%f, %f\n", vDetections[0].p[i][0], vDetections[0].p[i][1]);

    int id = vDetections[0].id;

    Eigen::Vector6d T_tc = CalcPose( vDetections[0].p, pts_3d, K, p[4] );

    Eigen::Vector6d T_wc;
    T_wc = _T2Cart(_Cart2T(tags[vDetections[0].id].pose) * _Cart2T(T_tc) );
//    if (id == 26)
      poses.push_back( T_tc );

    // 4) Extract measurements of the 4 corners of each detected target
    for( size_t ii = 0; ii < vDetections.size(); ii++ ){
      april_tag_detection_t* p = &vDetections[ii];

      // target not seen before? then add 4 new landmarks
      if( tag_seen.find(p->id) == tag_seen.end() ){
        // ok, remap unique survey landmark id to new 0-based index
        for( int cidx = 0; cidx < 4; cidx++ ){
          int survey_id = p->id*100+cidx;
          local_to_survey[ local_landmark_id ] = survey_id;
          survey_to_local[ survey_id ] = local_landmark_id;
          local_landmark_id++;
        }
        tag_seen[p->id] = true;
      }

      // add 4 measurements for this tag's corners
      for( int cidx = 0; cidx < 4; cidx++ ){
        int survey_id = p->id*100+cidx;
        int lid = survey_to_local[ survey_id ]; // look up local id
        measurement_t z = { local_pose_id, lid, p->p[cidx][0], p->p[cidx][1] };
        measurements.push_back( z );
      }

      // draw rectangle around tag
      cv::line( rgb,
                cv::Point( p->p[0][0], p->p[0][1] ),
          cv::Point( p->p[1][0], p->p[1][1] ),
          cv::Scalar( 255, 0, 0 ), 1, 8 );

      cv::line( rgb,
                cv::Point( p->p[1][0], p->p[1][1] ),
          cv::Point( p->p[2][0], p->p[2][1] ),
          cv::Scalar( 0, 255, 0 ), 1, 8 );

      cv::line( rgb,
                cv::Point( p->p[2][0], p->p[2][1] ),
          cv::Point( p->p[3][0], p->p[3][1] ),
          cv::Scalar( 0, 0, 255 ), 1, 8 );

      cv::line( rgb,
                cv::Point( p->p[3][0], p->p[3][1] ),
          cv::Point( p->p[0][0], p->p[0][1] ),
          cv::Scalar( 255, 0, 255 ), 1, 8 );
    }

    if (view_) {
      cv::imshow( "Tag Viewer", rgb );
      cv::waitKey(0);
    }
    local_pose_id++;
  }

  // ok, now print everything:
  fprintf( file, "%d\n", local_pose_id );
  fprintf( file, "%lu\n", local_to_survey.size() );
  fprintf( file, "%lu\n", measurements.size() );
  fprintf( file, "%d\n", tags.size());
  for( size_t ii = 0; ii < measurements.size(); ii++ ){
    measurement_t& z = measurements[ii];
    fprintf(file, "%d, %d, %f, %f\n", z.pose_idx, z.lm_idx, z.u, z.v );
  }
  for( size_t ii = 0; ii < poses.size(); ii++ ){
    fprintf( file,  "%f, %f, %f, %f, %f, %f\n", poses[ii][0],  poses[ii][1],
        poses[ii][2],  poses[ii][3], poses[ii][4],  poses[ii][5] );
  }
  for( size_t ii = 0; ii < local_to_survey.size(); ii++ ){
    int sid = local_to_survey[ii];
    Eigen::Vector3d& p3d = survey_map[sid];
    fprintf(file, "%d, %f, %f, %f\n", sid, p3d[0], p3d[1], p3d[2] );
  }
  for (std::map<int, tag_t>::iterator it = tags.begin(); it != tags.end(); it++) {
    fprintf(file, "%d, %f, %f, %f, %f, %f, %f\n", it->first,
            it->second.pose(0), it->second.pose(1), it->second.pose(2),
            it->second.pose(3), it->second.pose(4), it->second.pose(5) );
  }
  if (view_)
    fclose(file);
  return 0;
}

