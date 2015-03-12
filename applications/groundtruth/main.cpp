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
#include "ErrorMetric.h"
#include "usage.h"

#include <ceres/ceres.h>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include "ceres_cost_functions.h"
#include "dense_ceres.h"
#include "hess.h"
#include "ImageAlign/compute_homography.h"

#define MAX_FRAMES 116

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

void LoadGTPoses( const string& filename, std::vector< Eigen::Vector6d >& gtposes )
{
  std::ifstream ifs( filename );
  std::string line;
  fprintf(stderr, "Loading data file %s: \n", filename.c_str());
  std::getline ( ifs, line );
  while( ifs.good() ){
    float x, y, z, r, p, q;
    sscanf( line.c_str(), "%f\t%f\t%f\t%f\t%f\t%f\n", &x, &y, &z, &r, &p, &q );

    Eigen::Vector6d pz;
    Eigen::Vector3d t, rt;
    Eigen::Matrix3d rot;
    // Robotics to vision and invert 'z'
    rot << 0, 1, 0,
           1, 0, 0,
           0, 0, -1;
    t << x, y, z;
    t = rot*t;
    rt << r, p, q;
    rt = _R2Cart(rot*_Cart2R(rt));

    Eigen::Vector3d offset;
    offset << 81, 22, 0;
    offset /= 72;
    offset *= 0.0254;
    t += offset;

    //  Calibu offset from origin
    pz << t(0), t(1), t(2), rt(0), rt(1), rt(2);

    gtposes.push_back(pz);
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
  for (int count = 0; count < 6; count++)
    x[count] = ds[0]->pose.data()[count];
  for( int count = 0; count < ds.size(); count++) {
    std::shared_ptr< detection > d = ds[count];
    ceres::CostFunction* cost_function = ProjectionCost( d, K, cmod);
    if (cost_function == NULL) {
      fprintf(stderr, "cost_function is null\n");
      fflush(stderr);
    }
    problem.AddResidualBlock( cost_function, NULL, x);
  }
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  for (int count = 0; count < 6; count++)
    ds[0]->pose.data()[count] = x[count];
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
    make_hessian(sim_cam, dets[0], l);
  }
}

void dense_optimize( DMap detections,
                     SceneGraph::GLSimCam* sim_cam,
                     Eigen::Matrix3d K)
{
  for (DMap::iterator it = detections.begin(); it != detections.end(); it++) {
    dense_frame_optimize(it->second, sim_cam, K);
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
//  for (int ii = 0; ii < 2; ii++) {
    c_new = 20;
    while (c_new > 0.01) {
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
    }
//  }
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

void print_tag_(FILE* f, int id) {
  TagDetector td;
  unsigned long long value = td.tf_->codes[id];
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
    if (t_reverse[35 - count]) {
      fprintf(f, "1\t");
    } else {
      fprintf(f, "0\t");
    }
    if ((count % 6) == 0) {
      fprintf(f, "\n");
    }
  }

}


void print_test_data_( std::shared_ptr< detection > ds)
{
  FILE* f;
  detection d = *ds;
  f = fopen("data.out", "w");
  fprintf(f, "%f %f %f %f %f %f\n", d.pose(0), d.pose(1), d.pose(2),
                                    d.pose(3), d.pose(4), d.pose(5));
  Eigen::Vector3d c3d;
  c3d = d.tag_data.tl;
  fprintf(f, "%f %f %f\n", c3d(0), c3d(1), c3d(2) );
  c3d = d.tag_data.tr;
  fprintf(f, "%f %f %f\n", c3d(0), c3d(1), c3d(2) );
  c3d = d.tag_data.br;
  fprintf(f, "%f %f %f\n", c3d(0), c3d(1), c3d(2) );
  c3d = d.tag_data.bl;
  fprintf(f, "%f %f %f\n", c3d(0), c3d(1), c3d(2) );

  Eigen::Vector2d c2d;
  c2d = d.tag_corners.tl;
  fprintf(f, "%f %f\n", c2d(0), c2d(1));
  c2d = d.tag_corners.tr;
  fprintf(f, "%f %f\n", c2d(0), c2d(1));
  c2d = d.tag_corners.br;
  fprintf(f, "%f %f\n", c2d(0), c2d(1));
  c2d = d.tag_corners.bl;
  fprintf(f, "%f %f\n", c2d(0), c2d(1));

  fprintf(f, "%d %d\n", d.tag_data.color_low, d.tag_data.color_high);
  print_tag_(f, d.tag_id);

  fclose(f);
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
  std::vector< Eigen::Vector6d > gt_poses_temp, gt_poses;
  if (cl.search("-poses")) {
    LoadGTPoses(cl.follow("", "-poses"), gt_poses_temp);
  }
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
  if( dynamic_cast<calibu::LinearCamera<double>*>( cmod ) ){
    fprintf(stderr, "Opening Linear Camera\n");
  }
  else if( dynamic_cast<calibu::FovCamera<double>*>( cmod ) ){
    fprintf(stderr, "Opening Fov Camera\n");
  }
  else if( dynamic_cast<calibu::Poly3Camera<double>*>( cmod ) ){
    fprintf(stderr, "Opening Poly3 Camera\n");
  }
  else {
    fprintf(stderr, "No matching camera model.\n");
  }

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
  while( start || capture && (count < MAX_FRAMES)){
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

    // 3) For all tags detected in a frame, add a detection to that frame
    std::vector< std::shared_ptr< detection > > ds;
    for( size_t ii = 0; ii < vDetections.size(); ii++ ){
      std::shared_ptr< detection > d(new detection);
      d->frame = count;

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
    gt_poses.push_back(gt_poses_temp[count]);
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

  std::vector< Eigen::Vector6d > poses;
  for( std::map<int, std::vector< std::shared_ptr< detection > > >::iterator it = detections.begin();
       it != detections.end(); it++){
    poses.push_back(it->second[0]->pose);
  }

  if (!cl.search("-v")) {
    sparse_optimize(detections, K, cmod);
    sift_optimize(detections, &sim_cam, &depth_cam, K);
    dense_optimize(detections, &sim_cam, K);
    if (cl.search("-o")) {
      std::cerr << cl.follow("", "-o") << std::endl;
      FILE* ofile = fopen(cl.follow("", "-o").c_str(), "w");
      for( std::map<int, std::vector< std::shared_ptr< detection > > >::iterator it = detections.begin();
           it != detections.end(); it++){
        Eigen::Vector6d poze = it->second[0]->pose;
        fprintf(ofile, "%d\t%f\t%f\t%f\t%f\t%f\t%f\n", it->second[0]->frame, poze(0),
                poze(1),
                poze(2),
                poze(3),
                poze(4),
                poze(5));
      }
      fclose(ofile);
    }

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

  std::vector< SceneGraph::GLAxis > gt_axis;
  gt_axis.resize(gt_poses.size());
  for (int ii = 0; ii < gt_poses.size(); ii++) {
    SceneGraph::GLAxis a;
    a.SetPose(gt_poses[ii]);
    a.SetScale(0.05);
    gt_axis[ii] = a;
    glGraph.AddChild( &gt_axis[ii] );
  }

  bool bStep = false;
  bool use_gt_pose_ = false;
  unsigned long nFrame=0;
  int pose_number = 0;
  DMap::iterator it;
  it = detections.begin();
  Eigen::Vector3d del;
  float dx = 1e-3;

  pangolin::RegisterKeyPressCallback(pangolin::PANGO_SPECIAL + pangolin::PANGO_KEY_RIGHT, [&](){bStep=true; pose_number++;} );
  pangolin::RegisterKeyPressCallback(pangolin::PANGO_SPECIAL + pangolin::PANGO_KEY_LEFT, [&](){bStep=true; pose_number--;} );
  pangolin::RegisterKeyPressCallback('p', [&](){ pose_shift(it->second[0], &sim_cam, K, &depth_cam);
    update_objects(detections,
                   camPoses,
                   campose);});
  pangolin::RegisterKeyPressCallback('s', [&](){ sparse_optimize(detections, K, cmod);
    update_objects(detections,
                   camPoses,
                   campose);} );
  pangolin::RegisterKeyPressCallback('d', [&](){ dense_optimize(detections, &sim_cam, &depth_cam, K);
    update_objects(detections,
                   camPoses,
                   campose);} );
  pangolin::RegisterKeyPressCallback('w', [&](){ sparse_frame_optimize(it->second, K, cmod);
    update_objects(detections,
                   camPoses,
                   campose);} );
  pangolin::RegisterKeyPressCallback('t', [&](){ std::cout<< it->second.size() <<std::endl;} );
  pangolin::RegisterKeyPressCallback(']', [&](){ use_gt_pose_ = !use_gt_pose_;} );
  pangolin::RegisterKeyPressCallback('g', [&](){
    for (int ii = 0; ii < gt_axis.size(); ii++) { gt_axis[ii].SetVisible(!gt_axis[ii].IsVisible());} ;} );
  pangolin::RegisterKeyPressCallback('c', [&](){
    for (int ii = 0; ii < campose.size(); ii++) { campose[ii].SetVisible(!campose[ii].IsVisible());} ;} );
  pangolin::RegisterKeyPressCallback(';', [&](){ print_test_data_(it->second[0]);} );
  pangolin::RegisterKeyPressCallback('j', [&](){
    //  Printing of Error Metric Stuff
    fprintf(stderr, "RMSE frame to frame: %f\n", ErrorMetric::RMSE(gt_poses, camPoses, 1));
    fprintf(stderr, "RMSE_average: %f\n", ErrorMetric::RMSE_ave(gt_poses, camPoses));
//    fprintf(stderr, "ATE: %f\n", ErrorMetric::ATE(gt_poses, camPoses));
    fflush(stderr);
  } );
  pangolin::RegisterKeyPressCallback('e', [&](){ dense_frame_optimize(it->second, &sim_cam, K);
    update_objects(detections,
                   camPoses,
                   campose);} );
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

    if (use_gt_pose_) {
      sim_cam.SetPoseVision(_Cart2T(gt_poses[pose_number]));
    } else {
      sim_cam.SetPoseVision(_Cart2T(camPoses[pose_number]));
    }
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
