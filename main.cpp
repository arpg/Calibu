/**
 * @author  Steven Lovegrove
 * Copyright (C) 2010  Steven Lovegrove
 *                     Imperial College London
 **/

#include <vector>

#include <boost/ptr_container/ptr_vector.hpp>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <pangolin/pangolin.h>
#include <pangolin/video.h>
#include <pangolin/firewire.h>

#include <TooN/se3.h>

#include <cvd/image.h>
#include <cvd/rgb.h>
#include <cvd/image_convert.h>
#include <cvd/integral_image.h>
#include <cvd/vision.h>
#include <cvd/gl_helpers.h>

#include "adaptive_threshold.h"
#include "label.h"
#include "conics.h"
#include "find_conics.h"
#include "target.h"
#include "camera.h"
#include "drawing.h"

using namespace std;
using namespace pangolin;
using namespace TooN;
using namespace CVD;

const int PANEL_WIDTH = 200;

struct Keyframe
{
  SE3<> T_kw;
  vector<Conic> conics;
  vector<int> conics_target_map;
};

double ReprojectionErrorRMS(
  const AbstractCamera& cam,
  const SE3<>& T_cw,
  const Target& target,
  const vector<Vector<2> >& ellipses,
  const vector<int>& ellipse_target_map
) {
  int n=0;
  double sse =0;

  for( unsigned i=0; i<ellipses.size(); ++i )
  {
    const int ti = ellipse_target_map[i];
    if( ti >= 0 )
    {
      const Vector<2> t = cam.project_map(T_cw * target.circles3D()[ti]);
      sse += norm_sq(t - ellipses[i]);
      ++n;
    }
  }

  return sqrt(sse / n);
}

SE3<> StaticPoseFromMirroredVirtualPoses(const boost::ptr_vector<Keyframe>& keyframes )
{
  // As described in this paper
  // "Camera Pose Estimation using Images of Planar Mirror Reflections" ECCV 2010
  // Rui Rodrigues, Joao Barreto, Urbano Nunes

  if( keyframes.size() >= 3 )
  {
    // Method 1
    const unsigned Nkf = keyframes.size() -1;
    const SE3<> T_w0 = keyframes[0].T_kw.inverse();

    Matrix<> As(Nkf*3,4);
    As = Zeros;
    for( int i=0; i<Nkf; ++i )
    {
      const SE3<> T_iw = keyframes[i+1].T_kw * T_w0;
      const Vector<3> t = T_iw.get_translation();
      const Vector<3> theta_omega = T_iw.get_rotation().ln();
      const double theta = norm(theta_omega);
      const Vector<3> omega = theta_omega / theta;
      const double tanthby2 = tan(theta/2.0);
      const Matrix<3,3> skew_t = SkewSym(t);

      As.slice(3*i,0,3,3) = skew_t + tanthby2 * omega.as_col() * t.as_row();
      As.slice(3*i,3,3,1) = -2 * tanthby2 * omega.as_col();
    }

    // Solve system using SVD
    SVD<> svd(As);

    const Vector<4> _N_0 = svd.get_VT()[3];
    const Vector<4> N_0 = _N_0 / norm(_N_0.slice<0,3>());

//    {
//    // Compute Symmetry transformation S in ss(3) induced by N_0
//    const Matrix<4,4> S = SymmetryTransform(N_0);
//    const Vector<4> x_0 = S * makeVector(0,0,0,1);

//    // Compensate for fact we were detecting reflected target
//    const Vector<3> x_w_ref = project(T_w0 * x_0);
//    const Vector<3> x_w = makeVector(x_w_ref[0],x_w_ref[1],-x_w_ref[2]);
//    return SE3<>(SO3<>(), x_w );
//    }

    // Transform plane to world frame of reference
    const Vector<4> N_w = T_4x4(T_w0.inverse()).T() * N_0;
    const Vector<4> N_z0 = makeVector(0,0,1,0);

    const Matrix<4,4> S1 = SymmetryTransform(N_z0);
    const Matrix<4,4> S2 = SymmetryTransform(N_w);

    const SE3<> T_00 = FromMatrix(S2*S1);
    const SE3<> T_w0r = T_w0 * T_00;
    return T_w0r;
  }else{
    return SE3<>();
  }
}

#include <map>
#include <vector>
#include <opencv/cv.h>
#include <set>
#include <cvd/random.h>

void opencv_pnp(
    const LinearCamera& cam,
    const Target& target,
    const vector<Vector<2> >& ellipses,
    const vector<int>& ellipse_target_map,
    SE3<>& T_cw,
    bool use_guess = false
) {
  // Attempt to compute pose
  if( ellipses.size() >= 4 )
  {
    vector<cv::Point2f> cvimg;
    vector<cv::Point3f> cvpts;
    for( unsigned int i=0; i < ellipses.size(); ++ i)
    {
      const int ti = ellipse_target_map[i];
      if( 0 <= ti)
      {
        assert( ti < (int)target.circles3D().size() );
        const Vector<2> m = cam.unmap(ellipses[i]);
        const Vector<3> t = target.circles3D()[ti];
        cvimg.push_back( cv::Point2f( m[0],m[1] ) );
        cvpts.push_back( cv::Point3f( t[0], t[1], t[2] ) );
      }
    }
    if( cvimg.size() >= 4 )
    {
      cv::Mat cv_matched_3d(cvpts);
      cv::Mat cv_matched_obs(cvimg);
      cv::Mat cv_K(3,3,CV_64FC1);
      cv::setIdentity(cv_K);
      cv::Mat cv_dist(4,1,CV_64FC1,0.0);

      Vector<3> rot_vec = T_cw.get_rotation().ln();
      Vector<3> trans = T_cw.get_translation();
      cv::Mat cv_rot(3,1,CV_64FC1,rot_vec.my_data);
      cv::Mat cv_trans(3,1,CV_64FC1,trans.my_data);
      cv::solvePnP(
        cv_matched_3d, cv_matched_obs,
        cv_K, cv_dist, cv_rot, cv_trans, use_guess
      );
      T_cw =  SE3<double>(TooN::SO3<double>(rot_vec),trans);
    }
  }
}

int main( int /*argc*/, char* argv[] )
{
  // Load configuration data
  pangolin::ParseVarsFile("app.cfg");

  // Setup Video
  Var<string> video_uri("video_uri");
  VideoInput video(video_uri);

  const unsigned w = video.Width();
  const unsigned h = video.Height();

  // Create Glut window
  pangolin::CreateGlutWindowAndBind("Main",2*w+PANEL_WIDTH,h);

  // Pangolin 3D Render state
  pangolin::OpenGlRenderState s_cam;
  s_cam.Set(ProjectionMatrixRDF_TopLeft(640,480,420,420,320,240,1,1E6));
  s_cam.Set(IdentityMatrix(GlModelViewStack));
  pangolin::Handler3D handler(s_cam);

  // Create viewport for video with fixed aspect
  View& vPanel = pangolin::CreatePanel("ui").SetBounds(1.0,0.0,0,PANEL_WIDTH);
  View& vVideo = pangolin::Display("Video").SetAspect((float)w/h);
  View& v3D    = pangolin::Display("3D").SetAspect((float)w/h).SetHandler(&handler);
//  View& vDebug = pangolin::Display("Debug").SetAspect((float)w/h);

  Display("Container")
      .SetBounds(1.0,0.0,PANEL_WIDTH,1.0,false)
      .SetLayout(LayoutEqual)
      .AddDisplay(vVideo)
      .AddDisplay(v3D)
//      .AddDisplay(vDebug)
      ;

  // OpenGl Texture for video frame
  GlTexture texRGB(w,h,GL_RGBA8);
  GlTexture tex(w,h,GL_LUMINANCE8);

  // Declare Image buffers
  CVD::ImageRef size(w,h);
  CVD::ConvertImage<Rgb<byte>,byte> rgb_to_grey;
  Image<Rgb<byte> > Irgb(size);
  Image<byte> I(size);
  Image<float> intI(size);
  Image<short> lI(size);
  Image<float[2]> dI(size);
  Image<byte> tI(size);

  // Camera parameters
  Vector<5,float> cam_params = Var<Vector<5,float> >("cam_params");
  FovCamera cam( w,h, w*cam_params[0],h*cam_params[1], w*cam_params[2],h*cam_params[3], cam_params[4] );

  // Target to track from
  Target target;
  target.GenerateRandom(60,25/(842.0/297.0),75/(842.0/297.0),15/(842.0/297.0),makeVector(297,210));
//  target.GenerateCircular(60,20,50,15,makeVector(210,210));
//  target.GenerateEmptyCircle(60,25,75,15,200,makeVector(297,210));
  target.SaveEPS("test.eps");

  // Current pose
  SE3<> T_cw;

  // Fixed mirrored pose
  SE3<> T_0w;

  // Stored keyframes
  boost::ptr_vector<Keyframe> keyframes;

  // Variables
  Var<bool> add_keyframe("ui.Add Keyframe",false,false);

  Var<bool> use_mirror("ui.Use Mirror",false,true);
  Var<bool> calc_mirror_pose("ui.Calculate Mirrored Pose",false,false);

  Var<float> at_threshold("ui.Adap Threshold",0.5,0,1.0);
  Var<int> at_window("ui.Adapt Window",20,1,200);
  Var<float> conic_min_area("ui.Conic min area",40, 0, 1E5);
  Var<float> conic_max_area("ui.Conic max area",1E4, 0, 1E5);
  Var<float> conic_min_density("ui.Conic min density",0.7, 0, 1.0);
  Var<float> conic_min_aspect("ui.Conic min aspect",0.1, 0, 1.0);
  Var<float> conic_max_residual("Conic max residual",1);
  Var<int> target_match_neighbours("ui.Match Descriptor Neighbours",10, 5, 20);
  Var<int> target_ransac_its("ui.Ransac Its", 100, 20, 500);
  Var<int> target_ransac_min_pts("ui.Ransac Min Pts", 5, 5, 10);
  Var<float> target_ransac_max_inlier_err_mm("ui.Ransac Max Inlier Err (mm)", 15, 0, 50);
  Var<float> target_plane_inlier_thresh("ui.Plane inlier thresh", 1.5, 0.1, 10);
  Var<double> rms("ui.RMS", 0);
  Var<double> max_rms("ui.max RMS", 1.0, 0.01, 10);

  for(int frame=0; !pangolin::ShouldQuit(); ++frame)
  {
    glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

    // Get newest frame from camera and upload to GPU as texture
    video.GrabNewest((byte*)Irgb.data(),true);

    // Generic processing
    rgb_to_grey.convert(Irgb,I);
    gradient<>(I,dI);
    integral_image(I,intI);

    // Threshold and label image
    AdaptiveThreshold(I,intI,tI,at_threshold,at_window,(byte)255,(byte)0);
    vector<PixelClass> labels;
    Label(tI,lI,labels);

    // Find conics
    vector<Conic> conics;
    FindConics(
      labels,dI,conics,
      conic_min_area,conic_max_area,conic_min_density,
      conic_min_aspect,conic_max_residual
    );

    // Generate map and point structures
    vector<int> conics_target_map(conics.size(),-1);
    vector<Vector<2> > ellipses;
    for( size_t i=0; i < conics.size(); ++i )
      ellipses.push_back(conics[i].center);

    // Find target given approximate pose T_cw
    // TODO: Make this a bit better!
    target.FindTarget( T_cw, cam, conics, conics_target_map );

    // Update pose given correspondences
    opencv_pnp(cam,target,ellipses,conics_target_map,T_cw,true);
    // TODO: put rms in terms of target units.
    rms = ReprojectionErrorRMS(cam,T_cw,target,ellipses,conics_target_map);

    if( !isfinite((double)rms) || rms > max_rms )
    {
      // Undistort Conics
      vector<Conic> conics_camframe;
      for( unsigned int i=0; i<conics.size(); ++i )
        conics_camframe.push_back(UnmapConic(conics[i],cam));

      // Find target given (approximately) undistorted conics
      const static LinearCamera idcam(-1,-1,1,1,0,0);
      target.FindTarget(
          idcam,conics_camframe, conics_target_map, target_match_neighbours,
          target_ransac_its, target_ransac_min_pts, target_ransac_max_inlier_err_mm,
          target_plane_inlier_thresh, use_mirror
        );

      // Estimate camera pose relative to target coordinate system
      opencv_pnp(cam,target,ellipses,conics_target_map,T_cw,false);
      rms = ReprojectionErrorRMS(cam,T_cw,target,ellipses,conics_target_map);
    }

    if( pangolin::Pushed(add_keyframe) )
    {
      Keyframe* kf = new Keyframe();
      kf->T_kw = T_cw;
      kf->conics.insert(kf->conics.begin(),conics.begin(),conics.end());
      kf->conics_target_map.insert(kf->conics_target_map.begin(),conics_target_map.begin(),conics_target_map.end());
      keyframes.push_back(kf);
    }

    if( pangolin::Pushed(calc_mirror_pose) )
    {
      T_0w = StaticPoseFromMirroredVirtualPoses(keyframes);
    }

    // Display Live Image
    glColor3f(1,1,1);
    vVideo.ActivateScissorAndClear();
    texRGB.Upload(Irgb.data(),GL_RGB,GL_UNSIGNED_BYTE);
    texRGB.RenderToViewportFlipY();

    // Display detected ellipses
    glOrtho(-0.5,w-0.5,h-0.5,-0.5,0,1.0);
    for( int i=0; i<ellipses.size(); ++i ) {
      glColorBin(conics_target_map[i],ellipses.size());
      DrawCross(ellipses[i],2);
    }

//    // Display thresholded image
//    glColor3f(1,1,1);
//    vDebug.ActivateScissorAndClear();
//    tex.Upload(tI.data(),GL_LUMINANCE,GL_UNSIGNED_BYTE);
//    tex.RenderToViewportFlipY();

    // Display 3D Vis
    glEnable(GL_DEPTH_TEST);
    v3D.ActivateScissorAndClear(s_cam);
    glDepthFunc(GL_LEQUAL);
    glDrawAxis(30);
    DrawTarget(target,makeVector(0,0),1,0.2,0.2);
    DrawTarget(conics_target_map,target,makeVector(0,0),1);

    // Static pose
    glColor3f(0,0,1);
    DrawCross(T_0w.get_translation());
//    glDrawFrustrum(cam.Kinv(),w,h,T_0w.inverse(),30);


    // Live pose
    glColor3f(1,0,0);
    glDrawFrustrum(cam.Kinv(),w,h,T_cw.inverse(),30);

    // Keyframes
    glColor3f(0.5,0.5,0.5);
    foreach (Keyframe& kf, keyframes) {
      glDrawFrustrum(cam.Kinv(),w,h,kf.T_kw.inverse(),30);
    }

    vPanel.Render();

    // Swap back buffer with front
    glutSwapBuffers();

    // Process window events via GLUT
    glutMainLoopEvent();
  }

  return 0;
}
