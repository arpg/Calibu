/**
 * @author  Steven Lovegrove
 * Copyright (C) 2010  Steven Lovegrove
 *                     Imperial College London
 **/

#include <vector>

#include <pangolin/pangolin.h>
#include <pangolin/video.h>

#include <TooN/se3.h>

#include <cvd/image.h>
#include <cvd/rgb.h>
#include <cvd/image_convert.h>
#include <cvd/integral_image.h>
#include <cvd/vision.h>

#include "adaptive_threshold.h"
#include "label.h"
#include "conics.h"
#include "find_conics.h"
#include "target.h"
#include "camera.h"

using namespace std;
using namespace pangolin;
using namespace TooN;
using namespace CVD;

const int PANEL_WIDTH = 200;

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

int main( int /*argc*/, char* argv[] )
{
  std::string video_uri = "v4l:///dev/video0";

  // Setup Firewire Camera
//  FirewireVideo video(0,DC1394_VIDEO_MODE_640x480_RGB8,DC1394_FRAMERATE_30,DC1394_ISO_SPEED_400,50);
  VideoInput video(video_uri);

  const unsigned w = video.Width();
  const unsigned h = video.Height();

  // Create Glut window
  pangolin::CreateGlutWindowAndBind("Main",2*w+PANEL_WIDTH,h);

  // Create viewport for video with fixed aspect
  View& vPanel = CreatePanel("ui").SetBounds(1.0,0.0,0,PANEL_WIDTH);
  View& vVideo = Display("Video").SetAspect((float)w/h);
  View& vDebug = Display("Debug").SetAspect((float)w/h);

  Display("Container")
      .SetBounds(1.0,0.0,PANEL_WIDTH,1.0,false)
      .SetLayout(LayoutEqual)
      .AddDisplay(vVideo)
      .AddDisplay(vDebug);

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
  //camera =[ 1.32234 1.76001 0.500052 0.483979 -0.739503 ]
  LinearCamera cam(w,h,w/2,h/3,w*1.32234, h*1.76001);

  // Target to track from
  Target target;
  target.Generate(60,25,75,15,makeVector(842,595));
  target.SaveEPS("test.eps");

  // Current pose
  SE3<> T_cw;

  // Variables
  Var<float> at_threshold("ui.Adap Threshold",0.5,0,1.0);
  Var<int> at_window("ui.Adapt Window",20,1,200);
  Var<float> conic_min_area("ui.Conic min area",40, 0, 1E5);
  Var<float> conic_max_area("ui.Conic max area",1E4, 0, 1E5);
  Var<float> conic_min_density("ui.Conic min density",0.7, 0, 1.0);
  Var<float> conic_min_aspect("ui.Conic min aspect",0.1, 0, 1.0);
  Var<float> conic_max_residual("Conic max residual",1);
  Var<int> target_match_neighbours("ui.Match Neighbours",10, 5, 20);
  Var<int> target_ransac_its("ui.Ransac Its", 100, 20, 500);
  Var<int> target_ransac_min_pts("ui.Ransac Min Pts", 5, 5, 10);
  Var<float> target_ransac_max_err("ui.Ransac Max Err", 2000, 1E2, 1E5);
  Var<float> target_plane_inlier_thresh("ui.Plane inlier thresh", 1.5, 0.1, 10);
  Var<double> rms("ui.RMS", 0);
  Var<double> max_rms("ui.max RMS", 5, 0.1, 10);

  for(int frame=0; !pangolin::ShouldQuit(); ++frame)
  {
    glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

    // Get newest frame from camera and upload to GPU as texture
    video.GrabNext((byte*)Irgb.data(),true);

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
    target.FindTarget(
      T_cw, cam, conics, conics_target_map, target_match_neighbours,
      target_ransac_its, target_ransac_min_pts, target_ransac_max_err,
      target_plane_inlier_thresh
    );

    // Update pose given correspondences
//    PnP(cam,target,ellipses,conics_target_map,T_cw,true);
    rms = ReprojectionErrorRMS(cam,T_cw,target,ellipses,conics_target_map);

    if( !isfinite((double)rms) || rms > max_rms )
    {
      // Undistort Conics
      vector<Conic> conics_camframe;
      for( unsigned int i=0; i<conics.size(); ++i )
        conics_camframe.push_back(UnmapConic(conics[i],cam));

      // Find target given (approximately) undistorted conics
      const static LinearCamera idcam(-1,-1,0,0,1,1);
      target.FindTarget(
          idcam,conics_camframe, conics_target_map, target_match_neighbours,
          target_ransac_its, target_ransac_min_pts, target_ransac_max_err,
          target_plane_inlier_thresh
        );

      // Estimate camera pose relative to target coordinate system
//      PnP(cam,target,ellipses,conics_target_map,T_cw,false);
      rms = ReprojectionErrorRMS(cam,T_cw,target,ellipses,conics_target_map);
    }

    // Display images
    vVideo.Activate();
    texRGB.Upload(Irgb.data(),GL_RGB,GL_UNSIGNED_BYTE);
    texRGB.RenderToViewportFlipY();

    vDebug.Activate();
    tex.Upload(tI.data(),GL_LUMINANCE,GL_UNSIGNED_BYTE);
    tex.RenderToViewportFlipY();

    vPanel.Render();

    // Swap back buffer with front
    glutSwapBuffers();

    // Process window events via GLUT
    glutMainLoopEvent();
  }

  return 0;
}
