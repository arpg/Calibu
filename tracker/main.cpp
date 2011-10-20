#include <vector>
#include <ctime>

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

#include <fiducials/adaptive_threshold.h>
#include <fiducials/label.h>
#include <fiducials/conics.h>
#include <fiducials/find_conics.h>
#include <fiducials/target.h>
#include <fiducials/pnp.h>
#include <fiducials/camera.h>
#include <fiducials/drawing.h>

using namespace std;
using namespace pangolin;
using namespace TooN;
using namespace CVD;

const int PANEL_WIDTH = 200;

int main( int /*argc*/, char* argv[] )
{
  // Load configuration data
  pangolin::ParseVarsFile("app.cfg");

  // Target to track from
  Target target;
  target.GenerateRandom(60,25/(842.0/297.0),75/(842.0/297.0),15/(842.0/297.0),makeVector(297,210));
//  target.GenerateCircular(60,20,50,15,makeVector(210,210));
//  target.GenerateEmptyCircle(60,25,75,15,200,makeVector(297,210));
  target.SaveEPS("target_A4.eps");
  cout << "Calibration target saved as: target_A4.eps" << endl;

  // Setup Video
  Var<string> video_uri("video_uri");
  VideoInput video(video_uri);

  const unsigned w = video.Width();
  const unsigned h = video.Height();

  // Create Glut window
  pangolin::CreateGlutWindowAndBind("Main",2*w+PANEL_WIDTH,h);
  glEnable (GL_BLEND);
  glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  // Pangolin 3D Render state
  pangolin::OpenGlRenderState s_cam;
  s_cam.Set(ProjectionMatrixRDF_TopLeft(640,480,420,420,320,240,1,1E6));
  s_cam.Set(FromTooN(SE3<>(SO3<>(),makeVector(-target.Size()[0]/2,-target.Size()[1]/2,500))));
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

  // Last good pose
  int good_frames = 0;
  clock_t last_good = 0;
  SE3<> T_gw;

  // Pose hypothesis
  SE3<> T_hw;

  // Variables
  Var<bool> disp_thresh("ui.Display Thresh",false);
  Var<float> at_threshold("ui.Adap Threshold",1.0,0,1.0);
  Var<int> at_window("ui.Adapt Window",w/3,1,200);
  Var<float> conic_min_area("ui.Conic min area",25, 0, 1E5);
  Var<float> conic_max_area("ui.Conic max area",4E4, 0, 1E5);
  Var<float> conic_min_density("ui.Conic min density",0.4, 0, 1.0);
  Var<float> conic_min_aspect("ui.Conic min aspect",0.1, 0, 1.0);
  Var<int> target_match_neighbours("ui.Match Descriptor Neighbours",10, 5, 20);
  Var<int> target_ransac_its("ui.Ransac Its", 100, 20, 500);
  Var<int> target_ransac_min_pts("ui.Ransac Min Pts", 5, 5, 10);
  Var<float> target_ransac_max_inlier_err_mm("ui.Ransac Max Inlier Err (mm)", 15, 0, 50);
  Var<float> target_plane_inlier_thresh("ui.Plane inlier thresh", 1.5, 0.1, 10);
  Var<double> rms("ui.RMS", 0);
  Var<double> max_rms("ui.max RMS", 1.0, 0.01, 10);
  Var<bool> lock_to_cam("ui.AR",false);

  Var<double> robust_4pt_inlier_tol("ui.Ransac 4Pt Inlier", 0.006, 1E-3, 1E-2);
  Var<int>    robust_4pt_its("ui.Ransac 4pt its", 200, 10, 500);
  Var<double> max_mmps("ui.max mms per sec", 1500, 100, 2000);

  for(int frame=0; !pangolin::ShouldQuit(); ++frame)
  {
    Viewport::DisableScissor();
    glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

    // Get newest frame from camera and upload to GPU as texture
    video.GrabNewest((byte*)Irgb.data(),true);

    // Generic processing
    rgb_to_grey.convert(Irgb,I);
    gradient<>(I,dI);
    integral_image(I,intI);

    // Threshold and label image
    AdaptiveThreshold(I,intI,tI,at_threshold,at_window,(byte)0,(byte)255);
    vector<PixelClass> labels;
    Label(tI,lI,labels);

    // Find conics
    vector<PixelClass> candidates;
    vector<Conic> conics;
    FindCandidateConicsFromLabels(
      I.size(),labels,candidates,
      conic_min_area,conic_max_area,conic_min_density, conic_min_aspect
    );
    FindConics( candidates,dI,conics );

    // Generate map and point structures
    vector<int> conics_target_map(conics.size(),-1);
    vector<Vector<2> > ellipses;
    for( size_t i=0; i < conics.size(); ++i )
      ellipses.push_back(conics[i].center);

    // Find target given approximate pose T_cw
    target.FindTarget( T_hw, cam, conics, conics_target_map );

    // Update pose given correspondences
    T_hw = FindPose(cam,target.circles3D(),ellipses,conics_target_map,robust_4pt_inlier_tol,robust_4pt_its);
    rms = ReprojectionErrorRMS(cam,T_hw,target.circles3D(),ellipses,conics_target_map);

    int inliers =0;
    for( int i=0; i < conics_target_map.size(); ++i)
        if( conics_target_map[i] >=0 ) inliers++;

    if( !isfinite((double)rms) || rms > max_rms || (good_frames < 5 && inliers < 6) )
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
          target_plane_inlier_thresh
        );

      // Estimate camera pose relative to target coordinate system
      T_hw = FindPose(cam,target.circles3D(),ellipses,conics_target_map,robust_4pt_inlier_tol,robust_4pt_its);
      rms = ReprojectionErrorRMS(cam,T_hw,target.circles3D(),ellipses,conics_target_map);
    }

    if( isfinite((double)rms) && rms < max_rms )
    {
        // seconds since good
        const double t = std::difftime(clock(),last_good) / (double) CLOCKS_PER_SEC;
        const double dx = norm(T_hw.inverse().get_translation() - T_gw.inverse().get_translation());
        if( dx / t < max_mmps || last_good == 0) {
            good_frames++;
        }else{
            good_frames = 0;
        }
        if( good_frames > 5 )
        {
            T_gw = T_hw;
            last_good = clock();
        }
    }else{
        good_frames = 0;
    }

    if( lock_to_cam )
    {
      // Follow Live camera
      s_cam.Set(FromTooN(T_gw));
    }


    // Display Live Image
    glColor3f(1,1,1);
    vVideo.ActivateScissorAndClear();

    if(!disp_thresh) {
        texRGB.Upload(Irgb.data(),GL_RGB,GL_UNSIGNED_BYTE);
        texRGB.RenderToViewportFlipY();
    }else{
        tex.Upload(tI.data(),GL_LUMINANCE,GL_UNSIGNED_BYTE);
        tex.RenderToViewportFlipY();
    }

    // Display detected ellipses
    glOrtho(-0.5,w-0.5,h-0.5,-0.5,0,1.0);
    for( int i=0; i<ellipses.size(); ++i ) {
      glColorBin(conics_target_map[i],target.circles3D().size());
      DrawCross(ellipses[i],2);
    }

    // Display 3D Vis
    glEnable(GL_DEPTH_TEST);
    v3D.ActivateScissorAndClear(s_cam);
    glDepthFunc(GL_LEQUAL);
    glDrawAxis(30);
    DrawTarget(target,makeVector(0,0),1,0.2,0.2);
    DrawTarget(conics_target_map,target,makeVector(0,0),1);

    // Draw Camera
    if( good_frames > 5 )
    {
        glColor3f(1,0,0);
        glDrawFrustrum(cam.Kinv(),w,h,T_gw.inverse(),10);
    }

    vPanel.Render();

    // Swap back buffer with front
    glutSwapBuffers();

    // Process window events via GLUT
    glutMainLoopEvent();
  }

  return 0;
}
