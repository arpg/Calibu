#include <pangolin/pangolin.h>
#include <pangolin/video.h>

#include <cvd/image_convert.h>
#include <cvd/gl_helpers.h>

#include <fiducials/tracker.h>
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

  // Setup Video
  Var<string> video_uri("video_uri");
  VideoInput video(video_uri);

  const unsigned w = video.Width();
  const unsigned h = video.Height();
  CVD::ImageRef size(w,h);

  // Setup Tracker and associated target
  Tracker tracker(size);
  tracker.target.GenerateRandom(
    60,25/(842.0/297.0),75/(842.0/297.0),15/(842.0/297.0),makeVector(297,210)
  );

  // Create Glut window
  pangolin::CreateGlutWindowAndBind("Main",2*w+PANEL_WIDTH,h);
  glEnable (GL_BLEND);
  glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  // Pangolin 3D Render state
  pangolin::OpenGlRenderState s_cam;
  s_cam.Set(ProjectionMatrixRDF_TopLeft(640,480,420,420,320,240,1,1E6));
  s_cam.Set(FromTooN(SE3<>(SO3<>(),makeVector(-tracker.target.Size()[0]/2,-tracker.target.Size()[1]/2,500))));
  pangolin::Handler3D handler(s_cam);

  // Create viewport for video with fixed aspect
  View& vPanel = pangolin::CreatePanel("ui").SetBounds(1.0,0.0,0,PANEL_WIDTH);
  View& vVideo = pangolin::Display("Video").SetAspect((float)w/h);
  View& v3D    = pangolin::Display("3D").SetAspect((float)w/h).SetHandler(&handler);

  Display("Container")
      .SetBounds(1.0,0.0,PANEL_WIDTH,1.0,false)
      .SetLayout(LayoutEqual)
      .AddDisplay(vVideo)
      .AddDisplay(v3D);

  // OpenGl Texture for video frame
  GlTexture texRGB(w,h,GL_RGBA8);
  GlTexture tex(w,h,GL_LUMINANCE8);

  // Declare Image buffers
  CVD::ConvertImage<Rgb<byte>,byte> rgb_to_grey;
  Image<Rgb<byte> > Irgb(size);
  Image<byte> I(size);

  // Camera parameters
  Vector<5,float> cam_params = Var<Vector<5,float> >("cam_params");
  FovCamera cam( w,h, w*cam_params[0],h*cam_params[1], w*cam_params[2],h*cam_params[3], cam_params[4] );

  // Variables
  Var<bool> disp_thresh("ui.Display Thresh",false);
  Var<bool> lock_to_cam("ui.AR",false);

  for(int frame=0; !pangolin::ShouldQuit(); ++frame)
  {
    Viewport::DisableScissor();
    glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

    video.GrabNewest((byte*)Irgb.data(),true);
    rgb_to_grey.convert(Irgb,I);

    const bool tracking_good =
        tracker.ProcessFrame(cam,I);

    if( lock_to_cam )
        s_cam.Set(FromTooN(tracker.T_gw));

    // Display Live Image
    glColor3f(1,1,1);
    vVideo.ActivateScissorAndClear();

    if(!disp_thresh) {
        texRGB.Upload(Irgb.data(),GL_RGB,GL_UNSIGNED_BYTE);
        texRGB.RenderToViewportFlipY();
    }else{
        tex.Upload(tracker.tI.data(),GL_LUMINANCE,GL_UNSIGNED_BYTE);
        tex.RenderToViewportFlipY();
    }

    // Display detected ellipses
    glOrtho(-0.5,w-0.5,h-0.5,-0.5,0,1.0);
    for( int i=0; i<tracker.conics.size(); ++i ) {
      glColorBin(tracker.conics_target_map[i],tracker.target.circles3D().size());
      DrawCross(tracker.conics[i].center,2);
    }

    // Display 3D Vis
    glEnable(GL_DEPTH_TEST);
    v3D.ActivateScissorAndClear(s_cam);
    glDepthFunc(GL_LEQUAL);
    glDrawAxis(30);
    DrawTarget(tracker.target,makeVector(0,0),1,0.2,0.2);
//    DrawTarget(conics_target_map,target,makeVector(0,0),1);

    if( tracking_good )
    {
        // Draw Camera
        glColor3f(1,0,0);
        glDrawFrustrum(cam.Kinv(),w,h,tracker.T_gw.inverse(),10);
    }

    vPanel.Render();

    // Swap back buffer with front
    glutSwapBuffers();

    // Process window events via GLUT
    glutMainLoopEvent();
  }

  return 0;
}
