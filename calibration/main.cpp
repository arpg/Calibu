#include <pangolin/pangolin.h>
#include <pangolin/video.h>

#include <cvd/image_convert.h>
#include <cvd/gl_helpers.h>

#include <fiducials/tracker.h>
#include <fiducials/drawing.h>

#include <CameraModel.h>
#include <CCameraModel/GridCalibrator.h>

#include <Eigen/Eigen>

#define USE_COLOUR 1
//#define USE_USHORT 1

using namespace std;
using namespace pangolin;
using namespace TooN;
using namespace CVD;

using namespace CCameraModel;

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
  tracker.target.SaveEPS("target.eps");

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
  View& vPanel = pangolin::CreatePanel("ui").SetBounds(1.0,0.0,0,Attach::Pix(PANEL_WIDTH));
  View& vVideo = pangolin::Display("Video").SetAspect((float)w/h);
  View& v3D    = pangolin::Display("3D").SetAspect((float)w/h).SetHandler(&handler);

  Display("Container")
      .SetBounds(1.0,0.0,Attach::Pix(PANEL_WIDTH),1.0,false)
      .SetLayout(LayoutEqual)
      .AddDisplay(vVideo)
      .AddDisplay(v3D);

  // OpenGl Texture for video frame
  GlTexture texRGB(w,h,GL_RGBA8);
  GlTexture tex(w,h,GL_LUMINANCE8);

  // Declare Image buffers
#ifdef USE_COLOUR
  CVD::ConvertImage<Rgb<byte>,byte> rgb_to_grey;
  Image<Rgb<byte> > Irgb(size);
#elif USE_USHORT
  CVD::ConvertImage<unsigned short,byte> rgb_to_grey;
  Image<unsigned short> Irgb(size);
#endif
  Image<byte> I(size);

  // Camera parameters
  Vector<9,float> cam_params = Var<Vector<9,float> >("cam_params");
//  FovCamera cam( w,h, w*cam_params[0],h*cam_params[1], w*cam_params[2],h*cam_params[3], cam_params[4] );
  MatlabCamera cam( w,h, w*cam_params[0],h*cam_params[1], w*cam_params[2],h*cam_params[3], cam_params[4], cam_params[5], cam_params[6], cam_params[7], cam_params[8]);

  // Variables
  Var<bool> disp_thresh("ui.Display Thresh",false);
  Var<bool> lock_to_cam("ui.AR",false);
  Var<bool> add_image("ui.add Image",false,false);
  Var<bool> minimise("ui.minimise",false,false);

  Eigen::MatrixXd pattern = Eigen::MatrixXd(3, tracker.target.circles().size() );
  for(size_t i=0; i < tracker.target.circles().size(); ++i )
  {
      TooN::Vector<3> circ = tracker.target.circles3D()[i];
      pattern.col(i) << circ[0] , circ[1], circ[2];
  }

  GridCalibrator calibrator(
//      "Arctan", size.x, size.y, pattern
      "PinholeRadTan", size.x, size.y, pattern
  );

  double rms = 0;
  Var<double> var_rms("ui.rms");

  for(int frame=0; !pangolin::ShouldQuit(); ++frame)
  {
    var_rms = rms;

    Viewport::DisableScissor();
    glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

#if (USE_COLOUR || USE_USHORT )
    video.GrabNewest((byte*)Irgb.data(),true);
    rgb_to_grey.convert(Irgb,I);
#else
    video.GrabNewest((byte*)I.data(),true);
#endif

    const bool tracking_good =
        tracker.ProcessFrame(cam,I);

    if( Pushed(add_image) ) {
        vector<int> target_conics_map(tracker.target.circles().size(), -1);
        for( int i=0; i < tracker.conics_target_map.size(); ++i ) {
            if(tracker.conics_target_map[i] >= 0) {
                target_conics_map[tracker.conics_target_map[i]] = i;
            }
        }

        bool complete = true;
        int unseen = 0;
        for( int i=0; i < target_conics_map.size(); ++i ) {
            if( target_conics_map[i] == -1 ) {
                complete = false;
                ++unseen;
            }
        }

        if(unseen > 0 ) {
            cout << "Unseen: " << unseen << endl;
        }

        if( complete ) {
            cout << "Calling Calibrator" << endl;
            Eigen::MatrixXd obs = Eigen::MatrixXd(2, tracker.target.circles().size() );
            for(size_t i=0; i < tracker.target.circles().size(); ++i ) {
                const int c = target_conics_map[i];
                TooN::Vector<2> circ = tracker.conics[c].center;
                obs.col(i) <<  circ[0] , circ[1];
            }

            cout << obs << endl;
            calibrator.add_view(obs);

            cout <<
                    calibrator.get_camera_copy()->get<double>("fx") / size.x << " " <<
                    calibrator.get_camera_copy()->get<double>("fy") / size.y << " " <<
                    calibrator.get_camera_copy()->get<double>("cx") / size.x << " " <<
                    calibrator.get_camera_copy()->get<double>("cy") / size.y << " " <<
                    calibrator.get_camera_copy()->get<double>("k1") << " " <<
                    calibrator.get_camera_copy()->get<double>("k2") << " " <<
                    calibrator.get_camera_copy()->get<double>("p1") << " " <<
                    calibrator.get_camera_copy()->get<double>("p2") << " 0.0" << endl;

            calibrator.save("camparams.txt");
        }

    }

    if(Pushed(minimise)) {
        calibrator.minimise();
        calibrator.save("camparams.txt");

        cout <<
                calibrator.get_camera_copy()->get<double>("fx") / size.x << " " <<
                calibrator.get_camera_copy()->get<double>("fy") / size.y << " " <<
                calibrator.get_camera_copy()->get<double>("cx") / size.x << " " <<
                calibrator.get_camera_copy()->get<double>("cy") / size.y << " " <<
                calibrator.get_camera_copy()->get<double>("k1") << " " <<
                calibrator.get_camera_copy()->get<double>("k2") << " " <<
                calibrator.get_camera_copy()->get<double>("p1") << " " <<
                calibrator.get_camera_copy()->get<double>("p2") << " 0.0" << endl;
    }

//    calibrator.iterate(rms);

    if( lock_to_cam )
        s_cam.Set(FromTooN(tracker.T_gw));

    // Display Live Image
    glColor3f(1,1,1);
    vVideo.ActivateScissorAndClear();

    if(!disp_thresh) {
#ifdef USE_COLOUR
        texRGB.Upload(Irgb.data(),GL_RGB,GL_UNSIGNED_BYTE);
#elif USE_USHORT
        glPixelTransferScale(100);
        texRGB.Upload(Irgb.data(),GL_LUMINANCE,GL_UNSIGNED_SHORT);
        glPixelTransferScale(1);
#else
        texRGB.Upload(I.data(),GL_LUMINANCE,GL_UNSIGNED_BYTE);
#endif
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
    DrawTarget(tracker.conics_target_map,tracker.target,makeVector(0,0),1);

//    if( tracking_good )
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
