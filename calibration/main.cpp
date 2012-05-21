#include <pangolin/pangolin.h>
#include <pangolin/video.h>

#include <cvd/image_convert.h>
#include <cvd/gl_helpers.h>

#include <fiducials/tracker.h>
#include <fiducials/drawing.h>
#include <fiducials/utils.h>

#include <CameraModel.h>
#include <CCameraModel/GridCalibrator.h>

#include <Eigen/Eigen>
#include <unsupported/Eigen/MatrixFunctions>
#include <unsupported/Eigen/OpenGLSupport>

#define USE_VICON 1
#define USE_COLOUR 1
//#define USE_USHORT 1

using namespace std;
using namespace pangolin;
using namespace Eigen;
using namespace CVD;

using namespace CCameraModel;

const int PANEL_WIDTH = 200;

#ifdef USE_VICON
#include <vrpn_Tracker.h>
#include <quat.h>

#include <boost/thread.hpp>

inline double Tic()
{
    struct timeval tv;
    gettimeofday(&tv, 0);
    return tv.tv_sec + 1e-6 * (tv.tv_usec);
}

struct ViconTracking
{
    ViconTracking( std::string objectName, std::string host)
    {
        const std::string uri = objectName + "@" + host;
        m_object = new vrpn_Tracker_Remote( uri.c_str() );
        m_object->register_change_handler(this, &ViconTracking::c_callback );
        Start();
    }

    ~ViconTracking()
    {
        Stop();
        delete m_object;
    }

    void EventLoop() {
        while(m_run) {
            m_object->mainloop();
        }
    }

    void Start() {
        m_run = true;
        m_event_thread = boost::thread(&ViconTracking::EventLoop, this);
    }

    void Stop() {
        m_run = false;
        m_event_thread.join();
    }

    void TrackingEvent(const vrpn_TRACKERCB tData )
    {
//        double local_timestamp = Tic();
//        double vicon_timestamp = tData.msg_time.tv_sec + (1e-6 * tData.msg_time.tv_usec);

        q_to_ogl_matrix(&T_wc(0,0), tData.quat);
        T_wc(0,3) = tData.pos[0];
        T_wc(1,3) = tData.pos[1];
        T_wc(2,3) = tData.pos[2];
    }

    static void VRPN_CALLBACK c_callback(void* userData, const vrpn_TRACKERCB tData )
    {
        ViconTracking* self = reinterpret_cast<ViconTracking*>(userData);
        self->TrackingEvent(tData);
    }

    Eigen::Matrix4d T_wc;

    bool m_run;
    vrpn_Tracker_Remote* m_object;
    boost::thread m_event_thread;
};

struct Observation
{
    Eigen::MatrixXd obs;
    Eigen::Matrix4d T_fw;
};

void OptimiseTargetVicon(
    const MatlabCamera& cam,
    const Target& target,
    const std::vector<Observation>& vicon_obs,
    Eigen::Matrix4d& T_cf,
    Eigen::Matrix4d& T_wt
) {
    for( int i=0; i< vicon_obs.size(); ++i ) {
        const Observation& sample = vicon_obs[i];
        for( int j=0; j < target.circles3D().size(); ++j ) {
            Eigen::Vector3d p_t = target.circles3D()[j];
            Eigen::Vector4d P_c = T_cf * sample.T_fw * T_wt * Eigen::Vector4d(p_t[0],p_t[1],p_t[2],1);
//            cam.map()
        }
    }
}
#endif

int main( int /*argc*/, char* argv[] )
{
#ifdef USE_VICON
    ViconTracking vicon("BOX","192.168.10.1");
    std::vector<Observation> vicon_obs;
    Eigen::Matrix4d T_cf;
    Eigen::Matrix4d T_tw;
#endif

    // Load configuration data
    pangolin::ParseVarsFile("app.cfg");
    
    // Setup Video
    Var<string> video_uri("video_uri");
    VideoInput video(video_uri);
    
    const unsigned w = video.Width();
    const unsigned h = video.Height();
    CVD::ImageRef size(w,h);

    // Unit hell!
    const double ppi = 72; // Points Per Inch
    const double USwp = 11 * ppi;
    const double UShp = 8.5 * ppi;
    const double mpi = 0.0254; // meters per inch
    const double mpp = mpi / ppi; // meters per point
    const double unit = mpp;

    // Setup Tracker and associated target
    Tracker tracker(size);
    tracker.target.GenerateRandom(
                //    60,25/(842.0/297.0),75/(842.0/297.0),15/(842.0/297.0),Eigen::Vector2d(297,210) // A4
                60,unit*USwp*25/(842.0),unit*USwp*75/(842.0),unit*USwp*40/(842.0),Eigen::Vector2d(unit*USwp,unit*UShp) // US Letter
                );
    tracker.target.SaveEPS("target.eps");
    
    // Create Glut window
    pangolin::CreateGlutWindowAndBind("Main",2*w+PANEL_WIDTH,h);
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_LINE_SMOOTH);
    glHint(GL_LINE_SMOOTH_HINT,GL_NICEST);

    // Pangolin 3D Render state
    pangolin::OpenGlRenderState s_cam;
    s_cam.Set(ProjectionMatrixRDF_TopLeft(640,480,420,420,320,240,1E-3,1E6));
    s_cam.Set(FromTooN(toTooN(Sophus::SE3(Sophus::SO3(),Vector3d(-tracker.target.Size()[0]/2,-tracker.target.Size()[1]/2,500) ))));
    pangolin::Handler3D handler(s_cam);
    
    // Create viewport for video with fixed aspect
    View& vPanel = pangolin::CreatePanel("ui").SetBounds(1.0,0.0,0,Attach::Pix(PANEL_WIDTH));
    View& vVideo = pangolin::CreateDisplay().SetAspect((float)w/h);
    View& v3D    = pangolin::CreateDisplay().SetAspect((float)w/h).SetHandler(&handler);
    View& v3D2   = pangolin::CreateDisplay().SetAspect((float)w/h).SetHandler(&handler);

    Display("Container")
            .SetBounds(1.0,0.0,Attach::Pix(PANEL_WIDTH),1.0,false)
            .SetLayout(LayoutEqual)
            .AddDisplay(vVideo)
            .AddDisplay(v3D)
#ifdef USE_VICON
            .AddDisplay(v3D2)
#endif
            ;
    
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
    Matrix<double,9,1> cam_params; // = Var<Matrix<double,9,1> >("cam_params");
    cam_params << 0.808936, 1.06675, 0.495884, 0.520504, 0.180668, -0.354284, -0.00169838, 0.000600873, 0.0;
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
        pattern.col(i) = tracker.target.circles3D()[i];
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
            // Reverse map (target -> conic from conic -> target)
            vector<int> target_conics_map(tracker.target.circles().size(), -1);
            for( int i=0; i < tracker.conics_target_map.size(); ++i ) {
                if(tracker.conics_target_map[i] >= 0) {
                    target_conics_map[tracker.conics_target_map[i]] = i;
                }
            }
            
            // Generate list of visible indices
            int unseen = 0;
            std::vector<short int> visibleCircles;
            for( int i=0; i < target_conics_map.size(); ++i ) {
                if( target_conics_map[i] != -1 ) {
                    visibleCircles.push_back(i);
                }else{
                    ++unseen;
                }
            }
            
            // Add observations to calibrator if most of circles are visable
            if( unseen < 10 ) {
                cout << "Adding Observations (unseen: " << unseen << ")" << endl;
                Eigen::MatrixXd obs = Eigen::MatrixXd(2, tracker.target.circles().size() );
                for(size_t i=0; i < tracker.target.circles().size(); ++i ) {
                    const int c = target_conics_map[i];
                    if(c >= 0 ) {
                        Eigen::Vector2d circ = tracker.conics[c].center;
                        obs.col(i) <<  circ[0] , circ[1];
                    }else{
                        obs.col(i) << NAN, NAN;
                    }
                }
                
                calibrator.add_view(obs, visibleCircles);

#ifdef USE_VICON
                vicon_obs.push_back((Observation){obs,vicon.T_wc});
#endif // USE_VICON
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
            s_cam.Set(FromTooN(toTooN(tracker.T_gw)));
        
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
        DrawTarget(tracker.target,Vector2d(0,0),1,0.2,0.2);
        DrawTarget(tracker.conics_target_map,tracker.target,Vector2d(0,0),1);
        
        //    if( tracking_good )
        {
            // Draw Camera
            glColor3f(1,0,0);
            glDrawFrustrum(cam.Kinv(),w,h,tracker.T_gw.inverse(),10);
        }

#ifdef USE_VICON
        v3D2.ActivateScissorAndClear(s_cam);

        glColor3f(0.5,0.5,0.5);
        glDrawGrid(20,0.25);

        glDisable(GL_DEPTH_TEST);
        glColor3f(0.8,0.8,0.8);
        glDrawGrid(5,1.0);
        glDrawAxis(1);
        glEnable(GL_DEPTH_TEST);

        {
            glPushMatrix();
            glMultMatrix(vicon.T_wc);
            glColor3f(1,0,0);
            glDrawAxis(0.1);
            glPopMatrix();
        }
#endif // USE_VICON

        vPanel.Render();
        
        // Swap back buffer with front
        glutSwapBuffers();
        
        // Process window events via GLUT
        glutMainLoopEvent();
    }
    
    return 0;
}
