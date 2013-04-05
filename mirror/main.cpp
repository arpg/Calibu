#include <map>
#include <vector>

#include <boost/ptr_container/ptr_vector.hpp>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <pangolin/pangolin.h>
#include <pangolin/video.h>
#include <pangolin/video_record_repeat.h>
#include <pangolin/input_record_repeat.h>

#include <sophus/se3.hpp>

#include <fiducials/config.h>
#include <cvd/image.h>
//#include <cvd/image_io.h>
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
using namespace Eigen;
using namespace CVD;

const int PANEL_WIDTH = 200;

struct Keyframe
{
    Sophus::SE3d T_kw;
    vector<Conic> conics;
    vector<int> conics_target_map;
};

void ConvertRGBtoI(BasicImage<Rgb<byte> >& Irgb, BasicImage<byte>& I)
{
    static CVD::ConvertImage<Rgb<byte>,byte> rgb_to_grey;
    rgb_to_grey.convert(Irgb,I);
}

void DrawLabels(const vector<PixelClass>& labels )
{
    foreach(const PixelClass& p, labels)
    {
        if( p.equiv == -1 )
        {
            glBegin(GL_LINES);
            glVertex2f(p.bbox.x1,p.bbox.y1);
            glVertex2f(p.bbox.x2,p.bbox.y1);
            glVertex2f(p.bbox.x1,p.bbox.y2);
            glVertex2f(p.bbox.x2,p.bbox.y2);
            glVertex2f(p.bbox.x1,p.bbox.y1);
            glVertex2f(p.bbox.x1,p.bbox.y2);
            glVertex2f(p.bbox.x2,p.bbox.y1);
            glVertex2f(p.bbox.x2,p.bbox.y2);
            glEnd();
        }
    }
}

int main( int /*argc*/, char* argv[] )
{
    const static string ui_file = "app.clicks";

    // Load configuration data
    pangolin::ParseVarsFile("app.cfg");

    InputRecordRepeat input("ui.");
    input.LoadBuffer(ui_file);

    // Target to track from
    Target target;
    target.GenerateRandom(60,25/(842.0/297.0),75/(842.0/297.0),15/(842.0/297.0),Vector2d(297,210));
    //  target.GenerateCircular(60,20,50,15,makeVector(210,210));
    //  target.GenerateEmptyCircle(60,25,75,15,200,makeVector(297,210));
    target.SaveEPS("target_A4.eps");
    cout << "Calibration target saved as: target_A4.eps" << endl;

    // Setup Video
    Var<string> video_uri("video_uri");
    VideoRecordRepeat video(video_uri, "video.pvn", 1024*1024*200);
//    VideoInput video(video_uri);

    const unsigned w = video.Width();
    const unsigned h = video.Height();

    // Create Glut window
    pangolin::CreateGlutWindowAndBind("Main",2*w+PANEL_WIDTH,h);

    // Pangolin 3D Render state
    pangolin::OpenGlRenderState s_cam;
    s_cam.SetProjectionMatrix(ProjectionMatrixRDF_TopLeft(640,480,420,420,320,240,1,1E6));
    s_cam.SetModelViewMatrix(Sophus::SE3d(Sophus::SO3d(),Vector3d(-target.Size()[0]/2,-target.Size()[1]/2,500) ).matrix() );
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
    Image<Rgb<byte> > Irgb(size);
    Image<byte> I(size);
    Image<float> intI(size);
    Image<short> lI(size);
    Image<boost::array<float,2> > dI(size);
    Image<byte> tI(size);

    // Camera parameters
    Matrix<float,5,1> cam_params; // = Var<Matrix<float,5,1> >("cam_params");
    cerr << "Not loading params properly" << endl;
    cam_params << 0.694621, 0.925258, 0.505055, 0.484551, 0.968455;
    FovCamera cam( w,h, w*cam_params[0],h*cam_params[1], w*cam_params[2],h*cam_params[3], cam_params[4] );

    // Last good pose
    Sophus::SE3d T_gw;
    std::clock_t last_good;
    int good_frames;

    // Pose hypothesis
    Sophus::SE3d T_hw;

    // Fixed mirrored pose
    Sophus::SE3d T_0w;

    // Stored keyframes
    boost::ptr_vector<Keyframe> keyframes;

    // Variables
    Var<bool> record("ui.Record",false,false);
    Var<bool> play("ui.Play",false,false);
    Var<bool> source("ui.Source",false,false);

    Var<bool> add_keyframe("ui.Add Keyframe",false,false);

    Var<bool> use_mirror("ui.Use Mirror",false,true);
    Var<bool> draw_mirror("ui.Draw Mirror",false,true);
//    Var<bool> calc_mirror_pose("ui.Calculate Mirrored Pose",false,false);

    Var<bool> disp_thresh("ui.Display Thresh",false);
    Var<float> at_threshold("ui.Adap Threshold",1.0,0,1.0);
    Var<int> at_window("ui.Adapt Window",w/3,1,200);
    Var<float> conic_min_area(".Conic min area",25, 0, 100);
    Var<float> conic_max_area(".Conic max area",4E4, 0, 1E5);
    Var<float> conic_min_density(".Conic min density",0.4, 0, 1.0);
    Var<float> conic_min_aspect(".Conic min aspect",0.1, 0, 1.0);
    Var<int> target_match_neighbours(".Match Descriptor Neighbours",10, 5, 20);
    Var<int> target_ransac_its("ui.Ransac Its", 100, 20, 500);
    Var<int> target_ransac_min_pts("ui.Ransac Min Pts", 5, 5, 10);
    Var<float> target_ransac_max_inlier_err_mm("ui.Ransac Max Inlier Err (mm)", 15, 0, 50);
    Var<float> target_plane_inlier_thresh("ui.Plane inlier thresh", 1.5, 0.1, 10);
    Var<double> rms("ui.RMS", 0);
    Var<double> max_rms("ui.max RMS", 1.0, 0.01, 10);
    Var<double> robust_4pt_inlier_tol("ui.Ransac 4Pt Inlier", 0.006, 1E-3, 1E-2);
    Var<int>    robust_4pt_its("ui.Ransac 4pt its", 100, 10, 500);
    Var<double> max_mmps("ui.max mms per sec", 1500, 100, 2000);

    Var<bool> lock_to_cam("ui.AR",false);

    for(int frame=0; !pangolin::ShouldQuit(); ++frame)
    {
        Viewport::DisableScissor();
        glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

        // Get newest frame from camera and upload to GPU as texture
        video.GrabNewest((byte*)Irgb.data(),true);

        // Associate input with this video frame
        input.SetIndex(video.FrameId());

        // Generic processing
        ConvertRGBtoI(Irgb,I);
        gradient(w,h,I.data(),dI.data());
        integral_image(w,h,I.data(),intI.data());

        // Threshold and label image
        AdaptiveThreshold(w,h,I.data(),intI.data(),tI.data(),at_threshold,at_window,(byte)0,(byte)255);
        vector<PixelClass> labels;
        Label(w,h,tI.data(),lI.data(),labels);

        // Find conics
        vector<PixelClass> candidates;
        vector<Conic> conics;
        FindCandidateConicsFromLabels(
                    w,h,labels,candidates,
                    conic_min_area,conic_max_area,conic_min_density, conic_min_aspect
                    );
        FindConics(w,h,candidates,dI.data(),conics );

        // Generate map and point structures
        vector<int> conics_target_map(conics.size(),-1);
        vector<Vector2d > ellipses;
        for( size_t i=0; i < conics.size(); ++i )
            ellipses.push_back(conics[i].center);

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
            const static CameraModelBase idcam(-1,-1,1,1,0,0);
            target.FindTarget(
                        idcam,conics_camframe, conics_target_map, target_match_neighbours,
                        target_ransac_its, target_ransac_min_pts, target_ransac_max_inlier_err_mm,
                        target_plane_inlier_thresh,use_mirror
                        );

            // Estimate camera pose relative to target coordinate system
            T_hw = FindPose(cam,target.circles3D(),ellipses,conics_target_map,robust_4pt_inlier_tol,robust_4pt_its);
            rms = ReprojectionErrorRMS(cam,T_hw,target.circles3D(),ellipses,conics_target_map);
        }

        if( isfinite((double)rms) && rms < max_rms )
        {
            // seconds since good
            const double t = std::difftime(clock(),last_good) / (double) CLOCKS_PER_SEC;
            const double dx = (T_hw.inverse().translation() - T_gw.inverse().translation()).norm();
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

        if( good_frames > 5 )
        {
            s_cam.Follow(T_gw.matrix(), lock_to_cam);

            if( pangolin::Pushed(add_keyframe) ) {
                Keyframe* kf = new Keyframe();
                kf->T_kw = T_gw;
                kf->conics.insert(kf->conics.begin(),conics.begin(),conics.end());
                kf->conics_target_map.insert(kf->conics_target_map.begin(),conics_target_map.begin(),conics_target_map.end());
                keyframes.push_back(kf);
            }
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
            glColorBin(conics_target_map[i],target.circles().size());
            DrawCross(ellipses[i],2);
        }

        glColor3f(0,1,0);
        DrawLabels(candidates);

        // Display 3D Vis
        glEnable(GL_DEPTH_TEST);
        v3D.ActivateScissorAndClear(s_cam);
        glDepthFunc(GL_LEQUAL);
        glDrawAxis(30);
        DrawTarget(target,Vector2d(0,0),1,0.2,0.2);
        DrawTarget(conics_target_map,target,Vector2d(0,0),1);

        // Draw Camera
        glColor3f(1,0,0);
        DrawFrustrum(cam.Kinv(),w,h,T_gw.inverse(),10);

        Vector3d r_w = Vector3d::Zero();

        if( keyframes.size() >= 3 )
        {
            // Method 1
            // As described in this paper
            // "Camera Pose Estimation using Images of Planar Mirror Reflections" ECCV 2010
            // Rui Rodrigues, Joao Barreto, Urbano Nunes

            const unsigned Nkf = keyframes.size() -1;
            const Sophus::SE3d T_w0 = keyframes[0].T_kw.inverse();

            MatrixXd As(Nkf*3,4);
            As.setZero();
            for( int i=0; i<Nkf; ++i )
            {
                const Sophus::SE3d T_iw = keyframes[i+1].T_kw * T_w0;
                const Vector3d t = T_iw.translation();
                const Vector3d theta_omega = T_iw.so3().log();
                const double theta = theta_omega.norm();
                const Vector3d omega = theta_omega / theta;
                const double tanthby2 = tan(theta/2.0);
                const Matrix3d skew_t = SkewSym(t);

                As.block<3,3>(3*i,0) = skew_t + tanthby2 * omega * t.transpose();
                As.block<3,1>(3*i,3) = -2 * tanthby2 * omega;
            }

            // Solve system using SVD
            Eigen::JacobiSVD<MatrixXd> svd(As, ComputeFullV);

            // Get mirror plane for virtual camera 0 in cam0 FoR
            const Vector4d _N_0 = svd.matrixV().col(3);
            Vector4d N_0 = _N_0 / (_N_0.head<3>()).norm();
            // d has different meaning in paper. Negate to match my thesis.
            N_0(3) *= -1;

            // Render plane corresponding to first keyframe
            glSetFrameOfReferenceF(keyframes[0].T_kw.inverse());
            glColor3f(0.2,0,0);
            DrawPlane(N_0,10,100);
            glUnsetFrameOfReference();

            // Attempt to render real camera.
            // We seem to have the correct translation. Rotation is wrong.
            // It's because the real camera (from the paper), relates to the real
            // points, Q, but we're using Qhat which doesn't match the real Q.
            {
                const Vector4d Nz = Vector4d(0,0,1,0);
                const Matrix4d T_wr(SymmetryTransform(Nz) * T_w0.matrix() * SymmetryTransform(N_0));
                glColor3f(0,0,1);

                glMatrixMode(GL_MODELVIEW);
                glPushMatrix();
                glMultMatrix( T_wr );
                DrawFrustrum(cam.Kinv(),w,h,30);
                glPopMatrix();
            }

            // Draw live mirror
            if( draw_mirror )
            {
                Vector3d l_w = T_gw.inverse().translation();
                l_w[2] *= -1;
                const Vector3d N = l_w - r_w;
                const double dist = N.norm();
                const Vector3d n = N / dist;
                const double d = -(r_w + N/2.0).dot(n);

                glColor3f(1,0,0);
                DrawCross(l_w);

                glColor4f(0.2,0.2,0.2,0.2);
                DrawPlane(Vector4d(n[0],n[1],n[2],d),10,100);
            }
        }

        // Keyframes
        glColor3f(0.5,0.5,0.5);
//        foreach (Keyframe& kf, keyframes) {
//            DrawFrustrum(cam.Kinv(),w,h,kf.T_kw.inverse(),30);
//        }
        if(keyframes.size() > 0 )
        {
          glSetFrameOfReferenceF(keyframes[0].T_kw.inverse());
          glDrawAxis(30);
          glUnsetFrameOfReference();

          glColor3f(1,0.5,0.5);
          DrawFrustrum(cam.Kinv(),w,h,keyframes[0].T_kw.inverse(),30);
        }

        if(pangolin::Pushed(record)) {
            video.Record();
            input.Record();
        }

        if(pangolin::Pushed(play)) {
            video.Play(true);
            input.PlayBuffer(0,input.Size()-1);
            input.SaveBuffer(ui_file);
        }

        if(pangolin::Pushed(source)) {
            video.Source();
            input.Stop();
            input.SaveBuffer(ui_file);
        }

       pangolin::FinishGlutFrame();
    }

    return 0;
}
