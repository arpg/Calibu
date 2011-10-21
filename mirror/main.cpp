#include <map>
#include <vector>

#include <boost/ptr_container/ptr_vector.hpp>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <pangolin/pangolin.h>
#include <pangolin/video.h>
#include <pangolin/firewire.h>

#include <TooN/se3.h>

#include <cvd/image.h>
#include <cvd/image_io.h>
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

struct Keyframe
{
    SE3<> T_kw;
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
    TooN::SE3<> T_gw;
    std::clock_t last_good;
    int good_frames;

    // Pose hypothesis
    TooN::SE3<> T_hw;

    // Fixed mirrored pose
    SE3<> T_0w;

    // Stored keyframes
    boost::ptr_vector<Keyframe> keyframes;

    // Variables
    Var<bool> add_keyframe("ui.Add Keyframe",false,false);

    Var<bool> use_mirror("ui.Use Mirror",false,true);
    Var<bool> calc_mirror_pose("ui.Calculate Mirrored Pose",false,false);

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

        // Generic processing
        ConvertRGBtoI(Irgb,I);
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

        if( good_frames > 5 )
        {
            if( lock_to_cam ) {
                s_cam.Set(FromTooN(T_gw));
            }

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
        DrawTarget(target,makeVector(0,0),1,0.2,0.2);
        DrawTarget(conics_target_map,target,makeVector(0,0),1);

        // Draw Camera
        glColor3f(1,0,0);
        glDrawFrustrum(cam.Kinv(),w,h,T_gw.inverse(),10);

        Vector<3> r_w = Zeros;

        if( keyframes.size() >= 3 )
        {
            // Method 1
            // As described in this paper
            // "Camera Pose Estimation using Images of Planar Mirror Reflections" ECCV 2010
            // Rui Rodrigues, Joao Barreto, Urbano Nunes

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
            Vector<4> N_0 = _N_0 / norm(_N_0.slice<0,3>());
            N_0[3] *= -1;

            //      glSetFrameOfReferenceF(keyframes[0].T_kw.inverse());
            //      glColor3f(0.2,0,0);
            //      DrawPlane(N_0,10,100);
            //      glUnsetFrameOfReference();

            const Vector<4> _N0_wm  = T_4x4(T_w0.inverse()).T() * N_0;
            const Vector<4> N0_wm  = _N0_wm / norm(_N0_wm.slice<0,3>());

            //      glColor3f(0,0.2,0);
            //      DrawPlane(N0_wm,15,100);


            {
                // Compute Symmetry transformation S in ss(3) induced by N_0
                const Matrix<4,4> S_0_r0 = SymmetryTransform(N_0);
                const Vector<4> tr0_r0 = makeVector(0,0,0,1);
                const Vector<4> t0_0 = S_0_r0 * tr0_r0;

                // Compensate for fact we were detecting reflected target
                const Vector<3> t0_wm = project(T_w0 * t0_0);
                const Vector<3> t0_w = makeVector(t0_wm[0],t0_wm[1],-t0_wm[2]);

                r_w = t0_w;

                glColor3f(0,1,0);
                DrawCross(t0_wm);

                glColor3f(0,1,0);
                DrawCross(r_w);
            }

            //      {
            //        // Compute Symmetry transformation S in ss(3) induced by N_0
            //        const Matrix<4,4> S_wm_wm0 = SymmetryTransform(N0_wm);
            //        const Vector<3> t0_wm0 = T_w0 * makeVector(0,0,0);
            //        const Vector<3> t0_wm = project(S_wm_wm0 * unproject(t0_wm0));

            //        glColor3f(1,0,0);
            //        DrawCross(t0_wm0);

            //        glColor3f(0,0,1);
            //        DrawCross(t0_wm);
            //      }

            {
                //        glSetFrameOfReferenceF(T_w0);

                const Vector<4> N1_0 = makeVector(N_0[0],N_0[1],N_0[2],N_0[3]);
                const Vector<4> N1_w = T_4x4(T_w0.inverse()).T() * N1_0;
                const Vector<4> N1_wr = makeVector(N1_w[0],N1_w[1],-N1_w[2],N1_w[3]);
                const Vector<4> N1_wr_norm = N1_wr / norm(N1_wr.slice<0,3>());
                const Vector<4> N1_0r = T_4x4(T_w0).T() * N1_wr_norm;

                const Vector<4> Nz = makeVector(0,0,-1,0);
                //        SE3<> T_wf = FromMatrix(SymmetryTransform(Nz)*SymmetryTransform(Nz)) ;

                const Matrix<4,4> T_wm = T_w0 * SymmetryTransform(N1_0);
                const Vector<4> Nz_m = T_wm.T() * Nz;

                const SE3<> T_wr = FromMatrix(T_wm * SymmetryTransform(Nz_m));

                glColor3f(0,0,1);
                glDrawFrustrum(cam.Kinv(),w,h,T_wr,30);


                //        cout << "----------------" << endl;
                //        cout << N1_0 << endl;
                //        cout << N1_w << endl;
                //        cout << N1_wr << endl;
                //        cout << N1_wr_norm << endl;
                //        cout << N1_0r << endl;

                //        glColor3f(0,0.2,0);
                //        DrawPlane(Nz,15,100);
                //        DrawPlane(N1_0r,15,100);

                //        glUnsetFrameOfReference();

            }

            //      {
            //        // Compute Symmetry transformation S in ss(3) induced by N_0
            //        const Vector<4> N1 = makeVector(N_0[0],N_0[1],-N_0[2],N_0[3]);
            //        const Vector<4> N2 = makeVector(0,0,-1,0);

            //        TooN::SE3<> T = FromMatrix(SymmetryTransform(N2)*SymmetryTransform(N1));
            //        glColor3f(0,0,1);
            //        glDrawFrustrum(cam.Kinv(),w,h,T_w0*T,30);

            //        glColor3f(0,1,1);
            //        glDrawFrustrum(cam.Kinv(),w,h,(T_w0*T).inverse(),30);
            //        glColor3f(1,1,0);
            //        glDrawFrustrum(cam.Kinv(),w,h,(T*T_w0).inverse(),30);
            //        glColor3f(1,0,1);
            //        glDrawFrustrum(cam.Kinv(),w,h,(T*T_w0),30);
            //      }

            // Draw live mirror
            {
                Vector<3> l_w = T_gw.inverse().get_translation();
                l_w[2] *= -1;
                const Vector<3> N = l_w - r_w;
                const double dist = norm(N);
                const Vector<3> n = N / dist;
                const double d = -(r_w + N/2.0) * n;

                glColor3f(1,0,0);
                DrawCross(l_w);

                //        glColor4f(0.2,0.2,0.2,0.2);
                //        DrawPlane(makeVector(n[0],n[1],n[2],d),10,100);
            }
        }

        //    glColor3f(0.2,0.2,0.2);
        //    Draw_z0(10,100);

        // Static pose
        //    glColor3f(0,0,1);
        //    glDrawFrustrum(cam.Kinv(),w,h,T_0w.inverse(),30);
        //    glColor3f(0,1,0);
        //    glDrawFrustrum(cam.Kinv(),w,h,T_0w,30);
        //    DrawCross(T_0w.get_translation());


        // Keyframes
        glColor3f(0.5,0.5,0.5);
        foreach (Keyframe& kf, keyframes) {
            glDrawFrustrum(cam.Kinv(),w,h,kf.T_kw.inverse(),30);
        }
        //    if(keyframes.size() > 0 )
        //    {
        //      glColor3f(1,0.5,0.5);
        //      glDrawFrustrum(cam.Kinv(),w,h,keyframes[0].T_kw.inverse(),30);
        //    }

        vPanel.Render();

        // Swap back buffer with front
        glutSwapBuffers();

        // Process window events via GLUT
        glutMainLoopEvent();
    }

    return 0;
}
