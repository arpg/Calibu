#include <memory>

#include <pangolin/pangolin.h>
#include <Pangolin/gldraw.h>

#include <sophus/se3.hpp>

#include <calibu/calib/Calibrator.h>
#include <calibu/image/ImageProcessing.h>
#include <calibu/target/TargetGridDot.h>
#include <calibu/gl/Drawing.h>
#include <calibu/pose/Pnp.h>
#include <calibu/conics/ConicFinder.h>

#include <CVars/CVar.h>

using namespace calibu;

int main( int argc, char** argv)
{    
    if(argc != 2) {
        std::cout << "Usage:" << std::endl;
        std::cout << "\t" << argv[0] << " video_uri" << std::endl;
        return -1;
    }

    // Setup Video Source
    std::string video_uri = argv[1];
    pangolin::VideoInput video(video_uri);
    
    // Allocate stream buffer and vector of images (that will point into buffer)
    unsigned char image_buffer[video.SizeBytes()];
    std::vector<pangolin::Image<unsigned char> > images;
    
    // For the moment, assume all N cameras have same resolution
    const size_t N = video.Streams().size();
    const size_t w = video.Streams()[0].Width();
    const size_t h = video.Streams()[0].Height();
    
    for(size_t i=0; i<N; ++i) {
        if( video.Streams()[i].PixFormat().channels != 1) {
            throw pangolin::VideoException("Video channels must be GRAY8 format. Use Convert:// or fmt=GRAY8 option");
        }
    }
    
    // Make grid of 3d points
    const double grid_spacing = 0.02;
    const Eigen::Vector2i grid_size(19,10);
    const Eigen::Vector2i grid_center(9, 5);
    Calibrator<Fov> calibrator;   
    
    // Setup GUI
    const int PANEL_WIDTH = 150;
    pangolin::CreateGlutWindowAndBind("Main",(N+1)*w/2.0+PANEL_WIDTH,h/2.0);
    
    // Make things look prettier...        
    glEnable(GL_LINE_SMOOTH);
    glHint( GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST );
    glHint( GL_LINE_SMOOTH_HINT, GL_NICEST );
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthFunc( GL_LEQUAL );
    glEnable( GL_DEPTH_TEST );    
    glLineWidth(1.7);

    // Pangolin 3D Render state
    pangolin::OpenGlRenderState stacks;
    stacks.SetProjectionMatrix(pangolin::ProjectionMatrixRDF_TopLeft(640,480,420,420,320,240,0.01,1E6));
    stacks.SetModelViewMatrix(pangolin::ModelViewLookAtRDF(0,0,-0.5, 0,0,0, 0, -1, 0) );
    
    // Create viewport for video with fixed aspect
    pangolin::CreatePanel("ui").SetBounds(1.0,0.0,0,pangolin::Attach::Pix(PANEL_WIDTH));
    
    pangolin::View& container = pangolin::CreateDisplay()
            .SetBounds(1.0,0.0, pangolin::Attach::Pix(PANEL_WIDTH),1.0)
            .SetLayout(pangolin::LayoutEqual);
    
    // Add view for each camera stream
    for(size_t c=0; c < N; ++c) {
        container.AddDisplay( pangolin::CreateDisplay().SetAspect(w/(float)h) );
    }
    
    // Add 3d view, attach input handler
    pangolin::Handler3D handler(stacks);
    pangolin::View& v3D = pangolin::CreateDisplay().SetAspect((float)w/h).SetHandler(&handler);
    container.AddDisplay(v3D);
        
    // OpenGl Texture for video frame
    pangolin::GlTexture tex(w,h,GL_LUMINANCE8);
            
    pangolin::Var<bool> reset("ui.reset", false, false);
    pangolin::Var<bool> step("ui.step", false, false);
    pangolin::Var<bool> run("ui.run", false, true);
    pangolin::Var<bool> add("ui.add", false, true);
    
    pangolin::Var<bool> disp_thresh("ui.Display Thresh",false);
    pangolin::Var<bool> disp_lines("ui.Display Lines",true);
    pangolin::Var<bool> disp_cross("ui.Display crosses",true);
    pangolin::Var<bool> disp_bbox("ui.Display bbox",true);

    pangolin::Var<double> disp_mse("ui.MSE");
    pangolin::Var<int> disp_frame("ui.frame");
    
    for(size_t i=0; i<container.NumChildren(); ++i) {
        pangolin::RegisterKeyPressCallback('1'+i, [&container,i](){container[i].ToggleShow();} );
    }
    
    int calib_cams[N];    
    bool tracking_good[N];
    Sophus::SE3d T_hw[N];
        
    pangolin::RegisterKeyPressCallback('[', [&](){calibrator.Start();} );
    pangolin::RegisterKeyPressCallback(']', [&](){calibrator.Stop();} );

    pangolin::RegisterKeyPressCallback(pangolin::PANGO_SPECIAL+ GLUT_KEY_RIGHT, [&](){step = true;} );
    pangolin::RegisterKeyPressCallback(' ', [&](){run = !run;} );
    pangolin::RegisterKeyPressCallback(pangolin::PANGO_CTRL + 'r', [&](){reset = true;} );
    
    ImageProcessing image_processing(w,h);
//    image_processing.Params().black_on_white = false;
//    image_processing.Params().at_threshold = 1.6;
//    image_processing.Params().at_window_ratio = 30.0;
    image_processing.Params().black_on_white = true;
    image_processing.Params().at_threshold = 0.9;
    image_processing.Params().at_window_ratio = 30.0;
    
    CVarUtils::AttachCVar("proc.adaptive.threshold", &image_processing.Params().at_threshold);
    CVarUtils::AttachCVar("proc.adaptive.window_ratio", &image_processing.Params().at_window_ratio);
    CVarUtils::AttachCVar("proc.black_on_white", &image_processing.Params().black_on_white);
    
    ConicFinder conic_finder;
    conic_finder.Params().conic_min_area = 2.0;
    conic_finder.Params().conic_min_density = 0.208;
    conic_finder.Params().conic_min_aspect = 0.1;
        
    TargetGridDot target(grid_spacing, grid_size, grid_center);

    for(size_t i=0; i<N; ++i) {
        // Add (arbitrary) starting camera params
        const pangolin::StreamInfo& si = video.Streams()[i];
        CameraModel<Fov> default_cam( si.Width(), si.Height() );
        default_cam.Params()  << 300, 300, w/2.0, h/2.0, 0.2;
        calib_cams[i] = calibrator.AddCamera(default_cam);
    }            

    for(int frame=0; !pangolin::ShouldQuit();)
    {     
        if(pangolin::Pushed(reset) ) {
            calibrator.Clear();
            video.Reset();
            frame=0;
            
            // Re-add camers
            for(size_t i=0; i<N; ++i) {
                const pangolin::StreamInfo& si = video.Streams()[i];
                CameraModel<Fov> default_cam( si.Width(), si.Height() );
                default_cam.Params()  << 300, 300, w/2.0, h/2.0, 0.2;
                calib_cams[i] = calibrator.AddCamera(default_cam);
            }            
        }

        const bool go = (frame==0) || run || pangolin::Pushed(step);
        
        int calib_frame = -1;
        
        if( go ) {
            if( video.Grab(image_buffer, images, true, true) ) {
                if(add) calib_frame = calibrator.AddFrame(Sophus::SE3d(Sophus::SO3(), Eigen::Vector3d(0,0,1000)) );
                ++frame;
            }else{
                run = false;
            }
        }  
        
        glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);            
                
        for(size_t iI = 0; iI < N; ++iI)
        {
            image_processing.Process( images[iI].ptr, images[iI].pitch );
            conic_finder.Find(image_processing);
            
            const std::vector<Conic>& conics = conic_finder.Conics();
            std::vector<int> ellipse_target_map;
            
            tracking_good[iI] = target.FindTarget(image_processing, conic_finder.Conics(), ellipse_target_map);
            
            if(tracking_good[iI]) {
                // Generate map and point structures
                std::vector<Eigen::Vector2d> ellipses;
                for( size_t i=0; i < conics.size(); ++i ) {
                    ellipses.push_back(conics[i].center);
                }
                                    
                // find camera pose given intrinsics
                PosePnPRansac(
                    calibrator.GetCamera(iI).camera, ellipses, target.Circles3D(),
                    ellipse_target_map,
                    0, 0, &T_hw[iI]
                );
                                
                if(calib_frame >= 0) {
                    if(iI==0) {
                        calibrator.GetFrame(calib_frame) = T_hw[iI];
                    }
                    
                    for(size_t p=0; p < ellipses.size(); ++p) {
                        const Eigen::Vector2d pc = ellipses[p];
                        const Eigen::Vector2i pg = target.Map()[p].pg;
                        
                        const Eigen::Vector2i pgz = pg + grid_center;
                        if( 0<= pgz(0) && pgz(0) < grid_size(0) &&  0<= pgz(1) && pgz(1) < grid_size(1) )
                        {
                            const Eigen::Vector3d pg3d = grid_spacing * Eigen::Vector3d(pg(0), pg(1), 0);
                            calibrator.AddObservation(calib_frame, calib_cams[iI], pg3d, pc );
                        }
                    }
                }
            }

            if(container[iI].IsShown()) {
                container[iI].ActivateScissorAndClear();                
                glColor3f(1,1,1);
                
                // Display camera image
                if(!disp_thresh) {
                    tex.Upload(image_processing.Img(),GL_LUMINANCE,GL_UNSIGNED_BYTE);
                    tex.RenderToViewportFlipY();
                }else{
                    tex.Upload(image_processing.ImgThresh(),GL_LUMINANCE,GL_UNSIGNED_BYTE);
                    tex.RenderToViewportFlipY();
                }
    
                // Setup orthographic pixel drawing
                glMatrixMode(GL_PROJECTION);
                glLoadIdentity();
                glOrtho(-0.5,w-0.5,h-0.5,-0.5,0,1.0);
                glMatrixMode(GL_MODELVIEW);
                
//                    if(target.CenterId() >= 0) {
//                        //now draw a circle around the center cross
//                        glColor3f(1.0,1.0,1.0);
//                        const Conic& center_conic = conics[target.CenterId()];
//                        pangolin::glDrawCirclePerimeter(center_conic.center,center_conic.bbox.Width()/2.0);
//                    }
                
                    if(disp_lines) { 
                        for(std::list<LineGroup>::const_iterator i = target.LineGroups().begin(); i != target.LineGroups().end(); ++i)
                        {
                            //glColorBin(i->k,3);
                            //glColorHSV(i->theta*180/M_PI, 1.0, 1.0);
                            glColor3f(0.5,0.5,0.5);
    //                        glColorBin(di, target.LineGroups().size());
                            glBegin(GL_LINE_STRIP);
                            for(std::list<size_t>::const_iterator el = i->ops.begin(); el != i->ops.end(); ++el)
                            {
                                const Eigen::Vector2d p = conics[*el].center;
                                glVertex2d(p(0), p(1));
                            }
                            glEnd();
                        }            
                    }

                    if(disp_cross) {          
                        for( size_t i=0; i < conics.size(); ++i ) {   
                            const Eigen::Vector2d pc = conics[i].center;
                            const Eigen::Vector2i pg = tracking_good[iI] ? target.Map()[i].pg : Eigen::Vector2i(0,0);
                            
                            const Eigen::Vector2i pgz = pg + grid_center;
                            if( 0<= pgz(0) && pgz(0) < grid_size(0) &&  0<= pgz(1) && pgz(1) < grid_size(1) )
                            {
//                                    glColorBin(pgz(1)*grid_size(0)+pgz(0), grid_size(0)*grid_size(1));
                                glColorBin( target.Map()[i].value, 2);
                                glDrawCross(pc, conics[i].bbox.Width()*0.75 );
                            }
                        }
                    }
                    
                    if(disp_bbox) {
                        for( size_t i=0; i < conics.size(); ++i ) {   
                            const Eigen::Vector2i pg = tracking_good[iI] ? target.Map()[i].pg : Eigen::Vector2i(0,0);                    
                            const Eigen::Vector2i pgz = pg + grid_center;
                            if( 0<= pgz(0) && pgz(0) < grid_size(0) &&  0<= pgz(1) && pgz(1) < grid_size(1) )
                            {
                                glColorBin(pgz(1)*grid_size(0)+pgz(0), grid_size(0)*grid_size(1));
                                glDrawRectangle(conics[i].bbox);
                            }
                        }
                    }
            }
        }
        
        if(v3D.IsShown()) {
            v3D.ActivateScissorAndClear(stacks);
            
            calibu::glDrawTarget(target, Eigen::Vector2d(0,0), 1.0, 0.8, 1.0);
                        
            for(size_t c=0; c< calibrator.NumCameras(); ++c) {
                const Eigen::Matrix3d Kinv = calibrator.GetCamera(c).camera.Kinv();
                
                const CameraAndPose<Fov> cap = calibrator.GetCamera(c);
                const Sophus::SE3d T_ck = cap.T_ck;
    
                // Draw keyframes
                glColorBin(c, 2, 0.2);
                for(size_t k=0; k< calibrator.NumFrames(); ++k) {
                    glDrawAxis((T_ck * calibrator.GetFrame(k)).inverse().matrix(), 0.01);
                }
                
                // Draw current camera
                if(tracking_good[c]) {
                    glColorBin(c, 2, 0.5);
                    glDrawFrustrum(Kinv,w,h,T_hw[c].inverse().matrix(),0.01);
                }
            }
        }
                 
        disp_mse = calibrator.MeanSquareError();
        disp_frame = frame;
        
        // Process window events via GLUT
        pangolin::FinishGlutFrame();    
    }
    
    calibrator.Stop();
}

