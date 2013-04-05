#include <memory>

#include <pangolin/pangolin.h>
#include <Pangolin/gldraw.h>

#include <sophus/se3.hpp>

//#include "find_grid.h"
#include "calib.h"
#include <fiducials/gradient.h>
#include <fiducials/integral_image.h>

#include <fiducials/target_grid_dot.h>
#include <fiducials/conic_finder.h>
#include <fiducials/drawing.h>
#include <fiducials/pnp.h>

using namespace fiducials;

int main( int argc, char** argv)
{    
    if(argc < 2) {
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
            throw "Video channels must be GRAY8 format. Use Convert:// or fmt=GRAY8 option";
        }
    }
    
    // Make grid of 3d points
    const double grid_spacing = 0.02;
    const Eigen::Vector2i grid_size(19,10);
    const Eigen::Vector2i grid_center(9, 5);
    Calibrator<Fov> calibrator(grid_spacing);   
    
    // Setup GUI
    const int PANEL_WIDTH = 150;
    pangolin::CreateGlutWindowAndBind("Main",2*w+PANEL_WIDTH,2*h);
    
    // Make things look prettier...        
    glEnable(GL_LINE_SMOOTH);
    glHint( GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST );
    glHint( GL_LINE_SMOOTH_HINT, GL_NICEST );
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthFunc( GL_LEQUAL );
    glEnable( GL_DEPTH_TEST );    
    glLineWidth(1.7);

    // TODO: Fix this in pangolin so video recording doesn't assume this alignment.
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);        
    
    // Pangolin 3D Render state
    pangolin::OpenGlRenderState stacks;
    stacks.SetProjectionMatrix(pangolin::ProjectionMatrixRDF_TopLeft(640,480,420,420,320,240,0.01,1E6));
    stacks.SetModelViewMatrix(pangolin::ModelViewLookAtRDF(0,0,-10, 0,0,0, 0, -1, 0) );
    
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
            
    pangolin::Var<bool> step("ui.step", false, false);
    pangolin::Var<bool> run("ui.run", false, true);
    pangolin::Var<bool> reset("ui.reset", false, false);
    
    pangolin::Var<bool> disp_thresh("ui.Display Thresh",false);
    pangolin::Var<bool> disp_lines("ui.Display Lines",false);
                    
    int calib_cams[N];
    for(size_t i=0; i<N; ++i) {
        // Add relatively arbitrary starting camera params
        const pangolin::StreamInfo& si = video.Streams()[i];
        CameraModel<Fov> default_cam( si.Width(), si.Height() );
        default_cam.Params()  << 300, 300, w/2.0, h/2.0, 0.2;
        calib_cams[i] = calibrator.AddCamera(default_cam);
    }
    
    bool tracking_good[N];
    Sophus::SE3d T_hw[N];
        
    pangolin::RegisterKeyPressCallback('[', [&](){calibrator.Start();} );
    pangolin::RegisterKeyPressCallback(']', [&](){calibrator.Stop();} );
    
    ImageProcessing image_processing(w,h);
    image_processing.Params().at_threshold = 1.0;
    image_processing.Params().at_window_ratio = 3.6;
    image_processing.Params().black_on_white = false;
    
    ConicFinder conic_finder;
    conic_finder.Params().conic_min_area = 3.96;
    conic_finder.Params().conic_min_density = 0.208;
    
    TargetGridDot target(grid_spacing, grid_size, grid_center);
    
    for(int frame=0; !pangolin::ShouldQuit(); ++frame)
    {
        glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);    
     
        if(pangolin::Pushed(reset)) {
            video.Reset();
            calibrator.Clear();
            frame=0;
        }

        const bool go = (frame==0) || run || pangolin::Pushed(step);
        
        int calib_frame = -1;
        
        if( go ) {
            if( video.Grab(image_buffer, images, true, true) ) {
                calib_frame = calibrator.AddFrame(Sophus::SE3d(Sophus::SO3(), Eigen::Vector3d(0,0,1000)) );
            }else{
                run = false;
            }
        }  
                
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
                        const Eigen::Vector2i pg = target.Grid()[p];
                        
                        const Eigen::Vector2i pgz = pg + grid_center;
                        if( 0<= pgz(0) && pgz(0) < grid_size(0) &&  0<= pgz(1) && pgz(1) < grid_size(1) )
                        {
                            calibrator.AddObservation(calib_frame, calib_cams[iI], pg, pc );
                        }
                    }
                }
            }

            container[iI].ActivateScissorAndClear();                
            glColor3f(1,1,1);
            
            if(!disp_thresh) {
                tex.Upload(image_processing.Img(),GL_LUMINANCE,GL_UNSIGNED_BYTE);
                tex.RenderToViewportFlipY();
            }else{
                tex.Upload(image_processing.ImgThresh(),GL_LUMINANCE,GL_UNSIGNED_BYTE);
                tex.RenderToViewportFlipY();
            }

            // Display detected ellipses
            glMatrixMode(GL_PROJECTION);
            glLoadIdentity();
            glOrtho(-0.5,w-0.5,h-0.5,-0.5,0,1.0);
            glMatrixMode(GL_MODELVIEW);

            if(tracking_good[iI]) {
                //now draw a circle around the center cross
                glColor3f(1.0,1.0,1.0);
                const Conic& center_conic = conics[target.CenterId()];
                pangolin::glDrawCirclePerimeter(center_conic.center,center_conic.bbox.Width()/2.0);
            
                if(disp_lines) { 
                    for(std::list<LineGroup>::const_iterator i = target.LineGroups().begin(); i != target.LineGroups().end(); ++i)
                    {
                        glHsvColor(i->theta*180/M_PI, 1.0, 1.0);
                        glBegin(GL_LINE_STRIP);
                        for(std::list<size_t>::const_iterator el = i->ops.begin(); el != i->ops.end(); ++el)
                        {
                            const Eigen::Vector2d p = conics[*el].center;
                            glVertex2d(p(0), p(1));
                        }
                        glEnd();
                    }            
                }else{                        
                    for( size_t i=0; i < conics.size(); ++i ) {   
                        const Eigen::Vector2d pc = conics[i].center;
                        const Eigen::Vector2i pg = target.Grid()[i];
                        
                        const Eigen::Vector2i pgz = pg + grid_center;
                        if( 0<= pgz(0) && pgz(0) < grid_size(0) &&  0<= pgz(1) && pgz(1) < grid_size(1) )
                        {
                            glBinColor(std::abs(pg(0)), 20);
                            DrawCross(pc, 10 );
                        }
                    }
                }
            }
            
        }
        
        v3D.ActivateScissorAndClear(stacks);
        
        pangolin::glColorHSV(100, 0.2, 1.0);
        pangolin::glDraw_z0(1.0, 10);
        
        for(size_t c=0; c< calibrator.NumCameras(); ++c) {
            const Eigen::Matrix3d Kinv = calibrator.GetCamera(c).camera.Kinv();
            
            const CameraAndPose<Fov> cap = calibrator.GetCamera(c);
            const Sophus::SE3d T_ck = cap.T_ck;

            // Draw keyframes
            glBinColor(c, 2, 0.2);
            for(size_t k=0; k< calibrator.NumFrames(); ++k) {
                DrawFrustrum(Kinv,w,h,(T_ck * calibrator.GetFrame(k)).inverse(),0.01);
            }
            
            // Draw current camera
            if(tracking_good[c]) {
                glBinColor(c, 2, 0.5);
                DrawFrustrum(Kinv,w,h,T_hw[c].inverse(),0.05);
            }
        }
                    
        // Process window events via GLUT
        pangolin::FinishGlutFrame();    
    }
    
    calibrator.Stop();
}

