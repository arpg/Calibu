#include <pangolin/pangolin.h>
#include <pangolin/video.h>

#include <fiducials/pose/Tracker.h>
#include <fiducials/target/TargetRandomDot.h>
#include <fiducials/gl/Drawing.h>
#include <fiducials/utils/Utils.h>
#include <fiducials/cam/CameraModel.h>

using namespace std;
using namespace pangolin;
using namespace Eigen;
using namespace fiducials;

const int PANEL_WIDTH = 200;

int main( int argc, char* argv[] )
{
    if(argc != 2) {
        std::cout << "Usage:" << std::endl;
        std::cout << "\t" << argv[0] << " video_uri" << std::endl;
        std::cout << "\t e.g. " << argv[0] << " files:[fmt=GRAY8,read_ahead=50,fps=1]//test_images/image_%05d.png" << std::endl;
        return -1;
    }
    
    const std::string video_uri = argv[1];
    
    // Setup Video
    VideoInput video(video_uri);
    
    if(video.Streams().size() < 1 || video.Streams()[0].PixFormat().channels != 1)
        throw pangolin::VideoException("Unsupported image format");
    
    // Allocate stream buffer and vector of images (that will point into buffer)
    unsigned char image_buffer[video.SizeBytes()];
    std::vector<pangolin::Image<unsigned char> > images;    
    
    const unsigned w = video.Width();
    const unsigned h = video.Height();
    
    // Setup Tracker and associated target
    TargetRandomDot target;  
    if(!target.LoadEPS("target.eps", 72/0.0254)) {
        Vector2d target_size_in_meters = Vector2d(11, 8.5)*0.0254;
        double radius = 0.0075;//target_size_in_meters[0]/40;
        target.GenerateRandom(60, radius, 3*radius, radius, target_size_in_meters);
        target.SaveEPS("target.eps", 72/0.0254);
        target.SaveRotatedEPS("target_to_print.eps", 72/0.0254);
    }
    
    Tracker tracker(target,w,h);  
    
    // Create Glut window
    pangolin::CreateGlutWindowAndBind("Main",2*w+PANEL_WIDTH,h);
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    
    // Pangolin 3D Render state
    pangolin::OpenGlRenderState s_cam;
    s_cam.SetProjectionMatrix(ProjectionMatrixRDF_TopLeft(640,480,420,420,320,240,0.01,1E6));
    s_cam.SetModelViewMatrix(Sophus::SE3d(Sophus::SO3d(),Vector3d(-target.Size()[0]/2,-target.Size()[1]/2,0.5) ).matrix());
    pangolin::Handler3D handler(s_cam);
    
    // Create viewport for video with fixed aspect
    pangolin::CreatePanel("ui").SetBounds(1.0,0.0,0,Attach::Pix(PANEL_WIDTH));
    View& vVideo = pangolin::Display("Video").SetAspect((float)w/h);
    View& v3D    = pangolin::Display("3D").SetAspect((float)w/h).SetHandler(&handler);
    
    Display("Container")
            .SetBounds(1.0,0.0,Attach::Pix(PANEL_WIDTH),1.0,false)
            .SetLayout(LayoutEqual)
            .AddDisplay(vVideo)
            .AddDisplay(v3D);
    
    // OpenGl Texture for video frame
    GlTexture tex(w,h,GL_LUMINANCE8);
        
    // Camera parameters
    CameraModel<Pinhole> cam(w, h, Eigen::Vector4d(525 , 525, w/2.0, h/2.0) );
    
    // Variables
    Var<bool> step("ui.step", false, false);
    Var<bool> run("ui.run", false, true);
    
    Var<bool> disp_thresh("ui.Display Thresh",false);
    Var<bool> lock_to_cam("ui.AR",false);
    
    bool tracking_good = false;  
    
    for(int frame=0; !pangolin::ShouldQuit(); ++frame)
    {
        bool go = frame==0 || run || Pushed(step);
        
        if(go) {
            if( video.Grab(image_buffer, images, true, true) ) {
                tracking_good = tracker.ProcessFrame(cam, images[0].ptr, images[0].pitch );
            }else{
                run = false;
            }
        }
        
        Viewport::DisableScissor();
        glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);    
        
        // Display Live Image
        glColor3f(1,1,1);
        vVideo.ActivateScissorAndClear();
        
        if(!disp_thresh) {
            tex.Upload(tracker.Images().Img(),GL_LUMINANCE,GL_UNSIGNED_BYTE);
            tex.RenderToViewportFlipY();
        }else{
            tex.Upload(tracker.Images().ImgThresh(),GL_LUMINANCE,GL_UNSIGNED_BYTE);
            tex.RenderToViewportFlipY();
        }
        
        // Display detected ellipses
        glOrtho(-0.5,w-0.5,h-0.5,-0.5,0,1.0);
        for( size_t i=0; i<tracker.GetConicFinder().Conics().size(); ++i ) {
            glColorBin(tracker.ConicsTargetMap()[i], target.Circles3D().size());
            DrawCross(tracker.GetConicFinder().Conics()[i].center,2);
        }
        
        if(lock_to_cam) {
            s_cam.SetModelViewMatrix(tracker.PoseT_gw().matrix());
        }        
        
        // Display 3D Vis
        glEnable(GL_DEPTH_TEST);
        v3D.ActivateScissorAndClear(s_cam);
        
        glDepthFunc(GL_LEQUAL);
        glDrawAxis(0.3);
        DrawTarget(target,Vector2d(0,0),1,0.2,0.2);
        
        if( tracking_good )
        {
            // Draw Camera
            glColor3f(1,0,0);
            DrawFrustrum(cam.Kinv(),w,h,tracker.PoseT_gw().inverse(),0.05);
        }
        
        // Process window events via GLUT
        pangolin::FinishGlutFrame();
    }
    
    return 0;
}
