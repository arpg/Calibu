#include <memory>

#include <pangolin/pangolin.h>
#include <pangolin/gldraw.h>

#include <sophus/se3.hpp>

#include <calibu/calib/Calibrator.h>
#include <calibu/image/ImageProcessing.h>
#include <calibu/target/TargetGridDot.h>
#include <calibu/gl/Drawing.h>
#include <calibu/pose/Pnp.h>
#include <calibu/conics/ConicFinder.h>

#include <CVars/CVar.h>

#include <HAL/Camera/CameraDevice.h>

#include "GetPot"

using namespace calibu;

const char* sUriInfo =
"Usage:"
"\tcalibgrid <options> video_uri\n"
"Options:\n"
"\t-output,-o <file>        Output XML file to write camera models to.\n"
"\t                         By default calibgrid will output to cameras.xml\n"
"\t-cameras,-c <file>       Input XML file to read starting intrinsics from.\n"
"\t-fix-intrinsics,-f       Fix camera intrinsics during optimisation.\n"
"\t-paused,-p               Start video paused.\n"
"\t-grid-preset <value>     Grid preset: 0 (small grid), 1 (large grid).\n"
"\t-grid-spacing <value>    Distance between circles in grid (m)\n"
"\t-grid-seed <value>       Random seed used when creating grid (=71)\n"
"\t-grid-rows <value>       Number of rows in the grid pattern.\n"
"\t-grid-cols <value>       Number of cols in the grid pattern.\n"
"\t-grid-large-rad <value>  Radius of large grid dot (m).\n"
"\t-grid-small-rad <value>  Radius of small grid dot (m).\n"
"\t-save-grid <file>        Save grid in an EPS file and exit.\n"

"e.g.:\n"
"\tcalibgrid-hal -c leftcam.xml -c rightcaml.xml video_uri\n"
"\tcalibgrid-hal -grid-preset 1 video_uri (large grid)\n\n"
"\tcalibgrid-hal -grid-preset 0 video_uri (default)\n\n"
"\tcalibgrid-hal -grid-spacing 0.03156 -grid-rows 36 -grid-cols 25 video_uri (large grid)\n"
"\tcalibgrid-hal -grid-spacing 0.01411 -grid-rows 10 -grid-cols 19 video_uri (default)\n"
"\tcalibgrid-hal -grid-preset 1 -save-grid target.eps\n\n"
"Video URI's take the following form:\n"
" scheme:[param1=value1,param2=value2,...]//device\n"
"\n"
"scheme = file | dc1394 | v4l | openni | convert | split | mjpeg\n"
"\n"
"file/files - read PVN file format (pangolin video) or other formats using ffmpeg\n"
" e.g. \"file:[realtime=1]///home/user/video/movie.pvn\"\n"
" e.g. \"file:[stream=1]///home/user/video/movie.avi\"\n"
" e.g. \"files:///home/user/sequence/foo%03d.jpeg\"\n"
"\n"
"dc1394 - capture video through a firewire camera\n"
" e.g. \"dc1394:[fmt=RGB24,size=640x480,fps=30,iso=400,dma=10]//0\"\n"
" e.g. \"dc1394:[fmt=FORMAT7_1,size=640x480,pos=2+2,iso=400,dma=10]//0\"\n"
"\n"
"v4l - capture video from a Video4Linux (USB) camera (normally YUVY422 format)\n"
" e.g. \"v4l:///dev/video0\"\n"
"\n"
"openni - capture video / depth from an OpenNI streaming device (Kinect / Xtrion etc)\n"
" e.g. \"openni://'\n"
" e.g. \"openni:[img1=rgb,img2=depth]//\"\n"
" e.g. \"openni:[img1=ir]//\"\n"
"\n"
"convert - use FFMPEG to convert between video pixel formats\n"
" e.g. \"convert:[fmt=RGB24]//v4l:///dev/video0\"\n"
" e.g. \"convert:[fmt=GRAY8]//v4l:///dev/video0\"\n"
"\n"
"mjpeg - capture from (possibly networked) motion jpeg stream using FFMPEG\n"
" e.g. \"mjpeg://http://127.0.0.1/?action=stream\"\n"
"\n"
"split - split a single stream video into a multi stream video based on Region of Interest\n"
" e.g. \"split:[roi1=0+0+640x480,roi2=640+0+640x480]//files:///home/user/sequence/foo%03d.jpeg\"\n"
" e.g. \" split:[roi1=0+0+640x480,roi2=640+0+640x480]//uvc://\"\n"
"\n"
"log - run google protobuf data log from HAL (see https://github.com/gwu-robotics):\n"
" e.g. \"log://~/Data/calib.log\"\n\n";

void saveGrid(const TargetGridDot& grid, const std::string& filename,
              double large, double small);
static inline Eigen::MatrixXi GWUSmallGrid();
static inline Eigen::MatrixXi GoogleLargeGrid();

enum GridPreset {
  GridPresetGWUSmall  = 0,  // 19x10 grid at GWU
  GridPresetGoogleLarge = 1,  // 25x36 from Google folks
};

int main(int argc, char** argv)
{
    ////////////////////////////////////////////////////////////////////
    // Create command line options. Check if we should print usage.

    GetPot cl(argc,argv);

    if(cl.search(3, "-help", "-h", "?") || argc < 2) {
        std::cout << sUriInfo << std::endl;
        return -1;
    }

    ////////////////////////////////////////////////////////////////////
    // Default configuration values

    // Default grid printed on US Letter
    int grid_preset = GridPresetGWUSmall;
    double grid_spacing;
    int grid_rows;
    int grid_cols;
    uint32_t grid_seed;
    double grid_large_rad;
    double grid_small_rad;
    std::string save_grid;

    // Use no input cameras by default
    std::vector<calibu::CameraAndPose > input_cameras;

    // Fix cameras intrinsic parameters during optimisation, changing
    // only their relative poses.
    bool fix_intrinsics = false;

    // Require user to start playing the video
    bool start_paused = false;

    // Output file for camera rig
    std::string output_filename = "cameras.xml";

    ////////////////////////////////////////////////////////////////////
    // Parse command line

    grid_preset = cl.follow(grid_preset, "-grid-preset");
    switch(grid_preset)
    {
      case GridPresetGWUSmall:
        grid_spacing = 0.254 / 18;  // meters
        grid_large_rad = 0.00423; // m
        grid_small_rad = 0.00283; // m
        grid_rows = 10; // grid dots
        grid_cols = 19; // grid dots
        grid_seed = 71;
        break;
      case GridPresetGoogleLarge:
        grid_spacing = 0.03156;  // meters
        grid_large_rad = 0.00889; // m
        grid_small_rad = 0.00635; // m
        grid_rows = 36; // grid dots
        grid_cols = 25; // grid dots
        grid_seed = 71;
        break;
    }

    grid_spacing = cl.follow(grid_spacing,"-grid-spacing");
    grid_seed = cl.follow((int)grid_seed,"-grid-seed");
    grid_cols = cl.follow((int)grid_cols,"-grid-cols");
    grid_rows = cl.follow((int)grid_rows,"-grid-rows");
    grid_large_rad = cl.follow(grid_large_rad,"-grid-large-rad");
    grid_small_rad = cl.follow(grid_small_rad,"-grid-small-rad");
    fix_intrinsics = cl.search(2, "-fix-intrinsics", "-f");
    start_paused = cl.search(2, "-paused", "-p");
    output_filename = cl.follow(output_filename.c_str(), 2, "-output", "-o");
    save_grid = cl.follow("", "-save-grid");
    const Eigen::Vector2i grid_size(grid_cols, grid_rows);

    ////////////////////////////////////////////////////////////////////
    // Setup Grid pattern

    ConicFinder conic_finder;
    conic_finder.Params().conic_min_area = 4.0;
    conic_finder.Params().conic_min_density = 0.6;
    conic_finder.Params().conic_min_aspect = 0.2;

    std::unique_ptr<TargetGridDot> target;
    if(grid_preset == GridPresetGoogleLarge)
        target.reset(new TargetGridDot(grid_spacing, GoogleLargeGrid()));
    else if(grid_preset == GridPresetGWUSmall)
        target.reset(new TargetGridDot(grid_spacing, GWUSmallGrid()));
    else
        target.reset(new TargetGridDot(grid_spacing, grid_size, grid_seed));

    // Save grid and exit if required
    if(!save_grid.empty()) {
        saveGrid(*target, save_grid, grid_large_rad, grid_small_rad);
        return 0;
    }

    ////////////////////////////////////////////////////////////////////
    // Setup Video Source

    // Last argument or parameter - Video URI
    std::string cam_param = cl.follow("", 2, "-video_url", "-cam");
    std::string uri = cam_param.empty() ? argv[argc-1] : cam_param;

    hal::Camera  cam;
    try {
      cam = hal::Camera( uri );
    } catch (...) {
        if(!cam_param.empty())
          std::cerr << "Could not create camera from URI: " << uri
                    << std::endl;
        return -1;
    }
    if(cam.Empty()) return -1;

    // For the moment, assume all N cameras have same resolution
    const size_t N = cam.NumChannels();
    const size_t w = cam.Width();
    const size_t h = cam.Height();

    std::shared_ptr<pb::ImageArray> imageArray = pb::ImageArray::Create();
    cam.Capture(*imageArray);

    std::vector<cv::Mat> vImages;
    std::vector<uint64_t> vSerialNos;
    vImages.reserve(imageArray->Size());
    vSerialNos.reserve(imageArray->Size());
    for( int i = 0; i < imageArray->Size(); ++i ){
        pb::Image& image = *(*imageArray)[i];
        cv::Mat im = image.Mat();
        if( im.type() != CV_8UC1 ){
           std::cerr << "Input channels must be GRAY8 format. Use "
               "Convert:[fmt=MONO8]// video scheme." << std::endl;
        } else {
            vImages.emplace_back(im);
            vSerialNos.emplace_back(image.SerialNumber());
        }
    }

    // Load camera hints from command line
    cl.disable_loop();
    cl.reset_cursor();
    for(std::string filename = cl.follow("",2,"-cameras","-c");
        !filename.empty(); filename = cl.follow("",2,"-cameras","-c") ) {
        const size_t i = input_cameras.size();
        if(i < N) {
            if(filename == "fov") {
                CameraModelT<Fov> starting_cam(w, h);
                starting_cam.Params()  << 300, 300, w/2.0, h/2.0, 0.2;
                input_cameras.push_back( CameraAndPose(CameraModel(starting_cam), Sophus::SE3d() ) );
            }else if(filename == "poly2") {
                CameraModelT<Poly2> starting_cam(w, h);
                starting_cam.Params()  << 300, 300, w/2.0, h/2.0, 0.0, 0.0;
                input_cameras.push_back( CameraAndPose(CameraModel(starting_cam), Sophus::SE3d() ) );
            }else if(filename == "poly3" || filename =="poly") {
                CameraModelT<Poly3> starting_cam(w, h);
                starting_cam.Params()  << 300, 300, w/2.0, h/2.0, 0.0, 0.0, 0.0;
                input_cameras.push_back( CameraAndPose(CameraModel(starting_cam), Sophus::SE3d() ) );
            }else if(filename == "kb4") {
                CameraModelT<ProjectionKannalaBrandt> starting_cam(w, h);
                starting_cam.Params()  << 300, 300, w/2.0, h/2.0, 0.0, 0.0, 0.0, 0.0;
                input_cameras.push_back( CameraAndPose(CameraModel(starting_cam), Sophus::SE3d() ) );
            }else{
                const CameraRig rig = ReadXmlRig(filename);
                for(const CameraModelAndTransform& cop : rig.cameras ) {
                    input_cameras.push_back( CameraAndPose(cop.camera, cop.T_wc.inverse()) );
                }
            }
        }else{
            throw std::runtime_error("Too many camera files provided.");
        }
    }


    if(input_cameras.size() > 0 && input_cameras.size() != N) {
        std::cerr << "Number of cameras specified in files does not match video source" << std::endl;
        return -1;
    }

    ////////////////////////////////////////////////////////////////////
    // Setup image processing pipeline

    ImageProcessing image_processing(w,h);
    image_processing.Params().black_on_white = true;
    image_processing.Params().at_threshold = 0.9;
    image_processing.Params().at_window_ratio = 30.0;

    CVarUtils::AttachCVar("proc.adaptive.threshold", &image_processing.Params().at_threshold);
    CVarUtils::AttachCVar("proc.adaptive.window_ratio", &image_processing.Params().at_window_ratio);
    CVarUtils::AttachCVar("proc.black_on_white", &image_processing.Params().black_on_white);

    ////////////////////////////////////////////////////////////////////
    // Initialize Calibration object and tracking params

    Calibrator calibrator;
    calibrator.FixCameraIntrinsics(fix_intrinsics);

    int calib_cams[N];
    bool tracking_good[N];
    std::vector<Sophus::SE3d> T_hw;
    T_hw.resize(N);

    for(size_t i=0; i<N; ++i) {
        const int w_i = cam.Width();
        const int h_i = cam.Height();
        if(i < input_cameras.size() ) {
            input_cameras[i].camera.SetSerialNumber(vSerialNos[i]);
            calib_cams[i] = calibrator.AddCamera(
                        input_cameras[i].camera, input_cameras[i].T_ck
                        );
        }else{
            // Generic starting set of parameters.
            CameraModelT<Fov> starting_cam(w_i, h_i);
            starting_cam.Params()  << 300, 300, w_i/2.0, h_i/2.0, 0.2;
            starting_cam.SetSerialNumber(vSerialNos[i]);

            calib_cams[i] = calibrator.AddCamera(
                        CameraModel(starting_cam),
                        Sophus::SE3d()
                        );
        }
    }

    ////////////////////////////////////////////////////////////////////
    // Setup GUI

    const int PANEL_WIDTH = 150;
    pangolin::CreateWindowAndBind("Main",(N+1)*w/2.0+PANEL_WIDTH,h/2.0);

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

    ////////////////////////////////////////////////////////////////////
    // Display Variables

    pangolin::Var<bool> run("ui.Play video", !start_paused, true);

    pangolin::Var<double> disp_mse("ui.MSE");
    pangolin::Var<int> disp_frame("ui.frame");

    pangolin::Var<bool> add("ui.Add Frames", true, true);

    pangolin::Var<bool> disp_thresh("ui.Display Thresh",false);
    pangolin::Var<bool> disp_lines("ui.Display Lines",true);
    pangolin::Var<bool> disp_cross("ui.Display crosses",true);
    pangolin::Var<bool> disp_bbox("ui.Display bbox",true);

    ////////////////////////////////////////////////////////////////////
    // Key shortcuts

    // 1,2,3,... keys hide and show viewports
    for(size_t i=0; i<container.NumChildren(); ++i) {
        pangolin::RegisterKeyPressCallback('1'+i, [&container,i](){container[i].ToggleShow();} );
    }

    pangolin::RegisterKeyPressCallback('[', [&](){calibrator.Start();} );
    pangolin::RegisterKeyPressCallback(']', [&](){calibrator.Stop();} );

    bool step = false;
    pangolin::RegisterKeyPressCallback(pangolin::PANGO_SPECIAL+ pangolin::PANGO_KEY_RIGHT, [&](){step = true;} );
    pangolin::RegisterKeyPressCallback(' ', [&](){run = !run;} );

    pangolin::RegisterKeyPressCallback('r', [&](){calibrator.PrintResults();} );
    pangolin::RegisterKeyPressCallback('q', &pangolin::Quit);

    ////////////////////////////////////////////////////////////////////
    // Main event loop

    for(int frame=0; !pangolin::ShouldQuit();){
        const bool go = (frame==0) || run || pangolin::Pushed(step);

        int calib_frame = -1;

        if( go ) {
            if( cam.Capture( vImages ) ){
                if(add) {
                    calib_frame = calibrator.AddFrame(Sophus::SE3d(Sophus::SO3d(), Eigen::Vector3d(0,0,1000)) );
                }
                ++frame;
            }else{
                run = false;
            }
        }

        glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

        for(size_t iI = 0; iI < N; ++iI)
        {
            if (vImages.size() != N) break;

            image_processing.Process( vImages[iI].data, vImages[iI].cols, vImages[iI].rows, vImages[iI].cols );
            conic_finder.Find( image_processing );

            const std::vector<Conic, Eigen::aligned_allocator<Conic> >& conics =
                conic_finder.Conics();
            std::vector<int> ellipse_target_map;

            tracking_good[iI] = target->FindTarget(
                        image_processing, conic_finder.Conics(),
                        ellipse_target_map
                        );

            if(tracking_good[iI]) {
                // Generate map and point structures
              std::vector<Eigen::Vector2d,
                          Eigen::aligned_allocator<Eigen::Vector2d> > ellipses;
                for( size_t i=0; i < conics.size(); ++i ) {
                    ellipses.push_back(conics[i].center);
                }

                // find camera pose given intrinsics
                PosePnPRansac(
                    calibrator.GetCamera(iI).camera, ellipses, target->Circles3D(),
                    ellipse_target_map,
                    0, 0, &T_hw[iI]
                );

                if(calib_frame >= 0) {
                    if(iI==0 || !tracking_good[0]) {
                        // Initialize pose of frame for least squares optimisation
                        calibrator.GetFrame(calib_frame) = T_hw[iI];
                    }

                    for(size_t p=0; p < ellipses.size(); ++p) {
                        const Eigen::Vector2d pc = ellipses[p];
                        const Eigen::Vector2i pg = target->Map()[p].pg;

                        if( 0<= pg(0) && pg(0) < grid_size(0) &&  0<= pg(1) && pg(1) < grid_size(1) )
                        {
                            const Eigen::Vector3d pg3d = grid_spacing * Eigen::Vector3d(pg(0), pg(1), 0);
                            // TODO: Add these correspondences in bulk to avoid
                            //       hitting mutex each time.
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

                if(disp_lines) {
                    for(std::list<LineGroup>::const_iterator i = target->LineGroups().begin(); i != target->LineGroups().end(); ++i)
                    {
                        glColor3f(0.5,0.5,0.5);
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
                        pangolin::glColorBin( target->Map()[i].value, 2);
                        pangolin::glDrawCross(pc, conics[i].bbox.Width()*0.75 );
                    }
                }

                if(disp_bbox) {
                    for( size_t i=0; i < conics.size(); ++i ) {
                        const Eigen::Vector2i pg = tracking_good[iI] ? target->Map()[i].pg : Eigen::Vector2i(0,0);
                        if( 0<= pg(0) && pg(0) < grid_size(0) &&  0<= pg(1) && pg(1) < grid_size(1) ) {
                            pangolin::glColorBin(pg(1)*grid_size(0)+pg(0), grid_size(0)*grid_size(1));
                            glDrawRectPerimeter(conics[i].bbox);
                        }
                    }
                }
            }
        }

        if(v3D.IsShown()) {
            v3D.ActivateScissorAndClear(stacks);

            calibu::glDrawTarget(*target, Eigen::Vector2d(0,0), 1.0, 0.8, 1.0);

            for(size_t c=0; c< calibrator.NumCameras(); ++c) {
                const Eigen::Matrix3d Kinv = calibrator.GetCamera(c).camera.Kinv();

                const CameraAndPose cap = calibrator.GetCamera(c);
                const Sophus::SE3d T_ck = cap.T_ck;

                // Draw keyframes
                pangolin::glColorBin(c, 2, 0.2);
                for(size_t k=0; k< calibrator.NumFrames(); ++k) {
                    pangolin::glDrawAxis((T_ck * calibrator.GetFrame(k)).inverse().matrix(), 0.01);
                }

                // Draw current camera
                if(tracking_good[c]) {
                    pangolin::glColorBin(c, 2, 0.5);
                    pangolin::glDrawFrustrum(Kinv,w,h,T_hw[c].inverse().matrix(),0.05);
                }
            }
        }

        disp_mse = calibrator.MeanSquareError();
        disp_frame = frame;

        // Process window events via GLUT
        pangolin::FinishFrame();
    }

    calibrator.Stop();
    calibrator.PrintResults();
    calibrator.WriteCameraModels(output_filename);

}

void saveGrid(const TargetGridDot& grid, const std::string& filename,
              double large, double small) {
  const double pts_per_unit = 72. / 2.54 * 100.; // points per meter
  const Eigen::Vector2d offset(0,0);
  grid.SaveEPS(filename, offset, small, large, pts_per_unit);
  std::cout << "File " << filename << " saved" << std::endl;
}

inline Eigen::MatrixXi GWUSmallGrid() {
  Eigen::MatrixXi m(10, 19);
  m <<
      0, 1, 0, 0, 1, 1, 0, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 0,
      1, 0, 1, 1, 0, 0, 0, 1, 1, 0, 1, 1, 0, 0, 0, 1, 1, 1, 1,
      0, 0, 0, 0, 0, 0, 1, 1, 0, 1, 1, 1, 1, 0, 0, 0, 0, 1, 0,
      1, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0,
      0, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 1, 1, 0, 1, 0, 0, 0, 1,
      0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 0, 1, 1, 0, 0,
      1, 1, 0, 1, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1,
      0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 1, 1, 1, 1, 0,
      0, 1, 0, 1, 0, 0, 1, 0, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 0,
      0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 1, 0, 1, 1, 1, 1, 0, 1, 0;
  return m;
}

// Large google pattern
inline Eigen::MatrixXi GoogleLargeGrid() {
  Eigen::MatrixXi m(36, 25);
  m <<
      0, 1, 0, 0, 1, 1, 0, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0,
      0, 1, 1, 0, 1, 1, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1, 1, 1,
      1, 0, 0, 0, 0, 1, 0, 1, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 0, 1, 0,
      0, 0, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 1, 1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0,
      1, 1, 1, 0, 0, 0, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 1, 1, 0, 0, 1, 0, 0,
      0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 1, 1, 1,
      1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 0, 0, 0, 0, 1,
      0, 1, 0, 1, 0, 1, 1, 0, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 0, 0,
      1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 1, 0, 0, 0, 1, 1, 0, 1, 0, 1, 0,
      1, 1, 0, 0, 0, 0, 1, 1, 0, 1, 0, 0, 0, 1, 0, 0, 1, 1, 1, 1, 0, 1, 1, 0, 1,
      0, 0, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 0, 0, 1, 1, 1, 1, 0, 0, 1, 1, 0, 1,
      1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1, 0, 0, 0, 0, 0, 1,
      0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 1, 0, 0, 0, 1, 1, 0, 1, 1, 0, 0, 1, 1, 0, 0,
      1, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 0, 0,
      0, 0, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 0, 0, 1, 1, 0, 0, 1, 1, 0, 1, 1, 1, 1,
      1, 1, 1, 0, 1, 0, 1, 0, 1, 1, 1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0,
      1, 1, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 1, 1, 1, 0, 0, 1, 1, 0, 0, 0, 1, 1,
      1, 0, 1, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 0, 0, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1,
      0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1,
      1, 0, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1,
      0, 0, 0, 0, 0, 1, 0, 1, 1, 1, 1, 0, 1, 1, 0, 1, 1, 1, 1, 1, 0, 0, 0, 1, 0,
      0, 1, 0, 0, 0, 0, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 0, 0, 1, 1, 0, 0, 0, 1, 1,
      0, 0, 1, 0, 0, 1, 0, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 1,
      1, 1, 0, 1, 0, 1, 1, 0, 0, 1, 1, 1, 0, 1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1,
      1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 1, 0, 1,
      0, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 0, 0, 0, 1, 1, 0, 1, 0, 1, 1, 0,
      1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 1, 1, 0, 1, 1, 0, 0, 0, 0, 0, 1, 0,
      1, 1, 0, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0,
      0, 0, 0, 1, 0, 0, 0, 0, 1, 1, 0, 1, 1, 1, 0, 0, 1, 0, 0, 1, 0, 1, 0, 1, 1,
      1, 1, 1, 1, 0, 1, 1, 1, 0, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 0, 1, 1, 1, 1, 0,
      1, 0, 1, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 1, 0, 1, 0, 0,
      0, 1, 1, 1, 0, 1, 0, 0, 0, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 0, 0, 1, 1, 0, 1,
      1, 1, 1, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 0, 1,
      1, 1, 0, 1, 0, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 1, 1, 0, 1, 1, 1, 0,
      0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 1, 1, 1, 0, 0, 0, 1, 0, 1, 1, 1, 0, 0, 0, 0,
      0, 0, 1, 1, 0, 0, 1, 0, 1, 0, 1, 1, 0, 1, 1, 1, 0, 0, 1, 1, 1, 1, 0, 0, 0;
  return m;
}
