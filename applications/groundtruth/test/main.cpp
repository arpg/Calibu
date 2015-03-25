#include <Eigen/Eigen>
#include <SceneGraph/SceneGraph.h>
#include <pangolin/pangolin.h>
#include <HAL/Camera/CameraDevice.h>
#include <HAL/Utils/GetPot>
#include <calibu/Calibu.h>
#include <PbMsgs/Logger.h>
#include "../math.h"
#include "codes.h"

class GLTag : public SceneGraph::GLObject {

public:
  Eigen::Vector3d tr, tl, br, bl;
  Eigen::Vector3d tr_wb, tl_wb, br_wb, bl_wb;
  unsigned long long data;
  bool px[36];
  bool has_data;
  float color_high;
  float color_low;
  float wb;

  GLTag(){
    has_data = false;
    color_high = 1.0f;
    color_low = 0.0f;
  }

  GLTag( Eigen::Vector3d atr, Eigen::Vector3d atl, Eigen::Vector3d abr, Eigen::Vector3d abl ) : tr(atr), tl(atl), br(abr),  bl(abl)
  {
    has_data = false;
    color_high = 1.0f;
    color_low = 0.0f;
  }

  void CreateData( void )
  {
    if (data == 0)
      return;
    has_data = true;
    unsigned long long value = data;
    bool t_reverse[36];
    int idx = 0;
    for (int count = 0; count < 9; count++) {
      unsigned long long temp = value - ((value >> 4) << 4);
      t_reverse[idx + 0] = temp & 1;
      t_reverse[idx + 1] = temp & 2;
      t_reverse[idx + 2] = temp & 4;
      t_reverse[idx + 3] = temp & 8;
      idx += 4;
      value >>= 4;
    }
    for (int count = 0; count < 36; count++){
      px[count] = t_reverse[35 - count];
    }

  }

  void add_white_border( void )
  {
    Eigen::Vector3d i, j, k;

    i = 0.5*((br - bl) + (tr - tl));
    i /= i.norm();
    j = 0.5*((tl - bl) + (tr - br));
    j /= j.norm();
    k = i.cross(j);

    k = k / k.norm();
    j = k.cross(i);

    tr_wb = tr + wb*i + wb*j;
    tl_wb = tl - wb*i + wb*j;
    br_wb = br + wb*i - wb*j;
    bl_wb = bl - wb*i - wb*j;
  }

  inline void DrawCanonicalObject()
  {
    glPushMatrix();
    glColor4f( 1.0, 1.0, 1.0, 1.0 );
    if (has_data) {
      Eigen::Vector3d dx, dy;
      dx = (tl - tr) / 8;
      dy = (tl - bl) / 8;
      glBegin(GL_QUADS);
      if (wb != 0) {
        glColor3f(color_high, color_high, color_high);

        glVertex3d(tl_wb(0), tl_wb(1), tl_wb(2));
        glVertex3d(tl(0), tl(1), tl(2));
        glVertex3d(bl(0), bl(1), bl(2));
        glVertex3d(bl_wb(0), bl_wb(1), bl_wb(2));

        glVertex3d(bl_wb(0), bl_wb(1), bl_wb(2));
        glVertex3d(bl(0), bl(1), bl(2));
        glVertex3d(br(0), br(1), br(2));
        glVertex3d(br_wb(0), br_wb(1), br_wb(2));

        glVertex3d(br_wb(0), br_wb(1), br_wb(2));
        glVertex3d(br(0), br(1), br(2));
        glVertex3d(tr(0), tr(1), tr(2));
        glVertex3d(tr_wb(0), tr_wb(1), tr_wb(2));

        glVertex3d(tr_wb(0), tr_wb(1), tr_wb(2));
        glVertex3d(tr(0), tr(1), tr(2));
        glVertex3d(tl(0), tl(1), tl(2));
        glVertex3d(tl_wb(0), tl_wb(1), tl_wb(2));
      }
      for (int jj = 0; jj < 8; jj++) {
        for (int ii = 0; ii < 8; ii++) {
          if ((ii == 0) || (ii == 7) || (jj == 0) || (jj == 7)) {
            glColor3f(color_low, color_low, color_low);
          }
          else {
            int i = ii - 1;
            int j = jj - 1;
            if (px[i + 6*j]) {
              glColor3f(color_high, color_high, color_high);
            } else {
              glColor3f(color_low, color_low, color_low);
            }
          }
          Eigen::Vector3d o, t, r, f;
          o = tl - ii*dx - jj*dy;
          t = tl - (ii + 1)*dx - jj*dy;
          r = tl - (ii + 1)*dx - (jj + 1)*dy;
          f = tl - (jj + 1)*dy - ii*dx;

          glVertex3d(o(0), o(1), o(2));
          glVertex3d(f(0), f(1), f(2));
          glVertex3d(r(0), r(1), r(2));
          glVertex3d(t(0), t(1), t(2));
        }
      }
      glEnd();
    }

    glPopMatrix();
  }

};


void load_poses( std::string filename, std::vector< Eigen::Vector6d > *poses )
{
  std::ifstream ifs( filename );
  std::string line;
  std::getline ( ifs, line );
  while( ifs.good() ){

    float x, y, z, r, p, q;
    sscanf( line.c_str(), "%f\t%f\t%f\t%f\t%f\t%f\n", &x, &y, &z, &r, &p, &q );

    Eigen::Vector6d pz;
    Eigen::Vector3d t, rt;
    Eigen::Matrix3d rot;
    // Robotics to vision and invert 'z'
    rot << 0, 1, 0,
        1, 0, 0,
        0, 0, -1;
    t << x, y, z;
    t = rot*t;
    rt << r, p, q;
    rt = _R2Cart(rot*_Cart2R(rt));

    pz << t(0), t(1), t(2), rt(0), rt(1), rt(2);

    poses->push_back(pz);
    std::getline ( ifs, line );
  }

}

void add_corner(GLTag& t, int corner, double x, double y, double z)
{
  Eigen::Vector3d pt(x, y, z);
  switch (corner) {
  case 0: t.tl = pt; break;
  case 1: t.bl = pt; break;
  case 2: t.br = pt; break;
  case 3: t.tr = pt; break;
  }
}

void load_tags( std::string filename, std::vector< GLTag > *tags )
{
  std::ifstream ifs( filename );
  std::string line;
  std::getline( ifs, line );
  while (ifs.good()) {
    int tid, id, corner;
    double x, y, z;
    GLTag t;
    sscanf(line.c_str(), "%d\t%f\%f\%f\n", &tid, &x, &y, &z);
    id = tid  / 100;
    t.color_high = 1;
    t.color_low = 0;
    t.data = codes[id];
    t.CreateData();
    corner = tid % 100;
    add_corner(t, corner, x, y, z);
    sscanf(line.c_str(), "%d\t%f\%f\%f\n", &tid, &x, &y, &z);
    corner = tid % 100;
    add_corner(t, corner, x, y, z);
    sscanf(line.c_str(), "%d\t%f\%f\%f\n", &tid, &x, &y, &z);
    corner = tid % 100;
    add_corner(t, corner, x, y, z);
    sscanf(line.c_str(), "%d\t%f\%f\%f\n", &tid, &x, &y, &z);
    corner = tid % 100;
    add_corner(t, corner, x, y, z);
    tags->push_back(t);
    std::getline( ifs, line );
  }
}


int main( int argc, char** argv )
{
  GetPot cl(argc, argv);

  if (!cl.search("-cam")) {
    fprintf(stderr, "Needs a camera file.\n");
    fflush(stderr);
    exit(1);
  }

  std::vector< Eigen::Vector6d > poses;
  if (!cl.search("-poses")) {
    fprintf(stderr, "Needs poses!\n");
    fflush(stderr);
    exit(1);
  }
  load_poses( cl.follow("", "-poses"), &poses );

  std::vector< GLTag > tags;
  if (!cl.search("-tags")) {
    fprintf(stderr, "Needs tags!\n");
    fflush(stderr);
    exit(1);
  }
  init_codes();
  load_tags(cl.follow("", "-tags"), &tags);

  calibu::CameraRig rig = calibu::ReadXmlRig(cl.follow("", "-cam"));
  Eigen::Matrix3d K = rig.cameras[0].camera.K();

  pangolin::CreateWindowAndBind("Visualizer");
  glewInit();
  SceneGraph::GLSceneGraph::ApplyPreferredGlSettings();
  glClearColor(0.0f,0.0f,0.0f,1.0f);


  glPixelStorei(GL_PACK_ALIGNMENT,1);
  glPixelStorei(GL_UNPACK_ALIGNMENT,1);
  glEnable(GL_CULL_FACE);

  pangolin::View& container = pangolin::DisplayBase();

  pangolin::View view3d;
  const double far = 1000;
  const double near = 1E-3;
  SceneGraph::GLSceneGraph scene;
  pangolin::OpenGlRenderState stacks3d(
        pangolin::ProjectionMatrix(640,480,420,420,320,240,near,far),
        pangolin::ModelViewLookAt(-10, 0, -5, 0, 0, 0, pangolin::AxisNegZ)
        );


  SceneGraph::GLSimCam simcam;
  Eigen::Matrix4d p;
  p << 1, 0, 0, 0,
      0, 1, 0, 0,
      0, 0, 1, 0,
      0, 0, 0, 0;
  simcam.Init( &scene, p, K, 640, 480, SceneGraph::eSimCamLuminance);

  pb::Logger& logger = pb::Logger::GetInstance();
  logger.LogToFile("", "synthetic");

  for ( GLTag g : tags ){
    scene.AddChild( &g );
  }

  unsigned char* image = (unsigned char *) malloc (640*480);
  for ( Eigen::Vector6d pose : poses ) {
//    simcam.SetPoseVision( _Cart2T(pose) );
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glColor4f(1.0f,1.0f,1.0f,1.0f);

    view3d.Activate(stacks3d);

    simcam.SetPoseRobot( _Cart2T(pose) );
    simcam.RenderToTexture();
    simcam.CaptureGrey( image );

    glColor4f(1, 1, 1, 1);

    pangolin::FinishFrame();

    pb::Msg msg;
    pb::CameraMsg cam_msg;
    pb::ImageMsg* image_msg = cam_msg.add_image();
    image_msg->set_width(640);
    image_msg->set_height(480);
    image_msg->set_format(pb::Format::PB_LUMINANCE);
    image_msg->set_type(pb::Type::PB_BYTE);
    image_msg->set_data(image, 640*480);
    msg.mutable_camera()->Swap( &cam_msg );
    logger.LogMessage( msg );
  }

  free(image);
  return 0;
}
