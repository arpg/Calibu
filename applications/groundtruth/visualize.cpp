using namespace std;

#include <iostream>
#include <cstring>
#include <vector>
#include <sys/time.h>
#include <calibu/Calibu.h>
#include <pangolin/pangolin.h>
#include <pangolin/timer.h>
#include <HAL/Camera/CameraDevice.h>
#include <HAL/Utils/GetPot>
#include <SceneGraph/SceneGraph.h>
#include <sophus/sophus.hpp>
#include <sophus/se3.hpp>
#include <pangolin/video.h>
#include <Eigen/Eigen>


class GLTag : public SceneGraph::GLObject {

public:
  Eigen::Vector3d tr, tl, br, bl;

  GLTag(){

  }

  GLTag( Eigen::Vector3d atr, Eigen::Vector3d atl, Eigen::Vector3d abr, Eigen::Vector3d abl ) : tr(atr), tl(atl), br(abr),  bl(abl)
  {

  }

  inline void DrawCanonicalObject()
  {
    glPushMatrix();
    glColor4f( 1.0, 1.0, 1.0, 1.0 );
    pangolin::glDrawLine(tl[0],tl[1],tl[2],tr[0],tr[1],tr[2]);
    pangolin::glDrawLine(bl[0],bl[1],bl[2],br[0],br[1],br[2]);
    pangolin::glDrawLine(tl[0],tl[1],tl[2],bl[0],bl[1],bl[2]);
    pangolin::glDrawLine(tr[0],tr[1],tr[2],br[0],br[1],br[2]);
    glPopMatrix();
  }

};

int main(int argc, char* argv[]) {


  // Load from file
  FILE* fptr = fopen(argv[1], "r");
  if (fptr == NULL) {
    return false;
  };

  std::vector< Eigen::Vector6d > camPoses;
  std::vector< Eigen::Vector6d > ceresPoses;
  std::vector< Eigen::Vector6d > tagPoses;
  std::vector< GLTag > glTags;
  std::vector< SceneGraph::GLAxis > glTagPoses;

  FILE* file;
  bool ceres = false;
  if (argc == 3) {
    ceres = true;
    file = fopen(argv[2], "r");
    int x, y, z, p, q, r;
    int count = 0;
    while (fscanf(file, "%f, %f, %f, %f, %f, %f", &x, &y, &z, &p, &q, &r) != EOF) {
      count++;
      Eigen::Vector6d pose;
      pose(0) = x;
      pose(1) = y;
      pose(2) = z;
      pose(3) = p;
      pose(4) = q;
      pose(5) = r;
      ceresPoses.push_back(pose);
    }
  }

  fprintf(stdout, "Read %d ceres poses\n", ceresPoses.size());

  int num_poses_, num_landmarks_, num_observations_, num_tags_;
  fscanf( fptr, "%d", &num_poses_ );
  fscanf( fptr, "%d", &num_landmarks_ );
  fscanf( fptr, "%d", &num_observations_ );
  fscanf( fptr, "%d", &num_tags_ );

  tagPoses.resize(num_poses_);
  glTags.resize(num_landmarks_ / 4);
  tagPoses.resize(num_tags_);
  glTagPoses.resize(num_tags_);
  camPoses.resize(num_poses_);

  for (int i = 0; i < num_observations_; ++i) {
    double u, v;
    int a, b;
    int n = fscanf( fptr, "%d, %d, %lf, %lf",
                    &a, &b, &u, &v );
  }
  // poses
  for (int i = 0; i < num_poses_; ++i) {
    double x,y,z,p,q,r;
    int n = fscanf( fptr, "%lf, %lf, %lf, %lf, %lf, %lf",
                    &x, &y , &z , &p , &q , &r );
    camPoses[i] << x, y, z, p, q, r;
  }

  // landmarks
  int num = 0;
  for (int i = 0; i < num_landmarks_; ++i) {
    double x,y,z;
    int uid;
    for (int count = 0; count < 4; count++) {
      int n = fscanf( fptr, "%d, %lf, %lf, %lf", &uid, &x, &y , &z );
      switch(uid % 100) {
        case 0 : glTags[num].tl << x, y, z; break;
        case 1 : glTags[num].bl << x, y, z; break;
        case 2 : glTags[num].br << x, y, z; break;
        case 3 : glTags[num].tr << x, y, z; break;
      }
      i++;
    }
    num++;
  }

  for (int i = 0 ; i < num_tags_; i++) {
    double x,y,z,p,q,r;
    int id;
    int n = fscanf( fptr, "%d, %lf, %lf, %lf, %lf, %lf, %lf",
                    &id, &x, &y , &z , &p , &q , &r );
    glTagPoses[i].SetPose(x, y, z, p, q, r);
    glTagPoses[i].SetAxisSize(0.5);
  }

  // Setup OpenGL Display (based on GLUT)
  pangolin::CreateWindowAndBind("Visualizer");
  glewInit();
  SceneGraph::GLSceneGraph::ApplyPreferredGlSettings();
  glClearColor(0.0f,0.0f,0.0f,1.0f);


  glPixelStorei(GL_PACK_ALIGNMENT,1);
  glPixelStorei(GL_UNPACK_ALIGNMENT,1);

  pangolin::View& container = pangolin::DisplayBase();

  pangolin::View view3d;
  const double far = 1000;
  const double near = 1E-3;
  SceneGraph::GLSceneGraph glGraph;
  SceneGraph::GLGrid grid;
  glGraph.AddChild( &grid );
  pangolin::OpenGlRenderState stacks3d(
        pangolin::ProjectionMatrix(640,480,420,420,320,240,near,far),
        pangolin::ModelViewLookAt(-10, 0, -5, 0, 0, 0, pangolin::AxisNegZ)
        );

  view3d.SetBounds(0, 1, 0, 1)
      .SetHandler(new SceneGraph::HandlerSceneGraph(glGraph,stacks3d, pangolin::AxisNegZ))
      .SetDrawFunction(SceneGraph::ActivateDrawFunctor(glGraph, stacks3d));

  container.AddDisplay(view3d);

  for (int i = 0; i < glTags.size(); i++) {
    glGraph.AddChild( &glTags[i] );
  }

  for (int i = 0; i < glTagPoses.size(); i++) {
    glGraph.AddChild( &glTagPoses[i] );
  }

  std::vector< SceneGraph::GLAxis > campose;
  campose.resize(num_poses_);
  for (int i = 0; i < num_poses_; i++)
  {
    campose[i].SetPose(camPoses[i]);
    campose[i].SetScale(0.1);
    glGraph.AddChild( &campose[i]);
  }

  std::vector< SceneGraph::GLAxis > cerescampose;
  cerescampose.resize(ceresPoses.size());
  for (int i = 0; i < ceresPoses.size(); i++)
  {
    cerescampose[i].SetPose(ceresPoses[i]);
    cerescampose[i].SetScale(0.1);
    glGraph.AddChild( &cerescampose[i]);
  }


  bool bRun = false;
  bool bStep = false;
  unsigned long nFrame=0;

  pangolin::RegisterKeyPressCallback(pangolin::PANGO_SPECIAL + pangolin::PANGO_KEY_RIGHT, [&bStep](){bStep=true;} );
  pangolin::RegisterKeyPressCallback(' ', [&](){bRun = !bRun;} );

  //    SceneGraph::GLWireSphere Sphere(1);
//  SceneGraph::GLAxis Sphere(1);
//  Sphere.SetPose(0, 0, 0, 0, 0, 0);
//  glGraph.AddChild(&Sphere);

//  SceneGraph::GLAxis coord(1);
//  Sphere.SetPose(0, 0, 0, 0, 0, 0);
//  glGraph.AddChild(&coord);

  for(; !pangolin::ShouldQuit(); nFrame++)
  {
    const bool bGo = bRun || pangolin::Pushed(bStep);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glColor4f(1.0f,1.0f,1.0f,1.0f);

    view3d.Activate(stacks3d);

    glColor4f(1.0, 0, 0, 1);
    for (int ii = 1; ii < camPoses.size(); ii++) {
      pangolin::glDrawLine(camPoses[ii - 1][0], camPoses[ii - 1][1], camPoses[ii - 1][2],
          camPoses[ii][0], camPoses[ii][1], camPoses[ii][2]);
    }
    glColor4f(0, 1.0, 0, 1);
    for (int ii = 1; ii < ceresPoses.size(); ii++) {
      pangolin::glDrawLine(ceresPoses[ii - 1][0], ceresPoses[ii - 1][1], ceresPoses[ii - 1][2],
          ceresPoses[ii][0], ceresPoses[ii][1], ceresPoses[ii][2]);
    }

    glColor4f(1, 1, 1, 1);
//            for (int ii = 0; ii < demo.detectionsL.size(); ii++ ) {
    //            Eigen::Vector3d p = demo.PoseFromId( demo.detectionsL[ii].id ).block<3,1>(0,3);

    //            Sophus::SE3d pose_cam = Sophus::SE3d(Robot2Vision(demo.print_detection( demo.detectionsL[ii] ))).inverse(); // Transform Tag to Camera
    ////                    (Sophus::SE3d(demo.PoseFromId( demo.detectionsL[ii].id ))*Sophus::SE3d(demo.print_detection( demo.detectionsL[ii] )).inverse());
    ////                    (Sophus::SE3d(demo.PoseFromId( demo.detectionsL[ii].id ))*Sophus::SE3d(demo.print_detection( demo.detectionsL[ii] )));
    //            pose_cam =  Sophus::SE3d(demo.PoseFromId( demo.detectionsL[ii].id )) * pose_cam;

    //            Eigen::Vector3d d = pose_cam.translation();
    //            Sphere.SetPose( pose_cam.matrix() );
    //            pangolin::glDrawLine(p(0), p(1), p(2), -d(0), d(1), d(2) );
    //        }

    pangolin::FinishFrame();
  }

  return 0;
}
