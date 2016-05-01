/*
   Example demonstrating camera model reading and writing.
*/

#include <calibu/Calibu.h>
#include <glog/logging.h>

using namespace calibu;


void Test1()
{
  // Tests the rig read/write methods.
  std::shared_ptr<Rig<double>> rig = calibu::ReadXmlRig("cameras_in.xml");
  for(size_t ii=0; ii< rig->cameras_.size(); ++ii) {
    std::shared_ptr<CameraInterface<double>> cam = rig->cameras_[ii];
    cam->PrintInfo();
    std::cout << "    Params       = \n" << cam->K().transpose() << std::endl;
    std::cout << "    T_wc         = \n" << cam->Pose().matrix3x4() << std::endl;
  }

  WriteXmlRig("cameras_out.xml", rig);
}

void Test2()
{
  // Tests the camera projection methods.
  std::shared_ptr<Rig<double>> rig = ReadXmlRig("cameras_in.xml");
  for( size_t ii=0; ii < rig->cameras_.size(); ++ii) {
    std::shared_ptr<CameraInterface<double>> cam = rig->cameras_[ii];

    Eigen::Vector2d p;
    Eigen::Vector3d P;
    double d = 10;
    P << 1,1,d;

    // project and lift into the z=1 plane
    p = cam->Project( P );

    // unproject form the image back into 3D
    Eigen::Vector3d P2;
    P2 = d*cam->Unproject( p );

    std::cout << "These vectors should match: \n" <<
                 P << "\n" <<
                 P2 << std::endl;
  }
}

int main( int argc, char* argv[] )
{
  std::cout << "Running Test1(). \n" << std::endl;
  Test1();
  std::cout << "Running Test2(). \n" << std::endl;
  Test2();
  return 0;
}
