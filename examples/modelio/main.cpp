/* 
   Example demonstrating camera model reading and writing.
*/

#include <calibu/Calibu.h>

using namespace calibu;

void Test1()
{
    CameraRig rig = ReadXmlRig("cameras.xml");
    CameraModel cam = rig.cameras[0].camera;

    Eigen::Vector2d p;
    Eigen::Vector3d P;
    double d = 10;
    P << 0,0,d;
 
    // project and lift into the z=1 plane
    p = cam.ProjectMap( P );

    // unproject form the image back into 3D
    Eigen::Vector3d P2;
    P2 = d*cam.UnmapUnproject( p );
}

void Test2()
{
    CameraRigT<float> rig = ReadXmlRig("cameras.xml");
    
    for(size_t i=0; i< rig.cameras.size(); ++i) {
        CameraModelGeneric<float>& cam = rig.cameras[i].camera;
        cam.PrintInfo();
        std::cout << "    Params       = " << cam.GenericParams().transpose() << std::endl;
    }    
}

int main( int argc, char* argv[] )
{
    Test1();
    Test2();
    return 0;
}


