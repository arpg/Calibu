/* 
   Example demonstrating camera model reading and writing.
*/

#include <calibu/Calibu.h>

using namespace calibu;

void Test1()
{
    Rig rig = ReadXmlRig("cameras.xml");
    CameraInterface cam = rig.cameras_[0];

    Eigen::Vector2d p;
    Eigen::Vector3d P;
    double d = 10;
    P << 0,0,d;
 
    // project and lift into the z=1 plane
    p = cam.Project( P );

    // unproject form the image back into 3D
    Eigen::Vector3d P2;
    P2 = d*cam.Unproject( p );
}

void Test2()
{
    Rig<float> rig = ReadXmlRig("cameras.xml");
    
    for(size_t i=0; i< rig.cameras_.size(); ++i) {
        CameraInterface<float>& cam = rig.cameras_[i];
        // cam.PrintInfo();
        std::cout << "    Params       = " << cam.GetParams().transpose() << std::endl;
    }    
}

int main( int argc, char* argv[] )
{
    Test1();
    Test2();
    return 0;
}


