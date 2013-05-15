/* 
   Example demonstrating camera model reading and writing.
*/

#include <calibu/Calibu.h>

using namespace calibu;

int main( int argc, char* argv[] )
{
    CameraModel cam = ReadXmlCameraModel("camera.xml");

    Eigen::Vector2d p;
    Eigen::Vector3d P;
    double d = 10;
    P << 0,0,d;
 
    // project and lift into the z=1 plane
    p = cam.ProjectMap( P );

    // unproject form the image back into 3D
    Eigen::Vector3d P2;
    P2 = d*cam.UnmapUnproject( p );

//    cam.PrintInfo();

    return 0;
}


