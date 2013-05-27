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
#include <Mvlpp/Mvl.h>
#include "GetPot"

using namespace std;

const char* sUsage = 
"USAGE: cmod <options> <files>\n"
"\n"
"Command line tool for manipulating camera models (both MVL and calibu)\n"
"\n"
"Options:\n"
"    --combine,-c <files>  Combine list of .xml camera models <files> into \n"
"                          single cameras.xml rig file.  This will upgrade MVL\n"
"                          camera models to calibu models if necessary.\n"
"    --info,-i    <files>  Print out human readable information about \n"
"                          camera models listed in <files>.\n"
"    --upgrade-mvl-to-calibu,-u <files>\n"
"                          Upgrade listed MVL models to calibu modles.\n"
"\n";

////////////////////////////////////////////////////////////////////////////
/// Convert mvl model to calibu model.
CameraModelAndPose MvlToCalibu( const mvl::CameraModel& mvlcam )
{
    CameraModelAndPose CamAndPose;
    switch( mvlcam.Type() ){
        case MVL_CAMERA_LINEAR: 
            CamAndPose.T_wc = Sophus::SE3d( mvlcam.GetPose() );
            Eigen::Matrix3d K = mvlcam.K();
            if( K(0,1) == 0 ){
                Eigen::Vector4d p; p << K(0,0), K(1,1), K(0,2), K(1,2);
                CameraModelT<Pinhole> cam( mvlcam.Width(), mvlcam.Height(), p );
                CamAndPose.camera = cam;
            }
            else{
                Eigen::Matrix<double,5,1> params;
                params << K(0,0), K(1,1), K(0,2), K(1,2), K(0,1);
                assert(0); // FIX ME
//                CamAndPose.camera = CameraModelT<Pinhole>(
//                        mvlcam.Width(), mvlcam.Height(), params);
            }


            cout << "calibu " << CamAndPose.camera.RDF() << endl;
            cout << "mvl    " << mvlcam.RDF() << endl;

            CamAndPose.camera.SetRDF( mvlcam.RDF() );
            
            cout << "calibu " << CamAndPose.camera.RDF() << endl;

            break;
    }
    return CamAndPose;
}

////////////////////////////////////////////////////////////////////////////
/// Backwards compatible camera model read function that automatically 
//  updates old MVL models to calibu models.
CameraModelAndPose ReadCameraModel( const std::string& sFile )
{
    // quick check if the file is an old MVL file:
    double pose[16]; 
    if( mvl_read_camera( sFile.c_str(), pose ) ){
        return MvlToCalibu( mvl::CameraModel(sFile) );
    } 
    // else treat it as a normal calibu model
    return calibu::ReadXmlCameraModelAndPose( sFile );
}

////////////////////////////////////////////////////////////////////////////
/// Function to convert multiple cameras into a calibu camera rig file.
int MakeRig( int argc, char** argv ) 
{
    GetPot cl( argc, argv );
    cl.search(2, "-c", "--combine-cameras");

    CameraRig rig;
    for( string s = cl.next(""); !s.empty(); s = cl.next("") ){
        CameraModelAndPose cam = ReadCameraModel( s );
        if( cam.camera.IsInitialised() ){
            rig.Add( cam );
        }
    }

    std::cout << "Wrote calibu camera rig to 'cameras.xml'\n";
    std::ofstream out( "cameras.xml" );
    calibu::WriteXmlRig( out, rig );

    return 0;
}

////////////////////////////////////////////////////////////////////////////
int UpgradeToMvl( int argc, char** argv ) 
{
    GetPot cl( argc, argv );
    cl.search( 2, "--upgrade-mvl-to-calibu", "-u" );

    for( string s = cl.next(""); !s.empty(); s = cl.next("") ){
        CameraModelAndPose cam = ReadCameraModel( s );
        if( cam.camera.IsInitialised() ){
            std::ofstream out( std::string("calibu-")+s );
            calibu::WriteXmlCameraModelAndPose( out, cam );

            cout << "calibu " << cam.camera.RDF() << endl;
            std::cout << "Wrote calibu model 'calibu-" << s << "'\n";
        }
        else{
            std::cout << "WARNING: Failed to parse '" << s << "'\n";
        }
    }

    return 0;
}

////////////////////////////////////////////////////////////////////////////
/// Function to convert multiple MVL cameras into a calibu camera rig file.
int PrintInfo( int argc, char** argv ) 
{
    GetPot cl( argc, argv );
    cl.search(2, "--info", "-i");

    CameraRig rig;
    for( string s = cl.next(""); !s.empty(); s = cl.next("") ){
        CameraModelAndPose cam = ReadCameraModel( s );
        if( cam.camera.IsInitialised() ){
            std::cout << "\nFile '" << s << "': ";
            cam.camera.PrintInfo();
        }
        else{
            std::cout << "WARNING: Failed to parse '" << s << "'\n";
        }
    }
    std::cout << "\n";

    return 0;
}

////////////////////////////////////////////////////////////////////////////
int main( int argc, char** argv )
{
    GetPot cl( argc, argv );

    // user wants us to make a camera rig
    if( cl.search(2, "-c", "--combine-cameras") ){
        return MakeRig( argc, argv );
    }

    // user wants us to make a camera rig
    if( cl.search(2, "-u", "--upgrade-mvl-to-calibu") ){
        return UpgradeToMvl( argc, argv );
    }

    // user wants camera model info
    if( cl.search(2, "--info", "-i") ){
        return PrintInfo( argc, argv );
    }

    puts( sUsage );

    return 0;
}

