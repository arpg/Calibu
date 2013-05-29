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
calibu::CameraModelAndPose MvlToCalibu( const mvl::CameraModel& mvlcam )
{
    Eigen::Matrix3d K = mvlcam.K(); // doesn't always make sense
    calibu::CameraModelAndPose CamAndPose;

    switch( mvlcam.Type() ){
        case MVL_CAMERA_LINEAR: 
            if( K(0,1) == 0 ){
                Eigen::Vector4d p; p << K(0,0), K(1,1), K(0,2), K(1,2);
                calibu::CameraModelT<calibu::Pinhole> cam( mvlcam.Width(), mvlcam.Height(), p );
                CamAndPose.camera = cam;
            }
            else{
                Eigen::Matrix<double,5,1> params;
                params << K(0,0), K(1,1), K(0,2), K(1,2), K(0,1);
                assert(0); // FIX ME
//                CamAndPose.camera = CameraModelT<Pinhole>(
//                        mvlcam.Width(), mvlcam.Height(), params);
            }
            break;
        case MVL_CAMERA_LUT:
                Eigen::Vector4d p; p << K(0,0), K(1,1), K(0,2), K(1,2);
                CameraModelT<Pinhole> cam( mvlcam.Width(), mvlcam.Height(), p );
                CamAndPose.camera = cam;
            break;
    }

    CamAndPose.camera.SetName( mvlcam.GetModel()->name );
    CamAndPose.camera.SetSerialNumber( mvlcam.GetModel()->serialno );
    CamAndPose.camera.SetIndex( mvlcam.GetModel()->index );
    CamAndPose.camera.SetVersion( calibu::CAMRERA_MODEL_VERSION );
    CamAndPose.camera.SetRDF( mvlcam.RDF() );
    CamAndPose.T_wc = Sophus::SE3d( mvlcam.GetPose() );

    return CamAndPose;
}

////////////////////////////////////////////////////////////////////////////
/// Backwards compatible camera model read function that automatically 
//  updates old MVL models to calibu models.
calibu::CameraModelAndPose ReadCameraModel( const std::string& sFile )
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
/// Read the lookup table into sLut, with tags
bool ReadCameraModelLut( const std::string& sFile, std::string& sLut )
{
    double pose[16]; 
    mvl_camera_t* pCam = mvl_read_camera( sFile.c_str(), pose );
    if( !pCam || pCam->type != MVL_CAMERA_LUT ){
        return false;
    } 

    // get the lookup table element
    TiXmlDocument doc;
    sLut.clear();
    if( doc.LoadFile(sFile) ){
        TiXmlElement* pNode = doc.FirstChildElement("camera_model");
        pNode = pNode->FirstChildElement("lut");
        if( !pNode ){
            return false;
        }

        std::cout << "Converting lookup-tables...";

        std::string sRest = pNode->GetText(); 
        // every 6 terms insert a newline
        unsigned int cnt = 1, start = 0, end = 0;
        while( end < sRest.size() ){
            end = sRest.find_first_of(" ",start);
            std::string s = sRest.substr( start, end-start );
            sLut += sRest.substr( start, end-start );
            sLut += ( cnt++ % 6 == 0 ) ? "\n" : " ";
            start = end+1;
        }

        std::cout << "done\n";
    }
    sLut = std::string("    <lut>\n") + sLut + "\n    </lut>\n";

    return true;
}

////////////////////////////////////////////////////////////////////////////
/// Function to convert multiple cameras into a calibu camera rig file.
int MakeRig( int argc, char** argv ) 
{
    GetPot cl( argc, argv );
    cl.search(2, "-c", "--combine-cameras");

    std::string sLuts; // lookup tables, if present
    calibu::CameraRig rig;

    for( string s = cl.next(""); !s.empty(); s = cl.next("") ){
        calibu::CameraModelAndPose cam = ReadCameraModel( s );
        if( cam.camera.IsInitialised() ){
            rig.Add( cam );
        }
        std::string sLut;
        if( ReadCameraModelLut( s, sLut )){ // did the model have a lut?
           sLuts += sLut;
        }
    }

    std::cout << "Wrote calibu camera rig to 'cameras.xml'\n";
    std::ofstream out( "cameras.xml" );
    // calibu::WriteXmlRig( out, rig, sLuts );
    out << AttribOpen(NODE_RIG) << std::endl;    
    for(const CameraModelAndPose& cop : rig.cameras) {
        WriteXmlCameraModelAndPose( out, cop, 4 );
    }
    out << sLuts; // empty if no looktables present
    out << AttribClose(NODE_RIG) << std::endl;

    return 0;
}

////////////////////////////////////////////////////////////////////////////
int UpgradeToCalibu( int argc, char** argv ) 
{
    GetPot cl( argc, argv );
    cl.search( 2, "--upgrade-mvl-to-calibu", "-u" );

    for( string s = cl.next(""); !s.empty(); s = cl.next("") ){
        calibu::CameraModelAndPose cam = ReadCameraModel( s );
        if( cam.camera.IsInitialised() ){
            std::ofstream out( std::string("calibu-")+s );

            std::string sLut;
            ReadCameraModelLut( s, sLut );

            calibu::WriteXmlCameraModelAndPoseWithLut( out, sLut, cam );
 
            std::cout << "Wrote calibu model 'calibu-" << s << "'\n";
        }
        else{
            std::cout << "\nWARNING: Failed to parse '" << s << "'\n";
        }
    }

    return 0;
}

////////////////////////////////////////////////////////////////////////////
/// Function to print info
int PrintInfo( int argc, char** argv ) 
{
    GetPot cl( argc, argv );
    cl.search(2, "--info", "-i");

    for( string s = cl.next(""); !s.empty(); s = cl.next("") ){
        calibu::CameraModelAndPose cam = ReadCameraModel( s );
        if( cam.camera.IsInitialised() ){
            std::cout << "\nFile '" << s << "': ";
            cam.camera.PrintInfo();
        }
        else{
            std::cout << "\nWARNING: Failed to parse '" << s << "'\n";
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
        return UpgradeToCalibu( argc, argv );
    }

    // user wants camera model info
    if( cl.search(2, "-i", "--info") ){
        return PrintInfo( argc, argv );
    }

    puts( sUsage );

    return 0;
}

