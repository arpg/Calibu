#include <memory>

#include <calibu/Calibu.h>
#include <calibu/utils/Xml.h>

#include "MVL/CameraModel.h"
#include "GetPot"

using namespace std;
using namespace calibu;

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
std::shared_ptr<calibu::CameraInterface<double>> MvlToCalibu( const mvl::CameraModel& mvlcam )
{
    Eigen::Matrix3d K = mvlcam.K(); // doesn't always make sense
    std::shared_ptr<calibu::CameraInterface<double>> CamAndPose;

    switch( mvlcam.Type() ){
        case MVL_CAMERA_LINEAR:
            if( K(0,1) == 0 ){
              // fu, fv, sx, sy
                Eigen::Vector4d p; p << K(0,0), K(1,1), K(0,2), K(1,2);
                CamAndPose.reset(new calibu::LinearCamera<double>());
                CamAndPose->SetImageDimensions( mvlcam.Width(), mvlcam.Height());
                CamAndPose->SetParams(p);
                CamAndPose->SetType("calibu_fu_fv_u0_v0");
            }
            else{
                // Eigen::Matrix<double,5,1> params;
                // params << K(0,0), K(1,1), K(0,2), K(1,2), K(0,1);
                assert(0); // FIX ME
//                CamAndPose.camera = CameraModelT<Pinhole>(
//                        mvlcam.Width(), mvlcam.Height(), params);
            }
            break;
        case MVL_CAMERA_LUT:
                Eigen::Vector4d p; p << K(0,0), K(1,1), K(0,2), K(1,2);
                CamAndPose.reset(new calibu::LinearCamera<double>());
                CamAndPose->SetImageDimensions( mvlcam.Width(), mvlcam.Height());
                CamAndPose->SetParams(p);
                CamAndPose->SetType("calibu_fu_fv_u0_v0");
            break;
    }

    CamAndPose->SetName( mvlcam.GetModel()->name );
    CamAndPose->SetSerialNumber( mvlcam.GetModel()->serialno );
    CamAndPose->SetIndex( mvlcam.GetModel()->index );
    CamAndPose->SetVersion( mvlcam.GetModel()->version );
    CamAndPose->SetRDF( mvlcam.RDF().transpose() );
    CamAndPose->SetPose(Sophus::SE3d( mvlcam.GetPose() ));

    return CamAndPose;
}

////////////////////////////////////////////////////////////////////////////
/// Backwards compatible camera model read function that automatically
//  updates old MVL models to calibu models.
std::shared_ptr<calibu::CameraInterface<double>> ReadCamera( const std::string& sFile )
{
    // quick check if the file is an old MVL file:
    double pose[16];
    if( mvl_read_camera( sFile.c_str(), pose ) ){
        return MvlToCalibu( mvl::CameraModel(sFile) );
    }
    // else treat it as a normal calibu model
    return calibu::ReadXmlCameraAndTransform( sFile );
}

////////////////////////////////////////////////////////////////////////////
/// Read the lookup table into sLut, with tags
bool ReadCameraLut( const std::string& sFile, std::string& sLut )
{
    double pose[16];
    mvl_camera_t* pCam = mvl_read_camera( sFile.c_str(), pose );
    if( !pCam || pCam->type != MVL_CAMERA_LUT ){
        return false;
    }

    // get the lookup table element
    calibu::TiXmlDocument doc;
    sLut.clear();
    if( doc.LoadFile(sFile) ){
        calibu::TiXmlElement* pNode = doc.FirstChildElement("camera_model");
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
    std::shared_ptr<calibu::Rig<double>> rig(new calibu::Rig<double>());

    for( string s = cl.next(""); !s.empty(); s = cl.next("") ){
        std::shared_ptr<calibu::CameraInterface<double>> cam = ReadCamera( s );
        if( cam->IsInitialized() ){
            rig->AddCamera( cam );
        }
        std::string sLut;
        if( ReadCameraLut( s, sLut )){ // did the model have a lut?
           sLuts += sLut;
        }
    }

    std::cout << "Wrote calibu camera rig to 'cameras.xml'\n";
    std::ofstream out( "cameras.xml" );
    // calibu::WriteXmlRig( out, rig, sLuts );
    out << AttribOpen(NODE_RIG) << std::endl;
    for(const std::shared_ptr<CameraInterface<double>> cop : rig->cameras_) {
        WriteXmlCameraAndTransform( out, cop, 4 );
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
        std::shared_ptr<calibu::CameraInterface<double>> cam = ReadCamera( s );
        if( cam->IsInitialized() ){
            std::ofstream out( std::string("calibu-")+s );

            std::string sLut;
            ReadCameraLut( s, sLut );

            calibu::WriteXmlCameraAndTransformWithLut( out, sLut, cam );

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
        std::shared_ptr<calibu::CameraInterface<double>> cam = ReadCamera( s );
        if( cam->IsInitialized() ){
            std::cout << "\nFile '" << s << "': ";
            cam->PrintInfo();
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

    // read camera model, create lut
//    calibu::CameraModelAndTransform cp = ReadCameraModel( argv[1] );
//    if( !cp.camera.IsInitialized() ){
//        return -1;
//    }

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

