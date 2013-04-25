/* 
   This file is part of the Calibu Project.
https://robotics.gwu.edu/git/calibu

Copyright (C) 2013 George Washington University
Gabe Sibley 

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
 */

#pragma once

#include <sys/stat.h>
#include <iostream>
#include <fstream>

#include <zlib.h>

#include <calibu/cam/CameraModel.h>
#include <calibu/utils/Xml.h>

//#include "CameraModelLut.h"


///////////////////////////////////////////////////////////////////////////////
namespace calibu
{

    ///////////////////////////////////////////////////////////////////////////////
    // TODO modernize this.
    // Allocate and read entire bu
    char *AllocReadIntoBuffer( const std::string& sFilename )
    {
        int nBufSize = 10000000;  // 10M
        int nReadSize = 100000;  // 100K
        char* pBuf = (char*)malloc( nBufSize );

        gzFile f = gzopen( sFilename.c_str(), "rb" );
        if( !f ){
            return NULL;
        }

        int nPos = 0;
        bool done = false;
        do{
            int cc = gzread( f, &pBuf[nPos], nReadSize );
            if( gzeof(f) ){
                done = true;
            }
            nPos += cc;
            if( nPos + nReadSize > nBufSize ){ // next read could go over buf size
                nBufSize *= 2;
                pBuf = (char*)realloc( pBuf, nBufSize );
                assert(pBuf);
            }
            pBuf[nPos] = 0;
        }
        while( !done );
        gzclose( f );
        return pBuf;
    }

    /*
    ///////////////////////////////////////////////////////////////////////////////
    /// Helper for reading lookup-tables from xml files.
    static bool _read_lut_point( char* buf, char** endptr, _Bi_Point2D_f_nob& pt );

    ///////////////////////////////////////////////////////////////////////////////
    /// Helper for writing lookup-tables to xml files.
    static void _write_lut_point( const _Bi_Point2D_f_nob& pt, char* buf );


    ////////////////////////////////////////////////////////////////////////////////
    /// Reads lookup table from file specified by filename in lut camera model
    static int _read_lut_from_file(
    const char* pLUTFileName, //< Input: LUT file name.
    int nWidth,               //< Input: image width.
    int nHeight,              //< Input: image height.
    Bi_Point2D_f_nob*** pLUT  //< Output: lookup table.
    );
     */

    ///////////////////////////////////////////////////////////////////////////////
    static bool IsFile( const std::string& sFile )
    {
        struct stat StatBuf;
        if( stat( sFile.c_str(), &StatBuf ) < 0 ){
            return false;
        }
        return S_ISREG( StatBuf.st_mode );
    }

    ///////////////////////////////////////////////////////////////////////////////
    static const char* CameraModelType( const std::string& sType )
    {
        if( sType.empty() ){
            fprintf( stderr, "ERROR: Unknown camera model type (no type attribute)" );
            return NULL;
        }
        if( sType == "MVL_CAMERA_WARPED" || sType == "calibu_poly" ) {
            return "calibu_poly"; // standard polynomial radial distortion model
        }
        else if( sType == "MVL_CAMERA_LINEAR" || sType == "calibu_pinhole" ) {
            return "calibu_pinhole"; // standard pinhole projection model
        }
        else if( sType == "MVL_CAMERA_LUT" || sType == "calibu_lut" ) {
            return "calibu_lut"; // look-up table model
        }
        else if( sType == "calibu_fov" ) {
            return "calibu_fov"; // fov model
        }
        return "none";
    }

    ///////////////////////////////////////////////////////
    void ReadCameraModelAndPose(
            const std::string& sFile, //< Input: file name to read from. 
            CameraModel& rCam,        //< Output: camera model.
            Eigen::Matrix4d& rPose    //< Output: user specified pose of the camera.
            )
    {
        // if not found, try adding .gz
        std::string sFileName( sFile );
        std::string sPath;
        if( IsFile( sFileName ) == false ){
            if( IsFile( sFileName+".gz" ) ){
                sFileName = sFileName+".gz";
            }
        }

        int nPos = sFileName.find_last_of(".");
        std::string sExt = sFile.substr( nPos );
        std::transform( sExt.begin(), sExt.end(), sExt.begin(), ::tolower );

        // load from zipped file or not 
        TiXmlDocument doc;
        bool success = false;
        if( sExt == ".gz" ){
            char* pBuf = AllocReadIntoBuffer( sFileName.c_str() );
            success = doc.Parse( pBuf );
            free( pBuf );
        }
        else{
            success = doc.LoadFile( sFileName.c_str() );
        }
        if( !success ){
            fprintf( stderr,
                    "ERROR: opening or parsing camera model XML file '%s': %s\n",
                    sFile.c_str(), strerror(errno) );
            return;
        }

        // start parsing the xml
        TiXmlElement* pCamNode = doc.FirstChildElement( "camera_model" );

        /// Get Type, allocate and initialize specialized camera type
        std::string sType   = CameraModelType( pCamNode->Attribute("type"));
        std::string sVer    = pCamNode->Attribute("version");
        std::string sName   = pCamNode->Attribute("name");
        std::string sIndex  = pCamNode->Attribute("index");
        std::string sSerial = pCamNode->Attribute("serialno");

        rCam.Init( sType );
//        rCam.SetImageDimensions( nWidth, nHeight );

        /// Set version
        if( sVer.empty() ){
            fprintf( stderr, 
                    "ERROR: Unknown camera model version (no version attribute)" );
            return;
        }

        rCam.SetVersion( atoi(sVer.c_str()) );
        rCam.SetName( sName );
        rCam.SetIndex( sIndex.empty() ? 0 : atoi(sIndex.c_str()) );
        rCam.SetSerialNumber( sSerial.empty() ? -1 : atoi(sSerial.c_str()) );

        if( rCam.Type() == "lut" ){
            // Read camera model parameters
            //        rCam.Params() << 
        }
        else if( rCam.Type() == "pinhole" ){

        }
        else if( rCam.Type() == "lut" ){

        }

        /*
           if( g_nMvlCameraModelVerbosityLevel > 0 &&
           cam->version != CURRENT_CMOD_XML_FILE_VERSION ){
           printf( "WARNING: Camera model v%d is outdated -- things should be fine, but you\n"
           "         should consider updating your camera models.\n", cam->version );
           printf( "         To do so, just run the 'cmodupdate' utility.\n\n" );
        //        printf( "       *** WILL TRY TO CONTINUE BUT BEHAVIOUR IS UNDEFINED AFTER THIS POINT ***\n\n\n" );
        }
         */
    }

    ///////////////////////////////////////////////////////
    ///
    void WriteCameraModelAndPose(
            CameraModelInterface& rCam, //< Input: camera to save.
            Eigen::Matrix4d& rPose, //< Input: pose of the camera.
            const std::string& sFile  //< Input: file name to write.
            )
    {

        std::ofstream file( sFile.c_str() );
        if( !file.is_open() ){
            printf( "ERROR: opening '%s' -- %s\n", sFile.c_str(), strerror(errno) );
            return;
        }

        // save camera model to a string
        Eigen::Vector3d dEulerAngles = rPose.block<3,3>(0,0).eulerAngles( 0, 1, 2 );
        std::stringstream ss;
        ss << "<camera_model name=\"" << rCam.Name() << "\" "
            << "index=\"" << rCam.Index() << "\" "
            << "serialno=\"" << rCam.SerialNumber() << "\" "
            << "type=\"" << rCam.Type() << "\" "
            << "version=\"" << rCam.Version() << "\">\n";
        ss << "    <width> " << rCam.Width() << " </width>\n";
        ss << "    <height> " << rCam.Height() << " </height>\n";
        ss << "    <pose> "
            << rPose(0,3) << " "
            << rPose(1,3) << " "
            << rPose(2,3) << " "
            << dEulerAngles[0] << " "
            << dEulerAngles[1] << " "
            << dEulerAngles[2] << " </pose>\n";

        // now print the RDF info
        ss << "    <right> "
            << rCam.RDF()(0,0) << " "
            << rCam.RDF()(1,0) << " "
            << rCam.RDF()(2,0) << " </right>\n";

        ss << "    <down> "
            << rCam.RDF()(0,1) << " "
            << rCam.RDF()(1,1) << " "
            << rCam.RDF()(2,1) << " </down>\n";

        ss << "    <forward> "
            << rCam.RDF()(0,2) << " "
            << rCam.RDF()(1,2) << " "
            << rCam.RDF()(2,2) << " </forward>\n";

        ss.precision(7);
        ss << "    <params> "
            << rCam.GenericParams().transpose()
            << " </params>\n";
        ss << "</camera_model>\n";

        file << ss.str();
    }
}

