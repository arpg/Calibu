/**
 *  \file camera_model_io.h
 *
 *  Functions for reading/writing MVL camera models
 *
 *  $Id$
 */

#include <iostream>
#include <fstream>
#include "camera_model_io.h"

#include "camera_model_lut.h"

#include "mvl_tinyxml.h"

using namespace MVLUtils;
using namespace std;


///////////////////////////////////////////////////////////////////////////////
/// Helper for reading lookup-tables from xml files.
static bool _read_lut_point( char* buf, char** endptr, _Bi_Point2D_f_nob& pt );

///////////////////////////////////////////////////////////////////////////////
/// Helper for writing lookup-tables to xml files.
static void _write_lut_point( const _Bi_Point2D_f_nob& pt, char* buf );

///////////////////////////////////////////////////////////////////////////////
/// Local helper.
static bool _mvl_write_linear_camera_to_string(
        const char *name,
        const long int serialno,
        const int index,
        const int width,
        const int height,
        const double *hpose,
        const double *RDF,
        const double fx,
        const double fy,
        const double cx,
        const double cy,
        const double sx,
        std::string& sResult
        );

///////////////////////////////////////////////////////////////////////////////
static bool _mvl_write_warped_camera_to_string(
        const char *name,
        const long int serialno,
        const int index,
        const int width,
        const int height,
        const double *hpose,
        const double *RDF,
        const double fx,
        const double fy,
        const double cx,
        const double cy,
        const double sx,
        const double kappa1,
        const double kappa2,
        const double kappa3,
        const double tau1,
        const double tau2,
        std::string& sResult
        );

///////////////////////////////////////////////////////////////////////////////
static bool _mvl_write_lut_camera_to_string(
        const char *name,
        const long int serialno,
        const int index,
        const int width,
        const int height,
        const double *hpose,
        const double *RDF,
        const double fx,
        const double fy,
        const double cx,
        const double cy,
        const double sx,
        Bi_Point2D_f_nob** LUT,
        std::string& sResult
        );


////////////////////////////////////////////////////////////////////////////////
/// Reads lookup table from file specified by filename in lut camera model
static int _mvl_read_lut_from_file(
        const char* pLUTFileName, //< Input: LUT file name.
        int nWidth,               //< Input: image width.
        int nHeight,              //< Input: image height.
        Bi_Point2D_f_nob*** pLUT  //< Output: lookup table.
        );

///////////////////////////////////////////////////////////////////////////////
bool _FilePartsMVL( std::string sFullPath, std::string& sPath,
        std::string& sFile, std::string& sExtension )
{
    size_t nBreak;
    size_t nFS = sFullPath.find_last_of("/");

#ifdef _WIN32
    //Windows user use either forward ot backslash to delimit (or even
    // a combination...)
    /*unsigned int*/ size_t nBS = sFullPath.find_last_of("\\");
    if(nBS!=std::string::npos)
    {
        if(nFS!=std::string::npos)
        {
            //look like a mixture - which is the final one
            nBreak=nBS>nFS ? nBS:nFS;
        }
        else
        {
            //looks like they are using only back slashes
            nBreak= nBS;
        }
    }
    else
    {
        //looks like they are using nix style forward slashes
        nBreak = nFS;
    }
#else
    nBreak = nFS;
#endif

    std::string sFullFile;
    if(nBreak==std::string::npos){
        //there is no path
        sPath = "";
        sFullFile = sFullPath;
    }
    else{
        //split path and file
        sPath = sFullPath.substr(0,nBreak);
        sFullFile = sFullPath.substr(nBreak+1);
    }

    //finally look to split on "." for extension if it is there.
    size_t ii = sFullFile.find_last_of( "." );
    if( ii != string::npos ){
        sFile = sFullFile.substr( 0, ii );
        sExtension = sFullFile.substr( ii+1, sFullFile.size()-ii );
    }
    else{
        sFile = sFullFile;
        sExtension = "";
    }

    return true;
}


///////////////////////////////////////////////////////////////////////////////
// cut-n-paste helper for reading camera models
int _scan_double_array(
        const char *str,
        char delim,
        int maxnums,
        double *nums
        )
{
    int n, ii;
    double d;
    char *rem_str;

    n = 0;
    rem_str = (char*)str;
    for( ii = 0; ii < maxnums; ii++ ) {
        /* parse the next number */
        d = strtod( str, &rem_str );
        if( rem_str == str )
            return n;

        /* ignore the stuff we've already parsed */
        str = rem_str;

        nums[ii] = d;
        n++;

        /* skip seperator character, if any */
        while( isspace( *str ) ) str++;
        if( *str == delim) str++;
    }
    if( n == maxnums ){
        return n;
    }
    return 0;
}

///////////////////////////////////////////////////////////////////////////////
// cut-n-paste helper for reading camera models
int _parse_doubles(
        const std::string& str,
        const std::string& delims,
        int maxnums,
        double *nums )
{
    // try to scan double given any of the delimiters listed in delims
    for( size_t ii = 0; ii < delims.length(); ii ++ ){
        char delim = delims[ii];
        if( _scan_double_array(
                    str.c_str(),
                    delim,
                    maxnums,
                    nums ) == maxnums ){
            return maxnums;
        }
    }
    return -1;
}


///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

namespace MVLUtils
{
    ///////////////////////////////////////////////////////////////////////////////
    bool _mvl_read_linear_camera(
            TiXmlElement* pCamNode,
            int *width,
            int *height,
            double *hpose,
            double *RDF,
            double *fx,
            double *fy,
            double *cx,
            double *cy,
            double *sx
            )
    {
        double epose[6];
        double forward[3];
        double right[3];
        double down[3];
        const char* delims = " ;,"; // acceptable delimiters for vector of numbers

        for( TiXmlElement*  pNode = pCamNode->FirstChildElement();
                pNode != NULL;
                pNode = pNode->NextSiblingElement() ) {
            std::string sElement( pNode->Value()   );
            std::string sText(    pNode->GetText() );

            char* valstr = (char*)sText.c_str();

            if( sElement.compare("width") == 0 ){
                if( sscanf( valstr, "%d", width ) != 1){
                    return false;
                }
            }
            else if( sElement.compare("height") == 0 ){
                if( sscanf( valstr, "%d", height ) != 1 ){
                    return false;
                }
            }
            else if( sElement.compare("forward") == 0 ){
                if( _parse_doubles( valstr, delims, 3, forward ) != 3 ){
                    return false;
                }
            }
            else if( sElement.compare("right") == 0 ){
                if( _parse_doubles( valstr, delims, 3, right ) != 3 ){
                    return false;
                }
            }
            else if( sElement.compare("down") == 0 ){
                if( _parse_doubles( valstr, delims, 3, down ) != 3 ){
                    return false;
                }
            }
            else if( sElement.compare("pose") == 0 ){
                if( _parse_doubles( valstr, delims, 6, epose ) != 6 ){
                    return false;
                }
                epose_to_hpose_d( epose, hpose ); /* from libkinematics */
            }
            else if( sElement.compare("fx") == 0 ){
                if( sscanf( valstr, "%lf", fx ) != 1 ){
                    return false;
                }
            }
            else if( sElement.compare("fy") == 0 ){
                if( sscanf( valstr, "%lf", fy )!= 1 ){
                    return false;
                }
            }
            else if( sElement.compare("cx") == 0 ){
                if( sscanf( valstr, "%lf", cx )!= 1 ){
                    return false;
                }
            }
            else if( sElement.compare("cy") == 0 ){
                if( sscanf( valstr, "%lf", cy )!= 1 ){
                    return false;
                }
            }
            else if( sElement.compare("sx") == 0 ){
                if( sscanf( valstr, "%lf", sx )!= 1 ){
                    return false;
                }
            }
        }

        // convert forward, right and down into the RDF conversion matrix.
        // See camera_math.pdf in the docs dir for more info.

        RDF[0] = right[0];
        RDF[1] = right[1];
        RDF[2] = right[2];

        RDF[3] = down[0];
        RDF[4] = down[1];
        RDF[5] = down[2];

        RDF[6] = forward[0];
        RDF[7] = forward[1];
        RDF[8] = forward[2];

        return true;
    }

    ///////////////////////////////////////////////////////////////////////////////
    bool _mvl_read_warped_camera(
            TiXmlElement* pCamNode,
            int *width,
            int *height,
            double *hpose,
            double *RDF,
            double *fx,
            double *fy,
            double *cx,
            double *cy,
            double *sx,
            double *kappa1,
            double *kappa2,
            double *kappa3,
            double *tau1,
            double *tau2 )
    {
        double epose[6];
        double forward[3];
        double right[3];
        double down[3];
        const char* delims = " ;,"; // acceptable delimiters for vector of numbers

        for( TiXmlElement*  pNode = pCamNode->FirstChildElement();
             pNode != NULL;
             pNode = pNode->NextSiblingElement() ) {

            std::string sElement( pNode->Value()   );
            std::string sText(    pNode->GetText() );

            char* valstr = (char*)sText.c_str();

            if( sElement.compare("width") == 0 ){
                if( sscanf( valstr, "%d", width ) != 1){
                    return false;
                }
            }
            else if( sElement.compare("height") == 0 ){
                if( sscanf( valstr, "%d", height ) != 1 ){
                    return false;
                }
            }
            else if( sElement.compare("forward") == 0 ){
                if( _parse_doubles( valstr, delims, 3, forward ) != 3 ){
                    return false;
                }
            }
            else if( sElement.compare("right") == 0 ){
                if( _parse_doubles( valstr, delims, 3, right ) != 3 ){
                    return false;
                }
            }
            else if( sElement.compare("down") == 0 ){
                if( _parse_doubles( valstr, delims, 3, down ) != 3 ){
                    return false;
                }
            }
            else if( sElement.compare("pose") == 0 ){
                if( _parse_doubles( valstr, delims, 6, epose ) != 6 ){
                    return false;
                }
                epose_to_hpose_d( epose, hpose ); /* from libkinematics */
            }
            else if( sElement.compare("fx") == 0 ){
                if( sscanf( valstr, "%lf", fx ) != 1 ){
                    return false;
                }
            }
            else if( sElement.compare("fy") == 0 ){
                if( sscanf( valstr, "%lf", fy )!= 1 ){
                    return false;
                }
            }
            else if( sElement.compare("cx") == 0 ){
                if( sscanf( valstr, "%lf", cx )!= 1 ){
                    return false;
                }
            }
            else if( sElement.compare("cy") == 0 ){
                if( sscanf( valstr, "%lf", cy )!= 1 ){
                    return false;
                }
            }
            else if( sElement.compare("sx") == 0 ){
                if( sscanf( valstr, "%lf", sx ) != 1 ){
                    return false;
                }
            }
            else if( sElement.compare("kappa1") == 0 ) {
                if( sscanf( valstr, "%lf", kappa1 ) != 1 ){
                    return false;
                }
            }
            else if( sElement.compare("kappa2") == 0 ) {
                if( sscanf( valstr, "%lf", kappa2 ) != 1 ){
                    return false;
                }
            }
            else if( sElement.compare("kappa3") == 0 ) {
                if( sscanf( valstr, "%lf", kappa3 ) != 1 ){
                    return false;
                }
            }
            else if( sElement.compare("tau1") == 0 ) {
                if( sscanf( valstr, "%lf", tau1 ) != 1 ) {
                    return false;
                }
            }
            else if( sElement.compare("tau2") == 0 ) {
                if( sscanf( valstr, "%lf", tau2 ) != 1 ) {
                    return false;
                }
            }
        }

        // convert forward, right and down into the RDF conversion matrix.
        // See camera_math.pdf in the docs dir for more info.
        RDF[0] = right[0];
        RDF[1] = right[1];
        RDF[2] = right[2];

        RDF[3] = down[0];
        RDF[4] = down[1];
        RDF[5] = down[2];

        RDF[6] = forward[0];
        RDF[7] = forward[1];
        RDF[8] = forward[2];

        return true;
    }

    ///////////////////////////////////////////////////////////////////////////////
    bool _mvl_read_lut_camera(
            TiXmlElement* pCamNode,
            int *width,
            int *height,
            double *hpose,
            double *RDF,
            double *fx,
            double *fy,
            double *cx,
            double *cy,
            double *sx,
            Bi_Point2D_f_nob*** pLUT
            )
    {
        double epose[6];
        double forward[3];
        double right[3];
        double down[3];
        const char* delims = " ;,"; // acceptable delimiters for vector of numbers

        *width = 0;
        *height = 0;

        bool bSuccess = false;

        char* lutptr = NULL;
        int  nLutSize = 0;

        for( TiXmlElement*  pNode = pCamNode->FirstChildElement();
                pNode != NULL;
                pNode = pNode->NextSiblingElement() ) {
            std::string sElement( pNode->Value()   );
            std::string sText(    pNode->GetText() );

            char* valstr = (char*)sText.c_str();

            if( sElement.compare("width") == 0 ){
                if( sscanf( valstr, "%d", width ) != 1){
                    return false;
                }
            }
            else if( sElement.compare("height") == 0 ){
                if( sscanf( valstr, "%d", height ) != 1 ){
                    return false;
                }
            }
            else if( sElement.compare("forward") == 0 ){
                if( _parse_doubles( valstr, delims, 3, forward ) != 3 ){
                    return false;
                }
            }
            else if( sElement.compare("right") == 0 ){
                if( _parse_doubles( valstr, delims, 3, right ) != 3 ){
                    return false;
                }
            }
            else if( sElement.compare("down") == 0 ){
                if( _parse_doubles( valstr, delims, 3, down ) != 3 ){
                    return false;
                }
            }
            else if( sElement.compare("pose") == 0 ){
                if( _parse_doubles( valstr, delims, 6, epose ) != 6 ){
                    return false;
                }
                epose_to_hpose_d( epose, hpose ); /* from libkinematics */
            }
            else if( sElement.compare("fx") == 0 ){
                if( sscanf( valstr, "%lf", fx ) != 1 ){
                    return false;
                }
            }
            else if( sElement.compare("fy") == 0 ){
                if( sscanf( valstr, "%lf", fy )!= 1 ){
                    return false;
                }
            }
            else if( sElement.compare("cx") == 0 ){
                if( sscanf( valstr, "%lf", cx )!= 1 ){
                    return false;
                }
            }
            else if( sElement.compare("cy") == 0 ){
                if( sscanf( valstr, "%lf", cy )!= 1 ){
                    return false;
                }
            }
            else if( sElement.compare("sx") == 0 ){
                if( sscanf( valstr, "%lf", sx )!= 1 ){
                    return false;
                }
            }

            else if( sElement.compare("lut") == 0 ){
                lutptr = valstr;
                nLutSize = sElement.size();

                //rjs: have to read the LUT here
                //as valstr ptr no longer valid outside
                //of TiXML for loop

                // Read LUT from string of hex values
                mvl_alloc_lut( *width, *height, pLUT );
                for( int ii = 0; ii < *height; ++ii ) {
                    for( int jj = 0; jj < *width; ++jj ) {
                        _read_lut_point( lutptr, &lutptr, (*pLUT)[ii][jj] );
                    }

                }
            }
        }

        if( *width == 0 || *height == 0 ) {
            fprintf( stderr, "ERROR: width or height is zero\n" );
            return false;
        }

        //rjs: can't read LUT here, lutptr no longer valid

        // Read LUT from string of hex values
//        mvl_alloc_lut( *width, *height, pLUT );
//        for( int ii = 0; ii < *height; ++ii ) {
//            for( int jj = 0; jj < *width; ++jj ) {
//                 _read_lut_point( lutptr, &lutptr, (*pLUT)[ii][jj] );
//            }
//        }

        bSuccess = true;

        // convert forward, right and down into the RDF conversion matrix.
        // See camera_math.pdf in the docs dir for more info.

        RDF[0] = right[0];
        RDF[1] = right[1];
        RDF[2] = right[2];

        RDF[3] = down[0];
        RDF[4] = down[1];
        RDF[5] = down[2];

        RDF[6] = forward[0];
        RDF[7] = forward[1];
        RDF[8] = forward[2];

        return bSuccess;
    }



    ///////////////////////////////////////////////////////////////////////////////
    bool _mvl_read_old_lut_camera(
            TiXmlElement* pCamNode,
            int *width,
            int *height,
            double *hpose,
            double *RDF,
            double *fx,
            double *fy,
            double *cx,
            double *cy,
            double *sx,
            std::string& sPath,
            char* pLUTFilename,
            Bi_Point2D_f_nob*** pLUT
            )
    {
        double epose[6];
        double forward[3];
        double right[3];
        double down[3];
        const char* delims = " ;,"; // acceptable delimiters for vector of numbers

        *width = 0;
        *height = 0;

        for( TiXmlElement*  pNode = pCamNode->FirstChildElement();
                pNode != NULL;
                pNode = pNode->NextSiblingElement() ) {
            std::string sElement( pNode->Value()   );
            std::string sText(    pNode->GetText() );

            char* valstr = (char*)sText.c_str();

            if( sElement.compare("width") == 0 ){
                if( sscanf( valstr, "%d", width ) != 1){
                    return false;
                }
            }
            else if( sElement.compare("height") == 0 ){
                if( sscanf( valstr, "%d", height ) != 1 ){
                    return false;
                }
            }
            else if( sElement.compare("forward") == 0 ){
                if( _parse_doubles( valstr, delims, 3, forward ) != 3 ){
                    return false;
                }
            }
            else if( sElement.compare("right") == 0 ){
                if( _parse_doubles( valstr, delims, 3, right ) != 3 ){
                    return false;
                }
            }
            else if( sElement.compare("down") == 0 ){
                if( _parse_doubles( valstr, delims, 3, down ) != 3 ){
                    return false;
                }
            }
            else if( sElement.compare("pose") == 0 ){
                if( _parse_doubles( valstr, delims, 6, epose ) != 6 ){
                    return false;
                }
                epose_to_hpose_d( epose, hpose ); /* from libkinematics */
            }
            else if( sElement.compare("fx") == 0 ){
                if( sscanf( valstr, "%lf", fx ) != 1 ){
                    return false;
                }
            }
            else if( sElement.compare("fy") == 0 ){
                if( sscanf( valstr, "%lf", fy )!= 1 ){
                    return false;
                }
            }
            else if( sElement.compare("cx") == 0 ){
                if( sscanf( valstr, "%lf", cx )!= 1 ){
                    return false;
                }
            }
            else if( sElement.compare("cy") == 0 ){
                if( sscanf( valstr, "%lf", cy )!= 1 ){
                    return false;
                }
            }
            else if( sElement.compare("sx") == 0 ){
                if( sscanf( valstr, "%lf", sx )!= 1 ){
                    return false;
                }
            }

            else if( sElement.compare( "lut" ) == 0 ) {
                if( sprintf( pLUTFilename, "%s", valstr ) == 0 ){
                    return false;
                }
            }
        }

        if( *width == 0 || * height == 0 ) {
            fprintf( stderr, "ERROR: width or height is zero\n" );
            return false;
        }

        // Read LUT from file
        string sFile, sExt;
        _FilePartsMVL( pLUTFilename, sPath, sFile, sExt );
        sFile = sFile + "." + sExt;
        if( _mvl_read_lut_from_file( sFile.c_str(), *width, *height, pLUT ) != 0 ) {
            stringstream sFullLUTFilename;
#ifdef _WIN32
            sFullLUTFilename << sPath << "\\" << sFile;
#else
            sFullLUTFilename << sPath << "/" << sFile;
#endif
            if( _mvl_read_lut_from_file( sFullLUTFilename.str().c_str(), *width, *height, pLUT ) != 0 ) {
                fprintf( stderr, "ERROR: could not find LUT file '%s'!'\n", sFullLUTFilename.str().c_str() );
                return false;
            }
        }

        // convert forward, right and down into the RDF conversion matrix.
        // See camera_math.pdf in the docs dir for more info.

        RDF[0] = right[0];
        RDF[1] = right[1];
        RDF[2] = right[2];

        RDF[3] = down[0];
        RDF[4] = down[1];
        RDF[5] = down[2];

        RDF[6] = forward[0];
        RDF[7] = forward[1];
        RDF[8] = forward[2];

        return true;
    }

} // end MVLUtils

///////////////////////////////////////////////////////////////////////////////
///////////////////// WRITING ///////////////////

#include <zlib.h>
#include <algorithm>

/// Write the camera model specified in cam to the xml file filename.  Also
//  convert the 4x4 homogeneous pose matrix hpose to a 6x1 euler roll-pitch-yaw
//  pose and write it as the pose element of the xml file.
bool mvl_write_camera(
        const char *filename,   //< Input:
        const double *hpose,    //< Input:
        const mvl_camera_t *cam //< Input:
        )
{
    std::string sRes;
    if( mvl_write_camera_to_string( hpose, cam, sRes ) == false ) {
        return false;
    }

    string sName( filename );

    // automatically compress LUT models
    int nPos = sName.find_last_of(".");
    string sExt = sName.substr( nPos );
    std::transform( sExt.begin(), sExt.end(), sExt.begin(), ::tolower );

    bool bCompress = false;
    if( sExt == ".gz" ){
        bCompress = true;
    }
    else{
        if( cam->type == MVL_CAMERA_LUT ){ // force compression for LUT models
//            sName = sName+".gz";
//            bCompress = true;
        }
    }

    if( bCompress ){
        gzFile gzfile;
        gzfile = gzopen ( sName.c_str(), "w2b" );
        if ( gzfile == NULL ) {
            return false;
        }
        gzwrite ( gzfile, sRes.c_str(), sRes.length() );
        gzclose ( gzfile );
    }
    else{
        std::ofstream of(filename);
        if( of.is_open() == false ){
            return false;
        }
        of << sRes;
        of.close();
    }

    return true;
}

/*****************************************************************************/
/// Print a camera model to a string.
bool mvl_write_camera_to_string(
        const double *hpose,    /**< Input: */
        const mvl_camera_t *cam, /**< Input: */
        std::string& sRes       /**< Output: */
        )
{
    switch(cam->type) {
        case MVL_CAMERA_LINEAR:
            return _mvl_write_linear_camera_to_string(
                    cam->name,
                    cam->serialno,
                    cam->index,
                    cam->warped.width,
                    cam->warped.height,
                    hpose,
                    cam->RDF,
                    cam->warped.fx,
                    cam->warped.fy,
                    cam->warped.cx,
                    cam->warped.cy,
                    cam->warped.sx,
                    sRes
                    );
            break;
        case MVL_CAMERA_WARPED:
            return _mvl_write_warped_camera_to_string(
                    cam->name,
                    cam->serialno,
                    cam->index,
                    cam->warped.width,
                    cam->warped.height,
                    hpose,
                    cam->RDF,
                    cam->warped.fx,
                    cam->warped.fy,
                    cam->warped.cx,
                    cam->warped.cy,
                    cam->warped.sx,
                    cam->warped.kappa1,
                    cam->warped.kappa2,
                    cam->warped.kappa3,
                    cam->warped.tau1,
                    cam->warped.tau2,
                    sRes
                    );
            break;
        case MVL_CAMERA_LUT:
            // write out the full lut with the file.
            return _mvl_write_lut_camera_to_string(
                    cam->name,
                    cam->serialno,
                    cam->index,
                    cam->lut.width,
                    cam->lut.height,
                    hpose,
                    cam->RDF,
                    cam->lut.fx,
                    cam->lut.fy,
                    cam->lut.cx,
                    cam->lut.cy,
                    cam->lut.sx,
                    //cam->lut.lutfilename,
                    cam->lut.pLUT,
                    sRes
                    );
            break;
        default:
            return false;
    }
    return true;
}



/*****************************************************************************/
bool _mvl_write_linear_camera_to_string(
        const char *name,
        const long int serialno,
        const int index,
        const int width,
        const int height,
        const double *hpose,
        const double *RDF,
        const double fx,
        const double fy,
        const double cx,
        const double cy,
        const double sx,
        std::string& sResult
        )
{
    double epose[6];

    hpose_to_epose_d( hpose, epose );

    std::stringstream ss;
    ss << "<camera_model name=\"" << name << "\" "
        << "index=\"" << index << "\" "
        << "serialno=\"" << serialno << "\" "
        << "type=\"" << mvl_camera_model_type_strings[MVL_CAMERA_LINEAR] << "\" "
        << "version=\"" << CURRENT_CMOD_XML_FILE_VERSION << "\">\n";

    ss << "    <width> " << width << " </width>\n";
    ss << "    <height> " << height << " </height>\n";

    ss << "    <pose> "
        << epose[0] << "; "
        << epose[1] << "; "
        << epose[2] << "; "
        << epose[3] << "; "
        << epose[4] << "; "
        << epose[5] << " </pose>\n";

    // now print the RDF info
    ss << "    <right> "
        << RDF[0] << "; "
        << RDF[1] << "; "
        << RDF[2] << " </right>\n";

    ss << "    <down> "
        << RDF[3] << "; "
        << RDF[4] << "; "
        << RDF[5] << " </down>\n";

    ss << "    <forward> "
        << RDF[6] << "; "
        << RDF[7] << "; "
        << RDF[8] << " </forward>\n";

    ss << "    <fx> " << fx << " </fx>\n";
    ss << "    <cx> " << cx << " </cx>\n";
    ss << "    <fy> " << fy << " </fy>\n";
    ss << "    <cy> " << cy << " </cy>\n";
    ss << "    <sx> " << sx << " </sx>\n";

    ss << "</camera_model>\n";

    // copy to output parameter
    sResult = ss.str();

    return true;
}

/*****************************************************************************/
bool _mvl_write_warped_camera_to_string(
        const char *name,
        const long int serialno,
        const int index,
        const int width,
        const int height,
        const double *hpose,
        const double *RDF,
        const double fx,
        const double fy,
        const double cx,
        const double cy,
        const double sx,
        const double kappa1,
        const double kappa2,
        const double kappa3,
        const double tau1,
        const double tau2,
        std::string& sResult
        )
{
    double epose[6];

    hpose_to_epose_d( hpose, epose );

    std::stringstream ss;
    ss << "<camera_model name=\"" << name << "\" "
        << "index=\"" << index << "\" "
        << "serialno=\"" << serialno << "\" "
        << "type=\"" << mvl_camera_model_type_strings[MVL_CAMERA_WARPED] << "\" "
        << " version=\"" << CURRENT_CMOD_XML_FILE_VERSION << "\">\n";

    ss << "    <width> " << width << " </width>\n";
    ss << "    <height> " << height << " </height>\n";

    ss << "    <pose> "
        << epose[0] << "; "
        << epose[1] << "; "
        << epose[2] << "; "
        << epose[3] << "; "
        << epose[4] << "; "
        << epose[5] << " </pose>\n";

    // now print the RDF info
    ss << "    <right> "
        << RDF[0] << "; "
        << RDF[1] << "; "
        << RDF[2] << " </right>\n";

    ss << "    <down> "
        << RDF[3] << "; "
        << RDF[4] << "; "
        << RDF[5] << " </down>\n";

    ss << "    <forward> "
        << RDF[6] << "; "
        << RDF[7] << "; "
        << RDF[8] << " </forward>\n";

    ss << "    <fx> " << fx << " </fx>\n";
    ss << "    <cx> " << cx << " </cx>\n";
    ss << "    <fy> " << fy << " </fy>\n";
    ss << "    <cy> " << cy << " </cy>\n";
    ss << "    <sx> " << sx << " </sx>\n";

    ss << "    <kappa1> " << kappa1 << " </kappa1>\n";
    ss << "    <kappa2> " << kappa2 << " </kappa2>\n";
    ss << "    <kappa3> " << kappa3 << " </kappa3>\n";
    ss << "    <tau1> " << tau1 << " </tau1>\n";
    ss << "    <tau2> " << tau2 << " </tau2>\n";

    ss << "</camera_model>\n";

    // copy to output parameter
    sResult = ss.str();

    return true;
}

/*****************************************************************************/
/// The new way: keeps the LUT with the xml file.
bool _mvl_write_lut_camera_to_string(
        const char *name,
        const long int serialno,
        const int index,
        const int width,
        const int height,
        const double *hpose,
        const double *RDF,
        const double fx,
        const double fy,
        const double cx,
        const double cy,
        const double sx,
        Bi_Point2D_f_nob** LUT,
        std::string& sResult
        )
{
    double epose[6];

    hpose_to_epose_d( hpose, epose );

    std::stringstream ss;
    ss << "<camera_model name=\"" << name << "\" "
        << "index=\"" << index << "\" "
        << "serialno=\"" << serialno << "\" "
        << "type=\"" << mvl_camera_model_type_strings[MVL_CAMERA_LUT] << "\" "
        << " version=\"" << CURRENT_CMOD_XML_FILE_VERSION << "\">\n";

    ss << "    <width> " << width << " </width>\n";
    ss << "    <height> " << height << " </height>\n";

    ss << "    <pose> "
        << epose[0] << "; "
        << epose[1] << "; "
        << epose[2] << "; "
        << epose[3] << "; "
        << epose[4] << "; "
        << epose[5] << " </pose>\n";

    // now print the RDF info
    ss << "    <right> "
        << RDF[0] << "; "
        << RDF[1] << "; "
        << RDF[2] << " </right>\n";

    ss << "    <down> "
        << RDF[3] << "; "
        << RDF[4] << "; "
        << RDF[5] << " </down>\n";

    ss << "    <forward> "
        << RDF[6] << "; "
        << RDF[7] << "; "
        << RDF[8] << " </forward>\n";

    ss << "    <fx> " << fx << " </fx>\n";
    ss << "    <cx> " << cx << " </cx>\n";
    ss << "    <fy> " << fy << " </fy>\n";
    ss << "    <cy> " << cy << " </cy>\n";
    ss << "    <sx> " << sx << " </sx>\n";

    // copy to output parameter
    sResult = ss.str();

    // write the lut to the string (new in version 7)
    sResult += "    <lut>\n";

    for( int irow = 0; irow <  height; irow++ ){
        for( int jcol = 0; jcol < width; jcol++ ){
            char sPt[64] = {0};
            _write_lut_point( LUT[irow][jcol], sPt );
            sResult += sPt;
            sResult += "\n";
        }
    }
    sResult += "\n    </lut>\n";
    sResult += "</camera_model>\n";

    return true;
}


////////////////////////////////////////////////////////////////////////////////
bool _read_lut_point( char* buf, char** endptr, _Bi_Point2D_f_nob& pt )
{
    pt.v00 = strtol( buf, &buf, 16 );
    pt.v01 = strtol( buf, &buf, 16 );

    /// this is convoluted to deal with gcc fno-strict-aliasing...
    int32_t a = strtol( buf, &buf, 16 );
    float* pa = (float*)&a;

    int32_t b = strtol( buf, &buf, 16 );
    float* pb = (float*)&b;

    int32_t c = strtol( buf, &buf, 16 );
    float* pc = (float*)&c;

    int32_t d = strtol( buf, &buf, 16 );
    float* pd = (float*)&d;

    pt.one_m_c_mul_one_m_r = *pa;
    pt.c_mul_one_m_r       = *pb;
    pt.one_m_c_mul_r       = *pc;
    pt.c_mul_r             = *pd;

    *endptr = buf;

    return true;
}

////////////////////////////////////////////////////////////////////////////////
void _write_lut_point( const _Bi_Point2D_f_nob& pt, char* buf )
{
    unsigned int* a = (unsigned int*)&pt.one_m_c_mul_one_m_r;
    unsigned int* b = (unsigned int*)&pt.c_mul_one_m_r;
    unsigned int* c = (unsigned int*)&pt.one_m_c_mul_r;
    unsigned int* d = (unsigned int*)&pt.c_mul_r;
    sprintf( buf, "%X %X %08X %08X %08X %08X", pt.v00, pt.v01, *a, *b, *c, *d );
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int _mvl_read_lut_from_file( const char* pLUTFileName,  //< Input: file name model
                            int nWidth, int nHeight,
                            Bi_Point2D_f_nob*** pLUT   //< Output: lookup table
                            )
{
    mvl_alloc_lut( nWidth, nHeight, pLUT );

    bool bSuccess = true;

    FILE *fp;
    fp = fopen( pLUTFileName, "r" );

    if( fp != NULL ) {
        for( int ii = 0; ii < nHeight; ++ii ) {
            for( int jj = 0; jj < nWidth; ++jj ) {
                if( fread( &((*pLUT)[ii][jj].v00),                 sizeof(int), 1, fp ) != 1 ) {
                    bSuccess = false;
                    break;
                }
                if( fread( &((*pLUT)[ii][jj].v01),                 sizeof(int), 1, fp) != 1 ) {
                    bSuccess = false;
                    break;
                }
                if( fread( &((*pLUT)[ii][jj].one_m_c_mul_one_m_r), sizeof(float), 1, fp) != 1 ) {
                    bSuccess = false;
                    break;
                }
                if( fread( &((*pLUT)[ii][jj].c_mul_one_m_r),       sizeof(float), 1, fp) != 1 ) {
                    bSuccess = false;
                    break;
                }
                if( fread( &((*pLUT)[ii][jj].one_m_c_mul_r),       sizeof(float), 1, fp) != 1 ) {
                    bSuccess = false;
                    break;
                };
                if( fread( &((*pLUT)[ii][jj].c_mul_r),             sizeof(float), 1, fp) != 1 ) {
                    bSuccess = false;
                    break;
                }
            }
        }

        fclose( fp );

        if( !bSuccess ) {
            mvl_free_lut( *pLUT );
            return -1;
        }

        return 0;
    }
    else {
        mvl_free_lut( *pLUT );
        return -1;
    }
}

///////////////////////////////////////////////////////////////////////////////
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
///////////////////////////////////////////////////////////////////////////////
#include <sys/stat.h>
bool _mvl_camera_is_file( const std::string& sFile )
{
    struct stat StatBuf;
    if( stat( sFile.c_str(), &StatBuf ) < 0 ){
        return false;
    }
    return S_ISREG( StatBuf.st_mode );
}

///////////////////////////////////////////////////////////////////////////////
/// This is the main API for users.
mvl_camera_t *mvl_read_camera(
        const char *filename, //< Input:
        double *hpose         //< Output:
        )
{
    mvl_camera_t* cam;
    cam = (mvl_camera_t*)calloc( 1, sizeof( mvl_camera_t ));


    // if not found, try adding .gz
    string sFileName( filename ), sPath;
    if( _mvl_camera_is_file( sFileName ) == false ){
        if( _mvl_camera_is_file( sFileName+".gz" ) ){
            sFileName = sFileName+".gz";
        }
    }

    string sTmp1, sTmp2;
    _FilePartsMVL( sFileName, sPath, sTmp1, sTmp2 );




    int nPos = sFileName.find_last_of(".");
    string sExt = sFileName.substr( nPos );
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
        fprintf( stderr, "ERROR: opening or parsing camera model XML file '%s'\n", sFileName.c_str() );
        return NULL;
    }

    TiXmlElement* pCamNode = doc.FirstChildElement( "camera_model" );
    if( !pCamNode ){
        return NULL;
    }

    /// Get version
    const char* sVer = pCamNode->Attribute("version");
    if( !sVer ){
        fprintf( stderr, "ERROR: Unknown camera model version (no version attribute)" );
        return NULL;
    }
    cam->version = atoi(sVer);

    /// Get name
    const char* sName = pCamNode->Attribute("name");
    sprintf( cam->name, "%s", sName ? sName : "" );

    /// Get index
    const char* sIndex = pCamNode->Attribute("index");
    cam->index = sIndex ? atoi(sIndex) : 0;

    /// Get serial number
    const char* sSerial = pCamNode->Attribute("serialno");
    cam->serialno = sSerial ? atoi(sSerial) : -1;

    /// Get Type
    const char* sType = pCamNode->Attribute("type");
    if( !sType ){
        fprintf( stderr, "ERROR: Unknown camera model type (no type attribute)" );
        return NULL;
    }
    if( !strcmp( sType, "MVL_CAMERA_WARPED" ) ) {
        cam->type = MVL_CAMERA_WARPED;
    }
    else if( !strcmp( sType, "MVL_CAMERA_LINEAR") ) {
        cam->type = MVL_CAMERA_LINEAR;
    }
    else if( !strcmp( sType, "MVL_CAMERA_LUT") ) {
        cam->type = MVL_CAMERA_LUT;
    }
    else {
        cam->type = -1;
    }

    if( g_nMvlCameraModelVerbosityLevel > 0 &&
            cam->version != CURRENT_CMOD_XML_FILE_VERSION ){
        printf( "WARNING: Camera model v%d is outdated -- things should be fine, but you\n"
                "         should consider updating your camera models.\n", cam->version );
        printf( "         To do so, just run the 'cmodupdate' utility.\n\n" );
        //        printf( "       *** WILL TRY TO CONTINUE BUT BEHAVIOUR IS UNDEFINED AFTER THIS POINT ***\n\n\n" );
    }

    success = false;
    switch( cam->type ) {
        case MVL_CAMERA_LINEAR:
           success = _mvl_read_linear_camera(
                    pCamNode,
                    &cam->linear.width,
                    &cam->linear.height,
                    hpose,
                    cam->linear.RDF,
                    &cam->linear.fx,
                    &cam->linear.fy,
                    &cam->linear.cx,
                    &cam->linear.cy,
                    &cam->linear.sx );
            break;
        case MVL_CAMERA_WARPED:
           success = _mvl_read_warped_camera(
                    pCamNode,
                    &cam->warped.width,
                    &cam->warped.height,
                    hpose,
                    cam->warped.RDF,
                    &cam->warped.fx,
                    &cam->warped.fy,
                    &cam->warped.cx,
                    &cam->warped.cy,
                    &cam->warped.sx,
                    &cam->warped.kappa1,
                    &cam->warped.kappa2,
                    &cam->warped.kappa3,
                    &cam->warped.tau1,
                    &cam->warped.tau2 );
            break;
        case MVL_CAMERA_LUT:
            if( cam->version >= 7 ){ //  version when we switched to merged LUT+xml files.
                success = _mvl_read_lut_camera(
                        pCamNode,
                        &cam->lut.width,
                        &cam->lut.height,
                        hpose,
                        cam->lut.RDF,
                        &cam->lut.fx,
                        &cam->lut.fy,
                        &cam->lut.cx,
                        &cam->lut.cy,
                        &cam->lut.sx,
                        &(cam->lut.pLUT) );
            }
            else{
                success = _mvl_read_old_lut_camera(
                        pCamNode,
                        &cam->old_lut.width,
                        &cam->old_lut.height,
                        hpose,
                        cam->old_lut.RDF,
                        &cam->old_lut.fx,
                        &cam->old_lut.fy,
                        &cam->old_lut.cx,
                        &cam->old_lut.cy,
                        &cam->old_lut.sx,
                        sPath,
                        cam->old_lut.lutfilename,
                        &(cam->old_lut.pLUT)
                        );
            }
            break;
        default:
            printf("unknown camera model type?\n");
            break;

    }

    if( !success ) {
        free( cam );
        fprintf( stderr, "ERROR: reading camera model '%s', returning NULL.\n", filename );
        return NULL;
    }

    return cam;
}

