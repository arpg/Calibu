/**
 *  @file camera.h
 *
 *  Main camera model header.  The primary purpose of a camera model is to
 *  define how a light ray from the 3D world projects to a 2D image (3D-to-2D);
 *  and to define how a point in a 2D image projects back out along a ray into
 *  the 3D world (2D-to-3D).
 *
 *  $Id: camera.h 381 2008-02-29 01:31:34Z braskob $
 */

#ifndef __CAMERA_H__
#define __CAMERA_H__

#include <stdio.h>
#include <stdint.h>
#include <limits.h> // For NAME_MAX

#define MVL_DEBUG 1

// Forward declaration (see camera_model_lut.h)
typedef struct _Bi_Point2D_f_nob Bi_Point2D_f_nob;


/// Control verbosity in error reporting.
extern int g_nMvlCameraModelVerbosityLevel;

///  Camera model type enumeration.  Specifies the type of projection model
//   (linear, warped etc.).
//
enum {
    MVL_CAMERA_LINEAR = 0, //< projective.
    MVL_CAMERA_WARPED = 1, //< projective plus radial distortion.
    MVL_CAMERA_LUT    = 2  //< projective with distortion accounted for by lookup-table.
};

/// NOTE: these correspond to the above, and are defined in camera.cpp
extern char *mvl_camera_model_type_strings[4];

#define MAX_NAME_LEN 64

///  Linear camera model struct.  This structure contains parameters used to
//   define a pinhole projection model.
//   NB: all camera models have the same first 8 fields (in this order!)
typedef struct {
    int      type;                //< Model type (see above).
    int      version;             //< MVL camera model version.
    char     name[MAX_NAME_LEN]; //< particular camera name.
    long int serialno;            //< Camera serial number, if appropriate.
    int      index;               //< Camera index, for multi-camera systems.
    double   RDF[9];              //< derived from forward right down matrix at startup.
    int      width;               //< Image width in pixels.
    int      height;              //< Image height in pixels.

    // parameters specific to this particular model.
    double            fx;  //< Horizontal focal length. fx = f/px; px is horizontal pixel size.
    double            fy;  //< Vertical focal length. fy = f/py; py is vertical pixel size.
    double            cx;  //< Horizontal image center in pixels.
    double            cy;  //< Vertical image center in pixels.
    double            sx;  //< Skew factor.
} _linear_camera_t;


/// Warpped camera model struct.  This structure contains parameters used to
//  define a pinhole projection model as well as extra parameters for radial
//  and tangental distortion.
typedef struct {
    int      type;                //< Model type (see above).
    int      version;             //< MVL camera model version.
    char     name[MAX_NAME_LEN]; //< particular camera name.
    long int serialno;            //< Camera serial number, if appropriate.
    int      index;               //< Camera index, for multi-camera systems.
    double   RDF[9];              //< derived from forward right down matrix at startup.
    int      width;               //< Image width in pixels.
    int      height;              //< Image height in pixels.

    // parameters specific to this particular model
    double            fx;    //< Horizontal focal length. fx = f/px; px is horizontal pix. size.
    double            fy;    //< Vertical focal length. fy = f/py; py is vertical pixel size.
    double            cx;    //< Horizontal image center in pixels.
    double            cy;    //< Vertical image center in pixels.
    double            sx;    //< Skew factor.
    double            kappa1;//< Radial warping terms.
    double            kappa2;//< Radial warping terms.
    double            kappa3;//< Radial warping terms.
    double            tau1;  //< Tangental warping terms.
    double            tau2;  //< Tangental warping terms.
} _warped_camera_t;


/// Linear camera model with lookup-table struct.  This structure contains
//  parameters used to define a pinhole projection model. It also contains
//  a filename which points to a pre-computed lookup table for rectification.
typedef struct {
    int      type;                //< Model type (see above).
    int      version;             //< MVL camera model version.
    char     name[MAX_NAME_LEN]; //< particular camera name.
    long int serialno;            //< Camera serial number, if appropriate.
    int      index;               //< Camera index, for multi-camera systems.
    double   RDF[9];              //< derived from forward right down matrix at startup.
    int      width;               //< Image width in pixels.
    int      height;              //< Image height in pixels.

    // parameters specific to this particular model
    double            fx;    //< Horizontal focal length. fx = f/px; px is horizontal pix. size.
    double            fy;    //< Vertical focal length. fy = f/py; py is vertical pixel size.
    double            cx;    //< Horizontal image center in pixels.
    double            cy;    //< Vertical image center in pixels.
    double            sx;    //< Skew factor.
    Bi_Point2D_f_nob** pLUT; //< LUT for rectification.  See the GenerateBumblebeeLUT utility.
} _lut_camera_t;


/// Linear camera model with lookup-table struct.  This structure contains
//  parameters used to define a pinhole projection model. It also contains
//  a filename which points to a pre-computed lookup table for rectification.
typedef struct {
    int      type;                //< Model type (see above).
    int      version;             //< MVL camera model version.
    char     name[MAX_NAME_LEN]; //< particular camera name.
    long int serialno;            //< Camera serial number, if appropriate.
    int      index;               //< Camera index, for multi-camera systems.
    double   RDF[9];              //< derived from forward right down matrix at startup.
    int      width;               //< Image width in pixels.
    int      height;              //< Image height in pixels.

    // parameters specific to this particular model
    double            fx;    //< Horizontal focal length. fx = f/px; px is horizontal pix. size.
    double            fy;    //< Vertical focal length. fy = f/py; py is vertical pixel size.
    double            cx;    //< Horizontal image center in pixels.
    double            cy;    //< Vertical image center in pixels.
    double            sx;    //< Skew factor.
    Bi_Point2D_f_nob** pLUT; //< LUT for rectification.  See the GenerateBumblebeeLUT utility.
    char              lutfilename[NAME_MAX]; //< Filename of lookup table.
} _pre_version_7_lut_camera_t;



/// Main camera model structure is a union of all possible camera models. This
//  union is the primary abstraction for camera models.  Crucially, the first
//  structure with 6 fields overlaps with all sub-camera model structs.
typedef union _mvl_camera_t {
    struct {
        int      type;                //< Model type (see above).
        int      version;             //< MVL camera model version.
        char     name[MAX_NAME_LEN]; //< particular camera name.
        long int serialno;            //< Camera serial number, if appropriate.
        int      index;               //< Camera index, for multi-camera systems.
        double   RDF[9];              //< derived from forward right down matrix at startup.
        int      width;               //< Image width in pixels.
        int      height;              //< Image height in pixels.
    };

    _linear_camera_t   linear;       //< Linear camera model child type.
    _warped_camera_t   warped;       //< Warped camera model child type.
    _lut_camera_t      lut;          //< Lookup table camera model child type.
    _pre_version_7_lut_camera_t old_lut; //< Lookup table camera model child type.

    // Add new camera models here....

} mvl_camera_t;


/// Copies a camera model
mvl_camera_t* mvl_alloc_camera( int type );

/// Copies a camera model
mvl_camera_t* mvl_alloc_and_copy_camera( mvl_camera_t* pInputCamera );

/// Copies a camera model
void mvl_free_camera( mvl_camera_t* pCamera );

/// coordinate frame convention conversion functions
#include "camera_transform_cf.h"

/// 2d to 3d (i.e., a ray) functions.
#include "camera_2d_to_3d.h"

/// 3d to 2d projection functions.
#include "camera_3d_to_2d.h"

/// conversion functions.
#include "camera_convert.h"

// i/o functions.
#include "camera_model_io.h"

// misc functions.
//#include "camera_misc.h"

///   Specific camera model implementations

#include "camera_model_linear.h"
#include "camera_model_warped.h"
#include "camera_model_lut.h"

#endif

