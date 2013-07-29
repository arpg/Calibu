/**
 *  @file camera_misc.h
 *
 *  Miscellaneous camera helper functions.
 *
 *  $Id: camera_misc.h 372 2008-02-26 12:17:07Z gsibley $
 */ 
#ifndef __CAMERA_MISC_H__
#define __CAMERA_MISC_H__

#include <mvl/camera/camera.h>

/* Misc printing functions */

/****************************************************************************/
/**
 *  Print camera model to output file stream.
 */
void mvl_camera_fprint(
        FILE *file,        /**< Input: file stream */
        mvl_camera_t *cam  /**< Input: MVL camera model */
        );

/****************************************************************************/
/**
 *  Print linear camera model to output file stream.
 */
void mvl_camera_fprint_linear(
        FILE *file,       /**< Input: file stream */  
        mvl_camera_t *cam /**< Input: MVL camera model */
        );

/****************************************************************************/
/**
 *  Print warped camera model to output file stream.
 */
void mvl_camera_fprint_warped(
        FILE *file,        /**< Input: file stream */ 
        mvl_camera_t *cam  /**< Input: MVL camera model */
        );

/****************************************************************************/
/**
 *  Print linear (LUT) camera model to output file stream.
 */
void mvl_camera_fprint_lut(
        FILE *file,       /**< Input: file stream */  
        mvl_camera_t *cam /**< Input: MVL camera model */
        );


#endif

