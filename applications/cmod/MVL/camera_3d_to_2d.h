/**
 *  @file camera_3d_to_2d.h
 *
 *  Functions for projecting 3D points into 2D images.
 *
 *  $Id: camera_3d_to_2d.h 372 2008-02-26 12:17:07Z gsibley $
 */

#ifndef __CAMERA_3D_TO_2D_H__
#define __CAMERA_3D_TO_2D_H__

/**
 *  Project a 3D world point, pt, into the image pixel, px.  This is another
 *  standard projection routine.  dpx is a 2x3 Jacobian of the projection
 *  function wrt to the 3D world point, pt.
 *
 *  Note that this function is a "C wrapper" around the "pure C" interface. 
 */
void mvl_camera_3d_to_2d(
        const mvl_camera_t *camera, /**< Input: MVL camera model */ 
        const double *hpose,	    /**< Input: 4x4 homogeneous pose matrix */
        const double *pt,           /**< Input: 3D world point to project */
        double *px,                 /**< Output: 2D image pixel */
        double *dpx                 /**< Output: 2x3 Jacobian wrt to 3D pt */
        );

#endif

