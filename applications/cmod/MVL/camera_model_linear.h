/**
 *  @file camera_model_linear.h
 *
 *  Perspective projection camera model 2d-to-3d and 3d-to-2d functions.
 *
 *  $Id: camera_model_linear.h 373 2008-02-26 13:55:20Z gsibley $
 */

#ifndef __CAMERA_PERSPECTIVE_CAMERA__
#define __CAMERA_PERSPECTIVE_CAMERA__

/****************************************************************************/
/** 
 *  Project a 3D world point, pt, into the image pixel, px.  This function
 *  implements standard pinhole projection routine WITHOUT warping.  The return
 *  parameter dpx is a 2x3 Jacobian of the projection function wrt to the 3D
 *  world point, pt.
 */
void mvl_camera_linear_3d_to_2d(
        const double fx,     /**< Input: horizontal focal length */
        const double cx,     /**< Input: horizontal image center */
        const double fy,     /**< Input: vertical focal length */
        const double cy,     /**< Input: vertical image center */
        const double sx,     /**< Input: skew factor */
        const double *pt,    /**< Input: 3D world point to project */
        double *px,          /**< Output: projected 2D image point */
        double *dpx          /**< Output: 2x3 Jacobian wrt to the 3D point */
        );

/****************************************************************************/
/** 
 *  Project a 2D image point into a 3D ray using a standard projective model
 *  camera; ray is a unit vector and dray is the 3x2 jacobian of the ray
 *  equation.
 */
void mvl_camera_linear_2d_to_3d(
        const double fx,     /**< Input: horizontal focal length */
        const double cx,     /**< Input: horizontal image center */
        const double fy,     /**< Input: vertical focal length */
        const double cy,     /**< Input: vertical image center */
        const double sx,     /**< Input: skew factor */
        const double *px,    /**< Input: pixel position */
        double *ray,         /**< Output: 3x1 unit ray */
        double *dray         /**< Output: 2x3 Jacobian wrt to 2D px */
        );

/****************************************************************************/
/** 
  * standard vision frame perspective projection function.
  */
bool mvl_camera_K_3d_to_2d( //< Return: true if projection infront of camera.
        const double *K,
        const double *pt,
        double *px
        );

/****************************************************************************/
/** 
  *  compute jacobian of 3d_to_2d wrt 3d point
  */
void mvl_camera_K_3d_to_2d_point_jacobian(
        const double *K,
        const double *pt,
        const double *px,
        double *dpx
        );

#if 0
/****************************************************************************/
/**
 * compute 2x6 jacobian of 3d_to_2d wrt 6x1 Euler Angle pose... here be dragons. 
 */
inline void mvl_camera_K_3d_to_2d_point_jacobian(
        const double *hpose,  // 4x4 homogeneous pose
        const double *K,
        const double *xm, // 3x1 3d point
        double *Jac
        );
#endif

#endif

