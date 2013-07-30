/** 
 *  @file camera_2d_to_3d.h
 * 
 *  Functions to project a pixel in an image back out along a ray into the 3D
 *  world (2D-to-3D).
 *
 *  $Id: camera_2d_to_3d.h 372 2008-02-26 12:17:07Z gsibley $
 */
#ifndef __CAMERA_2D_TO_3D_H__
#define __CAMERA_2D_TO_3D_H__

/** 
 *  Project an image point px into a 3D ray using the camera model camera.
 *  Standard projection routine, in which ray is a unit vector and dray is the
 *  3x2 jacobian of the ray equation.
 *
 *  Before calling specialliazd functions, this function converts from the frame
 *  specified in the camera model to the cannonical computer vision coordinate
 *  frame (it also converts the results back).  These camera model functions are
 *  implemented in the models directory.
 *
 *  Note that this function is a "C wrapper" around the "pure C" interface. 
 */
void mvl_camera_2d_to_3d(
        const mvl_camera_t *camera, /**< Input: MVL camera model */
        const double *hpose,        /**< Input: 4x4 homogeneous pose matrix */
        const double *px,           /**< Input: 2D image pixel */
        double *ray,                /**< Output: 3x1 unit ray */
        double *dray                /**< Output: 3x2 Jacobian wrt to 2D px */
        );

#endif

