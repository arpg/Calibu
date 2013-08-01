/**
 *  @file camera_transform_cf.h
 *
 *  Utility functions to convert vectors from some coordinate frame, specified
 *  by an RDF matrix, to the Computer Vision coordinate frame convention.
 *
 *  The RDF matrix is the matrix that transforms a vector X in the source
 *  coordinate frame to a vector X_cv in the Comptuer Vision frame.  The RDF
 *  matrix is composed of the Right, Down and Forward row vectors associated
 *  with a camera model.  Note that the inverse of RDF will transform from the
 *  CV convention back to the source convention.  Since RDF is an orthornrmal
 *  matrix the inverese is the same as the transpose.
 *
 *  The RDF is used, for example, to convert between the Computer Vision
 *  coordinate frame convention (in which +z is forward, +x is right, and +y is
 *  down) and the NASA Aerospace convention (in which +x is forward, +y is
 *  right, and +z is down).
 *
 *  $Id: camera_transform_cf.h 372 2008-02-26 12:17:07Z gsibley $
 */

#ifndef __CAMERA_TRANSFORM_CF_H__
#define __CAMERA_TRANSFORM_CF_H__

//#include <mvl/camera/camera.h> // This shouldn't be here

/****************************************************************************/
void transform_cf_3x1_vision_to_aero(
        const double *pcv,
        double *paero
        );

/****************************************************************************/
void transform_cf_2x3_vision_to_aero(
        const double *H,
        double *Hm
        );

/****************************************************************************/
void transform_cf_2x3_transpose_rotate(
        const double *H,
        const double *Tvs,
        double *Hm
        );

/****************************************************************************/
/** Called after projecting 2d-to-3d to move the results to the users frame.*/
void transform_cf_post_2d_to_3d(
        const double *RDF,     /**< Input:  */
        const double *hpose,   /**< Input:  */
        double *ray,           /**< Input/Output:  */
        double *dray           /**< Input/Output:  */
        );

/** Called after projecting 2d-to-3d to move the results to the users frame.*/
void transform_cf_vision_to_areo_post_2d_to_3d(
        const double *hpose,
        double *ray,
        double *dray
        );

/****************************************************************************/
/* Called before projecting 3d-to-2d, to move point xwp into CV frame. */
void transform_cf_pre_3d_to_2d(
        const mvl_camera_t *camera,/**< Input:  */
        const double *hpose,       /**< Input:  */
        const double *xwp,         /**< Input: point in users frame */
        double *xcp                /**< Output: point in CV frame */
        );

/****************************************************************************/
/** Called before projecting 3d-to-2d to convert a point to cv frame */
void transform_cf_aero_to_vision_pre_3d_to_2d(
        const double *hpose,       /**< Input:  */
        const double *xwp,         /**< Input: point in users frame */
        double *xcp                /**< Output: point in CV frame */
        );

/****************************************************************************/
/**
 * Called after 3d_to_2d to orient Jacobian dpx to users coordinate frame.
 * This guy flips coordinates and rotates by hpose.
 */
void transform_cf_vision_to_aero_post_3d_to_2d_point_jacobian(
        const double *hpose,       /**< Input:  4x4 pose*/
        const double *dpx,         /**< Input: jacobian from cv cam model */
        double *H                  /**< Output: transformed jacobian */
        );

/****************************************************************************/
/** Called after 3d_to_2d to orient Jacobian dpx to users coordinate frame. */
void transform_cf_post_3d_to_2d(
        const mvl_camera_t *camera,/**< Input:  */
        const double *hpose,       /**< Input:  */
        double *dpx                /**< Input/Output: */
        );
/****************************************************************************/
/** Called after 3d_to_2d to orient Jacobian dpx to users coordinate frame. */
void transform_cf_vision_to_aero_post_3d_to_2d(
        const double *hpose,       /**< Input:  */
        double *dpx                /**< Input/Output: */
        );

/****************************************************************************/
// user must pre-compute
// 1) H with mvl_camera_k_3d_to_2d_jacobian
// 2) Hmij with transform_cf_vision_to_areo_post_3d_to_2d_point_jacobian
// See SWF sensor model writeup for more.  Note H is already in the roboics
// frame (as opp9ose to the visoin frame), so there's no need to flip axes.c
void transform_cf_aero_post_3d_to_2d_pose_jacobian(
        const double *Twr,  // homogeneous matrix robot pose in frame w
        const double *Trc,  // homogeneous matrix camera pose in robot frame
        const double *xwp,  // world 3d point
        const double *H,    // jacobian of 3d_to_2d in areo convention
        const double *Hm,   // 2x3 jacobian of 3d_to_2d wrt xwp in world frame
        const double *trigterms, //< 6x1 vector = [sin(r), cos(r), sin(p), cos(p), sin(q), cos(q)]
        double *Hp          // 2x6 jacobian of 3d_to_2d wrt to Euler pose from Twr
        );

/****************************************************************************/
void transform_cf_pose_jacobian(
        const double *T,
        const double *x,
        const double *A,
        const double *Hm,
        const double *trigterms, //< 6x1 vector = [sin(r), cos(r), sin(p), cos(p), sin(q), cos(q)]
        double *Hp );

#if 0
/****************************************************************************/
inline void convert_ray_from_CV_frame(
        double *RDF,
        double *hpose,
        double *ray
        );

/****************************************************************************/
/* Computes Jacobian = R*inv(RDF)*H */
inline void convert_dray_from_CV_frame(
        double *RDF,
        double *hpose,
        double *dray
        );

/****************************************************************************/
inline void convert_point_from_CV_frame(
        char *RDF,
        double *pts  /**< Input/Output: Convert this vector inplace*/
        );

/****************************************************************************/
void convert_points_from_cv_frame(
        char *RDF,
        int n,
        double *pts  /**< Input/Output: 3xn array of 3d points. */
        );
#endif

#endif

