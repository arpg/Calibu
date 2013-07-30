/**
 *  @file camera_model_warped.h
 *
 *  Warped camera model 2d-to-3d and 3d-to-2d functions.
 *
 *  $Id: camera_model_warped.h 373 2008-02-26 13:55:20Z gsibley $
 */

#ifndef __CAMERA_WARPED_PERSPECTIVE_CAMERA_H__
#define __CAMERA_WARPED_PERSPECTIVE_CAMERA_H__


/** 
 *  Lift an image point px to a 3D ray using the Tsai camera model.
 *  Standard projection routine, in which param ray is a unit vector
 *  and param dray is the 3x2 jacobian of the ray equation.
 *
 *  The input parameters are the inverse values from inv(K) @see
 *  mvl_camera_inv_projmat (this is to avoid the repeated inversion of
 *  K). The distortion function values correspond to the best inverse
 *  model. These values do not have an analytic formulations with
 *  respect to the initial model so a minimisation should be performed
 *  beforehand.
 *
 */
void mvl_camera_warped_2d_to_3d(const double ifx,       /**< Input: $ifx = 1/fx$ */
        const double icx,       /**< Input: $icx = (sx*cy-cx*fy)/(fx*fy)$ */
        const double ify,       /**< Input: $ify = 1/fy$ */
        const double icy,       /**< Input: $icy = -cy/fy$ */
        const double isx,       /**< Input: $isx = -sx/(fx*fy)$ */
        const double kappa1inv, /**< Input: first radial distortion parameter of inverse model */
        const double kappa2inv, /**< Input: second radial distortion parameter of inverse model */
        const double kappa3inv, /**< Input: third radial distortion parameter of inverse model */
        const double tau1inv,   /**< Input: first tangential distortion parameter of inverse model */
        const double tau2inv,   /**< Input: second tangential distortion parameter of inverse model */
        const double *px,       /**< Input: 2D image pixel */
        double *ray,            /**< Output: 3x1 unit ray */
        double *dray            /**< Output: 2x3 Jacobian wrt to 2D px */
        );
// DO NOT REMOVE THESE COMMENTS (USED BY MATWRAP)
//%input px(2,1) //%output ray(3,1) //%output dray(2,3)

/** 
 *  Project a 3D world point, pt, into the image pixel, px.  This function
 *  implements standard pinhole projection routine WITH warping.  The return
 *  parameter dpx is a 2x3 Jacobian of the projection function wrt to the 3D
 *  world point, pt.
 *
 */
void mvl_camera_warped_3d_to_2d(
        const double fx,     /**< Input: horizontal focal length */
        const double cx,     /**< Input: horizontal image center */
        const double fy,     /**< Input: vertical focal length */
        const double cy,     /**< Input: vertical image center */
        const double sx,     /**< Input: skew factor */
        const double kappa1, /**< Input: radial warping parameters */
        const double kappa2, /**< Input: radial warping parameters */
        const double kappa3, /**< Input: radial warping parameters */
        const double tau1,   /**< Input: tangental warping parameters */
        const double tau2,   /**< Input: tangental warping parameters */
        const double *pt,    /**< Input: 3D world point to project */
        double *px,          /**< Output: projected 2D image point */
        double *dpx          /**< Output: 2x3 Jacobian wrt to the 3D point*/
        );
// DO NOT REMOVE THESE COMMENTS (USED BY MATWRAP)
//%input pt(3,1) //%output px(2,1) //%output dpx(2,3)


/** 
 * \brief Distortion function with radial and tangential
 * components. The distortion is applied to input point (from the
 * normalised plane) and (optionally) a jacobian is calculated.
 */
void mvl_camera_warped_distortion(        
        const double kappa1, /**< Input: first radial distortion parameter*/
        const double kappa2, /**< Input: second radial distortion parameter*/
        const double kappa3, /**< Input: third radial distortion parameter*/
        const double tau1,   /**< Input: first tangential distortion parameter*/
        const double tau2,   /**< Input: second tangential distortion parameter*/
        const double *m_u,   /**< Input: undistorted (x,y) coordinates of point in the normalised plane*/
        double *m_d,         /**< Output: (x,y) coordinates after distortion*/
        double *dm_d         /**< Output: jacobian with respect to m_u, dxdmx=dm_d[0] dxdmy=dm_d[1] (...) to avoid calculation give NULL */
        );
// DO NOT REMOVE THESE COMMENTS (USED BY MATWRAP)
//%input m_u(2,1) //%output m_d(2,1) //%output dm_d(2,2)

/** 
 *  Lift an image point px to a 3D ray using the Tsai camera model and
 *  radial and tangential distortion.
 *
 *  The approach uses an iterative approach that is not guaranteed to
 *  converge. An initial estimate can be give in 'iray'
 *
 */
#ifdef HAVE_NETLIB // this guy relies on blas/lapack
int mvl_camera_warped_2d_to_3d_gauss(
        const double fx,     /**< Input: horizontal focal length */
        const double cx,     /**< Input: horizontal image center */
        const double fy,     /**< Input: vertical focal length */
        const double cy,     /**< Input: vertical image center */
        const double sx,     /**< Input: skew factor */
        const double kappa1, /**< Input: radial warping parameters */
        const double kappa2, /**< Input: radial warping parameters */
        const double kappa3, /**< Input: radial warping parameters */
        const double tau1,   /**< Input: tangental warping parameters */
        const double tau2,   /**< Input: tangental warping parameters */
        const double *px,    /**< Input: 2D image pixel */
        const int max_iter,  /**< Input: maximum number of iterations */
        const double max_row_error, /**< Input: maximum reprojection on row/x value */
        const double max_col_error, /**< Input: maximum reprojection on col/y value */
        double *ray,         /**< Output: 3x1 unit ray */
        double *iray         /**< Input: optional initial value for the ray (if NULL, the lifted value assuming no distortion will be used) */
        );
#endif

// DO NOT REMOVE THESE COMMENTS (USED BY MATWRAP)
//%input px(2,1) //%output ray(3,1) //%input iray(3,1)
#ifndef POINT_2D_D
#define POINT_2D_D
typedef struct {
    double x;
    double y;
} Point2D_d;
#endif 

#ifndef LINE_2D_D
#define LINE_2D_D
typedef struct {
    double beg_x;
    double beg_y;
    double end_x;
    double end_y;
} Line2D_d;
#endif 

/**
 * Calculates a lookup table to lift points from the image plane
 * (2d) to the normalised plane ("3d").  "lut" is a matrix that is
 * allocated as a continuous block of memory and indexed by row
 * and column (standard v,u coordinates)
 */
int mvl_camera_build_lut_2d_to_3d(
        const double fx,     /**< Input: horizontal focal length */
        const double cx,     /**< Input: horizontal image center */
        const double fy,     /**< Input: vertical focal length */
        const double cy,     /**< Input: vertical image center */
        const double sx,     /**< Input: skew factor */
        const double kappa1, /**< Input: radial warping parameters */
        const double kappa2, /**< Input: radial warping parameters */
        const double kappa3, /**< Input: radial warping parameters */
        const double tau1,   /**< Input: tangental warping parameters */
        const double tau2,   /**< Input: tangental warping parameters */
        const int image_width,
        const int image_height,
        const int max_iter,         /**< Input: maximum number of iterations */
        const double max_row_error, /**< Input: maximum reprojection on row/x value */
        const double max_col_error, /**< Input: maximum reprojection on col/y value */
        Point2D_d ***lut                    /**< Output: image/matrix of size 2*sizeof(double)*width*height containing the (x,y) ray coordinates aligned in memory*/
        );

#endif

