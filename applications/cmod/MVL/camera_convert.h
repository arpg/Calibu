/**
 *  @file camera_convert.h
 *
 *  Camera model conversion routines.  This file contains functions for
 *  converting from one camera model parameterization to another.
 *
 *  $Id: camera_convert.h 372 2008-02-26 12:17:07Z gsibley $
 */
#ifndef __CAMERA_CONVERT_H__
#define __CAMERA_CONVERT_H__

//#include <mvl/camera/camera.h>

/** create an Aerospace convention RDF coordinate frame matrix. */
void  mvl_camera_set_aero_rdf( double *RDF );

/** create a Computer Vision convention RDF coordinate frame matrix. */
void  mvl_camera_set_cv_rdf( double *RDF );

/**
 *  Create a 3x3 projection (K) matrix from individual intrinsic parameters.
 */
int mvl_camera_linear_to_projmat(
        const double fx,  /**< Input: horizontal focal length. */
        const double cx,  /**< Input: horizontal image center. */
        const double fy,  /**< Input: vertical focal length. */
        const double cy,  /**< Input: vertical image center. */
        const double sx,  /**< Input: skew factor. */
        double *K         /**< Output: 3x3 K matrix. */
        );

/**
 *  Create a camera model structure from individual intrinsic parameters.
 */
int mvl_camera_linear_to_model(
        const int width,  /**< Input: imager width. */
        const int height, /**< Input: imager height. */
        const double fx,  /**< Input: horizontal focal length. */
        const double cx,  /**< Input: horizontal image center. */
        const double fy,  /**< Input: vertical focal length. */
        const double cy,  /**< Input: vertical image center. */
        const double sx,  /**< Input: skew factor. */
        mvl_camera_t *camera
        );

/**
 *  Calculates the parameters for the inverse of the 3x3 projection (K) matrix.
 */
int mvl_camera_inv_projmat(
        const double fx,  /**< Input: horizontal focal length. */
        const double cx,  /**< Input: horizontal image center. */
        const double fy,  /**< Input: vertical focal length. */
        const double cy,  /**< Input: vertical image center. */
        const double sx,  /**< Input: skew factor. */
        double *ifx,      /**< Output: horizontal focal length. */
        double *icx,      /**< Output: horizontal image center. */
        double *ify,      /**< Output: vertical focal length. */
        double *icy,      /**< Output: vertical image center. */
        double *isx       /**< Output: skew factor. */
        );


/**
 *  Extract individual intrinsic parameters from a 3x3 projection (K) matrix
 *  (which implicitly uses the Computer Vision convention).
 */
int mvl_camera_projmat_to_linear(
        const double *K, /**< Input: 3x3 K matirx. */
        double *fx,      /**< Output: horizontal focal length. */
        double *cx,      /**< Output: horizontal image center. */
        double *fy,      /**< Output: vertical focal length. */
        double *cy,      /**< Output: vertical image center. */
        double *sx       /**< Output: skew factor. */
        );

/**
 *  Compute required intrinsic parameters for specified FOV.
 */
int mvl_camera_FOVXY_to_linear(
        const double fovx,   /**< Input: horizontal field of view. */
        const double fovy,   /**< Input: vertical field of view. */
        const double width,  /**< Input: image width. */
        const double height, /**< Input: image height. */
        double *fx,          /**< Output: horizontal focal length. */
        double *cx,          /**< Output: horizontal image center. */
        double *fy,          /**< Output: vertical focal length. */
        double *cy,          /**< Output: vertical image center. */
        double *sx           /**< Output: skew factor. */
        );

/**
 *  Compute required intrinsic parameters for specified FOV.
 */
int mvl_camera_FOVXY_to_model(
        const double fovx,   /**< Input: horizontal field of view. */
        const double fovy,   /**< Input: vertical field of view. */
        const double width,  /**< Input: image width. */
        const double height, /**< Input: image height. */
        mvl_camera_t* cmod /**< Output: camera model. */
        );

/**
 *  Extract K matrix from a camera model structure (implicitly uses the
 *  Computer Vision convention).
 */
int mvl_camera_model_to_projmat(
        const mvl_camera_t *camera_model, /**< Input: MVL camera model pointer. */
        double *K                         /**< Output: 3x3 "K matrix". */
        );


/**
 *  Convert a K matrix into a camera model structure.
 */
int mvl_camera_projmat_to_model(
        const double *K,           /**< Input: 3x3 K matrix. */
        const int width,           /**< Input: image width. */
        const int height,          /**< Input: image height. */
        mvl_camera_t *camera_model /**< Output: MVL camera model. */
        );

#endif

