/**
 *  @file camera_model_lut.h
 *
 *  Perspective projection camera model with lookup table 2d-to-3d and 3d-to-2d functions.
 *
 *  $Id: camera_model_lut.h 373 2008-02-26 13:55:20Z gsibley $
 */

#ifndef __CAMERA_LUT_PERSPECTIVE_CAMERA__
#define __CAMERA_LUT_PERSPECTIVE_CAMERA__



///////////////////////////////////////////////////////////////////////////////
/// This structure is used to build a LUT without requiring branching
//  when rectifying images. The values out of the image to the top left
//  pixels instead of having a test for out of bound access, the aim is
//  to avoid a branch in the code.
typedef struct _Bi_Point2D_f_nob 
{
    int v00;
    int v01;
    float one_m_c_mul_one_m_r;
    float c_mul_one_m_r;
    float one_m_c_mul_r;
    float c_mul_r;
} Bi_Point2D_f_nob;



///////////////////////////////////////////////////////////////////////////////
/// Project a 3D world point, pt, into the image pixel, px.  This function
//  implements standard pinhole projection routine WITHOUT warping.  The return
//  parameter dpx is a 2x3 Jacobian of the projection function wrt to the 3D
//  world point, pt.
void mvl_camera_lut_3d_to_2d(
        const double fx,     /**< Input: horizontal focal length */
        const double cx,     /**< Input: horizontal image center */
        const double fy,     /**< Input: vertical focal length */
        const double cy,     /**< Input: vertical image center */
        const double sx,     /**< Input: skew factor */
        const double *pt,    /**< Input: 3D world point to project */
        double *px,          /**< Output: projected 2D image point */
        double *dpx          /**< Output: 2x3 Jacobian wrt to the 3D point */
        );

///////////////////////////////////////////////////////////////////////////////
/// Project a 2D image point into a 3D ray using a standard projective model
//  camera; ray is a unit vector and dray is the 3x2 jacobian of the ray
//  equation.
void mvl_camera_lut_2d_to_3d(
        const double fx,     /**< Input: horizontal focal length */
        const double cx,     /**< Input: horizontal image center */
        const double fy,     /**< Input: vertical focal length */
        const double cy,     /**< Input: vertical image center */
        const double sx,     /**< Input: skew factor */
        const double *px,    /**< Input: pixel position */
        double *ray,         /**< Output: 3x1 unit ray */
        double *dray         /**< Output: 2x3 Jacobian wrt to 2D px */
        );


///////////////////////////////////////////////////////////////////////////////
/// Free bilinear lookup table.
void mvl_free_lut(
                    Bi_Point2D_f_nob** pLUT /**< Ouput: allocated mem */
                    );

///////////////////////////////////////////////////////////////////////////////
/// Allocate bilinear lookup table.
void mvl_alloc_lut(
                   int nWidth,              /**< Input: image width */
                   int nHeight,             /**< Input: image height */
                   Bi_Point2D_f_nob*** pLUT /**< Ouput: allocated mem */
                   );

///////////////////////////////////////////////////////////////////////////////
/// Allocate and copy a lookp table.
void mvl_alloc_and_copy_lut( Bi_Point2D_f_nob** pInputLUT,  /**< Input: LUT */
                             int nWidth,                    /**< Input: image width */
                             int nHeight,                   /**< Input: image height */
                             Bi_Point2D_f_nob*** pOutputLUT /**< Ouput: copied LUT */
                             );



#endif

