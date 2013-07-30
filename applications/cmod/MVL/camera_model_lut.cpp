/**
 *  @file camera_model_lut.c
 *
 *  Perspective projection camera model with lookup table 2d-to-3d and 3d-to-2d functions.
 *
 *  $Id: camera_model_lut.cpp 373 2008-02-26 13:55:20Z gsibley $
 */
#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <stdlib.h>
#include <string.h>

#include "camera.h"

///////////////////////////////////////////////////////////////////////////////
//void mvl_camera_lut_3d_to_2d(
//        const double fx,
//        const double cx,
//        const double fy,
//        const double cy,
//        const double sx,
//        const double *pt,
//        double *px,
//        double *dpx )
//{
//    // Just call the linear camera model 3d_to_2d function
//    // Assume users have rectified the images already...
//    mvl_camera_linear_3d_to_2d(
//         fx,
//         cx,
//         fy,
//         cy,
//         sx,
//         pt,
//         px,
//         dpx);
//}

///////////////////////////////////////////////////////////////////////////////
//void mvl_camera_lut_2d_to_3d(
//        const double fx,
//        const double cx,
//        const double fy,
//        const double cy,
//        const double sx,
//        const double *px,
//        double *ray,
//        double *dray )
//{
//    // Just call the linear camera model 2d_to_3d function
//    // Assume users have rectified the images already...
//    mvl_camera_linear_2d_to_3d(
//        fx,
//        cx,
//        fy,
//        cy,
//        sx,
//        px,
//        ray,
//        dray );
//}

///////////////////////////////////////////////////////////////////////////////
/// Free bilinear lookup table.
void mvl_free_lut(
                    Bi_Point2D_f_nob** pLUT /**< Ouput: allocated mem */
                    )
{
    free( pLUT[0] );
    free( pLUT );
}

///////////////////////////////////////////////////////////////////////////////
/// Allocate bilinear lookup table.
void mvl_alloc_lut(
                   int nWidth,              /**< Input: image width */
                   int nHeight,             /**< Input: image height */
                   Bi_Point2D_f_nob*** pLUT /**< Ouput: allocated mem */
                   )
{
    Bi_Point2D_f_nob* lut_array = (Bi_Point2D_f_nob*) malloc( sizeof(Bi_Point2D_f_nob) * nWidth * nHeight );

    // Linear memory but 'double' access
    *pLUT = (Bi_Point2D_f_nob**) malloc( sizeof(Bi_Point2D_f_nob*) * nHeight );

    Bi_Point2D_f_nob* ptr = lut_array;

    for( int r = 0; r < nHeight; r++ ) {
        (*pLUT)[r] = ptr;
        ptr += nWidth;
    }
}

///////////////////////////////////////////////////////////////////////////////
void mvl_alloc_and_copy_lut( Bi_Point2D_f_nob** pInputLUT,  /**< Input: LUT */
                             int nWidth,                    /**< Input: image width */
                             int nHeight,                   /**< Input: image height */
                             Bi_Point2D_f_nob*** pOutputLUT /**< Ouput: copied LUT */
                             )
{
    mvl_alloc_lut( nWidth, nHeight, pOutputLUT );
    memcpy( &(*pOutputLUT)[0][0], &pInputLUT[0][0], nWidth*nHeight*sizeof( Bi_Point2D_f_nob ) );
}


