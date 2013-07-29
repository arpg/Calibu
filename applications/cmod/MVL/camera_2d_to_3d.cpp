/**
 *  @file camera_2d_to_3d.c
 *
 *  Functions to project a pixel in an image back out along a ray into the 3D
 *  world (2D-to-3D).
 *
 *  $Id: camera_2d_to_3d.cpp 343 2008-02-14 12:05:37Z gsibley $
 */
#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <stdlib.h>
#include <string.h>

#include "matrix.h"

#include "camera.h"
#include "kinematics.h" /* for isvalid_hpose_d() */


/****************************************************************************/
//void mvl_camera_2d_to_3d(
//        const mvl_camera_t *camera,
//        const double *hpose,
//        const double *px,
//        double *ray,
//        double *dray )
//{
//#ifdef MVL_DEBUG
//    isvalid_hpose_d( hpose );
//#endif

//    // call appropriate 2d_to_3d projection function
//    switch( camera->type ){
//        /* add your favorite camera model here... */
//        case MVL_CAMERA_LINEAR:
//            mvl_camera_linear_2d_to_3d(
//                    camera->linear.fx,
//                    camera->linear.cx,
//                    camera->linear.fy,
//                    camera->linear.cy,
//                    camera->linear.sx,
//                    px, ray, dray );
//            break;
//        case MVL_CAMERA_WARPED:
//            mvl_camera_warped_2d_to_3d(
//                    camera->warped.fx,
//                    camera->warped.cx,
//                    camera->warped.fy,
//                    camera->warped.cy,
//                    camera->warped.sx,
//                    camera->warped.kappa1,
//                    camera->warped.kappa2,
//                    camera->warped.kappa3,
//                    camera->warped.tau1,
//                    camera->warped.tau2,
//                    px, ray, dray );
//            break;
//        case MVL_CAMERA_LUT:
//            mvl_camera_lut_2d_to_3d(
//                    camera->lut.fx,
//                    camera->lut.cx,
//                    camera->lut.fy,
//                    camera->lut.cy,
//                    camera->lut.sx,
//                    px, ray, dray );
//            break;
//        default:
//            printf("%s -- unknown camera model\n", __FUNCTION__);
//    }

//    // convert ray and dray to users coordinate frame and camera pose
//    transform_cf_post_2d_to_3d( camera->RDF, hpose, ray, dray );

//}

