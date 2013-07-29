/**
 *  @file camera_3d_to_2d.c
 *
 *  Functions for projecting 3D points into 2D images.
 *
 *  $Id: camera_3d_to_2d.cpp 358 2008-02-18 17:14:53Z gsibley $
 */
#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <stdlib.h>
#include <string.h>

#include "camera.h"
#include "kinematics.h"


/****************************************************************************/
/** This generic function figures out which specialized projection function to
 * call, based on the camera model type and the coordinate frame it is in.
 */
//void mvl_camera_3d_to_2d(
//        const mvl_camera_t* camera_model,
//        const double *hpose,
//        const double *pt,
//        double *px,
//        double *dpx
//        )
//{
//#ifdef MVL_DEBUG
//    if( !isvalid_hpose_d( hpose ) ){
//        printf("Error in 4x4 homogeneous pose matrix\n");
//        return;
//    }
//#endif

//    // move pt into CV frame
//    double xcv[3];
//    transform_cf_pre_3d_to_2d( camera_model, hpose, pt, xcv );

//    // call appropriate 3d_to_2d projection function
//    switch(camera_model->type){
//        case MVL_CAMERA_LINEAR:
//            mvl_camera_linear_3d_to_2d(
//                    camera_model->linear.fx,
//                    camera_model->linear.cx,
//                    camera_model->linear.fy,
//                    camera_model->linear.cy,
//                    camera_model->linear.sx,
//                    xcv, px, dpx );
//            break;
//        case MVL_CAMERA_WARPED:
//            mvl_camera_warped_3d_to_2d(
//                    camera_model->warped.fx,
//                    camera_model->warped.cx,
//                    camera_model->warped.fy,
//                    camera_model->warped.cy,
//                    camera_model->warped.sx,
//                    camera_model->warped.kappa1,
//                    camera_model->warped.kappa2,
//                    camera_model->warped.kappa3,
//                    camera_model->warped.tau1,
//                    camera_model->warped.tau2,
//                    xcv, px, dpx );
//            break;
//        case MVL_CAMERA_LUT:
//            mvl_camera_lut_3d_to_2d(
//                    camera_model->lut.fx,
//                    camera_model->lut.cx,
//                    camera_model->lut.fy,
//                    camera_model->lut.cy,
//                    camera_model->lut.sx,
//                    xcv, px, dpx );
//            break;

//        default:
//            printf("camera_model_3d_to_2d() -- unsupported camera model\n");
//    }

//    // orient Jacobian to users coordinate frame
//    transform_cf_post_3d_to_2d( camera_model, hpose, dpx );
//}

