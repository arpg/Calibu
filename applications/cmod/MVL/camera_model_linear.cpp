/**
 *  @file camera_model_linear.c
 * 
 *  Perspective projection camera model 2d-to-3d and 3d-to-2d functions.
 *
 *  $Id: camera_model_linear.cpp 373 2008-02-26 13:55:20Z gsibley $
 */
#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <stdlib.h>
#include <string.h>

#include "camera.h"

/****************************************************************************/
bool mvl_camera_K_3d_to_2d(
        const double *K,
        const double *pt,
        double *px
        )
{
        /* K*pt */
        px[0] = K[0]*pt[0] + K[1]*pt[1] + K[2]*pt[2];
        px[1] =              K[4]*pt[1] + K[5]*pt[2];
        //xn[2] =                          pt[2];

        /* divide by w to get pixel location */
        px[0] /= pt[2];
        px[1] /= pt[2];

        return pt[2] > 0;
}

/****************************************************************************/
// compute jacobian of 3d_to_2d wrt 3d point
void mvl_camera_K_3d_to_2d_point_jacobian(
        const double *K,
        const double *pt,
        const double *px,
        double *dpx
        )
{
    // 2x3 jacobian of K*p/|K*p| taken wrt to p
    dpx[0] = K[0]/pt[2];
    dpx[1] = K[1]/pt[2];
    dpx[2] = (K[2] - px[0])/pt[2];

    dpx[3] = 0;
    dpx[4] = K[4]/pt[2];
    dpx[5] = (K[5] - px[1])/pt[2];
}

/****************************************************************************/
void mvl_camera_linear_3d_to_2d(
        const double fx,
        const double cx,
        const double fy,
        const double cy,
        const double sx,
        const double *pt,
        double *px,
        double *dpx )
{

    // TODO; move this clipping crap out of here, and do it right anyway
    if( pt[2] < 0 ){
        /* K*pt */
        px[0] = fx*pt[0] + sx*pt[1] - cx*pt[2];
        px[1] =            fy*pt[1] - cy*pt[2];
        //xn[2] =                          pt[2];

        /* divide by w to get pixel location */
        px[0] /= -pt[2];
        px[1] /= -pt[2]; // arg! not so sure here
    }
    else{
        /* K*pt */
        px[0] = fx*pt[0] + sx*pt[1] + cx*pt[2];
        px[1] =            fy*pt[1] + cy*pt[2];
        //xn[2] =                          pt[2];

        /* divide by w to get pixel location */
        px[0] /= pt[2];
        px[1] /= pt[2];
    }

//    printf("pt = [%f %f %f],    px = %f %f\n", pt[0], pt[1], pt[2], px[0], px[1] );
//    printf(" px = %f %f\n", px[0], px[1] );
 
    /* compute Jacobian? */
    if( dpx ){
        dpx[0] = fx/pt[2];
        dpx[1] = sx/pt[2];
        dpx[2] = (cx - px[0])/pt[2];

        dpx[3] = 0;
        dpx[4] = fy/pt[2];
        dpx[5] = (cy - px[1])/pt[2];
    }
}

/****************************************************************************/
// TODO remove jacobians from these guys??
void mvl_camera_linear_2d_to_3d(
        const double fx,    
        const double cx,
        const double fy,    
        const double cy,    
        const double sx,    
        const double *px,   
        double *ray, 
        double *dray ) 
{
    double xn[2], dxn, nxn;
/*
     inv(K) = [ 1/fx, -sx/fx/fy, (sx*cy-cx*fy)/fx/fy  ;...
              [    0,      1/fy,              -cy/fy ;...
              [    0,         0,                   1 ];
*/
    // hmm, maybe should pre-compute these?
    double iK11 = 1/fx;
    double iK12 = -sx/fx/fy;
    double iK13 = (sx*cy-cx*fy)/fx/fy;
    double iK22 = 1/fy;
    double iK23 = -cy/fy;

    /* compute ray */
    xn[0] = iK11*px[0] + iK12*px[1] + iK13;
    xn[1] =              iK22*px[1] + iK23;
//    xn[2] =                              1;

    /* normalize ray */
    dxn = xn[0]*xn[0] + xn[1]*xn[1] + 1; // dot(xn,xn);
    nxn = sqrt( dxn ); // norm(xn)
    ray[0] = xn[0]/nxn;
    ray[1] = xn[1]/nxn;
    ray[2] = 1/nxn;

    /* compute Jacobian? */
    if( dray ){
        // 3x2 jacobian of inverse projection wrt to pt
        // iK = invK(1:3,1:2);
        // dray = (iK*normxn - (ray*xn.')*iK)/dotxn;
        // See the writeup camera_math.pdf in the docs dir for more.

        dray[0] = -iK11*(-nxn+ray[0]*xn[0])/dxn;
        dray[1] = -(-nxn*iK12+ray[0]*xn[0]*iK12+ray[0]*xn[1]*iK22)/dxn;

        dray[2] = -ray[1]*xn[0]*iK11/dxn;
        dray[3] = -(-nxn*iK22+ray[1]*xn[0]*iK12+ray[1]*xn[1]*iK22)/dxn;

        dray[4] = -ray[2]*xn[0]*iK11/dxn;
        dray[5] = -ray[2]*(xn[0]*iK12+xn[1]*iK22)/dxn;
    }
}

#if 0
/****************************************************************************/
void mvl_camera_K_2d_to_3d(
        double *hpose,
        double *K,
        double *ray,         /**< Output: 3x1 unit ray */
        double *dray         /**< Output: 2x3 Jacobian wrt to 2D px */
        )
{
    double xn[2], dxn, nxn;
/*
     inv(K) = [ 1/fx, -sx/fx/fy, (sx*cy-cx*fy)/fx/fy  ;...
              [    0,      1/fy,              -cy/fy ;...
              [    0,         0,                   1 ];
*/
    // hmm, maybe should pre-compute these?
    double iK11 = 1/K[0];
    double iK12 = -K[1]/K[0]/K[4];
    double iK13 = (K[1]*K[5]-K[2]*K[4])/K[0]/K[4];
    double iK22 = 1/K[4];
    double iK23 = -K[5]/K[4];

    /* compute ray */
    xn[0] = iK11*px[0] + iK12*px[1] + iK13;
    xn[1] =              iK22*px[1] + iK23;
//    xn[2] =                              1;

    /* normalize ray */
    dxn = xn[0]*xn[0] + xn[1]*xn[1] + 1; // dot(xn,xn);
    nxn = sqrt( dxn ); // norm(xn)
    ray[0] = xn[0]/nxn;
    ray[1] = xn[1]/nxn;
    ray[2] = 1/nxn;

    /* compute Jacobian? */
    if( dray ){
        // 3x2 jacobian of inverse projection wrt to pt
        // iK = invK(1:3,1:2);
        // dray = (iK*normxn - (ray*xn.')*iK)/dotxn;
        // See the writeup camera_math.pdf in the docs dir for more.

        dray[0] = -iK11*(-nxn+ray[0]*xn[0])/dxn;
        dray[1] = -(-nxn*iK12+ray[0]*xn[0]*iK12+ray[0]*xn[1]*iK22)/dxn;

        dray[2] = -ray[1]*xn[0]*iK11/dxn;
        dray[3] = -(-nxn*iK22+ray[1]*xn[0]*iK12+ray[1]*xn[1]*iK22)/dxn;

        dray[4] = -ray[2]*xn[0]*iK11/dxn;
        dray[5] = -ray[2]*(xn[0]*iK12+xn[1]*iK22)/dxn;
    }

}
#endif

