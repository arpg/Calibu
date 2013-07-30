/**
 *  @file camera_transform_cf.c
 *
 *  Utility functions to convert vectors from some coordinate frame, specified
 *  by an RDF matrix, to and from  the Computer Vision coordinate frame
 *  convention.
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
 *  $Id: camera_transform_cf.cpp 372 2008-02-26 12:17:07Z gsibley $
 */
#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <stdlib.h>
#include <string.h>

#include "camera.h"
#include "kinematics.h"

/* TODO speedup: for the most common conventions, do element swapping directly,
 instead of multilying by RDF.
#define AERO_TO_CV( ray ) { \
        double tmp;         \
        tmp = ray[0];       \
        ray[0] = ray[1];    \
        ray[1] = ray[2];    \
        ray[2] = tmp;   }
*/

/****************************************************************************/
// swap coordinates of a 3x1 point - i.e. compute pnew = M^-1*p, where M is a
// permutation matrix mapping from aero to cv.
void transform_cf_3x1_vision_to_aero( const double *pcv, double *paero )
{
    assert( paero != pcv );

    paero[0] = pcv[2];
    paero[1] = pcv[0];
    paero[2] = pcv[1];

    paero[3] = pcv[5];
    paero[4] = pcv[3];
    paero[5] = pcv[4];
}

/****************************************************************************/
// swap coordinates of a 2x3 matrix J, i.e. compute Jnew = J*M, where M is a permutation matrix
void transform_cf_2x3_vision_to_aero( const double *H, double *Hm )
{
    assert( H != Hm );

    Hm[0] = H[2];
    Hm[1] = H[0];
    Hm[2] = H[1];

    Hm[3] = H[5];
    Hm[4] = H[3];
    Hm[5] = H[4];
}

/****************************************************************************/
void transform_cf_2x3_transpose_rotate(
        const double *H,
        const double *T,
        double *Hm
        )
{
    assert( H != Hm );
    Hm[0] = H[0]*T[0] + H[1]*T[1] + H[2]* T[2];
    Hm[3] = H[3]*T[0] + H[4]*T[1] + H[5]* T[2];

    Hm[1] = H[0]*T[4] + H[1]*T[5] + H[2]* T[6];
    Hm[4] = H[3]*T[4] + H[4]*T[5] + H[5]* T[6];

    Hm[2] = H[0]*T[8] + H[1]*T[9] + H[2]*T[10];
    Hm[5] = H[3]*T[8] + H[4]*T[9] + H[5]*T[10];
}

/****************************************************************************/
/** Called after projecting 2d-to-3d to move the results to the users frame.*/
void transform_cf_post_2d_to_3d(
        const double *RDF,
        const double *hpose,
        double *ray,
        double *dray
        )
{
    double tmp[6];

    /* tmp = inv(RDF)*ray */
    tmp[0] = RDF[0]*ray[0] + RDF[3]*ray[1] + RDF[6]*ray[2];
    tmp[1] = RDF[1]*ray[0] + RDF[4]*ray[1] + RDF[7]*ray[2];
    tmp[2] = RDF[2]*ray[0] + RDF[5]*ray[1] + RDF[8]*ray[2];

    /* ray = R*inv(RDF)*ray */
    ray[0] = hpose[0]*tmp[0] + hpose[1]*tmp[1] +  hpose[2]*tmp[2];
    ray[1] = hpose[4]*tmp[0] + hpose[5]*tmp[1] +  hpose[6]*tmp[2];
    ray[2] = hpose[8]*tmp[0] + hpose[9]*tmp[1] + hpose[10]*tmp[2];

    if( dray ){
        /* convert col 1 from CV frame */
        tmp[0] = RDF[0]*dray[0] + RDF[3]*dray[2] + RDF[6]*dray[4];
        tmp[2] = RDF[1]*dray[0] + RDF[4]*dray[2] + RDF[7]*dray[4];
        tmp[4] = RDF[2]*dray[0] + RDF[5]*dray[2] + RDF[8]*dray[4];

        /* convert col 2 from CV frame */
        tmp[1] = RDF[0]*dray[1] + RDF[3]*dray[3] + RDF[6]*dray[5];
        tmp[3] = RDF[1]*dray[1] + RDF[4]*dray[3] + RDF[7]*dray[5];
        tmp[5] = RDF[2]*dray[1] + RDF[5]*dray[3] + RDF[8]*dray[5];

        /* orient cols of a 3x2 Jacobian, R*Jac, according to camera pose */
        /* col 1 */
        dray[0] = hpose[0]*tmp[0] + hpose[1]*tmp[2] +  hpose[2]*tmp[4];
        dray[2] = hpose[4]*tmp[0] + hpose[5]*tmp[2] +  hpose[6]*tmp[4];
        dray[4] = hpose[8]*tmp[0] + hpose[9]*tmp[2] + hpose[10]*tmp[4];

        /* col 2 */
        dray[1] = hpose[0]*tmp[1] + hpose[1]*tmp[3] +  hpose[2]*tmp[5];
        dray[3] = hpose[4]*tmp[1] + hpose[5]*tmp[3] +  hpose[6]*tmp[5];
        dray[5] = hpose[8]*tmp[1] + hpose[9]*tmp[3] + hpose[10]*tmp[5];
    }
}

/****************************************************************************/
/** Called after projecting 2d-to-3d to move the results to the users frame.*/
void transform_cf_vision_to_areo_post_2d_to_3d(
        const double *hpose,
        double *ray,
        double *dray
        )
{
    double tmp[6];

    /*
       inv(RDF) = [ 0 0 1
                    1 0 0
                    0 1 0 ]
     */

    /* tmp = inv(RDF)*ray */
    tmp[0] = ray[2];
    tmp[1] = ray[0];
    tmp[2] = ray[1];

    /* ray = R*inv(RDF)*ray */
    ray[0] = hpose[0]*tmp[0] + hpose[1]*tmp[1] +  hpose[2]*tmp[2];
    ray[1] = hpose[4]*tmp[0] + hpose[5]*tmp[1] +  hpose[6]*tmp[2];
    ray[2] = hpose[8]*tmp[0] + hpose[9]*tmp[1] + hpose[10]*tmp[2];

    if( dray ){
        /* convert col 1 from CV frame */
        tmp[0] = dray[4];
        tmp[2] = dray[0];
        tmp[4] = dray[2];

        /* convert col 2 from CV frame */
        tmp[1] = dray[5];
        tmp[3] = dray[1];
        tmp[5] = dray[3];

        /* orient cols of a 3x2 Jacobian, R*Jac, according to camera pose */
        /* col 1 */
        dray[0] = hpose[0]*tmp[0] + hpose[4]*tmp[2] +  hpose[8]*tmp[4];
        dray[2] = hpose[1]*tmp[0] + hpose[5]*tmp[2] +  hpose[9]*tmp[4];
        dray[4] = hpose[2]*tmp[0] + hpose[6]*tmp[2] + hpose[10]*tmp[4];

        /* col 2 */
        dray[1] = hpose[0]*tmp[1] + hpose[4]*tmp[3] +  hpose[8]*tmp[5];
        dray[3] = hpose[1]*tmp[1] + hpose[5]*tmp[3] +  hpose[9]*tmp[5];
        dray[5] = hpose[2]*tmp[1] + hpose[6]*tmp[3] + hpose[10]*tmp[5];
    }
}



/****************************************************************************/
// user must pre-compute
// 1) H with mvl_camera_k_3d_to_2d_jacobian
// 2) Hmij with transform_cf_vision_to_aero_post_3d_to_2d_point_jacobian
// See SWF sensor model writeup for more.  Note H is already in the robotics
// frame (as oppose to the vision frame), so there's no need to flip axes.
//void transform_cf_aero_post_3d_to_2d_pose_jacobian(
//        const double* Twr,  // homogeneous matrix robot pose in frame w
//        const double* ,  // TODO: Trc != I case is broken...? Check this. homogeneous matrix camera pose in robot frame
//        const double* xwp,  // world 3d point
//        const double* H,    // jacobian of 3d_to_2d in aero convention
//        const double* Hm,   // 2x3 jacobian of 3d_to_2d wrt xwp in world frame
//        const double* trigterms, //< 6x1 vector = [sin(r), cos(r), sin(p), cos(p), sin(q), cos(q)]
//        double* Hp          // 2x6 jacobian of 3d_to_2d wrt to Euler pose from Twr
//        )
//{
//    double J[9], dx[3];

//    // first 2x3 block is just -Hmij
//    Hp[0] = -Hm[0];
//    Hp[6] = -Hm[3];

//    Hp[1] = -Hm[1];
//    Hp[7] = -Hm[4];

//    Hp[2] = -Hm[2];
//    Hp[8] = -Hm[5];

//    // next 2x3 block is more complicated
//    // we need dR/dtheta * (xwp - Trw)
//    dx[0] = xwp[0] - Twr[3];
//    dx[1] = xwp[1] - Twr[7];
//    dx[2] = xwp[2] - Twr[11];
//    dRTdrpy_times_vector( Twr, dx, trigterms, J );

////    multmm_3d( H, J, Hpij );
//    Hp[3] = H[0]*J[0] + H[1]*J[3] + H[2]*J[6];
//    Hp[9] = H[3]*J[0] + H[4]*J[3] + H[5]*J[6];

//    Hp[4]  = H[0]*J[1] + H[1]*J[4] + H[2]*J[7];
//    Hp[10] = H[3]*J[1] + H[4]*J[4] + H[5]*J[7];

//    Hp[5]  = H[0]*J[2] + H[1]*J[5] + H[2]*J[8];
//    Hp[11] = H[3]*J[2] + H[4]*J[5] + H[5]*J[8];
//}

/****************************************************************************/
//void transform_cf_pose_jacobian(
//        const double *T,
//        const double *x,
//        const double *A,
//        const double *Hm,
//        const double *trigterms, //< 6x1 vector = [sin(r), cos(r), sin(p), cos(p), sin(q), cos(q)]
//        double *Hp )
//{
//    // first 2x3 block is just -Hmij
//    Hp[0] = -Hm[0];
//    Hp[6] = -Hm[3];

//    Hp[1] = -Hm[1];
//    Hp[7] = -Hm[4];

//    Hp[2] = -Hm[2];
//    Hp[8] = -Hm[5];

//    // next 2x3 block is more complicated
//    // we need dR/dtheta * (xwp - Trw)
//    double dx[3];
//    dx[0] = x[0] - T[3];
//    dx[1] = x[1] - T[7];
//    dx[2] = x[2] - T[11];
//    double J[9];
//    dRTdrpy_times_vector( T, dx, trigterms, J );

//    // now just compute A*J
//    Hp[3]  = A[0]*J[0] + A[1]*J[3] + A[2]*J[6];
//    Hp[9]  = A[3]*J[0] + A[4]*J[3] + A[5]*J[6];

//    Hp[4]  = A[0]*J[1] + A[1]*J[4] + A[2]*J[7];
//    Hp[10] = A[3]*J[1] + A[4]*J[4] + A[5]*J[7];

//    Hp[5]  = A[0]*J[2] + A[1]*J[5] + A[2]*J[8];
//    Hp[11] = A[3]*J[2] + A[4]*J[5] + A[5]*J[8];
//}

/****************************************************************************/
/** Called before projecting 3d-to-2d to convert a point to cv frame */
void transform_cf_aero_to_vision_pre_3d_to_2d(
        const double *hpose,       /**< Input:  */
        const double *xwp,         /**< Input: point in users frame */
        double *xcp                /**< Output: point in CV frame */
        )
{

    /* for vision to aero we use

       RDF = [ 0 1 0;
       0 0 1;
       1 0 0 ];
     */

    /* First, convert to CV convention: xcv = FRD*R'*pt */
    // See camera_math.pdf in the docs dir for more info.
    double tmp[3];

    /* xwp - twc */
    xcp[0] = xwp[0] - hpose[3];
    xcp[1] = xwp[1] - hpose[7];
    xcp[2] = xwp[2] - hpose[11];

    // Rwc'*(xwp-twc)
    tmp[0] = hpose[0]*xcp[0] + hpose[4]*xcp[1] + hpose[8]*xcp[2];
    tmp[1] = hpose[1]*xcp[0] + hpose[5]*xcp[1] + hpose[9]*xcp[2];
    tmp[2] = hpose[2]*xcp[0] + hpose[6]*xcp[1] + hpose[10]*xcp[2];

    // FRD*Rwc'*(xwp-twc)
    xcp[0] = tmp[1];
    xcp[1] = tmp[2];
    xcp[2] = tmp[0];
}

/****************************************************************************/
/** Called before projecting 3d-to-2d to convert a point to cv frame */
void transform_cf_pre_3d_to_2d(
        const mvl_camera_t *camera,/**< Input:  */
        const double *hpose,       /**< Input:  */
        const double *xwp,         /**< Input: point in users frame */
        double *xcp                /**< Output: point in CV frame */
        )
{
    /* First, convert to CV convention: xcv = FRD*R'*pt */
    // See camera_math.pdf in the docs dir for more info.
    const double *RDF = camera->RDF;
    double tmp[3];

    /* xwp - twc */
    xcp[0] = xwp[0] - hpose[3];
    xcp[1] = xwp[1] - hpose[7];
    xcp[2] = xwp[2] - hpose[11];

    // Rwc'*(xwp-twc)
    tmp[0] = hpose[0]*xcp[0] + hpose[4]*xcp[1] + hpose[8]*xcp[2];
    tmp[1] = hpose[1]*xcp[0] + hpose[5]*xcp[1] + hpose[9]*xcp[2];
    tmp[2] = hpose[2]*xcp[0] + hpose[6]*xcp[1] + hpose[10]*xcp[2];

    // FRD*Rwc'*(xwp-twc)
    xcp[0] = RDF[0]*tmp[0] + RDF[1]*tmp[1] + RDF[2]*tmp[2];
    xcp[1] = RDF[3]*tmp[0] + RDF[4]*tmp[1] + RDF[5]*tmp[2];
    xcp[2] = RDF[6]*tmp[0] + RDF[7]*tmp[1] + RDF[8]*tmp[2];
}


/****************************************************************************/
/**
 * Called after 3d_to_2d to orient Jacobian dpx to users coordinate frame.
 * This guy flips coordinates and rotates by hpose.
 */
void transform_cf_vision_to_aero_post_3d_to_2d_point_jacobian(
        const double *hpose,       /**< Input:  4x4 pose*/
        const double *dpx,         /**< Input: jacobian from cv cam model */
        double *H                  /**< Output: transformed jacobian */
        )
{
    double tmp[6];

    /* for vision to aero we use

       RDF = [ 0 1 0;
       0 0 1;
       1 0 0 ];
     */

    // transform jacobian to external CF
    // Compute H*RDF
    // first row of H*RDF
    tmp[0] = dpx[2];
    tmp[1] = dpx[0];
    tmp[2] = dpx[1];

    // second row H*RDF
    tmp[3] = dpx[5];
    tmp[4] = dpx[3];
    tmp[5] = dpx[4];

    // finish computing H*RDF*Rwc'
    // first row
    H[0] = tmp[0]*hpose[0] + tmp[1]*hpose[1] + tmp[2]*hpose[2];
    H[1] = tmp[0]*hpose[4] + tmp[1]*hpose[5] + tmp[2]*hpose[6];
    H[2] = tmp[0]*hpose[8] + tmp[1]*hpose[9] + tmp[2]*hpose[10];

    // second row
    H[3] = tmp[3]*hpose[0] + tmp[4]*hpose[1] + tmp[5]*hpose[2];
    H[4] = tmp[3]*hpose[4] + tmp[4]*hpose[5] + tmp[5]*hpose[6];
    H[5] = tmp[3]*hpose[8] + tmp[4]*hpose[9] + tmp[5]*hpose[10];
}




/****************************************************************************/
/** Called after 3d_to_2d to orient Jacobian dpx to users coordinate frame. */
void transform_cf_post_3d_to_2d(
        const mvl_camera_t *camera,/**< Input:  */
        const double *hpose,       /**< Input:  */
        double *dpx                /**< Input/Output: */
        )
{
    if( dpx ){
        const double *RDF = camera->RDF;
        double tmp[6];

        // transform jacobian to external CF
        // Compute H*RDF
        // first row of H*RDF
        tmp[0] = dpx[0]*RDF[0] + dpx[1]*RDF[3] + dpx[2]*RDF[6];
        tmp[1] = dpx[0]*RDF[1] + dpx[1]*RDF[4] + dpx[2]*RDF[7];
        tmp[2] = dpx[0]*RDF[2] + dpx[1]*RDF[5] + dpx[2]*RDF[8];

        // second row H*RDF
        tmp[3] = dpx[3]*RDF[0] + dpx[4]*RDF[3] + dpx[5]*RDF[6];
        tmp[4] = dpx[3]*RDF[1] + dpx[4]*RDF[4] + dpx[5]*RDF[7];
        tmp[5] = dpx[3]*RDF[2] + dpx[4]*RDF[5] + dpx[5]*RDF[8];

        // finish computing H*RDF*Rwc'
        // first row
        dpx[0] = tmp[0]*hpose[0] + tmp[1]*hpose[1] + tmp[2]*hpose[2];
        dpx[1] = tmp[0]*hpose[4] + tmp[1]*hpose[5] + tmp[2]*hpose[6];
        dpx[2] = tmp[0]*hpose[8] + tmp[1]*hpose[9] + tmp[2]*hpose[10];

        // second row
        dpx[3] = tmp[3]*hpose[0] + tmp[4]*hpose[1] + tmp[5]*hpose[2];
        dpx[4] = tmp[3]*hpose[4] + tmp[4]*hpose[5] + tmp[5]*hpose[6];
        dpx[5] = tmp[3]*hpose[8] + tmp[4]*hpose[9] + tmp[5]*hpose[10];
    }
}


#if 0
/****************************************************************************/
inline void convert_ray_from_CV_frame(
        double *RDF,
        double *hpose,
        double *ray
        )
{
    double tmp[3];

    /* tmp = inv(RDF)*ray */
    tmp[0] = RDF[0]*ray[0] + RDF[1]*ray[1] + RDF[2]*ray[2];
    tmp[1] = RDF[3]*ray[0] + RDF[4]*ray[1] + RDF[5]*ray[2];
    tmp[2] = RDF[6]*ray[0] + RDF[7]*ray[1] + RDF[8]*ray[2];

    /* ray = R*inv(RDF)*ray */
    ray[0] = hpose[0]*tmp[0] + hpose[1]*tmp[1] +  hpose[2]*tmp[2];
    ray[1] = hpose[4]*tmp[0] + hpose[5]*tmp[1] +  hpose[6]*tmp[2];
    ray[2] = hpose[8]*tmp[0] + hpose[9]*tmp[1] + hpose[10]*tmp[2];
}

/****************************************************************************/
/* Computes Jacobian = R*inv(RDF)*H */
inline void convert_dray_from_CV_frame(
        double *RDF,
        double *hpose,
        double *dray
        )
{
    double tmp[6];

    /* convert col 1 from CV frame */
    tmp[0] = RDF[0]*dray[0] + RDF[3]*dray[2] + RDF[6]*dray[4];
    tmp[2] = RDF[1]*dray[0] + RDF[4]*dray[2] + RDF[7]*dray[4];
    tmp[4] = RDF[2]*dray[0] + RDF[5]*dray[2] + RDF[8]*dray[4];

    /* convert col 2 from CV frame */
    tmp[1] = RDF[0]*dray[1] + RDF[3]*dray[3] + RDF[6]*dray[5];
    tmp[3] = RDF[1]*dray[1] + RDF[4]*dray[3] + RDF[7]*dray[5];
    tmp[5] = RDF[2]*dray[1] + RDF[5]*dray[3] + RDF[8]*dray[5];

    /* orient cols of a 3x2 jacobian, R*Jac, according to camera pose */
    /* col 1 */
    dray[0] = hpose[0]*tmp[0] + hpose[1]*tmp[2] +  hpose[2]*tmp[4];
    dray[2] = hpose[4]*tmp[0] + hpose[5]*tmp[2] +  hpose[6]*tmp[4];
    dray[4] = hpose[8]*tmp[0] + hpose[9]*tmp[2] + hpose[10]*tmp[4];

    /* col 2 */
    dray[1] = hpose[0]*tmp[1] + hpose[1]*tmp[3] +  hpose[2]*tmp[5];
    dray[3] = hpose[4]*tmp[1] + hpose[5]*tmp[3] +  hpose[6]*tmp[5];
    dray[5] = hpose[8]*tmp[1] + hpose[9]*tmp[3] + hpose[10]*tmp[5];
}

/****************************************************************************/
inline void convert_point_from_CV_frame(
        char *RDF,
        double *pts  /**< Input/Output: Convert this vector inplace*/
        )
{
    double tmp[3];

    tmp[0] = RDF[0]*pts[0] + RDF[3]*pts[1] + RDF[6]*pts[2];
    tmp[1] = RDF[1]*pts[0] + RDF[4]*pts[1] + RDF[7]*pts[2];
    tmp[2] = RDF[2]*pts[0] + RDF[5]*pts[1] + RDF[8]*pts[2];

    pts[0] = tmp[0];
    pts[1] = tmp[1];
    pts[2] = tmp[2];
}


/****************************************************************************/
void convert_points_from_CV_frame(
        char *RDF,
        int n,
        double *pts  /**< Input/Output: 3xn array of 3d points. */
        )
{
    double tmp[3];
    int ii;

    /* Convert points from Computer Vision cf to RDF cf */
    for( ii = 0; ii < n; ii++ ){
        tmp[0] = RDF[0]*pts[ii*n] + RDF[3]*pts[ii*n+1] + RDF[6]*pts[ii*n+2];
        tmp[1] = RDF[1]*pts[ii*n] + RDF[4]*pts[ii*n+1] + RDF[7]*pts[ii*n+2];
        tmp[2] = RDF[2]*pts[ii*n] + RDF[5]*pts[ii*n+1] + RDF[8]*pts[ii*n+2];

        pts[ii*n]   = tmp[0];
        pts[ii*n+1] = tmp[1];
        pts[ii*n+2] = tmp[2];
    }
}

/****************************************************************************/
void convert_point_to_CV_frame(
        char *RDF,
        double *pts  /**< Input/Output: Convert this vector inplace*/
        )
{
    double tmp[3];
    int ii;

    /* Convert points from RDF cf to Computer Vision cf*/
    for( ii = 0; ii < n; ii++ ){
        tmp[0] = RDF[0]*pts[ii*n] + RDF[1]*pts[ii*n+1] + RDF[2]*pts[ii*n+2];
        tmp[1] = RDF[3]*pts[ii*n] + RDF[4]*pts[ii*n+1] + RDF[5]*pts[ii*n+2];
        tmp[2] = RDF[6]*pts[ii*n] + RDF[7]*pts[ii*n+1] + RDF[8]*pts[ii*n+2];

        /* position point in the world */
        pts[ii*n]  = tmp[0]*hpose[0] + tmp[1]*hpose[1] + tmp[2]*hpose[2] + hpose[3];
        pts[ii*n+1] =tmp[0]*hpose[4] + tmp[1]*hpose[5] + tmp[2]*hpose[6] + hpose[7];
        pts[ii*n+2] =tmp[0]*hpose[8] + tmp[1]*hpose[9] + tmp[2]*hpose[10] + hpose[11];

        pts[ii*n]   = tmp[0];
        pts[ii*n+1] = tmp[1];
        pts[ii*n+2] = tmp[2];
    }
}
#endif

