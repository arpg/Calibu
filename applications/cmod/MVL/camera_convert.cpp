/**
 *  @file camera_convert.cpp  
 *
 *  Camera model conversion routines.  This file contains functions for
 *  converting from one camera model parameterization to another.
 *
 *  $Id: camera_convert.cpp 362 2008-02-25 15:10:09Z gsibley $
 */
#include <math.h>
#include <string.h>
#include <stdio.h>

#include "camera.h"

/****************************************************************************/
/** create an Aerospace convention RDF coordinate frame matrix */
void  mvl_camera_set_aero_rdf( double *RDF )
{
    RDF[0] = 0;
    RDF[1] = 1;
    RDF[2] = 0;

    RDF[3] = 0;
    RDF[4] = 0;
    RDF[5] = 1;

    RDF[6] = 1;
    RDF[7] = 0;
    RDF[8] = 0;
};

/****************************************************************************/
/** create a Computer Vision convention RDF coordinate frame matrix */
void  mvl_camera_set_cv_rdf( double *RDF )
{
    RDF[0] = 1;
    RDF[1] = 0;
    RDF[2] = 0;

    RDF[3] = 0;
    RDF[4] = 1;
    RDF[5] = 0;

    RDF[6] = 0;
    RDF[7] = 0;
    RDF[8] = 1;
};



/****************************************************************************/
int mvl_camera_FOVXY_to_linear(
        const double fovx,
        const double fovy,
        const double width,
        const double height,
        double *fx,
        double *cx,
        double *fy,
        double *cy,
        double *sx )
{
    /* focal lengths based on FOV and image plane size */
    *fx = width/2/tan( fovx/2 );
    *fy = height/2/tan( fovy/2 );

    /* principal point in exact center of image plane */
    *cx = width/2;
    *cy = height/2;

    /* no skew */
    *sx = 0;
    return 0;
}

/****************************************************************************/
int mvl_camera_linear_to_model(
        const int width, 
        const int height,
        const double fx,
        const double cx,
        const double fy,
        const double cy,
        const double sx,
        mvl_camera_t *camera
        )
{
    double K[9];
    int res = mvl_camera_linear_to_projmat( fx, cx, fy, cy, sx,K );
    if( res ){
        return res;
    }
    return mvl_camera_projmat_to_model( K, width, height, camera );
}

/****************************************************************************/
int mvl_camera_FOVXY_to_model(
        const double fovx,   /**< Input: horizontal field of view. */
        const double fovy,   /**< Input: vertical field of view. */
        const double width,  /**< Input: image width. */
        const double height, /**< Input: image height. */
        mvl_camera_t* cmod /**< Output: camera model. */
        )
{
    double fx;
    double cx;
    double fy;
    double cy;
    double sx;
    int res = mvl_camera_FOVXY_to_linear( fovx, fovy, width, height, &fx, &cx, &fy, &cy, &sx );
    if( res ){
        return res;
    }
    res = mvl_camera_linear_to_model( width, height, fx, cx, fy, cy, sx, cmod );
    return res;
}

/****************************************************************************/
int mvl_camera_model_to_projmat(
        const mvl_camera_t *camera,
        double *K )
{
    switch( camera->type ){
        case MVL_CAMERA_LUT:
        case MVL_CAMERA_LINEAR:
        case MVL_CAMERA_WARPED:
            mvl_camera_linear_to_projmat(
                    camera->linear.fx,
                    camera->linear.cx,
                    camera->linear.fy,
                    camera->linear.cy,
                    camera->linear.sx,        
                    K );
            break;
        default:
            printf("camera_model_to_projmat() -- unsupported camera model\n");
            return -1;
    }
    return 0;
}


/****************************************************************************/
int mvl_camera_projmat_to_model(
        const double *K,
        const int width, 
        const int height,
        mvl_camera_t *camera )
{
    /* only possilbe to goto a linear model anyway... */
    camera->linear.type = MVL_CAMERA_LINEAR;
    camera->linear.width = width;
    camera->linear.height = height;
    mvl_camera_projmat_to_linear( 
            K, 
            &camera->linear.fx,
            &camera->linear.cx,
            &camera->linear.fy,
            &camera->linear.cy,
            &camera->linear.sx );
    return 0;
}


/****************************************************************************/
int mvl_camera_linear_to_projmat(
        const double fx,
        const double cx,
        const double fy,
        const double cy,
        const double sx,        
        double *K )
{
    memset( K, 0, 9*sizeof(double) );
    K[0] = fx;
    K[1] = sx;
    K[2] = cx;
    K[4] = fy;
    K[5] = cy;
    K[8] = 1.0;
    return 0;
}

/****************************************************************************/
int mvl_camera_inv_projmat(const double fx, 
                            const double cx, 
                            const double fy, 
                            const double cy, 
                            const double sx, 
                            double *ifx,  
                            double *icx,  
                            double *ify,  
                            double *icy,  
                            double *isx) {
    *ifx = 1/fx;
    *icx = (sx*cy-cx*fy)/(fx*fy);
    *ify = 1/fy;
    *icy = -cy/fy;
    *isx = -sx/(fx*fy);
    return 0;
}


/****************************************************************************/
int mvl_camera_projmat_to_linear(
        const double *K, 
        double *fx,
        double *cx,
        double *fy,
        double *cy,
        double *sx )
{
    *fx = K[0];
    *sx = K[1];
    *cx = K[2];
    *fy = K[4];
    *cy = K[5];
    return 0;
}
