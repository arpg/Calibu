/**
 *  @file camera_misc.c
 *
 *  Miscellaneous camera helper functions.
 *
 *  $Id: camera_misc.cpp 343 2008-02-14 12:05:37Z gsibley $
 */ 
#include "camera.h"


/****************************************************************************/
void mvl_camera_fprint(
        FILE *file,
        mvl_camera_t *cam 
        )
{
    switch( cam->type ) {
        case MVL_CAMERA_LINEAR:
        case MVL_CAMERA_LUT:
            mvl_camera_fprint_linear( file, cam );
            break;
        case MVL_CAMERA_WARPED:
            mvl_camera_fprint_warped( file, cam );
            break;
        default:
            fprintf( file, "Unkown camera type!\n" );
            break;
    }
}


/****************************************************************************/
void mvl_camera_fprint_linear(
        FILE *file,
        mvl_camera_t *cam 
        )
{
    fprintf( file, "Camera: serial='%ld', name='%s' index='%d', "
            "version='%d',  type='Linear'):\n",
            cam->serialno,
            cam->name,
            cam->index,
            cam->version );

    fprintf( file, "    width   = %d\n", cam->linear.width );
    fprintf( file, "    height  = %d\n", cam->linear.height );
    fprintf( file, "    fx      = %f\n", cam->linear.fx );
    fprintf( file, "    fy      = %f\n", cam->linear.fy );
    fprintf( file, "    cx      = %f\n", cam->linear.cx );
    fprintf( file, "    cy      = %f\n", cam->linear.cy );
    fprintf( file, "    sx      = %f\n", cam->linear.sx );
}


/****************************************************************************/
void mvl_camera_fprint_warped(
        FILE *file,
        mvl_camera_t *cam 
        )
{
    fprintf( file, "Camera: serial='%ld', name='%s' index='%d', "
            "version='%d',  type='Linear'):\n",
            cam->serialno,
            cam->name,
            cam->index,
            cam->version );

    fprintf( file, "    width   = %d\n", cam->warped.width );
    fprintf( file, "    height  = %d\n", cam->warped.height );
    fprintf( file, "    fx      = %f\n", cam->warped.fx );
    fprintf( file, "    fy      = %f\n", cam->warped.fy );
    fprintf( file, "    cx      = %f\n", cam->warped.cx );
    fprintf( file, "    cy      = %f\n", cam->warped.cy );
    fprintf( file, "    sx      = %f\n", cam->warped.sx );
    fprintf( file, "    kappa1  = %f\n", cam->warped.kappa1 );
    fprintf( file, "    kappa2  = %f\n", cam->warped.kappa2 );
    fprintf( file, "    kappa3  = %f\n", cam->warped.kappa3 );
    fprintf( file, "    tau1    = %f\n", cam->warped.tau1 );
    fprintf( file, "    tau2    = %f\n", cam->warped.tau2 );
}

/****************************************************************************/
void mvl_camera_fprint_lut(
        FILE *file,
        mvl_camera_t *cam 
        )
{
    fprintf( file, "Camera: serial='%ld', name='%s' index='%d', "
            "version='%d',  type='Linear'):\n",
            cam->serialno,
            cam->name,
            cam->index,
            cam->version );
    fprintf( file, "    width   = %d\n", cam->lut.width );
    fprintf( file, "    height  = %d\n", cam->lut.height );
    fprintf( file, "    fx      = %f\n", cam->lut.fx );
    fprintf( file, "    fy      = %f\n", cam->lut.fy );
    fprintf( file, "    cx      = %f\n", cam->lut.cx );
    fprintf( file, "    cy      = %f\n", cam->lut.cy );
    fprintf( file, "    sx      = %f\n", cam->lut.sx );
}


