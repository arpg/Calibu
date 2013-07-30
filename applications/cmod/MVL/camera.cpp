/**
 *  \file camera.c
 *  
 *  Globals.  
 *  
 *  $Id: camera.cpp 373 2008-02-26 13:55:20Z gsibley $
 */ 

#include "camera.h"
#include "camera_model_lut.h"

/// Control verbosity in error reporting.
int g_nMvlCameraModelVerbosityLevel = 1; // 0 = silent

char *mvl_camera_model_type_strings[4] = { 
    (char*)"MVL_CAMERA_LINEAR", 
    (char*)"MVL_CAMERA_WARPED",
    (char*)"MVL_CAMERA_LUT",
    (char*)"your_new_model_here" 
};

///////////////////////////////////////////////////////////////////////////////
/// Copies a camera model
mvl_camera_t* mvl_alloc_and_copy_camera( mvl_camera_t* pInputCamera )
{
    mvl_camera_t* pNewCamera = (mvl_camera_t*)calloc( 1, sizeof( mvl_camera_t ) );

    *pNewCamera = *pInputCamera;

    switch( pInputCamera->type ) {
        case MVL_CAMERA_LINEAR:
            break;
        case MVL_CAMERA_WARPED:
            break;
        case MVL_CAMERA_LUT:
            mvl_alloc_and_copy_lut( pInputCamera->lut.pLUT,
                    pInputCamera->lut.width, pInputCamera->lut.height,  
                    &pNewCamera->lut.pLUT );
            break; 
        default:
            printf( "ERROR: unknown camera model type in mvl_alloc_and_copy_camera.\n");
            free( pNewCamera );
            pNewCamera = NULL;
            break;
    }
    return pNewCamera;
}

///////////////////////////////////////////////////////////////////////////////
/// Frees a camera model
void mvl_free_camera( mvl_camera_t* pCamera )
{
    switch( pCamera->type ) {
        case MVL_CAMERA_LINEAR:
            free( pCamera );
            break;
        case MVL_CAMERA_WARPED:
            free( pCamera );
            break;
        case MVL_CAMERA_LUT:
            mvl_free_lut( pCamera->lut.pLUT );
            free( pCamera );
            break; 
        default:
            printf( "ERROR: unknown camera model type in mvl_free_camera.\n");
            free( pCamera );
            break;
    }
}

