/**
 *  \file camera_model_io.h
 *
 *  Functions for reading/writing MVL camera models
 *
 *  $Id$
 */

#ifndef __CAMERA_MODEL_IO_H__
#define __CAMERA_MODEL_IO_H__

#include <string>

#include "conversions.h"
#include "camera.h"
//#include <mvl/camera/mvl_tinyxml.h>

#include <iostream>

/// useful
//#include <mvl/camera/mvl_tinyxmlhelpers.h>

///////////////////////////////////////////////////////////////////////////////
/// Read the serial number from a camera model xml file.
//bool mvl_read_camera_serialno( const char* filename );

///////////////////////////////////////////////////////////////////////////////
/// If you change the layout of the XML camera model file such that the new
//  parsing code is not backwards compatable, then you have to change the version
//  number.
#define CURRENT_CMOD_XML_FILE_VERSION 7

///////////////////////////////////////////////////////////////////////////////
/// allocate a mvl_camera_t structure and read a camera from the xml file
//  "filename"  into it.  Also, return the 4x4 homogeneous pose in hpose.
mvl_camera_t *mvl_read_camera(
        const char *filename, /**< Input: */
        double *hpose         /**< Output: */
        );

///////////////////////////////////////////////////////////////////////////////
/// Write the camera model specified in cam to the xml file filename.  Also
//  convert the 4x4 homogeneous pose matrix hpose to a 6x1 euler roll-pitch-yaw
//  pose and write it as the pose element of the xml file.
bool mvl_write_camera(
        const char *filename,   //< Input:
        const double *hpose,    //< Input:
        const mvl_camera_t *cam //< Input:
        );

///////////////////////////////////////////////////////////////////////////////
/// Write a camera model to a string.
bool mvl_write_camera_to_string(
        const double *hpose,     //< Input:
        const mvl_camera_t *cam, //< Input:
        std::string& sRes        //< Output:
        );

#endif

