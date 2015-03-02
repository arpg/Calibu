/*
   This file is part of the Calibu Project.
   https://github.com/arpg/Calibu

   Copyright (C) 2015 University of Colorado at Boulder,
                      Steven Lovegrove,
                      Christoffer Heckman,
                      Nima Keivan,
                      Gabe Sibley

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

   http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
 */

#pragma once

#include <calibu/Platform.h>
#include <calibu/cam/camera_crtp.h>
#include <calibu/cam/camera_crtp_impl.h>
#include <calibu/cam/camera_models_crtp.h>
#include <sstream>

namespace calibu
{

const std::string NODE_RIG     	= "rig";
const std::string NODE_CAM_POSE	= "camera_pose";
const std::string NODE_CAM      = "camera";
const std::string NODE_POSE     = "pose";

typedef CameraInterface<double> CameraInterfaced;
typedef Rig<double> Rigd;

///////////////////////////////////////////////////////////////////////////////
template <class T> inline
void StrToVal( T& t, const std::string& sValue )
{
    std::istringstream iss( sValue );
    iss >> t;
}

///////////////////////////////////////////////////////////////////////////////
template <class T> inline
T StrToVal( const std::string& sValue, const T& default_val = T() )
{
    T t = default_val;
    std::istringstream iss( sValue );
    iss >> t;
    return t;
}

///////////////////////////////////////////////////////////////////////////////
template <class T> inline
std::string ValToStr( const T& t )
{
    std::ostringstream oss;
    oss << t;
    return oss.str();
}

///////////////////////////////////////////////////////////////////////////////
CALIBU_EXPORT
std::string CameraType( const std::string& sType );

CALIBU_EXPORT
std::string IndentStr(int indent);

CALIBU_EXPORT
std::string AttribOpen(const std::string& attrib);

CALIBU_EXPORT
std::string AttribClose(const std::string& attrib);

CALIBU_EXPORT
void WriteXmlSE3(std::ostream& out, const Sophus::SE3d& t_rc, int indent);


CALIBU_EXPORT
void WriteXmlCamera(std::ostream& out, const std::shared_ptr<CameraInterfaced> cam,
                    int indent = 0);

CALIBU_EXPORT
void WriteXmlCamera(const std::string& filename,
                    const std::shared_ptr<CameraInterfaced> cam);


////////////////////////////////////////////////////////////////////////////
inline void WriteXmlCameraAndTransformWithLut(
        std::ostream& out,
        std::string& sLutXmlElement,
        const std::shared_ptr<CameraInterfaced> cam,
        int indent = 0
        )
{
    const std::string dd = IndentStr(indent);
    out << dd << AttribOpen(NODE_CAM_POSE) << std::endl;
    WriteXmlCamera(out, cam, indent+4);
    WriteXmlSE3(out,cam->Pose(),indent+4);

    out << dd << sLutXmlElement << std::endl; // LUT, if there is one!
    out << dd << AttribClose(NODE_CAM_POSE) << std::endl;
}


///////////////////////////////////////////////////////////////////////////////
CALIBU_EXPORT
std::shared_ptr<CameraInterfaced> ReadXmlCamera(
    const std::string& filename, const std::shared_ptr<CameraInterfaced> cam);

////////////////////////////////////////////////////////////////////////

CALIBU_EXPORT
void WriteXmlSE3(std::ostream& out, const Sophus::SE3d& t_rc, int indent = 0);

CALIBU_EXPORT
void WriteXmlSE3(const std::string& filename, const Sophus::SE3d& t_rc);

CALIBU_EXPORT
Sophus::SE3d ReadXmlSE3(const std::string& filename);

////////////////////////////////////////////////////////////////////////

CALIBU_EXPORT
void WriteXmlCameraAndTransform(std::ostream& out,
                                const std::shared_ptr<CameraInterfaced> cop,
                                int indent = 0);

CALIBU_EXPORT
void WriteXmlCameraAndTransform(const std::string& filename,
                                const std::shared_ptr<CameraInterfaced> cop);

CALIBU_EXPORT
std::shared_ptr<CameraInterfaced> ReadXmlCameraAndTransform(
    const std::string& filename, const std::shared_ptr<CameraInterfaced> cop);

CALIBU_EXPORT
std::shared_ptr<CameraInterfaced> ReadXmlCameraAndTransform(
    const std::string& filename);

////////////////////////////////////////////////////////////////////////
CALIBU_EXPORT
void WriteXmlRig(std::ostream& out, const std::shared_ptr<Rigd> rig, int indent = 0);

CALIBU_EXPORT
void WriteXmlRig(const std::string& filename, const std::shared_ptr<Rigd> rig);

CALIBU_EXPORT
std::shared_ptr<Rigd> ReadXmlRig(const std::string& filename);

} // namespace calibu
