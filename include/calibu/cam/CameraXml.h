/* 
   This file is part of the Calibu Project.
   https://github.com/gwu-robotics/Calibu

   Copyright (C) 2013 George Washington University,
                      Steven Lovegrove,
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
#include <calibu/cam/CameraRig.h>
#include <sstream>

namespace calibu
{

const std::string NODE_RIG           = "rig";
const std::string NODE_CAMMODEL_POSE = "camera";
const std::string NODE_CAMMODEL      = "camera_model";
const std::string NODE_POSE          = "pose";

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
std::string CameraModelType( const std::string& sType );

CALIBU_EXPORT
std::string IndentStr(int indent);

CALIBU_EXPORT
std::string AttribOpen(const std::string& attrib);

CALIBU_EXPORT
std::string AttribClose(const std::string& attrib);

CALIBU_EXPORT
void WriteXmlSE3(std::ostream& out, const Sophus::SE3d& T_wc, int indent);

CALIBU_EXPORT
void WriteXmlCameraModel(std::ostream& out, const CameraModelInterface& cam, int indent = 0);

CALIBU_EXPORT
void WriteXmlCameraModel(const std::string& filename, const CameraModelInterface& cam);


////////////////////////////////////////////////////////////////////////////
inline void WriteXmlCameraModelAndTransformWithLut(
        std::ostream& out,
        std::string& sLutXmlElement,
        const CameraModelAndTransform& cop,
        int indent = 0
        )
{
    const std::string dd = IndentStr(indent);
    out << dd << AttribOpen(NODE_CAMMODEL_POSE) << std::endl;
    WriteXmlCameraModel(out, cop.camera, indent+4);
    WriteXmlSE3(out, cop.T_wc, indent+4);
    
    out << dd << sLutXmlElement << std::endl; // LUT, if there is one!
    out << dd << AttribClose(NODE_CAMMODEL_POSE) << std::endl;
}


///////////////////////////////////////////////////////////////////////////////
CALIBU_EXPORT
CameraModel ReadXmlCameraModel(const std::string& filename);

////////////////////////////////////////////////////////////////////////

CALIBU_EXPORT
void WriteXmlSE3(std::ostream& out, const Sophus::SE3d& T_wc, int indent = 0);

CALIBU_EXPORT
void WriteXmlSE3(const std::string& filename, const Sophus::SE3d& T_wc);

CALIBU_EXPORT
Sophus::SE3d ReadXmlSE3(const std::string& filename);

////////////////////////////////////////////////////////////////////////

CALIBU_EXPORT
void WriteXmlCameraModelAndTransform(std::ostream& out, const CameraModelAndTransform& cop, int indent = 0);

CALIBU_EXPORT
void WriteXmlCameraModelAndTransform(const std::string& filename, const CameraModelAndTransform& cop);

CALIBU_EXPORT
CameraModelAndTransform ReadXmlCameraModelAndTransform(const std::string& filename);

////////////////////////////////////////////////////////////////////////
CALIBU_EXPORT
void WriteXmlRig(std::ostream& out, const CameraRig& rig, int indent = 0);

CALIBU_EXPORT
void WriteXmlRig(const std::string& filename, const CameraRig& rig);

CALIBU_EXPORT
CameraRig ReadXmlRig(const std::string& filename);


}
