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
std::string CameraModelType( const std::string& sType );

std::string IndentStr(int indent);

std::string AttribOpen(const std::string& attrib);

std::string AttribClose(const std::string& attrib);

void WriteXmlSE3(std::ostream& out, const Sophus::SE3d& T_wc, int indent);

void WriteXmlCameraModel(std::ostream& out, const CameraModelInterface& cam, int indent = 0);

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
//CameraModel ReadXmlCameraModel(TiXmlElement* pEl);
CameraModel ReadXmlCameraModel(const std::string& filename);

////////////////////////////////////////////////////////////////////////

void WriteXmlSE3(std::ostream& out, const Sophus::SE3d& T_wc, int indent = 0);
void WriteXmlSE3(const std::string& filename, const Sophus::SE3d& T_wc);
//Sophus::SE3d ReadXmlSE3(TiXmlNode* xmlcampose);
Sophus::SE3d ReadXmlSE3(const std::string& filename);

////////////////////////////////////////////////////////////////////////

void WriteXmlCameraModelAndTransform(std::ostream& out, const CameraModelAndTransform& cop, int indent = 0);
void WriteXmlCameraModelAndTransform(const std::string& filename, const CameraModelAndTransform& cop);
//CameraModelAndTransform ReadXmlCameraModelAndTransform(TiXmlNode* xmlcampose);
CameraModelAndTransform ReadXmlCameraModelAndTransform(const std::string& filename);

////////////////////////////////////////////////////////////////////////

void WriteXmlRig(std::ostream& out, const CameraRig& rig, int indent = 0);
void WriteXmlRig(const std::string& filename, const CameraRig& rig);
//CameraRig ReadXmlRig(TiXmlNode* xmlrig);
CameraRig ReadXmlRig(const std::string& filename);


}
