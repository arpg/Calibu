/*
   This file is part of the Calibu Project.
   https://github.com/arpg/Calibu

   Copyright (C) 2015 University of Colorado at Boulder,
                      Steven Lovegrove,
                      Christoffer Heckman,
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
void WriteXmlCamera(std::ostream& out, const calibu::CameraInterface<double>& cam, int indent = 0);

CALIBU_EXPORT
void WriteXmlCamera(const std::string& filename, const calibu::CameraInterface<double>& cam);


////////////////////////////////////////////////////////////////////////////
inline void WriteXmlCameraAndTransformWithLut(
        std::ostream& out,
        std::string& sLutXmlElement,
        const calibu::CameraInterface<double>& cam,
        const Sophus::SE3d& t_rc,
        int indent = 0
        )
{
    const std::string dd = IndentStr(indent);
    out << dd << AttribOpen(NODE_CAM_POSE) << std::endl;
    WriteXmlCameraAndTransform(out, cam, indent+4);

    out << dd << sLutXmlElement << std::endl; // LUT, if there is one!
    out << dd << AttribClose(NODE_CAM_POSE) << std::endl;
}


///////////////////////////////////////////////////////////////////////////////
template <typename Scalar = double>
CALIBU_EXPORT
inline
CameraInterface<Scalar> ReadXmlCamera(const std::string& filename);

////////////////////////////////////////////////////////////////////////

CALIBU_EXPORT
void WriteXmlSE3(std::ostream& out, const Sophus::SE3d& t_rc, int indent = 0);

CALIBU_EXPORT
void WriteXmlSE3(const std::string& filename, const Sophus::SE3d& t_rc);

CALIBU_EXPORT
Sophus::SE3d ReadXmlSE3(const std::string& filename);

////////////////////////////////////////////////////////////////////////

CALIBU_EXPORT
void WriteXmlCameraAndTransform(std::ostream& out, const calibu::CameraInterface<double>& cop, int indent = 0);

CALIBU_EXPORT
void WriteXmlCameraAndTransform(const std::string& filename, const calibu::CameraInterface<double>& cop);

template<typename Scalar = double>
CALIBU_EXPORT
inline
CameraInterface<Scalar> ReadXmlCameraAndTransform(const std::string& filename);

////////////////////////////////////////////////////////////////////////
CALIBU_EXPORT
void WriteXmlRig(std::ostream& out, const Rig<double>& rig, int indent = 0);

CALIBU_EXPORT
void WriteXmlRig(const std::string& filename, const Rig<double>& rig);

template<typename Scalar = double>
CALIBU_EXPORT
inline
Rig<Scalar> ReadXmlRig(const std::string& filename);

//template<typename Scalar>
//Rig ReadXmlRig(const std::string& filename) {
//for (auto& cam_and_transform: calibu::ReadXmlRig(filename)) {
//    if (cam_and_transform.Type() == "calibu_fu_fv_u0_v0_w") {
//      return new calibu::FovCamera<Scalar>(
//            cam_and_transform.Params(),
//            Eigen::Vector2i(cam_and_transform.Width(), cam_and_transform.Height()));
//    } else if (cam_and_transform.Type() == "calibu_fu_fv_u0_v0") {
//      return new calibu::LinearCamera<Scalar>(
//            cam_and_transform.Params(),
//            Eigen::Vector2i(cam_and_transform.Width(), cam_and_transform.Height()));
//    } else if (cam_and_transform.Type() == "calibu_fu_fv_u0_v0_k1_k2_k3") {
//      return new calibu::Poly3Camera<Scalar>(
//            cam_and_transform.Params(),
//            Eigen::Vector2i(cam_and_transform.Width(), cam_and_transform.Height()));
//    } else {
//      std::cerr << "Unknown old camera type " << old_cam.Type() << " please "
//          " implement this camera before initializing it using the "
//          " crtp camera system." << std::endl;
//      throw 0;
//     }
//    rig->AddCamera(cam_and_transform);
//}
//}

} // namespace calibu
