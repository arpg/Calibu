/*
   This file is part of the Calibu Project.
   https://github.com/arpg/Calibu

   Copyright (C) 2015 University of Colorado at Boulder,
                      Christoffer Heckman,
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
#include <calibu/cam/camera_crtp.h>
#include <calibu/cam/camera_crtp_impl.h>
#include <calibu/cam/camera_models_crtp.h>
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
void WriteXmlSE3(std::ostream& out, const Sophus::SE3d& t_wc, int indent);

CALIBU_EXPORT
void WriteXmlCameraModel(std::ostream& out, const calibu::CameraInterface<double>& cam, int indent = 0);

CALIBU_EXPORT
void WriteXmlCameraModel(const std::string& filename, const calibu::CameraInterface<double>& cam);


////////////////////////////////////////////////////////////////////////////
inline void WriteXmlCameraModelAndTransformWithLut(
        std::ostream& out,
        std::string& sLutXmlElement,
        const calibu::Rig& cop,
        int indent = 0
        )
{
    const std::string dd = IndentStr(indent);
    out << dd << AttribOpen(NODE_CAMMODEL_POSE) << std::endl;
    WriteXmlCameraModel(out, cop.camera, cip.transform, indent+4);
    WriteXmlSE3(out, cop.t_wc, indent+4);

    out << dd << sLutXmlElement << std::endl; // LUT, if there is one!
    out << dd << AttribClose(NODE_CAMMODEL_POSE) << std::endl;
}


///////////////////////////////////////////////////////////////////////////////
CALIBU_EXPORT
CameraModel ReadXmlCameraModel(const std::string& filename);

////////////////////////////////////////////////////////////////////////

CALIBU_EXPORT
void WriteXmlSE3(std::ostream& out, const Sophus::SE3d& t_wc, int indent = 0);

CALIBU_EXPORT
void WriteXmlSE3(const std::string& filename, const Sophus::SE3d& t_wc);

CALIBU_EXPORT
Sophus::SE3d ReadXmlSE3(const std::string& filename);

////////////////////////////////////////////////////////////////////////

CALIBU_EXPORT
void WriteXmlCameraModelAndTransform(std::ostream& out, const calibu::Rig& cop, int indent = 0);

CALIBU_EXPORT
void WriteXmlCameraModelAndTransform(const std::string& filename, const calibu::Rig& cop);

CALIBU_EXPORT
CameraModelAndTransform ReadXmlCameraModelAndTransform(const std::string& filename);

////////////////////////////////////////////////////////////////////////
CALIBU_EXPORT
void WriteXmlRig(std::ostream& out, const CameraRig& rig, int indent = 0);

CALIBU_EXPORT
void WriteXmlRig(const std::string& filename, const CameraRig& rig);

CALIBU_EXPORT
CameraRig ReadXmlRig(const std::string& filename);

template<typename Scalar>
inline void LoadRig(const std::string& filename, Rig<Scalar>* rig) {
  CameraRigT<Scalar> old_rig = calibu::ReadXmlRig(filename);
  for (auto& cam_and_transform: old_rig.cameras) {
    if (cam_and_transform.Type() == "calibu_fu_fv_u0_v0_w") {
      return new calibu::FovCamera<Scalar>(
            cam_and_transform.GenericParams(),
            Eigen::Vector2i(old_cam.Width(), old_cam.Height()));
    } else if (cam_and_transform.Type() == "calibu_fu_fv_u0_v0") {
      return new calibu::LinearCamera<Scalar>(
            cam_and_transform.GenericParams(),
            Eigen::Vector2i(old_cam.Width(), old_cam.Height()));
    } else if (cam_and_transform.Type() == "calibu_fu_fv_u0_v0_k1_k2_k3") {
      return new calibu::Poly3Camera<Scalar>(
            cam_and_transform.GenericParams(),
            Eigen::Vector2i(old_cam.Width(), old_cam.Height()));
    } else {
      std::cerr << "Unknown old camera type " << old_cam.Type() << " please "
          " implement this camera before initializing it using the "
          " crtp camera system." << std::endl;
      throw 0;
     }
    rig->AddCamera(CreateFromOldCamera(cam_and_transform.camera),
                   cam_and_transform.T_wc);
  }
}

template<typename Scalar>
inline CameraInterface<Scalar>* CreateFromOldCamera(
    const CameraModelGeneric<Scalar>& cam) {
}

template<typename Scalar>
inline CameraInterface<Scalar>* LoadCamera(
    const std::string& filename) {
  CameraRigT<Scalar> old_rig = calibu::ReadXmlRig(filename);
  if (old_rig.cameras.size() == 0) {
    std::cerr << "No cameras found in rig." << std::endl;
    throw 0;
  }
  auto& cam_and_transform = old_rig.cameras[0];
  return CreateFromOldCamera(cam_and_transform.camera,
                             cam_and_transform.T_wc);


}
} // namespace calibu
