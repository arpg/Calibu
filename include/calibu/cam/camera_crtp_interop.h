/*
  This file is part of the Calibu Project.
  https://github.com/gwu-robotics/Calibu

  Copyright (C) 2013 George Washington University,
  Steven Lovegrove,
  Nima Keivan
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
#include <calibu/cam/camera_crtp.h>
#include <calibu/cam/camera_models_crtp.h>
#include <calibu/cam/CameraModel.h>
#include <calibu/cam/CameraRig.h>
#include <calibu/cam/CameraXml.h>

namespace calibu {
template<typename Scalar>
inline CameraInterface<Scalar>* CreateFromOldCamera(
    const CameraModelGeneric<Scalar>& old_cam) {
  if (old_cam.Type() == "calibu_fu_fv_u0_v0_w") {
    return new calibu::FovCamera<Scalar>(
          old_cam.GenericParams(),
          Eigen::Vector2i(old_cam.Width(), old_cam.Height()));
  } else if (old_cam.Type() == "calibu_fu_fv_u0_v0") {
    return new calibu::LinearCamera<Scalar>(
          old_cam.GenericParams(),
          Eigen::Vector2i(old_cam.Width(), old_cam.Height()));
  } else if (old_cam.Type() == "calibu_fu_fv_u0_v0_k1_k2_k3") {
    return new calibu::Poly3Camera<Scalar>(
          old_cam.GenericParams(),
          Eigen::Vector2i(old_cam.Width(), old_cam.Height()));
  } else {
    std::cerr << "Unknown old camera type " << old_cam.Type() << " please "
        " implement this camera before initializing it using the "
        " crtp camera system." << std::endl;
    throw 0;
  }
}

template<typename Scalar>
inline void CreateFromOldRig(const CameraRigT<Scalar>* old_rig,
                             Rig<Scalar>* rig) {
  for (const auto& cam_and_transform: old_rig->cameras) {
    rig->AddCamera(CreateFromOldCamera(cam_and_transform.camera),
                   cam_and_transform.T_wc);
  }
}

template<typename Scalar>
inline void LoadRig(const std::string& filename, Rig<Scalar>* rig) {
  CameraRigT<Scalar> old_rig = calibu::ReadXmlRig(filename);
  for (auto& cam_and_transform: old_rig.cameras) {
    rig->AddCamera(CreateFromOldCamera(cam_and_transform.camera),
                   cam_and_transform.T_wc);
  }
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
}  // namespace calibu
