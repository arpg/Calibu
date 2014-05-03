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
#include <Eigen/Eigen>
#include <sophus/se3.hpp>

namespace calibu {

template<typename Scalar> using Vec2t = Eigen::Matrix<Scalar, 2, 1>;
template<typename Scalar> using Vec3t = Eigen::Matrix<Scalar, 3, 1>;
template<typename Scalar> using VecXt =
    Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;
template<typename Scalar> using SE3t = Sophus::SE3Group<Scalar>;

template<typename Scalar = double>
class CameraInterface {
 public:
  CameraInterface(const CameraInterface<Scalar>& other) :
      n_params_(other.n_params_), owns_memory_(other.owns_memory_) {
    CopyParams(other.params_);
  }

  virtual ~CameraInterface() {
    if (owns_memory_) {
      delete[] params_;
    }
  }

  /** Unproject an image location into world coordinates */
  virtual Vec3t<Scalar> Unproject(const Vec2t<Scalar>& pix) const = 0;

  /** Project a world point into an image location */
  virtual Vec2t<Scalar> Project(const Vec3t<Scalar>& ray) const = 0;

  /** Derivative of the Project along a ray */
  virtual Eigen::Matrix<Scalar, 2, 3> dProject_dray(
      const Vec3t<Scalar>& ray) const = 0;

  /**
   * Project a point into a camera located at t_ba.
   *
   * @param t_ba Location of camera to project into.
   * @param ray Homogeneous ray
   * @param rho Inverse depth of point
   * @returns A non-homegeneous pixel location in the second camera.
   */
  Vec2t<Scalar> Transfer3d(const SE3t<Scalar>& t_ba,
                           const Vec3t<Scalar>& ray,
                           const Scalar rho) const {
    const Vec3t<Scalar> ray_dehomogenized =
        t_ba.rotationMatrix() * ray + rho * t_ba.translation();
    return Project(ray_dehomogenized);
  }

  /**
   * The derivative of the projection from Transfer3d wrt the point
   * being transfered.
   */
  Eigen::Matrix<Scalar, 2, 4> dTransfer3d_dray(const SE3t<Scalar>& t_ba,
                                               const Vec3t<Scalar>& ray,
                                               const Scalar rho) const {
    const Eigen::Matrix<Scalar, 3, 3> rot_matrix = t_ba.rotationMatrix();
    const Vec3t<Scalar> ray_dehomogenized =
        rot_matrix * ray + rho * t_ba.translation();
    const Eigen::Matrix<Scalar, 2, 3> dproject_dray =
        dProject_dray(ray_dehomogenized);
    Eigen::Matrix<Scalar, 2, 4> dtransfer3d_dray;
    dtransfer3d_dray.template topLeftCorner<2, 3>() =
        dproject_dray * rot_matrix;
    dtransfer3d_dray.col(3) = dproject_dray * t_ba.translation();
    return dtransfer3d_dray;
  }

  Scalar* GetParams() {
    return params_;
  }

 protected:
  CameraInterface(Scalar* params_in, int n_params, bool owns_memory)
      : n_params_(n_params), owns_memory_(owns_memory) {
    CopyParams(params_in);
  }

  void CopyParams(Scalar* params) {
    if (owns_memory_) {
      params_ = new Scalar[n_params_];
      memcpy(params_, params, sizeof(Scalar) * n_params_);
    } else {
      params_ = params;
    }
  }

  // All the camera parameters (fu, fv, u0, v0, ...distortion)
  Scalar* params_;

  // Length of parameter array (including distortion parameters)
  int n_params_;

  // Is the parameter list memory managed by us or externally?
  bool owns_memory_;
};

template<typename Scalar = double>
class Rig {
 public:
  void AddCamera(CameraInterface<Scalar>* cam, const SE3t<Scalar>& t_wc) {
    cameras_.push_back(cam);
    t_wc_.push_back(t_wc);
  }

  ~Rig() {
    for (CameraInterface<Scalar>* ptr : cameras_) {
      delete ptr;
    }
  }

  std::vector<CameraInterface<Scalar>*> cameras_;
  std::vector<SE3t<Scalar>> t_wc_;
};
}  // namespace calibu
