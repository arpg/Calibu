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
template<typename Scalar = double>
class CameraInterface {
 protected:
  typedef Eigen::Matrix<Scalar, 2, 1> Vec2t;
  typedef Eigen::Matrix<Scalar, 3, 1> Vec3t;
  typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1> VecXt;
  typedef Sophus::SE3Group<Scalar> SE3t;

 public:
  CameraInterface(const CameraInterface<Scalar>& other) :
      image_size_(other.image_size_), params_(other.params_) {}
  virtual ~CameraInterface() {}

  /** Unproject an image location into world coordinates */
  virtual Vec3t Unproject(const Vec2t& pix) const = 0;

  /** Project a world point into an image location */
  virtual Vec2t Project(const Vec3t& ray) const = 0;

  /** Derivative of the Project along a ray */
  virtual Eigen::Matrix<Scalar, 2, 3>
  dProject_dray(const Vec3t& ray) const = 0;

  virtual Eigen::Matrix<Scalar, 2, Eigen::Dynamic>
  dProject_dparams(const Vec3t& ray) const = 0;

  virtual Eigen::Matrix<Scalar, 3, Eigen::Dynamic>
  dUnproject_dparams(const Vec2t& pix) const = 0;

  /**
   * Project a point into a camera located at t_ba.
   *
   * @param t_ba Location of camera to project into.
   * @param ray Homogeneous ray
   * @param rho Inverse depth of point
   * @returns A non-homegeneous pixel location in the second camera.
   */
  Vec2t Transfer3d(const SE3t& t_ba,
                   const Vec3t& ray,
                   const Scalar rho) const {
    const Vec3t ray_dehomogenized =
        t_ba.rotationMatrix() * ray + rho * t_ba.translation();
    return Project(ray_dehomogenized);
  }

  /**
   * The derivative of the projection from Transfer3d wrt the point
   * being transfered.
   */
  Eigen::Matrix<Scalar, 2, 4> dTransfer3d_dray(const SE3t& t_ba,
                                               const Vec3t& ray,
                                               const Scalar rho) const {
    const Eigen::Matrix<Scalar, 3, 3> rot_matrix = t_ba.rotationMatrix();
    const Vec3t ray_dehomogenized =
        rot_matrix * ray + rho * t_ba.translation();
    const Eigen::Matrix<Scalar, 2, 3> dproject_dray =
        dProject_dray(ray_dehomogenized);
    Eigen::Matrix<Scalar, 2, 4> dtransfer3d_dray;
    dtransfer3d_dray.template topLeftCorner<2, 3>() =
        dproject_dray * rot_matrix;
    dtransfer3d_dray.col(3) = dproject_dray * t_ba.translation();
    return dtransfer3d_dray;
  }

  Eigen::Matrix<Scalar, 2, Eigen::Dynamic> dTransfer_dparams(
      const SE3t& t_ba,
      const Vec2t& pix,
      const Scalar rho) const
  {
    const Vec3t ray = Unproject(pix);
    const Eigen::Matrix<Scalar, 3, 3> rot_matrix = t_ba.rotationMatrix();
    const Vec3t ray_dehomogenized =
        rot_matrix * ray + rho * t_ba.translation();
    const Eigen::Matrix<Scalar, 2, 3> dproject_dray =
        dProject_dray(ray_dehomogenized);
    const Eigen::Matrix<Scalar, 2, 3> dtransfer3d_dray =
        dproject_dray * rot_matrix;
    const Eigen::Matrix<Scalar, 3, Eigen::Dynamic> dray_dparams =
        dUnproject_dparams(pix);

    const Eigen::Matrix<Scalar, 2, Eigen::Dynamic> d_project_dparams =
        dProject_dparams(ray_dehomogenized);

    return d_project_dparams + dtransfer3d_dray * dray_dparams;
  }

  const Eigen::VectorXd& GetParams() const {
    return params_;
  }

  Eigen::VectorXd& GetParams() {
    return params_;
  }

  void SetParams(const Eigen::VectorXd& new_params) {
    params_ = new_params;
  }

  uint32_t NumParams() const {
    return params_.rows();
  }

  unsigned int Width() const {
    return image_size_[0];
  }

  unsigned int Height() const {
    return image_size_[1];
  }

 protected:
  CameraInterface(const Eigen::VectorXd& params_in,
                  const Eigen::Vector2i& image_size)
      : image_size_(image_size), params_(params_in) {
  }

  Eigen::Vector2i image_size_;

  // All the camera parameters (fu, fv, u0, v0, ...distortion)
  Eigen::VectorXd params_;
};

template<typename Scalar = double>
class Rig {
 public:
  typedef Sophus::SE3Group<Scalar> SE3t;
  void AddCamera(CameraInterface<Scalar>* cam, const SE3t& t_wc) {
    cameras_.push_back(cam);
    t_wc_.push_back(t_wc);
  }

  void Clear() {
    for (CameraInterface<Scalar>* ptr : cameras_) {
      delete ptr;
    }
    cameras_.clear();
    t_wc_.clear();
  }

  ~Rig() {
    Clear();
  }

  std::vector<CameraInterface<Scalar>*> cameras_;
  std::vector<SE3t> t_wc_;
};
}  // namespace calibu
