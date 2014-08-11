/*
  This file is part of the Calibu Project.
  https://github.com/gwu-robotics/Calibu

  Copyright (C) 2013 George Washington University,
  Steven Lovegrove,
  Nima Keivan
  Jack Morrison
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

/**
 * Avoids copying inherited function implementations for all camera models.
 *
 * Requires the following functions in the derived class:
 * - static void Unproject(const T* pix, const T* params, T* ray) {
 * - static void Project(const T* ray, const T* params, T* pix) {
 * - static void dProject_dray(const T* ray, const T* params, T* j) {
 * - static void dProject_dparams(const T* ray, const T* params, T* j)
 * - static void dUnproject_dparams(const T* pix, const T* params, T* j)
 */
namespace calibu {
template <typename Scalar, int ParamSize, typename Derived>
class CameraImpl : public CameraInterface<Scalar> {
  typedef typename CameraInterface<Scalar>::Vec2t Vec2t;
  typedef typename CameraInterface<Scalar>::Vec3t Vec3t;
  typedef typename CameraInterface<Scalar>::SE3t SE3t;

 public:
  static constexpr int kParamSize = ParamSize;

  CameraImpl() {}
  virtual ~CameraImpl() {}
  CameraImpl(const Eigen::VectorXd& params, const Eigen::Vector2i& image_size) :
      CameraInterface<Scalar>(params, image_size) {
  }

  Vec3t
  Unproject(const Vec2t& pix) const override {
    Vec3t ray;
    Derived::Unproject(pix.data(), this->params_.data(), ray.data());
    return ray;
  }

  Vec2t
  Project(const Vec3t& ray) const override {
    Vec2t pix;
    Derived::Project(ray.data(), this->params_.data(), pix.data());
    return pix;
  }

  Eigen::Matrix<Scalar, 2, Eigen::Dynamic>
  dProject_dparams(const Vec3t& ray) const override {
    Eigen::Matrix<Scalar, 2, kParamSize> j;
    Derived::dProject_dparams(ray.data(), this->params_.data(), j.data());
    return j;
  }

  Eigen::Matrix<Scalar, 3, Eigen::Dynamic>
  dUnproject_dparams(const Vec2t& pix) const override {
    Eigen::Matrix<Scalar, 3, kParamSize> j;
    Derived::dUnproject_dparams(pix.data(), this->params_.data(), j.data());
    return j;
  }

  Eigen::Matrix<Scalar, 2, 3>
  dProject_dray(const Vec3t& ray) const override {
    Eigen::Matrix<Scalar, 2, 3> j;
    Derived::dProject_dray(ray.data(), this->params_.data(), j.data());
    return j;
  }
};
}  // namespace calibu
