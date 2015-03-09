/*
  This file is part of the Calibu Project.
  https://github.com/arpg/Calibu

  Copyright (C) 2013 George Washington University,
  Steven Lovegrove,
  Nima Keivan
  Jack Morrison
  Christoffer Heckman
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
 * - static void Scale(const T* scale, const T* params)
 * - static void SetParams()
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
  CameraImpl(const Eigen::VectorXd& params, Eigen::Vector2i& image_size) :
      CameraInterface<Scalar>(params, image_size) {
  }

  void
  Scale(const Scalar& s) override {
    Derived::Scale( s, this->params_.data() );
  }

  void
  PrintInfo() const override {
      printf("camera model info:\n" );
      printf("    Right        = [%d; %d; %d] unit vector\n", (int)this->rdf_(0,0), (int)this->rdf_(0,1), (int)this->rdf_(0,2) );
      printf("    Down         = [%d; %d; %d] unit vector\n", (int)this->rdf_(1,0), (int)this->rdf_(1,1), (int)this->rdf_(1,2) );
      printf("    Forward      = [%d; %d; %d] unit vector\n", (int)this->rdf_(2,0), (int)this->rdf_(2,1), (int)this->rdf_(2,2) );
      printf("    Width        = %d pixels\n", (int) this->image_size_[0]);
      printf("    Height       = %d pixels\n", (int) this->image_size_[1]);

      // TODO ensure this is right for all models...
      printf("    Horiz Center = %.3f pixels\n", this->params_[2] );
      printf("    Vert Center  = %.3f pixels\n", this->params_[3] );
      printf("    Horiz FOV    = %.3f degrees\n", 180.0*2.0*atan2( this->image_size_[0]/2, this->params_[0] )/M_PI );
      printf("    Vert  FOV    = %.3f degrees\n", 180.0*2.0*atan2( this->image_size_[1]/2, this->params_[1] )/M_PI );
  }

  /** Model-dependent operations. */
  Eigen::Matrix<Scalar, 3, 3>
  K() const override {
    Eigen::Matrix<Scalar,3,3> Kmat;
    Derived::K( this->params_.data() , Kmat.data());
    return Kmat;
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
}; // public CameraInterface<Scalar>
}  // namespace calibu
