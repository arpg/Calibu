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

/*
   Common math utilities used in various camera models
*/

#pragma once
namespace calibu {
struct CameraUtils {
  /** Euclidean distance from (0, 0) to given pixel */
  template<typename T>
  static inline T PixNorm(const T* pix) {
    return sqrt(pix[0] * pix[0] + pix[1] * pix[1]);
  }

  template<typename T>
  static void Scale(const double s, T* params) {
    params[0] *= s;
    params[1] *= s;
    params[2] = s*(params[2]+0.5) - 0.5;
    params[3] = s*(params[3]+0.5) - 0.5;
  }

  template<typename T>
  static inline void K(const T* params, T* Kmat) {
    Kmat[0] = params[0];
    Kmat[1] = 0;
    Kmat[2] = 0;
    Kmat[3] = 0;
    Kmat[4] = params[1];
    Kmat[5] = 0;
    Kmat[6] = params[2];
    Kmat[7] = params[3];
    Kmat[8] = 1;
    //Kmat << params[0], 0, params[2],
    //    0, params[1], params[3],
    //    0, 0, 1;
  }

  /** (x, y, z) -> (x, y)
   *
   * @param ray A 3-vector of (x, y, z)
   * @param pix A 2-vector (x, y)
   * */
  template<typename T>
  static inline void Dehomogenize(const T* ray, T* px_dehomogenized) {
    px_dehomogenized[0] = ray[0] / ray[2];
    px_dehomogenized[1] = ray[1] / ray[2];
  }

  /**
   * (x, y) -> (x, y, z), with z = 1
   *
   * @param pix A 2-vector (x, y)
   * @param ray_homogenized A 3-vector to be filled in
   */
  template<typename T>
  static inline void Homogenize(const T* pix, T* ray_homogenized) {
    ray_homogenized[0] = pix[0];
    ray_homogenized[1] = pix[1];
    ray_homogenized[2] = (T)1.0;
  }

  /**
   * Derivative of "Dehomogenize" with respect to the homogenous ray
   * that is being dehomogenized.
   *
   * @param ray A 3-vector of (x, y, z)
   * @param j A 2x3 matrix stored in column-major order
   */
  template<typename T>
  static inline void dDehomogenize_dray(const T* ray, T* j) {
    const T z_sq = ray[2] * ray[2];
    const T z_inv = 1.0 / ray[2];
    // Column major storage order.
    j[0] = z_inv;   j[2] = 0;       j[4] = -ray[0] / z_sq;
    j[1] = 0;       j[3] = z_inv;   j[5] = -ray[1] / z_sq;
  }

  template<typename T>
  static inline void dMultK_dparams(const T*, const T* pix, T* j) {
    j[0] = pix[0];    j[2] = 0;       j[4] = 1;   j[6] = 0;
    j[1] = 0;         j[3] = pix[1];  j[5] = 0;   j[7] = 1;
  }

  template<typename T>
  static inline void dMultInvK_dparams(const T* params, const T* pix, T* j) {
    j[0] = -(pix[0] - params[2]) / (params[0] * params[0]);
    j[1] = 0;
    j[2] = 0;

    j[3] = 0;
    j[4] = -(pix[1] - params[3]) / (params[1] * params[1]);
    j[5] = 0;

    j[6] = -1 / params[0];
    j[7] = 0;
    j[8] = 0;

    j[9] = 0;
    j[10] = -1 / params[1];
    j[11] = 0;
  }

  /**
   * Transform a dehomogenized real-world point with calibration focal
   * length and principal point to place it in its imaged location.
   */
  template<typename T>
  static inline void MultK(const T* params, const T* pix, T* pix_k) {
    pix_k[0] = params[0] * pix[0] + params[2];
    pix_k[1] = params[1] * pix[1] + params[3];
  }

  /**
   * Take an image plane pixel and camera params and place it in
   * dehomogenized world coords.
   */
  template<typename T>
  static inline void MultInvK(const T* params, const T* pix, T* pix_kinv) {
    pix_kinv[0] = (pix[0] - params[2]) / params[0];
    pix_kinv[1] = (pix[1] - params[3]) / params[1];
  }
};

template<typename Scalar = double>
class LinearCamera : public CameraImpl<Scalar, 4, LinearCamera<Scalar> > {
  typedef CameraImpl<Scalar, 4, LinearCamera<Scalar> > Base;
 public:
  using Base::Base;

  static constexpr int NumParams = 4;

  template<typename T>
  static void Scale( const double s, T* params ) {
    CameraUtils::Scale( s, params );
  }

  // NOTE: We ASSUME the first four entries of params_ to be fu, fv, sx
  // and sy. If your camera model doesn't respect this ordering, then evaluating
  // K for it will result in an incorrect matrix.
  template<typename T>
  static void K( const T* params , T* Kmat) {
    CameraUtils::K( params , Kmat);
  }

  template<typename T>
  static void Unproject(const T* pix, const T* params, T* ray) {
    // First multiply by inverse K and calculate distortion parameter.
    T pix_kinv[2];
    CameraUtils::MultInvK(params, pix, pix_kinv);
    // Homogenize the point.
    CameraUtils::Homogenize<T>(pix_kinv, ray);
  }

  template<typename T>
  static void Project(const T* ray, const T* params, T* pix) {
    // De-homogenize and multiply by K.
    CameraUtils::Dehomogenize(ray, pix);
    CameraUtils::MultK<T>(params, pix, pix);
  }

  template<typename T>
  static void dProject_dparams(const T* ray, const T* params, T* j) {
    T pix[2];
    CameraUtils::Dehomogenize(ray, pix);
    CameraUtils::dMultK_dparams(params, pix, j);
  }

  template<typename T>
  static void dUnproject_dparams(const T* pix, const T* params, T* j) {
    CameraUtils::dMultInvK_dparams(params, pix, j);
  }

  template<typename T>
  static void dProject_dray(const T* ray, const T* params, T* j) {
    // De-homogenize and multiply by K.
    T pix[2];
    CameraUtils::Dehomogenize(ray, pix);
    // Calculte the dehomogenization derivative.
    CameraUtils::dDehomogenize_dray(ray, j);

    // Do the multiplication dkmult * ddehomogenized_dray
    j[0] *= params[0];
    // j[1] *= params[1];  // This equals zero.
    // j[2] *= params[0];  // This equals zero.
    j[3] *= params[1];
    j[4] *= params[0];
    j[5] *= params[1];
  }
};

}//end namespace calibu

