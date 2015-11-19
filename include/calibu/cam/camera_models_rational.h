/*
 * camera_models_rational.h
 *
 *  Created on: Nov 16, 2015
 *      Author: Gregory Kramida
 *   Copyright: 2015 Gregory Kramida
 *
 *   Licensed under the Apache License, Version 2.0 (the "License");
 *   you may not use this file except in compliance with the License.
 *   You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 *   Unless required by applicable law or agreed to in writing, software
 *   distributed under the License is distributed on an "AS IS" BASIS,
 *   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *   See the License for the specific language governing permissions and
 *   limitations under the License.
 */

/**
 * This file contains a camera models based on:
 * Claus, David, and Andrew W. Fitzgibbon. "A rational function lens distortion model for general
 * cameras." Computer Vision and Pattern Recognition, IEEE Computer Society Conference on (CVPR) 2005.
 * vol. 1, pp. 213-219. IEEE, 2005.
 */

#pragma once

#include <calibu/cam/camera_crtp.h>
#include <calibu/cam/camera_crtp_impl.h>
#include <calibu/cam/camera_utils.h>
#include <iostream>

namespace calibu {


/** A six-coefficient rational distortion model. */
template<typename Scalar = double>
class Rational6Camera : public CameraImpl<Scalar, 10, Rational6Camera<Scalar> > {
  typedef CameraImpl<Scalar, 10, Rational6Camera<Scalar> > Base;
 public:
  using Base::Base;

  static constexpr int NumParams = 10;

  template<typename T>
  static void Scale( const double s, T* params ) {
    CameraUtils::Scale( s, params );
  }

  // NOTE: A camera calibration matrix only makes sense for a LinearCamera.
  // Such a matrix only exists if derived for linear projection models. Ideally
  // any time we call for K it would be on a linear camera, but we provide this
  // functionality for obtaining approximate solutions.
  // FURTHER NOTE: We ASSUME the first four entries of params_ to be fu, fv, sx
  // and sy. If your camera model doesn't respect this ordering, then evaluating
  // K for it will result in an incorrect (even approximate) matrix.
  template<typename T>
  static void K( const T* params , T* Kmat) {
    CameraUtils::K( params , Kmat);
  }

  template<typename T>
  static T Factor(const T rad, const T* params) {
    T r2 = rad * rad;
    T r4 = r2 * r2;
    return ((static_cast<T>(1.0) +
            params[4] * r2 +
            params[5] * r4 +
            params[6] * r4 * r2)/
			(static_cast<T>(1.0) +
			params[7] * r2 +
			params[8] * r4 +
			params[9] * r4 * r2));
  }

  template<typename T>
  static T dFactor_drad(const T ru, const T* params, T* fac) {
    *fac = Factor(ru, params);

    T k1 = params[4];
	T k2 = params[5];
	T k3 = params[6];
	T k4 = params[7];
	T k5 = params[8];
	T k6 = params[9];

    T ru2 = ru * ru;
	T ru4 = ru2 * ru2;
	T ru6 = ru4 * ru2;
	T numer = k1 * ru2 + k2 * ru4 + k3 * ru6 + 1;
	T denom = k4 * ru2 + k5 * ru4 + k6 * ru6 + 1;
	T pol = numer / denom;
	T d_numer = ru * (2*k1 + 4*k2*ru2 * 6*k3*ru4);
	T d_denom = ru * (2*k4 + 4*k5*ru2 * 6*k6*ru4);
	T denom2 = (denom * denom);
	T numer2 = (d_numer * denom - numer * d_denom);
	T d_pol = numer2 / denom2;
    return (d_pol);
  }

  template<typename T>
  static T Factor_inv(const T rd, const T* params) {
    T k1 = params[4];
    T k2 = params[5];
    T k3 = params[6];
    T k4 = params[7];
    T k5 = params[8];
    T k6 = params[9];


    //Use Newton's method to solve (fixed number of iterations)
    // (for explanation, see notes on rational distortion models
    // in beginning of camera_models_crtp.h)
    T ru = rd;
    for (int i=0; i < 5; i++) {
      // Common sub-expressions of d, d2
      T ru2 = ru * ru;
      T ru4 = ru2 * ru2;
      T ru6 = ru4 * ru2;
      T numer = k1 * ru2 + k2 * ru4 + k3 * ru6 + 1;
      T denom = k4 * ru2 + k5 * ru4 + k6 * ru6 + 1;
      T pol = numer / denom;
      T d_numer = ru * (2*k1 + 4*k2*ru2 * 6*k3*ru4);
      T d_denom = ru * (2*k4 + 4*k5*ru2 * 6*k6*ru4);
      T denom2 = (denom * denom);
      T numer2 = (d_numer * denom - numer * d_denom);
      T d_pol = numer2 / denom2;
      T d_ru_pol = pol + ru * d_pol;

      T ru_pol_r = ru * (pol) - rd;

      // 1st derivative
      T d = 2 * ru_pol_r * d_ru_pol;

      T d2_numer = (2*k1 + 12*k2*ru2 + 30*k3*ru4);
      T d2_denom = (2*k4 + 12*k5*ru2 + 30*k6*ru4);
      // 2nd derivative
      T d2 = 2 * (d_ru_pol * d_ru_pol +
    		  ru_pol_r * (2 * d_pol +
    				  ((d2_numer*denom - d2_denom*numer)*denom2 - 2*denom2*d_denom*numer2)
					  /(denom2 * denom2)));
      // Delta update
      T delta = d / d2;
      ru -= delta;
    }

    // Return the undistortion factor
    return ru / rd;
  }

  template<typename T>
  static void Unproject(const T* pix, const T* params, T* ray) {
    // First multiply by inverse K and calculate distortion parameter.
    T pix_kinv[2];
    CameraUtils::MultInvK(params, pix, pix_kinv);

    // Homogenize the point.
    CameraUtils::Homogenize<T>(pix_kinv, ray);
    const T fac_inv = Factor_inv(CameraUtils::PixNorm(pix_kinv), params);
    pix_kinv[0] *= fac_inv;
    pix_kinv[1] *= fac_inv;
    CameraUtils::Homogenize<T>(pix_kinv, ray);
  }

  template<typename T>
  static void Project(const T* ray, const T* params, T* pix) {
    // De-homogenize and multiply by K.
    CameraUtils::Dehomogenize(ray, pix);

    // Calculate distortion parameter.
    const T fac = Factor(CameraUtils::PixNorm(pix), params);
    pix[0] *= fac;
    pix[1] *= fac;
    CameraUtils::MultK<T>(params, pix, pix);
  }

  template<typename T>
  static void dProject_dray(const T* ray, const T* params, T* j) {
    // De-homogenize and multiply by K.
    T pix[2];
    CameraUtils::Dehomogenize(ray, pix);

    // Calculate the dehomogenization derivative.
    T j_dehomog[6];
    CameraUtils::dDehomogenize_dray(ray, j_dehomog);

    T rad = CameraUtils::PixNorm(pix);
    T fac;
    T dfac_drad_byrad = dFactor_drad(rad, params, &fac) / rad;
    T dfac_dp[2] = { pix[0] * dfac_drad_byrad,
                     pix[1] * dfac_drad_byrad };

    // Calculate the k matrix and distortion derivative.
    T params0_pix0 = params[0] * pix[0];
    T params1_pix1 = params[1] * pix[1];
    T k00 = dfac_dp[0] * params0_pix0 + fac * params[0];
    T k01 = dfac_dp[1] * params0_pix0;
    T k10 = dfac_dp[0] * params1_pix1;
    T k11 = dfac_dp[1] * params1_pix1 + fac * params[1];

    // Do the multiplication dkmult * ddehomogenized_dray
    j[0] = j_dehomog[0] * k00;
    j[1] = j_dehomog[0] * k10;
    j[2] = j_dehomog[3] * k01;
    j[3] = j_dehomog[3] * k11;
    j[4] = j_dehomog[4] * k00 + j_dehomog[5] * k01;
    j[5] = j_dehomog[4] * k10 + j_dehomog[5] * k11;
  }

  template<typename T>
  static void dProject_dparams(const T*, const T*, T* ) {
    std::cerr << "dProjedt_dparams not defined for the poly3 rational model. "
        " Throwing exception." << std::endl;
    throw 0;
  }

  template<typename T>
  static void dUnproject_dparams(const T*, const T*, T* ) {
    std::cerr << "dUnproject_dparams not defined for the poly3 rational model. "
        " Throwing exception." << std::endl;
    throw 0;
  }
};

}//end namespace calibu
