/*
  This file is part of the Calibu Project.
  https://github.com/arpg/Calibu

  Copyright (C) 2015
  Steven Lovegrove,
  Nima Keivan
  Christoffer Heckman,
  Gabe Sibley,
  University of Colorado at Boulder,
  George Washington University.

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
#include <calibu/cam/camera_crtp_impl.h>
#include <calibu/cam/camera_utils.h>
#include <iostream>

namespace calibu {

/** Kannala-Brandt camera model.
 * Kannala and Brandt Like 'Generic' Projection Model
 * http://ieeexplore.ieee.org/iel5/34/34427/01642666.pdf
 * http://april.eecs.umich.edu/wiki/index.php/Camera_suite
 */
template<typename Scalar = double>
class KannalaBrandtCamera : public CameraImpl<Scalar, 8, KannalaBrandtCamera<Scalar> > {
  typedef CameraImpl<Scalar, 8, KannalaBrandtCamera<Scalar> > Base;
 public:
  using Base::Base;

  static constexpr int NumParams = 8;

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

    const T fu = params[0];
    const T fv = params[1];
    const T u0 = params[2];
    const T v0 = params[3];

    const T k0 = params[4];
    const T k1 = params[5];
    const T k2 = params[6];
    const T k3 = params[7];

    const T un = pix[0] - u0;
    const T vn = pix[1] - v0;
    const T psi = atan2( fu*vn, fv*un );

    const T rth = un / (fu * cos(psi) );

    // Use Newtons method to solve for theta.
    T th = rth;
    for (int i=0; i<5; i++)
    {
        // f = (th + k0*th**3 + k1*th**5 + k2*th**7 + k3*th**9 - rth)^2
        const T th2 = th*th;
        const T th3 = th2*th;
        const T th4 = th2*th2;
        const T th6 = th4*th2;
        const T x0 = k0*th3 + k1*th4*th + k2*th6*th + k3*th6*th3 - rth + th;
        const T x1 = 3*k0*th2 + 5*k1*th4 + 7*k2*th6 + 9*k3*th6*th2 + 1;
        const T d  = 2*x0*x1;
        const T d2 = 4*th*x0*(3*k0 + 10*k1*th2 + 21*k2*th4 + 36*k3*th6) + 2*x1*x1;
        const T delta = d / d2;
        th -= delta;
    }

    ray[0] = sin(th)*cos(psi);
    ray[1] = sin(th)*sin(psi);
    ray[2] = cos(th);
  }

  template<typename T>
  static void Project(const T* ray, const T* params, T* pix) {
    const T fu = params[0];
    const T fv = params[1];
    const T u0 = params[2];
    const T v0 = params[3];

    const T k0 = params[4];
    const T k1 = params[5];
    const T k2 = params[6];
    const T k3 = params[7];

    const T Xsq_plus_Ysq = ray[0]*ray[0]+ray[1]*ray[1];
    const T theta = atan2( sqrt(Xsq_plus_Ysq), ray[2] );
    const T psi = atan2( ray[1], ray[0] );

    const T theta2 = theta*theta;
    const T theta3 = theta2*theta;
    const T theta5 = theta3*theta2;
    const T theta7 = theta5*theta2;
    const T theta9 = theta7*theta2;
    const T r = theta + k0*theta3 + k1*theta5 + k2*theta7 + k3*theta9;

    pix[0] = fu*r*cos(psi) + u0;
    pix[1] = fv*r*sin(psi) + v0;
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
      const T fu = params[0];
      const T fv = params[1];
      const T k0 = params[4];
      const T k1 = params[5];
      const T k2 = params[6];
      const T k3 = params[7];

      const T x0 = ray[0]*ray[0];
      const T x1 = ray[1]*ray[1];
      const T x2 = x0 + x1;
      const T x3 = sqrt(x2);
      const T x4 = std::pow(x2,3.0/2.0);
      const T x5 = ray[2]*ray[2] + x2;
      const T a = atan2(x3, ray[2]);
      const T a2 = a*a;
      const T a4 = a2*a2;
      const T a6 = a4*a2;
      const T a8 = a4*a4;
      const T x11 = k0*a2 + k1*a4 + k2*a6 + k3*a8 + 1;
      const T x12 = 3*k0*a2 + 5*k1*a4 + 7*k2*a6 + 9*k3*a8 + 1;
      const T x13 = ray[2]*x12/(x2*x5) - x11*a/x4;
      const T x14 = ray[0]*fu;
      const T x15 = ray[1]*fv;
      const T x16 = -1/x4;
      const T x17 = x11*a;
      const T x18 = -x12/x5;
      const T x19 = x17/x3;
      const T x20 = ray[2]*x12/(x2*x5);

      j[0] = fu*(x0*x16*x17 + x0*x20 + x19);
      j[1] = ray[1]*x13*x14;
      j[2] = x14*x18;
      j[3] = ray[0]*x13*x15;
      j[4] = fv*(x1*x16*x17 + x1*x20 + x19);
      j[5] = x15*x18;
      }
};
}
