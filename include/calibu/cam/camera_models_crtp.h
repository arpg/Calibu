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

namespace calibu
{
  struct CameraUtils
  {
    template<typename T>
    static inline T PixNorm(const T* pix)
    {
      return sqrt(pix[0] * pix[0] + pix[1] * pix[1]);
    }

    template<typename T>
    static inline void Dehomogenize(const T* ray, T* px_dehomogenized)
    {
      px_dehomogenized[0] = ray[0] / ray[2];
      px_dehomogenized[1] = ray[1] / ray[2];
    }

    template<typename T>
    static inline void Homogenize(const T* pix, T* ray_homogenized)
    {
      ray_homogenized[0] = pix[0];
      ray_homogenized[1] = pix[1];
      ray_homogenized[2] = 1.0;
    }

    template<typename T>
    static inline void dDehomogenize_dray(const T* ray, T* j)
    {
      const T z_sq = ray[2] * ray[2];
      const T z_inv = 1.0 / ray[2];
      // Column major storage order.
      j[0] = z_inv;   j[2] = 0;       j[4] = -ray[0] / z_sq;
      j[1] = 0;       j[3] = z_inv;   j[5] = -ray[1] / z_sq;
    }

    template<typename T>
    static inline void MultK(const T* params, const T* pix, T* pix_k)
    {
      pix_k[0] = params[0] * pix[0] + params[2];
      pix_k[1] = params[1] * pix[1] + params[3];
    }

    template<typename T>
    static inline void MultInvK(const T* params, const T* pix, T* pix_kinv)
    {
      pix_kinv[0] = (pix[0] - params[2]) / params[0];
      pix_kinv[1] = (pix[1] - params[3]) / params[1];
    }
  };

  template<typename Scalar = double>
  class LinearCamera : public Camera<LinearCamera<Scalar>, Scalar>
  {
  public:
    LinearCamera(const Scalar* params_in)
    {
      memcpy(params, params_in, sizeof(params));
    }

    template<typename T>
    inline void UnprojectImpl(const T* pix, T* ray) const {
      // First multiply by inverse K and calculate distortion parameter.
      T pix_kinv[2];
      CameraUtils::MultInvK(params, pix, pix_kinv);
      // Homogenize the point.
      CameraUtils::Homogenize<T>(pix_kinv, ray);
    }

    template<typename T>
    inline void ProjectImpl(const T* ray, T* pix) const {
      // De-homogenize and multiply by K.
      CameraUtils::Dehomogenize(ray, pix);
      CameraUtils::MultK<T>(params, pix, pix);
    }

    template<typename T>
    inline void dProject_drayImpl(const T* ray, T* j) const {
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
    Scalar params[4];
  };

  template<typename Scalar = double>
  class FovCamera : public Camera<FovCamera<Scalar>, Scalar>
  {
  public:
    constexpr static const double kCamDistEps = 1e-5;
    static const uint32_t kMaxRadIncrements = 2000;
    constexpr static const double kMaxRad = 2.0;
    FovCamera(const Scalar* params_in)
    {
      memcpy(params, params_in, sizeof(params));
    }

    template<typename T>
    inline T Factor(T rad) const
    {
      const T param = params[4];
      if(param * param < kCamDistEps) {
        // limit w->0
        return (T)1;
      }else{
        const T mul2_tanw_by2 = (T)2.0 * tan(param / 2.0);
        const T mul2_tanw_by2_byw = mul2_tanw_by2 / param;

        if(rad * rad < kCamDistEps) {
          // limit r->0
          return mul2_tanw_by2_byw;
        }else{
          return atan(rad * mul2_tanw_by2) / (rad * param);
        }
      }
    }

    template<typename T>
    inline T dFactor_drad(T rad, T* fac) const
    {
      const T param = params[4];
      if(param * param < kCamDistEps) {
        *fac = (T)1;
        return 0;
      }else{
        const T tan_wby2 = tan(param / 2.0);
        const T mul2_tanw_by2 = (T)2.0 * tan_wby2;

        if(rad * rad < kCamDistEps) {
          *fac = mul2_tanw_by2 / param;
          return 0;
        }else{
          const T atan_mul2_tanw_by2 = atan(rad * mul2_tanw_by2);
          const T rad_by_param = (rad * param);

          *fac = atan_mul2_tanw_by2 / rad_by_param;
          return (2 * tan_wby2) /
              (rad * param * (4 * rad * rad * tan_wby2 * tan_wby2 + 1)) -
              atan_mul2_tanw_by2 / (rad * rad_by_param);
        }
      }
    }


    template<typename T>
    inline T Factor_inv(T rad) const
    {
      const T param = params[4];
      if(param * param < kCamDistEps) {
        // limit w->0
        return (T)1.0;
      }else{
        const T w_by2 = param / 2.0;
        const T mul_2tanw_by2 = tan(w_by2) * 2.0;

        if(rad * rad < kCamDistEps) {
          // limit r->0
          return param / mul_2tanw_by2;
        }else{
          return tan(rad * param) / (rad * mul_2tanw_by2);
        }
      }
    }

    template<typename T>
    inline void UnprojectImpl(const T* pix, T* ray) const {
      // First multiply by inverse K and calculate distortion parameter.
      T pix_kinv[2];
      CameraUtils::MultInvK(params, pix, pix_kinv);
      // Homogenize the point.
      CameraUtils::Homogenize<T>(pix_kinv, ray);
      const T fac_inv =
          Factor_inv(CameraUtils::PixNorm(pix_kinv));
      pix_kinv[0] *= fac_inv;
      pix_kinv[1] *= fac_inv;
      CameraUtils::Homogenize<T>(pix_kinv, ray);
    }

    template<typename T>
    inline void ProjectImpl(const T* ray, T* pix) const {
      // De-homogenize and multiply by K.
      CameraUtils::Dehomogenize(ray, pix);
      // Calculate distortion parameter.
      const T fac =
          Factor(CameraUtils::PixNorm(pix));
      pix[0] *= fac;
      pix[1] *= fac;
      CameraUtils::MultK<T>(params, pix, pix);
    }

    template<typename T>
    inline void dProject_drayImpl(const T* ray, T* j) const {
      // De-homogenize and multiply by K.
      T pix[2];
      CameraUtils::Dehomogenize(ray, pix);
      // Calculte the dehomogenization derivative.
      T j_dehomog[6];
      CameraUtils::dDehomogenize_dray(ray, j_dehomog);

      const T rad = CameraUtils::PixNorm(pix);
      T fac;
      const T dfac_drad_byrad =
          dFactor_drad(rad, &fac) / rad;
      const T dfac_dp[2] = { pix[0] * dfac_drad_byrad,
                             pix[1] * dfac_drad_byrad };

      // Calculate the k matrix and distortion derivative.
      const T params0_pix0 = params[0] * pix[0];
      const T params1_pix1 = params[1] * pix[1];
      const T k00 = dfac_dp[0] * params0_pix0 + fac * params[0];
      const T k01 = dfac_dp[1] * params0_pix0;
      const T k10 = dfac_dp[0] * params1_pix1;
      const T k11 = dfac_dp[1] * params1_pix1 + fac * params[1];

      // Do the multiplication dkmult * ddehomogenized_dray
      j[0] = j_dehomog[0] * k00;
      j[1] = j_dehomog[0] * k10;
      j[2] = j_dehomog[3] * k01;
      j[3] = j_dehomog[3] * k11;
      j[4] = j_dehomog[4] * k00 + j_dehomog[5] * k01;
      j[5] = j_dehomog[4] * k10 + j_dehomog[5] * k11;
    }
    Scalar params[5];
  };
}
