/* 
   This file is part of the Calibu Project.
   https://github.com/gwu-robotics/Calibu

   Copyright (C) 2013 George Washington University,
                      Steven Lovegrove

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

#include <stdexcept>

namespace calibu {

#define DIST_CAM_EPS 1E-5

//////////////////////////////////////////////////////////////////////////////
// Distortion Parametrizations
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
struct DistortionPinhole
{
    static const unsigned NUM_PARAMS = 0;
    
    inline static std::string Type() {
        return "";
    }
    
    template<typename T> inline
    static T RFactor(T /*r*/, T const* /*params*/) {
        return (T)1.0;
    }
    
    template<typename T> inline
    static T RinvFactor(T /*dr*/, T const* /*params*/) {
        return (T)1.0;
    }

    template<typename T> inline
    static T dRFactor_dr(T /*r*/, const T* /*params*/)
    {
        return (T)0.0;
    }

    template<typename T> inline
    static T dRFactor_dParam(T r, const T* params)
    {
        //TODO: implement this
        return (T)0;
    }
};

//////////////////////////////////////////////////////////////////////////////
struct DistortionPoly2
{
    static const unsigned NUM_PARAMS = 2;
    
    inline static std::string Type() { return "_k1_k2"; }    
    
    template<typename T> inline
    static T RFactor(T ru, const T* params)
    {
        const T r2 = ru*ru;
        const T r4 = r2*r2;
        return (T)1.0 + params[0]*r2 + params[1]*r4;
    }
        
    template<typename T> inline
    static T RinvFactor(T rd, const T* params)
    {
        const T k1 = params[0];
        const T k2 = params[1];
        
        // Use Newton's method to solve (fixed number of iterations)
        T ru = rd;
        for (int i=0; i<5; i++)
        {
            // Common sub-expressions of d, d2
            const T ru2 = ru*ru;
            const T ru4 = ru2*ru2;
            const T pol = k1*ru2 + k2*ru4 + 1;
            const T pol2 = 2*ru2*(k1 + 2*k2*ru2);
            const T pol3 = pol + pol2;
            // 1st derivative
            const T d = (ru*(pol) - rd) * 2*pol3;
            // 2nd derivative
            const T d2 = 4*ru*(ru*pol - rd)*(3*k1 + 10*k2*ru2) + 2*pol3*pol3;
            // Delta update
            const T delta = d / d2;
            ru -= delta;
        }
        return ru / rd;
    }    

    template<typename T> inline
    static T dRFactor_dr(T r, const T* params)
    {
        return 2.0*params[0]*r + 4.0*params[1]*r*r*r;
    }

    template<typename T> inline
    static T dRFactor_dParam(T r, const T* params)
    {
        //TODO: implement this
        assert(false);
        return (T)0;
    }
};

//////////////////////////////////////////////////////////////////////////////
struct DistortionPoly3
{
    static const unsigned NUM_PARAMS = 3;
    
    inline static std::string Type() { return "_k1_k2_k3"; }    
    
    template<typename T> inline
    static T RFactor(T ru, const T* params)
    {
        const T r2 = ru*ru;
        const T r4 = r2*r2;
        return (T)1.0 + params[0]*r2 + params[1]*r4 + params[2]*r4*r2;
    }
    
    template<typename T> inline
    static T RinvFactor(T rd, const T* params)
    {
        const T k1 = params[0];
        const T k2 = params[1];
        const T k3 = params[2];
        
        // Use Newton's method to solve (fixed number of iterations)
        T ru = rd;
        for (int i=0; i<5; i++)
        {
            // Common sub-expressions of d, d2
            const T ru2 = ru*ru;
            const T ru4 = ru2*ru2;
            const T ru6 = ru4*ru2;
            const T pol = k1*ru2 + k2*ru4 + k3*ru6 + 1;
            const T pol2 = 2*ru2*(k1 + 2*k2*ru2 + 3*k3*ru4);
            const T pol3 = pol + pol2;            
            // 1st derivative
            const T d = (ru*(pol) - rd) * 2*pol3;
            // 2nd derivative
            const T d2 = 4*ru*(ru*pol - rd)*(3*k1 + 10*k2*ru2 + 21*k3*ru4) + 2*pol3*pol3;
            // Delta update
            const T delta = d / d2;
            ru -= delta;
        }
        return ru / rd;
    }    

    template<typename T> inline
    static T dRFactor_dr(T r, const T* params)
    {
        const T r2 = r*r;
        const T r3 = r2*r;
        return 2.0*params[0]*r + 4.0*params[1]*r3 + 6.0*params[2]*r3*r2;
    }

    template<typename T> inline
    static T dRFactor_dParam(T r, const T* params)
    {
        //TODO: implement this
        assert(false);
        return (T)0;
    }
};

//////////////////////////////////////////////////////////////////////////////
struct DistortionFov
{
    static const unsigned NUM_PARAMS = 1;
    
    inline static std::string Type() { return "_w"; }
    
    template<typename T> inline
    static T RFactor(T r, const T* params)
    {
        if(params[0]*params[0] < DIST_CAM_EPS) {
            // limit w->0
            return (T)1;            
        }else{
            const T mul2tanwby2 = (T)2.0 * tan(params[0]/2.0);
            const T mul2tanwby2byw = mul2tanwby2 / params[0];
            
            if(r*r < DIST_CAM_EPS) {
                // limit r->0
                return mul2tanwby2byw;
            }else{
                return atan(r*mul2tanwby2) / (r*params[0]);
            }
        }
    }

    template<typename T> inline
    static T dRFactor_dParam(T r, const T* params)
    {
        if(params[0]*params[0] < DIST_CAM_EPS) {
            // limit w->0
            return (T)0;
        }else{
            const T mul2tanwby2 = (T)2.0 * tan(params[0]/2.0);
            const T mul2tanwby2byw = mul2tanwby2 / params[0];

            const T tanp_2 = tan(params[0]/2);
            const T tanp_2_sqr_over_2 = (tanp_2*tanp_2)/2;
            if(r*r < DIST_CAM_EPS) {
                // limit r->0
                return (2*(tanp_2_sqr_over_2 + 0.5))/params[0] -
                        (2*tan(params[0]/2))/(params[0]*params[0]);
            }else{
                return (2*(tanp_2_sqr_over_2 + 0.5))/(params[0]*(4*r*r*tanp_2*tanp_2 + 1)) -
                        atan(2*r*tan(params[0]/2))/(params[0]*params[0]*r);
            }
        }
    }
    
    template<typename T> inline
    static T RinvFactor(T dr, const T* params)
    {
        if(params[0]*params[0] < DIST_CAM_EPS) {
            // limit w->0
            return (T)1.0;
        }else{
            const T wby2 = params[0] / 2.0;
            const T mul2tanwby2 = tan(wby2) * 2.0;
            
            if(dr*dr < DIST_CAM_EPS) {
                // limit r->0
                return params[0] / mul2tanwby2;
            }else{
                return tan(dr*params[0]) / (dr*mul2tanwby2);
            }
        }
    }
    
    template<typename T> inline
    static T dRFactor_dr(T r, const T* params)
    {
        if(params[0]*params[0] < DIST_CAM_EPS) {
            return 0;
        }else{
            if(r*r < DIST_CAM_EPS) {
                return 0;
            }else{
                const T wby2 = params[0]/2.0;
                const T tanwby2 = tan(wby2);
                const T sq_tanwby2 = tanwby2 * tanwby2;
                const T rr=r*r;
                return (2*tanwby2)/(r*params[0]*(4*rr*sq_tanwby2+1))-atan(2*r*tanwby2)/(rr*params[0]);
            }
        }
    }
};

}
