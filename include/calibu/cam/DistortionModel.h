/* 
   This file is part of the Calibu Project.
   https://robotics.gwu.edu/git/calibu

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
    static double RFactor(double, T const*) {
        return 1;
    }
    
    template<typename T> inline
    static double RinvFactor(double, T const*) {
        return 1;
    }
};

//////////////////////////////////////////////////////////////////////////////
struct DistortionPoly
{
    static const unsigned NUM_PARAMS = 3;
    
    inline static std::string Type() { return "k1_k2_k3"; }    
    
    template<typename T> inline
    static T RFactor(T r, const T* params)
    {
        const T r2 = r*r;
        const T r4 = r2*r2;
        return (T)1.0 + params[0]*r2 + params[1]*r4 + params[2]*r4*r2;
    }
    
    template<typename T> inline
    static T RinvFactor(T /*dr*/, const T* /*params*/)
    {
        // TODO: imeplement
        throw std::exception();
    }    
};

//////////////////////////////////////////////////////////////////////////////
struct DistortionFov
{
    static const unsigned NUM_PARAMS = 1;
    
    inline static std::string Type() { return "w"; }
    
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
    
    inline static
    double dRFactor_dr(double r, const double* params)
    {
        if(params[0]*params[0] < DIST_CAM_EPS) {
            return 0;
        }else{
            if(r*r < DIST_CAM_EPS) {
                return 0;
            }else{
                const double wby2 = params[0]/2.0;
                const double tanwby2 = tan(wby2);
                const double sq_tanwby2 = tanwby2 * tanwby2;
                const double rr=r*r;
                return (2*tanwby2)/(r*params[0]*(4*rr*sq_tanwby2+1))-atan(2*r*tanwby2)/(rr*params[0]);
            }
        }
    }
};

}
