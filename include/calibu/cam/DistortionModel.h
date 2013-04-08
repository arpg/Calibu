/* This file is part of the calibu Project.
 * https://github.com/stevenlovegrove/calibu
 *
 * Copyright (C) 2013  Steven Lovegrove
 *                     George Washington University
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
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
    
    inline static std::string Name() {
        return "";
    }
    
    template<typename T> inline
    static double RFactor(double r, T const*) {
        return 1;
    }
    
    template<typename T> inline
    static double RinvFactor(double dr, T const*) {
        return 1;
    }
};

//////////////////////////////////////////////////////////////////////////////
struct DistortionPoly
{
    static const unsigned NUM_PARAMS = 3;
    
    inline static std::string Name() { return "k1_k2_k3"; }    
    
    template<typename T> inline
    static T RFactor(T r, const T* params)
    {
        const T r2 = r*r;
        const T r4 = r2*r2;
        return (T)1.0 + params[0]*r2 + params[1]*r4 + params[2]*r4*r2;
    }
    
    template<typename T> inline
    static T RinvFactor(T dr, const T* params)
    {
        // TODO: imeplement
        throw std::exception();
    }    
};

//////////////////////////////////////////////////////////////////////////////
struct DistortionFov
{
    static const unsigned NUM_PARAMS = 1;
    
    inline static std::string Name() { return "w"; }
    
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
