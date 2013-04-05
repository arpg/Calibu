/* This file is part of the fiducials Project.
 * https://github.com/stevenlovegrove/fiducials
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

#include <Eigen/Eigen>
#include "DistortionModel.h"

namespace fiducials {

//////////////////////////////////////////////////////////////////////////////
// Linear Projection parametrizations
//////////////////////////////////////////////////////////////////////////////

// Four parameters: fu, fv, u0, v0
template<typename DistortionModel>
struct ProjectionLinear
{
    static const unsigned NUM_LIN_PARAMS = 4;
    static const unsigned NUM_PARAMS = NUM_LIN_PARAMS + DistortionModel::NUM_PARAMS;
    
    inline static std::string Name() { return "fu_fv_u0_v0_" + DistortionModel::Name(); }      
    
    template<typename T> inline
    static Eigen::Matrix<T,2,1> Map(const Eigen::Matrix<T,2,1>& proj, T const* params)
    {
        const T fac = DistortionModel::template RFactor<T>(proj.norm(), params + NUM_LIN_PARAMS);
        return Eigen::Matrix<T,2,1>(
            fac* params[0] * proj(0) + params[2],
            fac* params[1] * proj(1) + params[3]
        );        
    }
    
    template<typename T> inline
    static Eigen::Matrix<T,2,1> Unmap(const Eigen::Matrix<T,2,1>& img, T const* params)
    {    
        const Eigen::Matrix<T,2,1> dproj = Eigen::Matrix<T,2,1>(
            (img[0] - params[2] ) / params[0],
            (img[1] - params[3] ) / params[1]
        );
        const T fac = DistortionModel::RinvFactor(dproj.norm(), params + NUM_LIN_PARAMS);
        return fac*dproj;
    }
    
    template<typename T> inline
    static Eigen::Matrix<T,3,3> MakeK(T const* params)
    {
        Eigen::Matrix<T,3,3> K;
        K << params[0], 0, params[2],
             0, params[1], params[3],
             0, 0, 1;
        return K;
    }

    template<typename T> inline
    static Eigen::Matrix<T,3,3> MakeKinv(T const* params)
    {
        Eigen::Matrix<T,3,3> K;
        K << 1.0/params[0], 0,    -params[2] / params[0],
             0, 1.0/params[1], -params[3] / params[1],
             0, 0, 1;
        return K;
    }
    
    static inline
    Eigen::Matrix<double,2,2> dMap_dp(const Eigen::Vector2d& p, const double* params)
    {
        const double r = p.norm();    
        const Eigen::Vector2d dNorm_dp(p(0)/r, p(1)/r);

        const double fac = DistortionModel::RFactor(r, params + NUM_LIN_PARAMS );
        const Eigen::Vector2d dfac_dp = DistortionModel::dRFactor_dr(r,params + NUM_LIN_PARAMS) * dNorm_dp;
                
        Eigen::Matrix<double,2,2> J;
        J.col(0) = Eigen::Matrix<double,2,1>(
            dfac_dp(0) *params[0]*p(0) + fac*params[2],
            dfac_dp(0) *params[1]*p(1)
        );
        J.col(1) = Eigen::Matrix<double,2,1>(
            dfac_dp(1) *params[0]*p(0),
            dfac_dp(1) *params[1]*p(1) + fac*params[3]
        );
        
        return J;
    }   
};

// Four parameters: f, u0, v0
template<typename DistortionModel>
struct ProjectionLinearSquare
{
    static const unsigned NUM_LIN_PARAMS = 3;
    static const unsigned NUM_PARAMS = NUM_LIN_PARAMS + DistortionModel::NUM_PARAMS;
    
    inline static std::string Name() { return "f_u0_v0_" + DistortionModel::Name(); }      
    
    template<typename T> inline
    static Eigen::Matrix<T,2,1> Map(const Eigen::Matrix<T,2,1>& proj, T const* params)
    {
        const T fac = DistortionModel::template RFactor<T>(proj.norm(), params + NUM_LIN_PARAMS);
        return Eigen::Matrix<T,2,1>(
            fac* params[0] * proj(0) + params[1],
            fac* params[0] * proj(1) + params[2]
        );        
    }
    
    template<typename T> inline
    static Eigen::Matrix<T,2,1> Unmap(const Eigen::Matrix<T,2,1>& img, T const* params)
    {    
        const Eigen::Matrix<T,2,1> dproj = Eigen::Matrix<T,2,1>(
            (img[0] - params[1] ) / params[0],
            (img[1] - params[2] ) / params[0]
        );
        const T fac = DistortionModel::RinvFactor(dproj.norm(), params + NUM_LIN_PARAMS);
        return fac*dproj;
    }  
    
    template<typename T> inline
    static Eigen::Matrix<T,3,3> MakeK(T const* params)
    {
        Eigen::Matrix<T,3,3> K;
        K << params[0], 0, params[1],
             0, params[0], params[2],
             0, 0, 1;
        return K;
    }

    template<typename T> inline
    static Eigen::Matrix<T,3,3> MakeKinv(T const* params)
    {
        Eigen::Matrix<T,3,3> K;
        K << 1.0/params[0], 0, -params[1] / params[0],
             0, 1.0/params[0], -params[2] / params[0],
             0, 0, 1;
        return K;
    }    
    
    static inline
    Eigen::Matrix<double,2,2> dMap_dp(const Eigen::Vector2d& p, const double* params)
    {
        const double r = p.norm();    
        const Eigen::Vector2d dNorm_dp(p(0)/r, p(1)/r);

        const double fac = DistortionModel::RFactor(r, params + NUM_LIN_PARAMS );
        const Eigen::Vector2d dfac_dp = DistortionModel::dRFactor_dr(r,params + NUM_LIN_PARAMS) * dNorm_dp;
                
        Eigen::Matrix<double,2,2> J;
        J.col(0) = Eigen::Matrix<double,2,1>(
            dfac_dp(0) *params[0]*p(0) + fac*params[1],
            dfac_dp(0) *params[0]*p(1)
        );
        J.col(1) = Eigen::Matrix<double,2,1>(
            dfac_dp(1) *params[0]*p(0),
            dfac_dp(1) *params[0]*p(1) + fac*params[1]
        );
        
        return J;
    }
};

//////////////////////////////////////////////////////////////////////////////
// Convenience typedefs
//////////////////////////////////////////////////////////////////////////////

typedef ProjectionLinear<DistortionPinhole>  Pinhole;
typedef ProjectionLinear<DistortionFov>      Fov;
typedef ProjectionLinear<DistortionPoly>     Poly;

}
