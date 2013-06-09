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

#include <Eigen/Eigen>
#include "DistortionModel.h"

namespace calibu {

//////////////////////////////////////////////////////////////////////////////
// Linear Projection parametrizations
//////////////////////////////////////////////////////////////////////////////

// Identity matrix projection
struct ProjectionLinearId
{
    typedef ProjectionLinearId DistortionFreeModel;

    static const unsigned NUM_PARAMS = 0;
    
    inline static std::string Type() { return "id"; }
    
    template<typename T> inline
    static Eigen::Matrix<T,2,1> Map(const Eigen::Matrix<T,2,1>& proj, T const* /*params*/)
    {
        return proj;
    }
    
    template<typename T> inline
    static Eigen::Matrix<T,2,1> Unmap(const Eigen::Matrix<T,2,1>& img, T const* /*params*/)
    {    
        return img;
    }
    
    template<typename T> inline
    static Eigen::Matrix<T,3,3> MakeK(T const* /*params*/)
    {
        return Eigen::Matrix<T,3,3>::Identity();
    }
    
    template<typename T> inline
    static Eigen::Matrix<T,3,3> MakeKinv(T const* /*params*/)
    {
        return Eigen::Matrix<T,3,3>::Identity();
    }
    
    static inline
    Eigen::Matrix<double,2,2> dMap_dp(const Eigen::Vector2d& /*p*/, const double* /*params*/)
    {
        return Eigen::Matrix<double,2,2>::Identity();
    }   
};

// Four parameters: fu, fv, u0, v0
template<typename DistortionModel>
struct ProjectionLinear
{
    typedef ProjectionLinear<DistortionPinhole> DistortionFreeModel;
    
    static const unsigned NUM_LIN_PARAMS = 4;
    static const unsigned NUM_PARAMS = NUM_LIN_PARAMS + DistortionModel::NUM_PARAMS;
 
    inline static std::string Type()
    {
        if(  DistortionModel::Type().empty() ){
            return "fu_fv_u0_v0";
        }
        return "fu_fv_u0_v0_" + DistortionModel::Type();
    }
 
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
                    dfac_dp(0) *params[0]*p(0) + fac*params[0],
                dfac_dp(0) *params[1]*p(1)
                );
        J.col(1) = Eigen::Matrix<double,2,1>(
                    dfac_dp(1) *params[0]*p(0),
                dfac_dp(1) *params[1]*p(1) + fac*params[1]
                );
        
        return J;
    }   
};

// Four parameters: f, u0, v0
template<typename DistortionModel>
struct ProjectionLinearSquare
{
    typedef ProjectionLinearSquare<DistortionPinhole> DistortionFreeModel;
    
    static const unsigned NUM_LIN_PARAMS = 3;
    static const unsigned NUM_PARAMS = NUM_LIN_PARAMS + DistortionModel::NUM_PARAMS;
    
    inline static std::string Type() 
    {
        if( DistortionModel::Type().empty() ){
            return "f_u0_v0"; 
        }
        return "f_u0_v0_" + DistortionModel::Type();
    }
    
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
