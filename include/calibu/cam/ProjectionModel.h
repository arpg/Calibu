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

#include <Eigen/Eigen>

#include <calibu/Platform.h>
#include <calibu/cam/DistortionModel.h>

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
    static Eigen::Matrix<T,2,1> Project(const Eigen::Matrix<T,3,1>& P, T const* /*params*/)
    {
        return calibu::Project(P);
    }

    template<typename T> inline
    static Eigen::Matrix<T,3,1> Unproject(const Eigen::Matrix<T,2,1>& p, T const* /*params*/)
    {
        return calibu::Unproject(p);
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
    
    template<typename T> inline
    static Eigen::Matrix<T,2,3> dProject_dP(const Eigen::Matrix<T,3,1>& P, const T* /*params*/)
    {
        Eigen::Matrix<T,2,3> _dp_dP;
        _dp_dP << 
            1.0/P(2), 0, -P(0)/(P(2)*P(2)),
            0, 1.0/P(2), -P(1)/(P(2)*P(2));

        return _dp_dP;        
    }

    template<typename T> inline
    static Eigen::Matrix<T,2,Eigen::Dynamic> dProject_dParams(const Eigen::Matrix<T,3,1>& P, const Eigen::Matrix<T,Eigen::Dynamic,1>& params)
    {
        //TODO: implement this
        assert(false);
        return Eigen::Matrix<T,Eigen::Dynamic,Eigen::Dynamic>();
    }
    
    template<typename T> inline
    static void Scale(T /*scale*/, T* /*params*/)
    {
        throw std::runtime_error("Cannot scale ProjectionLinearId camera");        
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
        return "fu_fv_u0_v0" + DistortionModel::Type();
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
    static Eigen::Matrix<T,2,Eigen::Dynamic> dProject_dParams(const Eigen::Matrix<T,3,1>& P, const Eigen::Matrix<T,Eigen::Dynamic,1>& params)
    {
        const Eigen::Matrix<T,2,1> proj = calibu::Project(P);
        const T fac = DistortionModel::template RFactor<T>(proj.norm(), &params[4]);
        const T dfac = DistortionModel::template dRFactor_dParam<T>(proj.norm(), &params[4]);
        const Eigen::Matrix<T,2,Eigen::Dynamic> dMap_dParams = (Eigen::Matrix<T,2,Eigen::Dynamic>(2,5) <<
                                                   fac*proj(0),           0, 1.0,   0, params[0]*proj(0)*dfac,
                                                             0, fac*proj(1),   0, 1.0, params[1]*proj(1)*dfac
                                                   ).finished();

        return dMap_dParams;
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
    static Eigen::Matrix<T,2,1> Project(const Eigen::Matrix<T,3,1>& P, T const* params)
    {
        return Map( calibu::Project(P) , params );
    }

    template<typename T> inline
    static Eigen::Matrix<T,3,1> Unproject(const Eigen::Matrix<T,2,1>& p, T const* params)
    {
        return calibu::Unproject( Unmap( p, params) );
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
    
    template<typename T> inline
    static Eigen::Matrix<T,2,2> dMap_dp(const Eigen::Matrix<T,2,1>& p, const T* params)
    {
        const T r = p.norm();
        const Eigen::Matrix<T,2,1> dNorm_dp(p(0)/r, p(1)/r);
        
        const T fac = DistortionModel::RFactor(r, params + NUM_LIN_PARAMS );
        const Eigen::Matrix<T,2,1> dfac_dp = DistortionModel::dRFactor_dr(r,params + NUM_LIN_PARAMS) * dNorm_dp;
        
        Eigen::Matrix<T,2,2> Jmap;
        Jmap.col(0).operator=( Eigen::Matrix<T,2,1>(
                    dfac_dp(0) *params[0]*p(0) + fac*params[0],
                dfac_dp(0) *params[1]*p(1)
                ) );
        Jmap.col(1).operator=( Eigen::Matrix<T,2,1>(
                    dfac_dp(1) *params[0]*p(0),
                dfac_dp(1) *params[1]*p(1) + fac*params[1]
                ) );
        
        return Jmap;
    }   
    
    template<typename T> inline
    static Eigen::Matrix<T,2,3> dProject_dP(const Eigen::Matrix<T,3,1>& P, const T* params)
    {
        const Eigen::Matrix<T,2,1> p(P(0) / P(2), P(1) / P(2));
        const Eigen::Matrix<T,2,2> _dMap_dp = dMap_dp(p, params);

        Eigen::Matrix<T,2,3> _dp_dP;
        _dp_dP << 
            1.0/P(2), 0, -P(0)/(P(2)*P(2)),
            0, 1.0/P(2), -P(1)/(P(2)*P(2));

        return _dMap_dp * _dp_dP;        
    }    
    
    template<typename T> inline
    static void Scale(T scale, T* params)
    {
        params[0] *= scale;
        params[1] *= scale;
        params[2] = scale*(params[2]+0.5) - 0.5;
        params[3] = scale*(params[3]+0.5) - 0.5;      
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
        return "f_u0_v0" + DistortionModel::Type();
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
    static Eigen::Matrix<T,2,1> Project(const Eigen::Matrix<T,3,1>& P, T const* params)
    {
        return Map( calibu::Project(P) , params );
    }

    template<typename T> inline
    static Eigen::Matrix<T,3,1> Unproject(const Eigen::Matrix<T,2,1>& p, T const* params)
    {
        return calibu::Unproject( Unmap( p, params) );
    }    

    template<typename T> inline
    static Eigen::Matrix<T,2,Eigen::Dynamic> dProject_dParams(const Eigen::Matrix<T,3,1>& /*P*/, const Eigen::Matrix<T,Eigen::Dynamic,1>& /*params*/)
    {
        //TODO: implement this
        assert(false);
        return Eigen::Matrix<T,Eigen::Dynamic,Eigen::Dynamic>();
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
    
    template<typename T> inline
    static Eigen::Matrix<T,2,2> dMap_dp(const Eigen::Matrix<T,2,1>& p, const T* params)
    {
        const T r = p.norm();
        const Eigen::Matrix<T,2,1> dNorm_dp(p(0)/r, p(1)/r);
        
        const T fac = DistortionModel::RFactor(r, params + NUM_LIN_PARAMS );
        const Eigen::Matrix<T,2,1> dfac_dp = DistortionModel::dRFactor_dr(r,params + NUM_LIN_PARAMS) * dNorm_dp;
        
        Eigen::Matrix<T,2,2> J;
        // Using operator= to get around clang bug.
        J.col(0).operator=( Eigen::Matrix<T,2,1>(
                    dfac_dp(0) *params[0]*p(0) + fac*params[1],
                    dfac_dp(0) *params[0]*p(1)
                ) );
        J.col(1).operator=( Eigen::Matrix<T,2,1>(
                    dfac_dp(1) *params[0]*p(0),
                    dfac_dp(1) *params[0]*p(1) + fac*params[1]
                ) );
        
        return J;
    }
    
    template<typename T> inline
    static Eigen::Matrix<T,2,3> dProject_dP(const Eigen::Matrix<T,3,1>& P, const T* params)
    {
        const Eigen::Matrix<T,2,1> p(P(0) / P(2), P(1) / P(2));
        const Eigen::Matrix<T,2,2> _dMap_dp = dMap_dp(p, params);

        Eigen::Matrix<T,2,3> _dp_dP;
        _dp_dP << 
            1.0/P(2), 0, -P(0)/(P(2)*P(2)),
            0, 1.0/P(2), -P(1)/(P(2)*P(2));

        return _dMap_dp * _dp_dP;        
    }    
    
    template<typename T> inline
    static void Scale(T scale, T* params)
    {
        params[0] *= scale;
        params[1] = scale*(params[1]+0.5) - 0.5;
        params[2] = scale*(params[2]+0.5) - 0.5;      
    }    
};

//////////////////////////////////////////////////////////////////////////////
// Convenience typedefs
//////////////////////////////////////////////////////////////////////////////

typedef ProjectionLinear<DistortionPinhole>  Pinhole;
typedef ProjectionLinear<DistortionFov>      Fov;
typedef ProjectionLinear<DistortionPoly2>    Poly2;
typedef ProjectionLinear<DistortionPoly3>    Poly3;

}
