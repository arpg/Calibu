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
#include <stdexcept>

#include "ProjectionModel.h"

namespace calibu {

// Kannala and Brandt Like 'Generic' Projection Model
// http://cs.iupui.edu/~tuceryan/pdf-repository/Kannala2006.pdf
// http://april.eecs.umich.edu/wiki/index.php/Camera_suite
struct ProjectionKannalaBrandt
{
    typedef ProjectionLinear<DistortionPinhole> DistortionFreeModel;
    static const unsigned NUM_PARAMS = 8;
 
    inline static std::string Type()
    {
        return "fu_fv_u0_v0_kb4";
    }
     
    template<typename T> inline
    static Eigen::Matrix<T,2,1> Project(const Eigen::Matrix<T,3,1>& P, T const* params)
    {
        // Projection
        const T fu = params[0];
        const T fv = params[1];
        const T u0 = params[2];
        const T v0 = params[3];
        
        const T k0 = params[4];
        const T k1 = params[5];
        const T k2 = params[6];
        const T k3 = params[7];
        
        const T Xsq_plus_Ysq = P(0)*P(0)+P(1)*P(1);
        const T theta = atan2( sqrt(Xsq_plus_Ysq), P(2) );
        const T psi = atan2( P(1), P(0) );
        
        const T theta2 = theta*theta;
        const T theta3 = theta2*theta;
        const T theta5 = theta3*theta2;
        const T theta7 = theta5*theta2;
        const T theta9 = theta7*theta2;
        const T r = theta + k0*theta3 + k1*theta5 + k2*theta7 + k3*theta9;
        
        return Eigen::Matrix<T,2,1>( fu*r*cos(psi) + u0, fv*r*sin(psi) + v0 );
    }

    template<typename T> inline
    static Eigen::Matrix<T,3,1> Unproject(const Eigen::Matrix<T,2,1>& p, T const* params)
    {
        // Unprojection
        const T fu = params[0];
        const T fv = params[1];
        const T u0 = params[2];
        const T v0 = params[3];
        
        const T k0 = params[4];
        const T k1 = params[5];
        const T k2 = params[6];
        const T k3 = params[7];

        const T un = p(0) - u0;
        const T vn = p(1) - v0;
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
        
        return Eigen::Matrix<T,3,1>( sin(th)*cos(psi), sin(th)*sin(psi), cos(th) );
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
        K << 1.0/params[0], 0, -params[2] / params[0],
                0, 1.0/params[1], -params[3] / params[1],
                0, 0, 1;
        return K;
    }
    
    template<typename T> inline
    static Eigen::Matrix<T,2,3> dProject_dP(const Eigen::Matrix<T,3,1>& P, const T* ps)
    {
        const T fu = ps[0];
        const T fv = ps[1];
        const T k0 = ps[4];
        const T k1 = ps[5];
        const T k2 = ps[6];
        const T k3 = ps[7];
        
        const T x0 = P(0)*P(0);
        const T x1 = P(1)*P(1);
        const T x2 = x0 + x1;
        const T x3 = sqrt(x2);
        const T x4 = std::pow(x2,3.0/2.0);
        const T x5 = P(2)*P(2) + x2;
        const T a = atan2(x3, P(2));
        const T a2 = a*a;
        const T a4 = a2*a2;
        const T a6 = a4*a2;
        const T a8 = a4*a4;
        const T x11 = k0*a2 + k1*a4 + k2*a6 + k3*a8 + 1;
        const T x12 = 3*k0*a2 + 5*k1*a4 + 7*k2*a6 + 9*k3*a8 + 1;
        const T x13 = P(2)*x12/(x2*x5) - x11*a/x4;
        const T x14 = P(0)*fu;
        const T x15 = P(1)*fv;
        const T x16 = -1/x4;
        const T x17 = x11*a;
        const T x18 = -x12/x5;
        const T x19 = x17/x3;
        const T x20 = P(2)*x12/(x2*x5);
              
        Eigen::Matrix<T,2,3> _dProj_dP;
        _dProj_dP << 
            fu*(x0*x16*x17 + x0*x20 + x19), P(1)*x13*x14, x14*x18,
            P(0)*x13*x15, fv*(x1*x16*x17 + x1*x20 + x19), x15*x18;
        
        return _dProj_dP;
    }

    template<typename T> inline
    static Eigen::Matrix<T,2,Eigen::Dynamic> dProject_dParams(const Eigen::Matrix<T,3,1>& P, const Eigen::Matrix<T,Eigen::Dynamic,1>& params)
    {
        //TODO: implement this
        assert(false);
        return Eigen::Matrix<T,Eigen::Dynamic,Eigen::Dynamic>();
    }
    
    // Scale parameters to cope with scaled image.
    // Scale = width_to / width_from
    template<typename T> inline
    static void Scale(T scale, T* params)
    {
        params[0] *= scale;
        params[1] *= scale;
        params[2] = scale*(params[2]+0.5) - 0.5;
        params[3] = scale*(params[3]+0.5) - 0.5;      
        // Distortion cooefficients aren't effected by scale
    }    
};

}
