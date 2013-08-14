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
        return "fu_fv_u0_v0_KannalaBrandt4";
    }
     
    template<typename T> inline
    static Eigen::Matrix<T,2,1> ProjectMap(const Eigen::Matrix<T,3,1>& P, T const* params)
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
    static Eigen::Matrix<T,3,1> UnmapUnproject(const Eigen::Matrix<T,2,1>& p, T const* params)
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
    static Eigen::Matrix<T,2,3> dMap_dP(const Eigen::Matrix<T,3,1>& P, const T* params)
    {
        throw std::runtime_error("Not implemented");
    }
};

}
