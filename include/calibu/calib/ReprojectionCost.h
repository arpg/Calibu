/* 
   This file is part of the Calibu Project.
   https://robotics.gwu.edu/git/calibu

   Copyright (C) 2013 George Washington University
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

#include <calibu/calib/CostFunctionAndParams.h>
#include <calibu/calib/AutoDiffArrayCostFunction.h>

namespace calibu
{

// Parameter block 0: T_kw // keyframe
// Parameter block 1: T_ck // keyframe to cam
// Parameter block 2: fu,fv,u0,v0,w
template<typename ProjModel>
struct ReprojectionCost
        : public ceres::AutoDiffArrayCostFunction<
        CostFunctionAndParams, ReprojectionCost<ProjModel>,
        2,  Sophus::SE3d::num_parameters, Sophus::SE3d::num_parameters,
        ProjModel::NUM_PARAMS>
{
    ReprojectionCost(Eigen::Vector3d Pw, Eigen::Vector2d pc)
        : m_Pw(Pw), m_pc(pc)
    {        
    }
    
    template<typename T=double>
    bool Evaluate(T const* const* parameters, T* residuals) const
    {
        Eigen::Map<Eigen::Matrix<T,2,1> > r(residuals);
        const Eigen::Map<const Sophus::SE3Group<T> > T_kw(parameters[0]);
        const Eigen::Map<const Sophus::SE3Group<T> > T_ck(parameters[1]);
        T const* camparam = parameters[2];
        
        const Eigen::Matrix<T,3,1> Pc = T_ck * (T_kw * m_Pw.cast<T>());
        const Eigen::Matrix<T,2,1> pc = ProjModel::template Map<T>(Project<T>(Pc), camparam);
        r = pc - m_pc.cast<T>();
        return true;
    }    
    
    Eigen::Vector3d m_Pw;
    Eigen::Vector2d m_pc;
};

}
