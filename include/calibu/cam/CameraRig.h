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

#include <calibu/cam/CameraModel.h>
#include <sophus/se3.hpp>
#include <vector>

namespace calibu
{

//////////////////////////////////////////////////////////////////////////////

class CameraModelAndPose
{
public:
    CameraModel camera;
    Sophus::SE3d T_wc;
};

//////////////////////////////////////////////////////////////////////////////

class CameraRig
{
public:
    inline void Add(const CameraModelAndPose& cop) {
        cameras.push_back(cop);
    }
    inline void Add(const CameraModelInterface& cam, const Sophus::SE3d& T_wc) {
        CameraModelAndPose cop;
        cop.camera = CameraModel(cam);
        cop.T_wc = T_wc;
        cameras.push_back(cop);
    }
    
    std::vector<CameraModelAndPose> cameras;
};

//////////////////////////////////////////////////////////////////////////////

static const Sophus::SO3d RdfVision =
        Sophus::SO3d( (Eigen::Matrix3d() << 1,0,0, 0,1,0, 0,0,1).finished() );

static const Sophus::SO3d RdfRobotics = 
        Sophus::SO3d( (Eigen::Matrix3d() << 0,1,0, 0,0,1, 1,0,0).finished() );

// T_2b_1b = T_ba * T_2a_1a * T_ab
inline Sophus::SE3d ToCoordinateConvention(
        const Sophus::SE3d& T_2a_1a,
        const Sophus::SO3d& R_ba
        )
{
    Sophus::SE3d T_2b_1b;
    T_2b_1b.so3() = R_ba * T_2a_1a.so3() * R_ba.inverse();
    T_2b_1b.translation() = R_ba * T_2a_1a.translation();
    return T_2b_1b;
}

inline CameraRig ToCoordinateConvention(const CameraRig& rig, const Sophus::SO3d& rdf)
{
    CameraRig ret = rig;
    for(size_t c=0; c<ret.cameras.size(); ++c) {
        const Sophus::SO3d M = rdf * Sophus::SO3d(rig.cameras[c].camera.RDF()).inverse();
        ret.cameras[c].T_wc = ToCoordinateConvention(rig.cameras[c].T_wc, M);
        ret.cameras[c].camera.SetRDF(rdf.matrix());
    }
    return ret;
}


}
