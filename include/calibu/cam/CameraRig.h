/*
   This file is part of the Calibu Project.
   https://github.com/gwu-robotics/Calibu

   Copyright (C) 2013 George Washington University,
                      Steven Lovegrove,
                      Gabe Sibley

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

#include <calibu/Platform.h>
#include <calibu/cam/camera_crtp.h>
#include <calibu/cam/camera_crtp_impl.h>
#include <calibu/cam/camera_models_crtp.h>
#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <sophus/se3.hpp>
#include <vector>

namespace calibu
{

//////////////////////////////////////////////////////////////////////////////
template<typename Scalar=double>
class CameraModelAndTransformT
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    // Default constructor
    CameraModelAndTransformT()
    {
    }

    CameraModelAndTransformT( const CameraInterface<Scalar>& camera,
                              const Sophus::SE3Group<Scalar>& t_wc )
        : camera(camera), t_wc(t_wc)
    {

    }

    // Templated Copy Constructor
    template<typename T>
    CameraModelAndTransformT(const CameraModelAndTransformT<T>& other) {
        camera = other.camera;
        t_wc = other.t_wc.template cast<Scalar>();
    }

    CameraInterface<Scalar> camera;
    Sophus::SE3Group<Scalar> t_wc;
};
typedef CameraModelAndTransformT<double> CameraModelAndTransform;

//////////////////////////////////////////////////////////////////////////////
template<typename Scalar=double>
class CameraRigT
{
public:
    // Default Constructor
    CameraRigT() {
    }

    // Templated Copy Constructor
    template<typename T>
    CameraRigT(const CameraRigT<T>& other) {
        for( const CameraModelAndTransformT<T>& cmat : other.cameras ) {
            cameras.push_back( CameraModelAndTransformT<Scalar>(cmat) );
        }
    }

    void Add(const CameraModelAndTransformT<Scalar>& cop) {
        cameras.push_back(cop);
    }
    void Add(const CameraModelInterfaceT<Scalar>& cam, const Sophus::SE3Group<Scalar>& t_wc) {
        CameraModelAndTransformT<Scalar> cop;
        cop.cameras = CameraInterface<Scalar>(cam);
        cop.t_wc = t_wc;
        cameras.push_back(cop);
    }

  std::vector<CameraModelAndTransformT<Scalar>,
              Eigen::aligned_allocator<CameraModelAndTransformT<Scalar> > > cameras;
};
typedef CameraRigT<double> CameraRig;

//////////////////////////////////////////////////////////////////////////////

static const Sophus::SO3d RdfVision =
        Sophus::SO3d( (Eigen::Matrix3d() << 1,0,0, 0,1,0, 0,0,1).finished() );

static const Sophus::SO3d RdfRobotics =
//        Sophus::SO3d( (Eigen::Matrix3d() << 0,1,0, 0,0,1, 1,0,0).finished() );
        Sophus::SO3d( (Eigen::Matrix3d() << 0,0,1, 1,0,0, 0,1,0).finished() );

// T_2b_1b = T_ba * T_2a_1a * T_ab
template<typename Scalar=double>
inline Sophus::SE3Group<Scalar> ToCoordinateConvention(
        const Sophus::SE3Group<Scalar>& T_2a_1a,
        const Sophus::SO3Group<Scalar>& R_ba
        )
{
    Sophus::SE3Group<Scalar> T_2b_1b;
    T_2b_1b.so3() = R_ba * T_2a_1a.so3() * R_ba.inverse();
    T_2b_1b.translation() = R_ba * T_2a_1a.translation();
    return T_2b_1b;
}

template<typename Scalar=double>
inline CameraRigT<Scalar> ToCoordinateConvention(const CameraRigT<Scalar>& rig, const Sophus::SO3Group<Scalar>& rdf)
{
    CameraRigT<Scalar> ret = rig;
    for(size_t c=0; c<ret.cameras.size(); ++c) {
        const Sophus::SO3Group<Scalar> M = rdf * Sophus::SO3Group<Scalar>(rig.cameras[c].camera.RDF()).inverse();
        ret.cameras[c].T_wc = ToCoordinateConvention(rig.cameras[c].T_wc, M);
        ret.cameras[c].camera.SetRDF(rdf.matrix());
    }
    return ret;
}


}
