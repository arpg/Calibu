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

#include "CameraModelBase.h"
#include "ProjectionModel.h"

namespace fiducials
{

//////////////////////////////////////////////////////////////////////////////
// Linear Projection and Distortion
//////////////////////////////////////////////////////////////////////////////

template<typename ProjectionModel>
class CameraModel
    : public CameraModelBase
{
public:
    static const unsigned NUM_PARAMS = ProjectionModel::NUM_PARAMS;
            
    ///////////////////////////////////////////////////////////////////////////
    // Static Utilities
    ///////////////////////////////////////////////////////////////////////////

    inline static std::string Name() { return ProjectionModel::Name(); }
    
    // Map from image coordinates to z=1 plane
    template<typename T> inline
    static Eigen::Matrix<T,2,1> Map(const Eigen::Matrix<T,2,1>& proj, T const* params)
    {
        return ProjectionModel::Map(proj, params);
    }
    
    // Map from z=1 plane to image coordinates
    template<typename T> inline
    static Eigen::Matrix<T,2,1> Unmap(const Eigen::Matrix<T,2,1>& img, T const* params)
    {    
        return ProjectionModel::Unmap(img, params);
    }
    
    static inline
    Eigen::Matrix<double,2,3> dMap_dP(const Eigen::Vector3d& P, const double* params)
    {
        const Eigen::Vector2d p(P(0) / P(2), P(1) / P(2));
        const Eigen::Matrix<double,2,2> _dMap_dp = ProjectionModel::dMap_dp(p, params);
        
        Eigen::Matrix<double,2,3> _dp_dP;
        _dp_dP << 
          1.0/P(2), 0, -P(0)/(P(2)*P(2)),
          0, 1.0/P(2), -P(1)/(P(2)*P(2));
                
        return _dMap_dp * _dp_dP;
    }    
    
    // Transfer point correspondence with known inv. depth to secondary camera frame.
    // Points at infinity are supported (rho = 0)
    // rhoPa = unproject(unmap(pa))
    template<typename T> inline
    static Eigen::Matrix<T,2,1> Transfer3D(const T* camparam, const Sophus::SE3Group<T>& T_ba, const Eigen::Matrix<T,3,1>& rhoPa, const T rho)
    {            
        // Inverse depth point in a transformed to b (homogeneous 2D)
        const Eigen::Matrix<T,3,1> Pb =
                T_ba.rotationMatrix() * rhoPa + rho * T_ba.translation();
    
        // to non-homogeneous 2D
        const Eigen::Matrix<T,2,1> proj( Pb(0)/Pb(2), Pb(1)/Pb(2) );
                
        // apply distortion and linear cam
        return Map(proj, camparam); 
    }
    
    // Transfer point correspondence with known inv. depth to secondary camera frame.
    // Points at infinity are supported (rho = 0)
    // rhoPa = unproject(unmap(pa))
    template<typename T> inline
    static Eigen::Matrix<T,2,1> Transfer3D(const T* camparam, const Sophus::SE3Group<T>& T_ba, const Eigen::Matrix<T,3,1>& rhoPa, const T rho, bool& in_front)
    {            
        // Inverse depth point in a transformed to b (homogeneous 2D)
        const Eigen::Matrix<T,3,1> Pb =
                T_ba.rotationMatrix() * rhoPa + rho * T_ba.translation();
    
        // to non-homogeneous 2D
        const Eigen::Matrix<T,2,1> proj( Pb(0)/Pb(2), Pb(1)/Pb(2) );
        in_front = Pb(2) > 0;
                
        // apply distortion and linear cam
        return Map(proj, camparam); 
    }
    
    // Transfer point correspondence with known inv. depth to secondary camera frame.
    // Points at infinity are supported (rho = 0)
    template<typename T> inline
    static Eigen::Matrix<T,2,1> Transfer(const T* camparam, const Sophus::SE3Group<T>& T_ba, const Eigen::Matrix<T,2,1>& pa, const T rho)
    {
        // rho*Pa (undo distortion, unproject, avoid division by inv depth)
        const Eigen::Matrix<T,3,1> rhoPa = Unproject<T>(
            Unmap<T>(pa, camparam)
        );
        
        return Transfer3D(camparam, T_ba, rhoPa, rho);
    }
    
    // Transfer point correspondence with known inv. depth to secondary camera frame.
    // Points at infinity are supported (rho = 0)
    template<typename T> inline
    static Eigen::Matrix<T,2,1> Transfer(const T* camparam, const Sophus::SE3Group<T>& T_ba, const Eigen::Matrix<T,2,1>& pa, const T rho, bool& in_front)
    {
        // rho*P1 (undo distortion, unproject, avoid division by inv depth)
        const Eigen::Matrix<T,3,1> rhoPa = Unproject<T>(
            Unmap<T>(pa, camparam)
        );
            
        return Transfer3D(camparam, T_ba, rhoPa, rho, in_front);
    }
    
    ///////////////////////////////////////////////////////////////////////////
    // Member functions
    ///////////////////////////////////////////////////////////////////////////

    CameraModel()
        : params(Eigen::Matrix<double,NUM_PARAMS,1>::Zero())
    {
    }    

    CameraModel(int w, int h)
        : width(w), height(h), params(Eigen::Matrix<double,NUM_PARAMS,1>::Zero())
    {
    }    
    
    CameraModel(const Eigen::Matrix<double,NUM_PARAMS,1>& params)
        : params(params)
    {
    }    
    
    CameraModel(int w, int h, const Eigen::Matrix<double,NUM_PARAMS,1>& params)
        : width(w), height(h), params(params)
    {
    }        

    CameraModel(double* cam_params)
        : params(Eigen::Map<Eigen::Matrix<double,NUM_PARAMS,1> >(cam_params))
    {
    }    
    
    CameraModel(const CameraModel& other)
        : params(other.params)
    {
    }    
    
    Eigen::Matrix<double,NUM_PARAMS,1>& Params() {
        return params;
    }

    const Eigen::Matrix<double,NUM_PARAMS,1>& Params() const {
        return params;
    }
    
    const double* data() const {
        return params.data();
    }

    double* data() {
        return params.data();
    }
    
    inline Eigen::Vector2d Map(const Eigen::Vector2d& proj) const
    {
        return Map(proj, params.data());
    }

    inline Eigen::Vector2d Unmap(const Eigen::Vector2d& img) const
    {
        return Unmap(img, params.data());
    }
    
    Eigen::Matrix3d MakeK() const
    {
        return ProjectionModel::MakeK(params.data());
    }
    
    Eigen::Matrix3d MakeKinv() const
    {
        return ProjectionModel::MakeKinv(params.data());
    }    
    
    inline Eigen::Vector2d ProjectMap(const Eigen::Vector3d& P) const
    {
        return Map( Project(P) , params.data() );
    }    
    
    inline Eigen::Vector3d UnmapUnproject(const Eigen::Vector2d& p) const
    {
        return Unproject( Unmap( p, params.data()) );
    }        
    
    inline Eigen::Vector2d Transfer(const Sophus::SE3d& T_ba, const Eigen::Vector2d& pa, double rho) const
    {
        return Transfer<double>(data(), T_ba, pa, rho);
    }

    inline Eigen::Vector2d Transfer(const Sophus::SE3d& T_ba, const Eigen::Vector2d& pa, double rho, bool& in_front) const
    {
        return Transfer<double>(data(), T_ba, pa, rho, in_front);
    }
    
    inline Eigen::Matrix3d K()
    {
        return MakeK(params.data());
    }

    inline Eigen::Matrix3d Kinv()
    {
        return MakeKinv(params.data());
    }
    
    int& Width() { return width; }
    int Width() const { return width; }

    int& Height() { return height; }
    int Height() const { return height; }
    
protected:
    int width;
    int height;
    Eigen::Matrix<double,NUM_PARAMS,1> params;
};

}
