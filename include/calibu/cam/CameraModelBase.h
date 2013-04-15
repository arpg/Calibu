/* 
   This file is part of the Calibu Project.
   https://robotics.gwu.edu/git/calibu

   Copyright (C) 2013 George Washington University

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

#include <sophus/se3.hpp>

namespace calibu
{

//////////////////////////////////////////////////////////////////////////////
// Projection / Unprojection utilities
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
/// Unproject returns .
template<typename T> inline
Eigen::Matrix<T,3,1> Unproject(
        const Eigen::Matrix<T,2,1>& P //< Input:
        )
{
    Eigen::Matrix<T,3,1> ret;
    ret.template head<2>() = P;
    ret[2] = (T)1.0;
    return ret;
}

//////////////////////////////////////////////////////////////////////////////
/// Unproject 
template<typename T> inline
Eigen::Matrix<T,4,1> Unproject(
        const Eigen::Matrix<T,3,1>& P //< Input:
        )
{
    Eigen::Matrix<T,4,1> ret;
    ret.template head<3>() = P;
    ret[3] = (T)1.0;
    return ret;
}

//////////////////////////////////////////////////////////////////////////////
/// This function Projects from 3D into an image, returning a 2x1 image point.
template<typename T> inline
Eigen::Matrix<T,2,1> Project(const Eigen::Matrix<T,3,1>& P)
{
    return Eigen::Matrix<T,2,1>(P(0)/P(2), P(1)/P(2));
}

//////////////////////////////////////////////////////////////////////////////
/// This function Project from homogeneous 3D into an image, returning a 3x1
//  image point.
template<typename T> inline
Eigen::Matrix<T,3,1> Project(
        const Eigen::Matrix<T,4,1>& P //< Input:
        )
{
    return Eigen::Matrix<T,3,1>(P(0)/P(3), P(1)/P(3), P(2)/P(3));
}

//////////////////////////////////////////////////////////////////////////////
/// dNorm_dx returns ... TODO
inline
Eigen::Matrix<double,1,2> dNorm_dx(
        const Eigen::Vector2d& x //< Input:
        )
{
    const double normx = x.norm();
    return Eigen::Matrix<double,1,2>(x(0)/normx, x(1)/normx);
}

//////////////////////////////////////////////////////////////////////////////
// Interface for polymorphic camera class
class CameraModelInterface
{
public:
    //////////////////////////////////////////////////////////////////////////////
    // Virtual member functions
    virtual ~CameraModelInterface(){}

    /// Map from image coordinates to z=1 plane.
    virtual Eigen::Vector2d Map( 
            const Eigen::Vector2d& proj  //< Input:
            ) const = 0;

    /// Map from z=1 plane to image coordinates.
    virtual Eigen::Vector2d Unmap( 
            const Eigen::Vector2d& img //< Input:
            ) const = 0;

    /// Return the perspective projection camera model "K" matrix    
    virtual Eigen::Matrix3d K() const = 0;
 
    /// Return the perspective projection camera model inverse "K" matrix
    virtual Eigen::Matrix3d Kinv() const = 0;

//    virtual static std::string Name() = 0;

    virtual int Width() const = 0;
 
    virtual int Height() const = 0;
 
    virtual void SetImageDimensions( 
            int nWidth,  //< Input:
            int nHeight  //< Input:
            ) = 0;
 
    /// Report camera model version number.
    virtual int Version() const = 0;

    /// Set the camera veriona nuber.
    virtual void SetVersion( int nVersion ) = 0;

    /// Report camera model 
    virtual const char* Type() const = 0;

    virtual void SetType( const std::string& sType ) = 0;

    /// Set the camera model name. e.g., "Left"
    virtual std::string Name() = 0;

    /// Set the camera model name. e.g., "Left"
    virtual void SetName( const std::string& sName ) = 0;
    
    /// Set the camera serial number.
    virtual long int SerialNumber() = 0;

    /// Set the camera serial number.
    virtual void SetSerialNumber( const long int nSerialNo ) = 0;

    /// Set the camera index (for multi-camera rigs).
    virtual int Index() = 0;

    /// Set the camera index (for multi-camera rigs).
    virtual void SetIndex( const int nIndex ) = 0;

    //////////////////////////////////////////////////////////////////////////////
    /// Return 3x3 RDF matrix, describing the coordinate-frame convention.
    virtual Eigen::Matrix3d RDF() const = 0;


    //////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////
    // Project point in 3d camera coordinates to image coordinates
    Eigen::Vector2d ProjectMap(
            const Eigen::Vector3d& P //< Input:
            ) const
    {
        return Map( Project(P) );
    }
 
    //////////////////////////////////////////////////////////////////////////////
    // Create 3D camera coordinates ray from image space coordinates
    Eigen::Vector3d UnmapUnproject(
            const Eigen::Vector2d& p //< Input:
            ) const
    {
        return Unproject( Unmap(p) );
    }        
 
    //////////////////////////////////////////////////////////////////////////////
    /// Transfer point correspondence with known inv. depth to secondary camera frame.
    //  Points at infinity are supported (rho = 0)
    //  rhoPa = unproject(unmap(pa)).
    Eigen::Vector2d Transfer3D(
            const Sophus::SE3d& T_ba,     //< Input:
            const Eigen::Vector3d& rhoPa, //< Input:
            const double rho              //< Input:
            ) const
    {
        // Inverse depth point in a transformed to b (homogeneous 2D)
        const Eigen::Vector3d Pb = T_ba.rotationMatrix() * rhoPa + rho * T_ba.translation();
        
        // to non-homogeneous 2D
        const Eigen::Vector2d proj( Pb(0)/Pb(2), Pb(1)/Pb(2) );
        
        // apply distortion and linear cam
        return Map(proj); 
    }
    
    //////////////////////////////////////////////////////////////////////////////
    // Transfer point correspondence with known inv. depth to secondary camera frame.
    // Points at infinity are supported (rho = 0)
    // rhoPa = unproject(unmap(pa))
    Eigen::Vector2d Transfer3D(
            const Sophus::SE3d& T_ba,     //< Input:
            const Eigen::Vector3d& rhoPa, //< Input:
            const double rho,             //< Input:
            bool& in_front                //< Output:
            ) const
    {            
        // Inverse depth point in a transformed to b (homogeneous 2D)
        const Eigen::Vector3d Pb = T_ba.rotationMatrix() * rhoPa + rho * T_ba.translation();
        
        // to non-homogeneous 2D
        const Eigen::Vector2d proj( Pb(0)/Pb(2), Pb(1)/Pb(2) );
        in_front = Pb(2) > 0;
        
        // apply distortion and linear cam
        return Map(proj); 
    }
    
    //////////////////////////////////////////////////////////////////////////////
    // Transfer point correspondence with known inv. depth to secondary camera frame.
    // Points at infinity are supported (rho = 0)
    Eigen::Vector2d Transfer(
            const Sophus::SE3d& T_ba,  //< Input:
            const Eigen::Vector2d& pa, //< Input:
            const double rho           //< Output:
            ) const
    {
        // rho*Pa (undo distortion, unproject, avoid division by inv depth)
        const Eigen::Vector3d rhoPa = UnmapUnproject(pa);
        return Transfer3D(T_ba, rhoPa, rho);
    }
    
    //////////////////////////////////////////////////////////////////////////////
    // Transfer point correspondence with known inv. depth to secondary camera frame.
    // Points at infinity are supported (rho = 0)
    Eigen::Vector2d Transfer(
            const Sophus::SE3d& T_ba,  //< Input:
            const Eigen::Vector2d& pa, //< Input:
            const double rho,          //< Input:
            bool& in_front             //< Output:
            ) const
    {
        // rho*P1 (undo distortion, unproject, avoid division by inv depth)
        const Eigen::Vector3d rhoPa = UnmapUnproject(pa);
        return Transfer3D(T_ba, rhoPa, rho, in_front);
    }

};

}
