/* 
   This file is part of the Calibu Project.
   https://robotics.gwu.edu/git/calibu

   Copyright (C) 2013 George Washington University
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

#include <sophus/se3.hpp>

#include "CameraUtils.h"

namespace calibu
{

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

    virtual size_t Width() const = 0;
 
    virtual size_t Height() const = 0;

    virtual Eigen::VectorXd GenericParams() const = 0;

    virtual void SetGenericParams(const Eigen::VectorXd& params) = 0;
 
    virtual void SetImageDimensions( 
            size_t nWidth,  //< Input:
            size_t nHeight  //< Input:
            ) = 0;
 
    /// Report camera model version number.
    virtual int Version() const = 0;

    /// Set the camera veriona nuber.
    virtual void SetVersion( int nVersion ) = 0;

    /// Report camera model 
    virtual std::string Type() const = 0;

    /// Set the camera model name. e.g., "Left"
    virtual std::string Name() const = 0;

    /// Set the camera model name. e.g., "Left"
    virtual void SetName( const std::string& sName ) = 0;
    
    /// Set the camera serial number.
    virtual long int SerialNumber() const = 0;

    /// Set the camera serial number.
    virtual void SetSerialNumber( const long int nSerialNo ) = 0;

    /// Set the camera index (for multi-camera rigs).
    virtual int Index() const = 0;

    /// Set the camera index (for multi-camera rigs).
    virtual void SetIndex( const int nIndex ) = 0;

    //////////////////////////////////////////////////////////////////////////////
    /// Return 3x3 RDF matrix, describing the coordinate-frame convention.
    /// Row0: Right vector in camera frame of reference
    /// Row1: Down vector in camera frame of reference
    /// Row2: Forward vector in camera frame of reference
    virtual Eigen::Matrix3d RDF() const = 0;
    
    //////////////////////////////////////////////////////////////////////////////
    /// Set the 3x3 RDF matrix, describing the coordinate-frame convention.
    /// Row0: Right vector in camera frame of reference
    /// Row1: Down vector in camera frame of reference
    /// Row2: Forward vector in camera frame of reference
    virtual void SetRDF( const Eigen::Matrix3d& RDF ) = 0;


    //////////////////////////////////////////////////////////////////////////////
    virtual void PrintInfo() = 0;
 
    //////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////
    // Project point in 3d camera coordinates to image coordinates
    inline Eigen::Vector2d ProjectMap(
            const Eigen::Vector3d& P //< Input:
            ) const
    {
        return Map( Project(P) );
    }
 
    //////////////////////////////////////////////////////////////////////////////
    // Create 3D camera coordinates ray from image space coordinates
    inline Eigen::Vector3d UnmapUnproject(
            const Eigen::Vector2d& p //< Input:
            ) const
    {
        return Unproject( Unmap(p) );
    }        
 
    //////////////////////////////////////////////////////////////////////////////
    /// Transfer point correspondence with known inv. depth to secondary camera frame.
    //  Points at infinity are supported (rho = 0)
    //  rhoPa = unproject(unmap(pa)).
    inline Eigen::Vector2d Transfer3D(
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
    inline Eigen::Vector2d Transfer3D(
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
    inline Eigen::Vector2d Transfer(
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
    inline Eigen::Vector2d Transfer(
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