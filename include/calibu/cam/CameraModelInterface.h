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
template<typename Scalar>
class CameraModelInterfaceT
{   
public:
    typedef Eigen::Matrix<Scalar,2,1> Vector2t;
    typedef Eigen::Matrix<Scalar,3,1> Vector3t;
    typedef Eigen::Matrix<Scalar,Eigen::Dynamic,1> VectorXt;
    typedef Eigen::Matrix<Scalar,3,3> Matrix3t;
    typedef Sophus::SE3Group<Scalar> SE3t;

    //////////////////////////////////////////////////////////////////////////////
    // Virtual member functions
    virtual ~CameraModelInterfaceT(){}

    /// Map from image coordinates to z=1 plane.
    virtual Vector2t Map(
            const Vector2t& proj  //< Input:
            ) const = 0;

    /// Map from z=1 plane to image coordinates.
    virtual Vector2t Unmap(
            const Vector2t& img //< Input:
            ) const = 0;

    /// Return the perspective projection camera model "K" matrix    
    virtual Matrix3t K() const = 0;
 
    /// Return the perspective projection camera model inverse "K" matrix
    virtual Matrix3t Kinv() const = 0;

    virtual size_t Width() const = 0;
 
    virtual size_t Height() const = 0;

    virtual VectorXt GenericParams() const = 0;

    virtual void SetGenericParams(const VectorXt& params) = 0;
 
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
    virtual Matrix3t RDF() const = 0;
    
    //////////////////////////////////////////////////////////////////////////////
    /// Set the 3x3 RDF matrix, describing the coordinate-frame convention.
    /// Row0: Right vector in camera frame of reference
    /// Row1: Down vector in camera frame of reference
    /// Row2: Forward vector in camera frame of reference
    virtual void SetRDF( const Matrix3t& RDF ) = 0;

    virtual Eigen::Matrix<Scalar,2,3> dMap_dP(const Vector3t& P) const = 0;
    virtual Eigen::Matrix<Scalar,2,4> dTransfer3D_dP(
            const SE3t& T_ba,   //< Input:
            const Eigen::Matrix<Scalar,3,1>& rhoPa, //< Input:
            const Scalar rho                        //< Input:
            ) const  = 0;

    //////////////////////////////////////////////////////////////////////////////
    virtual void PrintInfo() = 0;
 
    //////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////
    // Project point in 3d camera coordinates to image coordinates
    inline Vector2t ProjectMap(
            const Vector3t& P //< Input:
            ) const
    {
        return Map( Project(P) );
    }
 
    //////////////////////////////////////////////////////////////////////////////
    // Create 3D camera coordinates ray from image space coordinates
    inline Vector3t UnmapUnproject(
            const Vector2t& p //< Input:
            ) const
    {
        return Unproject( Unmap(p) );
    }        
 
    //////////////////////////////////////////////////////////////////////////////
    /// Transfer point correspondence with known inv. depth to secondary camera frame.
    //  Points at infinity are supported (rho = 0)
    //  rhoPa = unproject(unmap(pa)).
    inline Vector2t Transfer3D(
            const SE3t& T_ba,     //< Input:
            const Vector3t& rhoPa, //< Input:
            const Scalar rho              //< Input:
            ) const
    {
        // Inverse depth point in a transformed to b (homogeneous 2D)
        const Vector3t Pb = T_ba.rotationMatrix() * rhoPa + rho * T_ba.translation();
        
        // to non-homogeneous 2D
        const Vector2t proj( Pb(0)/Pb(2), Pb(1)/Pb(2) );
        
        // apply distortion and linear cam
        return Map(proj); 
    }
    
    //////////////////////////////////////////////////////////////////////////////
    // Transfer point correspondence with known inv. depth to secondary camera frame.
    // Points at infinity are supported (rho = 0)
    // rhoPa = unproject(unmap(pa))
    inline Vector2t Transfer3D(
            const SE3t& T_ba,     //< Input:
            const Vector3t& rhoPa, //< Input:
            const Scalar rho,             //< Input:
            bool& in_front                //< Output:
            ) const
    {
        // Inverse depth point in a transformed to b (homogeneous 2D)
        const Vector3t Pb = T_ba.rotationMatrix() * rhoPa + rho * T_ba.translation();
        
        // to non-homogeneous 2D
        const Vector2t proj( Pb(0)/Pb(2), Pb(1)/Pb(2) );
        in_front = Pb(2) > 0;
        
        // apply distortion and linear cam
        return Map(proj); 
    }
 
    //////////////////////////////////////////////////////////////////////////////
    // Transfer point correspondence with known inv. depth to secondary camera frame.
    // Points at infinity are supported (rho = 0)
    inline Vector2t Transfer(
            const SE3t& T_ba,  //< Input:
            const Vector2t& pa, //< Input:
            const Scalar rho           //< Output:
            ) const
    {
        // rho*Pa (undo distortion, unproject, avoid division by inv depth)
        const Vector3t rhoPa = UnmapUnproject(pa);
        return Transfer3D(T_ba, rhoPa, rho);
    }
 
    //////////////////////////////////////////////////////////////////////////////
    // Transfer point correspondence with known inv. depth to secondary camera frame.
    // Points at infinity are supported (rho = 0)
    inline Vector2t Transfer(
            const SE3t& T_ba,  //< Input:
            const Vector2t& pa, //< Input:
            const Scalar rho,          //< Input:
            bool& in_front             //< Output:
            ) const
    {
        // rho*P1 (undo distortion, unproject, avoid division by inv depth)
        const Vector3t rhoPa = UnmapUnproject(pa);
        return Transfer3D(T_ba, rhoPa, rho, in_front);
    }
    
    //////////////////////////////////////////////////////////////////////////////
    // Transfer point correspondence with known inv. depth to secondary camera frame.
    // Points at infinity are supported (rho = 0)
    inline Vector2t Transfer(
            const CameraModelInterfaceT<Scalar>& cam_a,
            const SE3t& T_ba,  //< Input:
            const Vector2t& pa, //< Input:
            const Scalar rho           //< Output:
            ) const
    {
        // rho*Pa (undo distortion, unproject, avoid division by inv depth)
        const Vector3t rhoPa = cam_a.UnmapUnproject(pa);
        return Transfer3D(T_ba, rhoPa, rho);
    }
    
    //////////////////////////////////////////////////////////////////////////////
    // Transfer point correspondence with known inv. depth to secondary camera frame.
    // Points at infinity are supported (rho = 0)
    inline Vector2t Transfer(
            const CameraModelInterfaceT<Scalar>& cam_a,            
            const SE3t& T_ba,  //< Input:
            const Vector2t& pa, //< Input:
            const Scalar rho,          //< Input:
            bool& in_front             //< Output:
            ) const
    {
        // rho*P1 (undo distortion, unproject, avoid division by inv depth)
        const Vector3t rhoPa = cam_a.UnmapUnproject(pa);
        return Transfer3D(T_ba, rhoPa, rho, in_front);
    }
};

typedef CameraModelInterfaceT<double> CameraModelInterface;

}
