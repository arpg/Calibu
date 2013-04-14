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
/// This function Project from 3D into an image, returning a 2x1 image point.
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
/// dNorm_dx returns ... 
inline
Eigen::Matrix<double,1,2> dNorm_dx(
        const Eigen::Vector2d& x //< Input:
        )
{
    const double normx = x.norm();
    return Eigen::Matrix<double,1,2>(x(0)/normx, x(1)/normx);
}

//////////////////////////////////////////////////////////////////////////////
// Polymorphic camera base class
//////////////////////////////////////////////////////////////////////////////

class CameraModelBase
{
public:
    //////////////////////////////////////////////////////////////////////////////    
    // Virtual member functions
    //////////////////////////////////////////////////////////////////////////////    
    
    virtual Eigen::Vector2d Map( 
            const Eigen::Vector2d& proj  //< Input:
            ) const = 0;
    virtual Eigen::Vector2d Unmap( 
            const Eigen::Vector2d& img //< Input:
            ) const = 0;

    /// Return the perspective projection camera model "K" matrix    
    virtual Eigen::Matrix3d K() const = 0;
 
    /// Return the perspective projection camera model inverse "K" matrix
    virtual Eigen::Matrix3d Kinv() const = 0;

    //////////////////////////////////////////////////////////////////////////////
    // Constructors
    
    CameraModelBase()
        : m_nWidth(0), m_nHeight(0)
    {
    }
    
    CameraModelBase(
            int nWidth, //< Input:
            int nHeight //< Input:
            )
        : m_nWidth(nWidth), m_nHeight(nHeight)
    {
    }
    
    //////////////////////////////////////////////////////////////////////////////
    // Image Dimentions
    
    int& Width() {
        return m_nWidth;
    }
    
    int Width() const
    {
        return m_nWidth;
    }
    
    int& Height()
    {
        return m_nHeight;
    }
    
    int Height() const
    {
        return m_nHeight;
    }
    
    void SetImageDimensions(
            int nWidth,  //< Input:
            int nHeight  //< Input:
            )
    {
        m_nWidth = nWidth;
        m_nHeight = nHeight;
    }
    
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



    //////////////////////////////////////////////////////////////////////////////
    /// Report camera model version number.
    int Version() const
    {
        return m_nVersion;
    }

    //////////////////////////////////////////////////////////////////////////////
    /// Set the camera veriona nuber.
    void SetVersion( int nVersion )
    {
        m_nVersion = nVersion;
    }

    //////////////////////////////////////////////////////////////////////////////
    /// Report camera model 
    const char* Type() const
    {
        return m_sType.c_str();
    }

    //////////////////////////////////////////////////////////////////////////////
    /// Set the camera model type.
    void SetType( const std::string& sType )
    {
        m_sType = sType;
    }

    //////////////////////////////////////////////////////////////////////////////
    /// Set the camera model name. e.g., "Left"
    void SetName( const std::string& sName )
    {
        m_sName = sName;
    }

    //////////////////////////////////////////////////////////////////////////////
    /// Set the camera serial number.
    void SetSerialNumber( const long int nSerialNo )
    {
        m_nSerialNo = nSerialNo;
    }

    //////////////////////////////////////////////////////////////////////////////
    /// Set the camera index (for multi-camera rigs).
    void SetIndex( const int nIndex )
    {
        m_nIndex = nIndex;
    }

    //////////////////////////////////////////////////////////////////////////////
    /// Return 3x3 RDF matrix, describing the coordinate-frame convention.
    Eigen::Matrix3d RDF() const
    {
        return m_RDF;
    }

    //////////////////////////////////////////////////////////////////////////////
protected:
    int              m_nWidth;    //< Camera width, in pixels
    int              m_nHeight;   //< Camera height, in pixels

    std::string      m_sType;     //< Model type name.
    std::string      m_sName;     //< particular camera name, e.g., "Left"
    int              m_nVersion;  //< Calibu or MVL camera model version.
    long int         m_nSerialNo; //< Camera serial number, if appropriate.
    int              m_nIndex;    //< Camera index, for multi-camera systems.
    Eigen::Matrix3d  m_RDF;       //< Define coordinate-frame convention from Right, Down, Forward vectors.
};

}
