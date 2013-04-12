/* This file is part of the calibu Project.
 * https://github.com/stevenlovegrove/calibu
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

#include <sophus/se3.hpp>

namespace calibu
{

//////////////////////////////////////////////////////////////////////////////
// Projection / Unprojection utilities
//////////////////////////////////////////////////////////////////////////////

template<typename T> inline
Eigen::Matrix<T,3,1> Unproject(const Eigen::Matrix<T,2,1>& P)
{
    Eigen::Matrix<T,3,1> ret;
    ret.template head<2>() = P;
    ret[2] = (T)1.0;
    return ret;
}

template<typename T> inline
Eigen::Matrix<T,4,1> Unproject(const Eigen::Matrix<T,3,1>& P)
{
    Eigen::Matrix<T,4,1> ret;
    ret.template head<3>() = P;
    ret[3] = (T)1.0;
    return ret;
}

template<typename T> inline
Eigen::Matrix<T,2,1> Project(const Eigen::Matrix<T,3,1>& P)
{
    return Eigen::Matrix<T,2,1>(P(0)/P(2), P(1)/P(2));
}

template<typename T> inline
Eigen::Matrix<T,3,1> Project(const Eigen::Matrix<T,4,1>& P)
{
    return Eigen::Matrix<T,3,1>(P(0)/P(3), P(1)/P(3), P(2)/P(3));
}

inline
Eigen::Matrix<double,1,2> dNorm_dx(const Eigen::Vector2d& x)
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
    
    virtual Eigen::Vector2d Map(const Eigen::Vector2d& proj) const = 0;
    virtual Eigen::Vector2d Unmap(const Eigen::Vector2d& img) const = 0;
    
    virtual Eigen::Matrix3d K() const = 0;
    virtual Eigen::Matrix3d Kinv() const = 0;
    
    //////////////////////////////////////////////////////////////////////////////
    // Constructors
    
    CameraModelBase()
        : m_width(0), m_height(0)
    {
    }
    
    CameraModelBase(int width, int height)
        : m_width(width), m_height(height)
    {
    }
    
    //////////////////////////////////////////////////////////////////////////////
    // Image Dimentions
    
    int& Width() {
        return m_width;
    }
    
    int Width() const
    {
        return m_width;
    }
    
    int& Height()
    {
        return m_height;
    }
    
    int Height() const
    {
        return m_height;
    }
    
    void SetImageDimensions(int width, int height)
    {
        m_width = width;
        m_height = height;
    }
    
    //////////////////////////////////////////////////////////////////////////////
    // Project point in 3d camera coordinates to image coordinates
    Eigen::Vector2d ProjectMap(const Eigen::Vector3d& P) const
    {
        return Map( Project(P) );
    }    
    
    //////////////////////////////////////////////////////////////////////////////
    // Create 3D camera coordinates ray from image space coordinates
    Eigen::Vector3d UnmapUnproject(const Eigen::Vector2d& p) const
    {
        return Unproject( Unmap(p) );
    }        
    
    //////////////////////////////////////////////////////////////////////////////
    // Transfer point correspondence with known inv. depth to secondary camera frame.
    // Points at infinity are supported (rho = 0)
    // rhoPa = unproject(unmap(pa))
    Eigen::Vector2d Transfer3D(const Sophus::SE3d& T_ba, const Eigen::Vector3d& rhoPa, const double rho) const
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
    Eigen::Vector2d Transfer3D(const Sophus::SE3d& T_ba, const Eigen::Vector3d& rhoPa, const double rho, bool& in_front) const
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
    Eigen::Vector2d Transfer(const Sophus::SE3d& T_ba, const Eigen::Vector2d& pa, const double rho) const
    {
        // rho*Pa (undo distortion, unproject, avoid division by inv depth)
        const Eigen::Vector3d rhoPa = UnmapUnproject(pa);
        return Transfer3D(T_ba, rhoPa, rho);
    }
    
    //////////////////////////////////////////////////////////////////////////////
    // Transfer point correspondence with known inv. depth to secondary camera frame.
    // Points at infinity are supported (rho = 0)
    Eigen::Vector2d Transfer(const Sophus::SE3d& T_ba, const Eigen::Vector2d& pa, const double rho, bool& in_front) const
    {
        // rho*P1 (undo distortion, unproject, avoid division by inv depth)
        const Eigen::Vector3d rhoPa = UnmapUnproject(pa);
        return Transfer3D(T_ba, rhoPa, rho, in_front);
    }
    
protected:
    int m_width;
    int m_height;    
};

}
