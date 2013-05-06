/* 
   This file is part of the Calibu Project.
   https://robotics.gwu.edu/git/calibu
   
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

#include "CameraModelInterface.h"
#include "CameraModelT.h"

#include <iostream>

namespace calibu
{

///////////////////////////////////////////////////////////////////////////
inline CameraModelInterface* CameraModelFactory( const std::string sModelName )
{
    if ( sModelName == "Pinhole" ){
        return new  CameraModelT<Pinhole>();
    }
    return NULL;
}

///////////////////////////////////////////////////////////////////////////
/// Generic CameraModel class.  Manual polymorphism.
///////////////////////////////////////////////////////////////////////////
class CameraModel : public CameraModelInterface
{
public:
    /////////////////////////////////////////////////////////////////////////
    // Constructors
    /////////////////////////////////////////////////////////////////////////
    CameraModel() : 
        m_pCam(NULL)
    {
    }
    
    CameraModel( const CameraModel& rRHS )
    {
        Init( rRHS.Type() );
    }
    
    CameraModel( CameraModelInterface* pCam )
    {
        m_pCam = pCam;
    }
    
    /////////////////////////////////////////////////////////////////////////
    // Member functions
    /////////////////////////////////////////////////////////////////////////
    
    /// Report camera model version number.
    int Version() const
    {
        return m_pCam->Version();
    }
    
    /// Set the camera veriona nuber.
    void SetVersion( int nVersion )
    {
        m_pCam->SetVersion( nVersion );
    }
    
    std::string Type() const
    {
        return m_pCam->Type();
    }
    
    /// Set the camera model type.
    void SetType( const std::string& sType )
    {
        m_pCam->SetType( sType );
    }
    
    long int SerialNumber()
    {
        return m_pCam->SerialNumber();
    }
    
    /// Set the camera serial number.
    void SetSerialNumber( const long int nSerialNo )
    {
        m_pCam->SetSerialNumber( nSerialNo );
    }
    
    int Index()
    {
        return m_pCam->Index();
    }
    
    /// Set the camera index (for multi-camera rigs).
    void SetIndex( const int nIndex )
    {
        m_pCam->SetIndex( nIndex );
    }
    
    /// Return 3x3 RDF matrix, describing the coordinate-frame convention.
    Eigen::Matrix3d RDF() const
    {
        return m_pCam->RDF();
    }
    
    int Width() const
    {
        return m_pCam->Width();
    }
    
    int Height() const
    {
        return m_pCam->Height();
    }
    
    
    Eigen::VectorXd GenericParams()
    {  
        return m_pCam->GenericParams();
    }
    
    void SetImageDimensions(
            int nWidth,  //< Input:
            int nHeight  //< Input:
            )
    {
        m_pCam->SetImageDimensions( nWidth, nHeight );
    }
    
    std::string Name()
    {
        return m_pCam->Name();    
    } 
    
    /// Set the camera model name. e.g., "Left"
    void SetName( const std::string& sName )
    {
        m_pCam->SetName( sName );
    }
    
    
    bool Init( const std::string& sType )
    {
        if( m_pCam ){
            delete m_pCam;
        }
        m_pCam = CameraModelFactory( sType );
        return true;
    }
    
    Eigen::Vector2d Map(const Eigen::Vector2d& proj) const
    {
        _AssertInit();
        return m_pCam->Map( proj );
    }
    
    Eigen::Vector2d Unmap(const Eigen::Vector2d& img) const
    {
        _AssertInit();
        return m_pCam->Unmap( img );
    }
    
    Eigen::Matrix3d K() const
    {
        _AssertInit();
        return m_pCam->K();
    }
    
    Eigen::Matrix3d Kinv() const
    {
        _AssertInit();
        return m_pCam->Kinv();
    }
    
/*
    ///////////////////////////////////////////////////////
    void Read(
            const std::string& sFile, //< Input: file name to read from. 
            Eigen::Matrix4d& rPose    //< Output: local pose of the camera.
            )
    {
        // ReadCameraModelAndPose( sFile, *this, rPose );
        ReadCameraModelHeaderAndPose( sFile, *this, rPose );
    }
    */
    
private:
    ///////////////////////////////////////////////////////
    void _AssertInit() const
    {
        if( !m_pCam ){
            std::cerr << "ERROR: Camera model not initialized" << 
                         " -- make srue to call Init()" << std::endl;
            assert(m_pCam);
        }
    }
    
protected:
    CameraModelInterface* m_pCam; // this will be a specialization
};

}



