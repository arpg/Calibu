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
#include <memory>

namespace calibu
{

struct CameraModelException : public std::exception
{
    CameraModelException(const std::string& what)
        : m_sWhat(what)
    {
    }

    ~CameraModelException() throw() {}
    
    const char* what() const throw() {
        return m_sWhat.c_str();
    }

    std::string m_sWhat;
};

///////////////////////////////////////////////////////////////////////////
inline CameraModelInterface* CameraModelFactory( const std::string sModelName )
{
    if ( sModelName == "calibu_id" ){
        return new CameraModelT<Pinhole>();
    }else if( sModelName == "calibu_f_u0_v0") {
        return new CameraModelT<ProjectionLinearSquare<DistortionPinhole> >();
    }else if( sModelName == "calibu_fu_fv_u0_v0") {
        return new CameraModelT<ProjectionLinear<DistortionPinhole> >();
    }else if( sModelName == "calibu_fu_fv_u0_v0_w") {
        return new CameraModelT<ProjectionLinear<DistortionFov> >();
    }else if( sModelName == "calibu_fu_fv_u0_v0_k1_k2_k3") {
        return new CameraModelT<ProjectionLinear<DistortionPoly> >();
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
        m_pCam(nullptr)
    {
    }
    
    CameraModel( const CameraModelInterface& rhs )
        : m_pCam( CameraModelFactory( rhs.Type() ) )
    {
        CopySameType(rhs);
    }
    
    CameraModel( std::string& sModelType)
        : m_pCam( CameraModelFactory( sModelType ) )
    {
    }
    
    /////////////////////////////////////////////////////////////////////////
    // Member functions
    /////////////////////////////////////////////////////////////////////////
        
    /// Returns if this CameraModel is initialised and can be used.
    bool IsInitialised() const
    {
        return (bool)m_pCam;
    }
    
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
        
    long int SerialNumber() const
    {
        return m_pCam->SerialNumber();
    }
    
    /// Set the camera serial number.
    void SetSerialNumber( const long int nSerialNo )
    {
        m_pCam->SetSerialNumber( nSerialNo );
    }
    
    int Index() const
    {
        return m_pCam->Index();
    }
    
    /// Set the camera index (for multi-camera rigs).
    void SetIndex( const int nIndex )
    {
        m_pCam->SetIndex( nIndex );
    }
    
    /// Return 3x3 RDF matrix, describing the coordinate-frame convention.
    /// Row0: Right vector in camera frame of reference
    /// Row1: Down vector in camera frame of reference
    /// Row2: Forward vector in camera frame of reference
    Eigen::Matrix3d RDF() const
    {
        return m_pCam->RDF();
    }
    
    /// Set the 3x3 RDF matrix, describing the coordinate-frame convention.
    /// Row0: Right vector in camera frame of reference
    /// Row1: Down vector in camera frame of reference
    /// Row2: Forward vector in camera frame of reference
    virtual void SetRDF( const Eigen::Matrix3d& RDF )
    {
        m_pCam->SetRDF( RDF );    
    }
 
    void PrintInfo() 
    {
        m_pCam->PrintInfo();
    }

    size_t Width() const
    {
        return m_pCam->Width();
    }
    
    size_t Height() const
    {
        return m_pCam->Height();
    }
    
    
    Eigen::VectorXd GenericParams() const
    {  
        return m_pCam->GenericParams();
    }
    
    void SetGenericParams(const Eigen::VectorXd& params)
    {
        m_pCam->SetGenericParams(params);
    }
    
    void SetImageDimensions(
            size_t nWidth,  //< Input:
            size_t nHeight  //< Input:
            )
    {
        m_pCam->SetImageDimensions( nWidth, nHeight );
    }
    
    std::string Name() const
    {
        return m_pCam->Name();    
    } 
    
    /// Set the camera model name. e.g., "Left"
    void SetName( const std::string& sName )
    {
        m_pCam->SetName( sName );
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
    
private:
    void CopySameType( const CameraModelInterface& other )
    {
        assert( Type() == other.Type() );
        SetGenericParams(other.GenericParams());
        SetImageDimensions(other.Width(), other.Height());
        SetIndex(other.Index());
        SetName(other.Name());
        SetSerialNumber(other.SerialNumber());
        SetVersion(other.Version());
        SetRDF( other.RDF() );
    }    
    
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
    std::shared_ptr<CameraModelInterface> m_pCam;
};

}



