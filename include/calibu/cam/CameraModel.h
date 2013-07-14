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

    const int CAMERA_MODEL_VERSION = 8;

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
template<typename Scalar=double>
inline CameraModelInterfaceT<Scalar>* CameraModelFactory( const std::string sModelName )
{
    if ( sModelName == "calibu_id" ){
        return new CameraModelT<Pinhole,Scalar>();
    }else if( sModelName == "calibu_f_u0_v0") {
        return new CameraModelT<ProjectionLinearSquare<DistortionPinholeT<Scalar>, Scalar >,Scalar >();
    }else if( sModelName == "calibu_fu_fv_u0_v0") {
        return new CameraModelT<ProjectionLinear<DistortionPinholeT<Scalar>, Scalar >,Scalar >();
    }else if( sModelName == "calibu_fu_fv_u0_v0_w") {
        return new CameraModelT<ProjectionLinear<DistortionFovT<Scalar>, Scalar>,Scalar >();
    }else if( sModelName == "calibu_fu_fv_u0_v0_k1_k2_k3") {
        return new CameraModelT<ProjectionLinear<DistortionPolyT<Scalar>, Scalar>,Scalar >();
    }
    return NULL;
}

///////////////////////////////////////////////////////////////////////////
/// Generic CameraModel class.  Manual polymorphism.
///////////////////////////////////////////////////////////////////////////
template<typename Scalar=double>
class CameraModelGeneric : public CameraModelInterfaceT<Scalar>
{
    typedef Eigen::Matrix<Scalar,2,1> Vector2t;
    typedef Eigen::Matrix<Scalar,3,1> Vector3t;
    typedef Eigen::Matrix<Scalar,Eigen::Dynamic,1> VectorXt;
    typedef Eigen::Matrix<Scalar,3,3> Matrix3t;
    typedef Sophus::SE3Group<Scalar> SE3t;

    template<typename S>
    friend bool IsLinearModel( const CameraModelGeneric<S>& cam );
public:

    /////////////////////////////////////////////////////////////////////////
    // Constructors
    /////////////////////////////////////////////////////////////////////////
    CameraModelGeneric() :
        m_pCam(nullptr)
    {
    }
    
    CameraModelGeneric( const CameraModelInterfaceT<Scalar>& rhs )
        : m_pCam( CameraModelFactory( rhs.Type() ) )
    {
        CopySameType(rhs);
    }
    
    CameraModelGeneric( std::string& sModelType)
        : m_pCam( CameraModelFactory( sModelType ) )
    {
    }
    
    /////////////////////////////////////////////////////////////////////////
    // Member functions
    /////////////////////////////////////////////////////////////////////////
        
    /// Returns if this CameraModel is initialized and can be used.
    bool IsInitialized() const
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
    Matrix3t RDF() const
    {
        return m_pCam->RDF();
    }
    
    /// Set the 3x3 RDF matrix, describing the coordinate-frame convention.
    /// Row0: Right vector in camera frame of reference
    /// Row1: Down vector in camera frame of reference
    /// Row2: Forward vector in camera frame of reference
    virtual void SetRDF( const Matrix3t& RDF )
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
    
    
    VectorXt GenericParams() const
    {  
        return m_pCam->GenericParams();
    }
    
    void SetGenericParams(const VectorXt& params)
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
    
    Vector2t Map(const Vector2t& proj) const
    {
        _AssertInit();
        return m_pCam->Map( proj );
    }
    
    Vector2t Unmap(const Vector2t& img) const
    {
        _AssertInit();
        return m_pCam->Unmap( img );
    }
    
    Matrix3t K() const
    {
        _AssertInit();
        return m_pCam->K();
    }
    
    Matrix3t Kinv() const
    {
        _AssertInit();
        return m_pCam->Kinv();
    }

    Eigen::Matrix<Scalar,2,3> dMap_dP(
            const Vector3t& P //< Input:
            ) const
    {
        return m_pCam->dMap_dP(P);
    }

    Eigen::Matrix<Scalar,2,4> dTransfer3D_dP(
            const SE3t& T_ba,   //< Input:
            const Eigen::Matrix<Scalar,3,1>& rhoPa, //< Input:
            const Scalar rho                        //< Input:
            ) const{
        return m_pCam->dTransfer3D_dP(T_ba,rhoPa,rho);
    }
    
    CameraModelInterface& GetCameraModelInterface()
    {
        return *m_pCam;
    }

    const CameraModelInterface& GetCameraModelInterface() const
    {
        return *m_pCam;
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
                         " -- make sure to call Init()" << std::endl;
            assert(m_pCam);
        }
    }
    
protected:
    std::shared_ptr<CameraModelInterface> m_pCam;
};

typedef CameraModelGeneric<double> CameraModel;
}



