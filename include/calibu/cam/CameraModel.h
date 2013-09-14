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
 
template<typename Scalar=double>
inline CameraModelInterfaceT<Scalar>* CameraModelFactory( const std::string sModelName )
{
    if ( sModelName == "calibu_id" ){
        return new CameraModelT<Pinhole,Scalar>();
    }else if( sModelName == "calibu_f_u0_v0") {
        return new CameraModelT<ProjectionLinearSquare<DistortionPinhole>,Scalar >();
    }else if( sModelName == "calibu_fu_fv_u0_v0") {
        return new CameraModelT<ProjectionLinear<DistortionPinhole>,Scalar >();
    }else if( sModelName == "calibu_fu_fv_u0_v0_w") {
        return new CameraModelT<ProjectionLinear<DistortionFov>,Scalar >();
    }else if( sModelName == "calibu_fu_fv_u0_v0_k1_k2") {
        return new CameraModelT<ProjectionLinear<DistortionPoly2>,Scalar >();
    }else if( sModelName == "calibu_fu_fv_u0_v0_k1_k2_k3") {
        return new CameraModelT<ProjectionLinear<DistortionPoly3>,Scalar >();
    }else if( sModelName == "calibu_fu_fv_u0_v0_kb4") {
        return new CameraModelT<ProjectionKannalaBrandt,Scalar>();
    }
    return NULL;
}

///////////////////////////////////////////////////////////////////////////
/// Generic CameraModel class.  Manual polymorphism.
///////////////////////////////////////////////////////////////////////////
template<typename Scalar=double>
class CameraModelGeneric : public CameraModelInterfaceT<Scalar>
{
public:
    typedef typename CameraModelInterfaceT<Scalar>::Vector2t Vector2t;
    typedef typename CameraModelInterfaceT<Scalar>::Vector3t Vector3t;
    typedef typename CameraModelInterfaceT<Scalar>::VectorXt VectorXt;
    typedef typename CameraModelInterfaceT<Scalar>::Matrix3t Matrix3t;
    typedef typename CameraModelInterfaceT<Scalar>::SE3t SE3t;

    template<typename S>
    friend bool IsLinearModel( const CameraModelGeneric<S>& cam );

    /////////////////////////////////////////////////////////////////////////
    // Constructors
    /////////////////////////////////////////////////////////////////////////
    CameraModelGeneric() :
        m_pCam(nullptr)
    {
    }
    
    // Actually create deep copy
    CameraModelGeneric(const CameraModelGeneric<Scalar>& rhs )
        : m_pCam( CameraModelFactory<Scalar>( rhs.Type() ) )
    {
        CopySameType(rhs);
    }

    void operator=(const CameraModelGeneric<Scalar>& rhs)
    {
        m_pCam = std::shared_ptr<CameraModelInterfaceT<Scalar> >(
                    CameraModelFactory<Scalar>( rhs.Type() )
                    );
        CopySameType(rhs);
    }
    
    // Or use move semantics if possible.
    CameraModelGeneric(CameraModelGeneric<Scalar>&& rhs)
        : m_pCam( std::move(rhs.m_pCam) )
    {
    }
    
    void operator=(CameraModelGeneric<Scalar>&& rhs)
    {
        m_pCam = std::move(rhs.m_pCam);
    }
    
    template<typename T>
    CameraModelGeneric( const CameraModelInterfaceT<T>& rhs )
        : m_pCam( CameraModelFactory<Scalar>( rhs.Type() ) )
    {
        CopySameType(rhs);
    }
    
    CameraModelGeneric( const std::string& sModelType)
        : m_pCam( CameraModelFactory<Scalar>( sModelType ) )
    {
    }
    
    /////////////////////////////////////////////////////////////////////////
    // Member functions
    /////////////////////////////////////////////////////////////////////////
        
    /// Returns if this CameraModel is initialized and can be used.
    bool IsInitialized() const
    {
        return m_pCam.get();
    }
    
    /// Report camera model version number.
    int Version() const
    {
        _AssertInit();
        return m_pCam->Version();
    }
    
    /// Set the camera veriona nuber.
    void SetVersion( int nVersion )
    {
        _AssertInit();
        m_pCam->SetVersion( nVersion );
    }
    
    std::string Type() const
    {
        _AssertInit();
        return m_pCam->Type();
    }
        
    long int SerialNumber() const
    {
        _AssertInit();
        return m_pCam->SerialNumber();
    }
    
    /// Set the camera serial number.
    void SetSerialNumber( const long int nSerialNo )
    {
        _AssertInit();
        m_pCam->SetSerialNumber( nSerialNo );
    }
    
    int Index() const
    {
        _AssertInit();
        return m_pCam->Index();
    }
    
    /// Set the camera index (for multi-camera rigs).
    void SetIndex( const int nIndex )
    {
        _AssertInit();
        m_pCam->SetIndex( nIndex );
    }
    
    /// Return 3x3 RDF matrix, describing the coordinate-frame convention.
    /// Row0: Right vector in camera frame of reference
    /// Row1: Down vector in camera frame of reference
    /// Row2: Forward vector in camera frame of reference
    Matrix3t RDF() const
    {
        _AssertInit();
        return m_pCam->RDF();
    }
    
    /// Set the 3x3 RDF matrix, describing the coordinate-frame convention.
    /// Row0: Right vector in camera frame of reference
    /// Row1: Down vector in camera frame of reference
    /// Row2: Forward vector in camera frame of reference
    void SetRDF( const Matrix3t& RDF )
    {
        _AssertInit();
        m_pCam->SetRDF( RDF );    
    }
 
    void PrintInfo() const
    {
        _AssertInit();
        m_pCam->PrintInfo();
    }

    size_t Width() const
    {
        _AssertInit();
        return m_pCam->Width();
    }
    
    size_t Height() const
    {
        _AssertInit();
        return m_pCam->Height();
    }
    
    
    VectorXt GenericParams() const
    {  
        _AssertInit();
        return m_pCam->GenericParams();
    }
    
    void SetGenericParams(const VectorXt& params)
    {
        _AssertInit();
        m_pCam->SetGenericParams(params);
    }
    
    size_t NumParams() const
    {
        _AssertInit();
        return m_pCam->NumParams();
    }    
    
    const Scalar* data() const {
        _AssertInit();
        return m_pCam->data();
    }

    Scalar* data() {
        _AssertInit();
        return m_pCam->data();
    }    
    
    void SetImageDimensions(
            size_t nWidth,  //< Input:
            size_t nHeight  //< Input:
            )
    {
        _AssertInit();
        m_pCam->SetImageDimensions( nWidth, nHeight );
    }
    
    std::string Name() const
    {
        _AssertInit();
        return m_pCam->Name();    
    } 
    
    /// Set the camera model name. e.g., "Left"
    void SetName( const std::string& sName )
    {
        _AssertInit();
        m_pCam->SetName( sName );
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
    
    Vector2t Project( const Vector3t& P ) const
    {
        _AssertInit();
        return m_pCam->Project(P);
    }
 
    Vector3t Unproject( const Vector2t& p ) const
    {
        _AssertInit();
        return m_pCam->Unproject(p);
    }
 
    Vector2t Transfer3D(
            const SE3t& T_ba, const Vector3t& rhoPa, const Scalar rho
            ) const
    {
        _AssertInit();
        return m_pCam->Transfer3D(T_ba, rhoPa, rho);
    }
    
    Vector2t Transfer3D(
            const SE3t& T_ba, const Vector3t& rhoPa, const Scalar rho,
            bool& in_front         
            ) const 
    {
        _AssertInit();
        return m_pCam->Transfer3D(T_ba, rhoPa, rho, in_front);
    }
 
    Vector2t Transfer(
            const SE3t& T_ba, const Vector2t& pa, const Scalar rho    
            ) const
    {
        _AssertInit();
        return m_pCam->Transfer(T_ba, pa, rho);
    }
 
    Vector2t Transfer(
            const SE3t& T_ba, const Vector2t& pa,  const Scalar rho,   
            bool& in_front
            ) const
    {
        _AssertInit();
        return m_pCam->Transfer(T_ba, pa, rho, in_front);
    }
    
    Vector2t Transfer(
            const CameraModelInterfaceT<Scalar>& cam_a,
            const SE3t& T_ba, const Vector2t& pa, 
            const Scalar rho    
            ) const
    {
        _AssertInit();
        return m_pCam->Transfer(cam_a, T_ba, pa, rho);
    }
    
    Vector2t Transfer(
            const CameraModelInterfaceT<Scalar>& cam_a,            
            const SE3t& T_ba,  const Vector2t& pa, 
            const Scalar rho, bool& in_front      
            ) const
    {
        _AssertInit();
        return m_pCam->Transfer(cam_a, T_ba, pa, rho, in_front);
    }

    Eigen::Matrix<Scalar,2,3> dProject_dP( const Vector3t& P ) const
    {
        _AssertInit();
        return m_pCam->dProject_dP(P);
    }

    Eigen::Matrix<Scalar,2,4> dTransfer3D_dP(
            const SE3t& T_ba,   //< Input:
            const Eigen::Matrix<Scalar,3,1>& rhoPa, //< Input:
            const Scalar rho                        //< Input:
            ) const
    {
        _AssertInit();
        return m_pCam->dTransfer3D_dP(T_ba,rhoPa,rho);
    }

    virtual Eigen::Matrix<Scalar,2,Eigen::Dynamic> dMap_dParams(const Eigen::Matrix<Scalar,3,1>& p,
                                                                const Eigen::Matrix<Scalar,Eigen::Dynamic,1>& params) const
    {
        _AssertInit();
        return m_pCam->dMap_dParams(p, params);
    }
    
    void Scale( Scalar scale) {
        _AssertInit();
        m_pCam->Scale(scale);        
    }   
    
    CameraModelGeneric<Scalar> Scaled(Scalar scale) const
    {
        _AssertInit();
        CameraModelGeneric<Scalar> scaled_cam(*this);
        scaled_cam.Scale( scale );
        return scaled_cam;
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
    template<typename T>
    void CopySameType( const CameraModelInterfaceT<T>& other )
    {
        assert( Type() == other.Type() );
        SetGenericParams( other.GenericParams().template cast<Scalar>() );
        SetImageDimensions(other.Width(), other.Height());
        SetIndex(other.Index());
        SetName(other.Name());
        SetSerialNumber(other.SerialNumber());
        SetVersion(other.Version());
        SetRDF( other.RDF().template cast<Scalar>() );
    }    
    
    ///////////////////////////////////////////////////////
    void _AssertInit() const
    {
        if( !m_pCam ){
            std::cerr << "ERROR: Camera model not initialized" << 
                         " -- make sure to call Init()" << std::endl;
            throw std::runtime_error("Camera model not initialized.");
        }
    }
    
protected:
    std::shared_ptr<CameraModelInterfaceT<Scalar> > m_pCam;
};

typedef CameraModelGeneric<double> CameraModel;
}



