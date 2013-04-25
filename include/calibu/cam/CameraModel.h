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

#include "CameraModelBase.h"
#include "ProjectionModel.h"

#include <exception>
#include <iostream>

namespace calibu
{

    //////////////////////////////////////////////////////////////////////////////
    // Linear Projection and Distortion
    //////////////////////////////////////////////////////////////////////////////
    template<typename ProjectionModel>
        class CameraModelSpecialization : public CameraModelInterface
    {
        public:
            static const unsigned NUM_PARAMS = ProjectionModel::NUM_PARAMS;

            /////////////////////////////////////////////////////////////////////////
            // Static Utilities
            /////////////////////////////////////////////////////////////////////////

//            inline static std::string Name() 
//            {
//                return ProjectionModel::Name(); 
//            }

            /////////////////////////////////////////////////////////////////////////
            /// Map from image coordinates to z=1 plane.
            template<typename T> inline static Eigen::Matrix<T,2,1> Map(
                    const Eigen::Matrix<T,2,1>& proj, //< Input:
                    T const* params                   //< Input:
                    )
            {
                return ProjectionModel::Map(proj, params);
            }

            /////////////////////////////////////////////////////////////////////////
            /// Map from z=1 plane to image coordinates.
            template<typename T> inline
                static Eigen::Matrix<T,2,1> Unmap(
                        const Eigen::Matrix<T,2,1>& img, //< Input:
                        T const* params                  //< Input:
                        )
                {    
                    return ProjectionModel::Unmap(img, params);
                }

            /////////////////////////////////////////////////////////////////////////
            /// TODO doxygen comment    
            static inline
                Eigen::Matrix<double,2,3> dMap_dP(
                        const Eigen::Vector3d& P, //< Input:
                        const double* params      //< Input:
                        )
                {
                    const Eigen::Vector2d p(P(0) / P(2), P(1) / P(2));
                    const Eigen::Matrix<double,2,2> _dMap_dp = ProjectionModel::dMap_dp(p, params);

                    Eigen::Matrix<double,2,3> _dp_dP;
                    _dp_dP << 
                        1.0/P(2), 0, -P(0)/(P(2)*P(2)),
                        0, 1.0/P(2), -P(1)/(P(2)*P(2));

                    return _dMap_dp * _dp_dP;
                }    

            /////////////////////////////////////////////////////////////////////////
            /// Transfer point correspondence with known inv. depth to secondary camera
            //  frame.  Points at infinity are supported (rho = 0) rhoPa =
            //  unproject(unmap(pa))
            template<typename T> inline
                static Eigen::Matrix<T,2,1> Transfer3D(
                        const T* camparam,                 //< Input:
                        const Sophus::SE3Group<T>& T_ba,   //< Input:
                        const Eigen::Matrix<T,3,1>& rhoPa, //< Input:
                        const T rho                        //< Input:
                        )
                {
                    // Inverse depth point in a transformed to b (homogeneous 2D)
                    const Eigen::Matrix<T,3,1> Pb =
                        T_ba.rotationMatrix() * rhoPa + rho * T_ba.translation();

                    // to non-homogeneous 2D
                    const Eigen::Matrix<T,2,1> proj( Pb(0)/Pb(2), Pb(1)/Pb(2) );

                    // apply distortion and linear cam
                    return Map(proj, camparam); 
                }

            /////////////////////////////////////////////////////////////////////////
            /// Transfer point correspondence with known inv. depth to secondary camera
            //  frame.  Points at infinity are supported (rho = 0) rhoPa =
            //  unproject(unmap(pa))
            template<typename T> inline
                static Eigen::Matrix<T,2,1> Transfer3D(
                        const T* camparam,                 //< Input:
                        const Sophus::SE3Group<T>& T_ba,   //< Input:
                        const Eigen::Matrix<T,3,1>& rhoPa, //< Input:
                        const T rho,                       //< Input:
                        bool& in_front                     //< Output:
                        )
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

            ///////////////////////////////////////////////////////////////////////////
            /// Transfer point correspondence with known inv. depth to secondary camera
            //  frame.  Points at infinity are supported (rho = 0)
            template<typename T> inline
                static Eigen::Matrix<T,2,1> Transfer(
                        const T* camparam,               //< Input:
                        const Sophus::SE3Group<T>& T_ba, //< Input:
                        const Eigen::Matrix<T,2,1>& pa,  //< Input:
                        const T rho                      //< Input:
                        )
                {
                    // rho*Pa (undo distortion, unproject, avoid division by inv depth)
                    const Eigen::Matrix<T,3,1> rhoPa = Unproject<T>( Unmap<T>(pa, camparam)); 
                    return Transfer3D(camparam, T_ba, rhoPa, rho);
                }

            ///////////////////////////////////////////////////////////////////////////
            /// Transfer point correspondence with known inv. depth to secondary camera
            //  frame.  Points at infinity are supported (rho = 0).
            template<typename T> inline
                static Eigen::Matrix<T,2,1> Transfer(
                        const T* camparam,               //< Input:
                        const Sophus::SE3Group<T>& T_ba, //< Input:
                        const Eigen::Matrix<T,2,1>& pa,  //< Input:
                        const T rho,                     //< Input:
                        bool& in_front                   //< Output:
                        )
                {
                    // rho*P1 (undo distortion, unproject, avoid division by inv depth)
                    const Eigen::Matrix<T,3,1> rhoPa = Unproject<T>( Unmap<T>(pa, camparam) ); 
                    return Transfer3D(camparam, T_ba, rhoPa, rho, in_front);
                }

            /////////////////////////////////////////////////////////////////////////
            // Constructors
            /////////////////////////////////////////////////////////////////////////

            CameraModelSpecialization() :  
                m_nWidth(0),
                m_nHeight(0),
                params(Eigen::Matrix<double,NUM_PARAMS,1>::Zero())
            {
                m_RDF << 0,0,1, 1,0,0, 0,1,0; 
                m_nIndex = 0;
                m_nSerialNo = 0;
                m_nVersion = 8;
            }    

            CameraModelSpecialization(int w, int h) :
                m_nWidth(w),
                m_nHeight(h),
                params(Eigen::Matrix<double,NUM_PARAMS,1>::Zero())
            {
                m_RDF << 0,0,1, 1,0,0, 0,1,0; 
                m_nIndex = 0;
                m_nSerialNo = 0;
                m_nVersion = 8;
             }    

            CameraModelSpecialization( const Eigen::Matrix<double,NUM_PARAMS,1>& params) :
                m_nWidth(0),
                m_nHeight(0),
                params(params)
            {
                m_RDF << 0,0,1, 1,0,0, 0,1,0; 
                m_nIndex = 0;
                m_nSerialNo = 0;
                m_nVersion = 8;
             }    

            CameraModelSpecialization( 
                    int w, 
                    int h, 
                    const Eigen::Matrix<double,NUM_PARAMS,1>& params) :
                m_nWidth(w),
                m_nHeight(h),
                params(params)
            {
                m_RDF << 0,0,1, 1,0,0, 0,1,0; 
                m_nIndex = 0;
                m_nSerialNo = 0;
                m_nVersion = 8;
             }

            CameraModelSpecialization(double* cam_params):
                m_nWidth(0),
                m_nHeight(0),
                params(Eigen::Map<Eigen::Matrix<double,NUM_PARAMS,1> >(cam_params))
            {
                m_RDF << 0,0,1, 1,0,0, 0,1,0; 
                m_nIndex = 0;
                m_nSerialNo = 0;
                m_nVersion = 8;
             }

            CameraModelSpecialization(int w, int h, double* cam_params) :
                m_nWidth(w),
                m_nHeight(h),
                params(Eigen::Map<Eigen::Matrix<double,NUM_PARAMS,1> >(cam_params))
            {
                m_RDF << 0,0,1, 1,0,0, 0,1,0; 
                m_nIndex = 0;
                m_nSerialNo = 0;
                m_nVersion = 8;
             }

            // copy constructor
            CameraModelSpecialization( const CameraModelSpecialization& rRHS ) 
            {
                m_nWidth    = rRHS.m_nWidth;
                m_nHeight   = rRHS.m_nHeight;
                params      = rRHS.params;
                m_RDF       = rRHS.m_RDF; 
                m_nIndex    = rRHS.m_nIndex;
                m_nSerialNo = rRHS.m_nSerialNo;
                m_nVersion  = rRHS.m_nVersion;
             }    

            ///////////////////////////////////////////////////////////////////////////
            // Member functions
            ///////////////////////////////////////////////////////////////////////////

            Eigen::VectorXd GenericParams() 
            {
                return params;
            }

            Eigen::Matrix<double,NUM_PARAMS,1>& Params() 
            {
                return params;
            }

            const Eigen::Matrix<double,NUM_PARAMS,1>& Params() const 
            {
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

            inline Eigen::Matrix3d K() const
            {
                return ProjectionModel::MakeK(params.data());
            }

            inline Eigen::Matrix3d Kinv() const
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

            /// Report camera model version number.
            int Version() const
            {
                return m_nVersion;
            }

            /// Set the camera veriona nuber.
            void SetVersion( int nVersion )
            {
            }

            /// Report camera model 
            std::string Type() const 
            {
                return m_sType.c_str();
            }

            void SetType( const std::string& sType )
            {
                m_sType = sType;
            }

            long int SerialNumber()
            {
                return m_nSerialNo;
            }

            /// Set the camera serial number.
            void SetSerialNumber( const long int nSerialNo )
            {
                m_nSerialNo = nSerialNo;
            }

            int Index()
            {
                return m_nIndex;
            }

            /// Set the camera index (for multi-camera rigs).
            void SetIndex( const int nIndex )
            {
               m_nIndex = nIndex;
            }

            /// Return 3x3 RDF matrix, describing the coordinate-frame convention.
            Eigen::Matrix3d RDF() const
            {
                return m_RDF;
            }

            int Width() const
            {
                return m_nWidth;
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

            std::string Name()
            {
                return m_sName;
            } 

            /// Set the camera model name. e.g., "Left"
            void SetName( const std::string& sName )
            {
                m_sName = sName;
            }

        protected:

            int              m_nWidth;    //< Camera width, in pixels
            int              m_nHeight;   //< Camera height, in pixels
            Eigen::Matrix<double,NUM_PARAMS,1> params;
            std::string      m_sName;     //< Model name, e.g., "left"
            std::string      m_sType;     //< Model type name.
            int              m_nVersion;  //< Calibu or MVL camera model version.
            long int         m_nSerialNo; //< Camera serial number, if appropriate.
            int              m_nIndex;    //< Camera index, for multi-camera systems.
            Eigen::Matrix3d  m_RDF;       //< Define coordinate-frame convention from Right, Down, Forward vectors.
    };



    ///////////////////////////////////////////////////////////////////////////
    inline CameraModelInterface* CameraModelFactory( const std::string sModelName )
    {
        if ( sModelName == "Pinhole" ){
            return new  CameraModelSpecialization<Pinhole>();
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



