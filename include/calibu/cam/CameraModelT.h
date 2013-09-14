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
#include "ProjectionModel.h"
#include "ProjectionKannalaBrandt.h"

namespace calibu
{

    //////////////////////////////////////////////////////////////////////////////
    // Linear Projection and Distortion
    //////////////////////////////////////////////////////////////////////////////
    template<typename ProjectionModel, typename Scalar=double>
        class CameraModelT : public CameraModelInterfaceT<Scalar>
    {
        public:
            typedef typename CameraModelInterfaceT<Scalar>::Vector2t Vector2t;
            typedef typename CameraModelInterfaceT<Scalar>::Vector3t Vector3t;
            typedef typename CameraModelInterfaceT<Scalar>::VectorXt VectorXt;
            typedef typename CameraModelInterfaceT<Scalar>::Matrix3t Matrix3t;
            typedef typename CameraModelInterfaceT<Scalar>::SE3t SE3t;
            
            typedef typename ProjectionModel::DistortionFreeModel DistortionFreeModel;

            static const unsigned NUM_PARAMS = ProjectionModel::NUM_PARAMS;

            /////////////////////////////////////////////////////////////////////////
            // Static Utilities
            /////////////////////////////////////////////////////////////////////////

            /////////////////////////////////////////////////////////////////////////
            /// Transfer point correspondence with known inv. depth to secondary camera
            //  frame.  Points at infinity are supported (rho = 0) rhoPa =
            //  unproject(unmap(pa))
            template<typename T> inline static
                Eigen::Matrix<T,2,1> Transfer3D(
                        const T* camparam,                 //< Input:
                        const Sophus::SE3Group<T>& T_ba,   //< Input:
                        const Eigen::Matrix<T,3,1>& rhoPa, //< Input:
                        const T rho                        //< Input:
                        )
                {
                    // Inverse depth point in a transformed to b (homogeneous 2D)
                    const Eigen::Matrix<T,3,1> Pb =
                        T_ba.rotationMatrix() * rhoPa + rho * T_ba.translation();

                    // apply projection, distortion and linear cam
                    return ProjectionModel::Project(Pb, camparam);
                }

            /////////////////////////////////////////////////////////////////////////
            /// Transfer point correspondence with known inv. depth to secondary camera
            //  frame.  Points at infinity are supported (rho = 0) rhoPa =
            //  unproject(unmap(pa))
            template<typename T> inline static
                Eigen::Matrix<T,2,1> Transfer3D(
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
                    
                    in_front = Pb(2) > 0;

                    // apply projection, distortion and linear cam
                    return ProjectionModel::Project(Pb, camparam);
                }

            ///////////////////////////////////////////////////////////////////////////
            /// Transfer point correspondence with known inv. depth to secondary camera
            //  frame.  Points at infinity are supported (rho = 0)
            template<typename T> inline static
                Eigen::Matrix<T,2,1> Transfer(
                        const T* camparam,               //< Input:
                        const Sophus::SE3Group<T>& T_ba, //< Input:
                        const Eigen::Matrix<T,2,1>& pa,  //< Input:
                        const T rho                      //< Input:
                        )
                {
                    // rho*Pa (undo distortion, unproject, avoid division by inv depth)
                    const Eigen::Matrix<T,3,1> rhoPa = ProjectionModel::Unproject(pa, camparam);
                    return Transfer3D(camparam, T_ba, rhoPa, rho);
                }

            ///////////////////////////////////////////////////////////////////////////
            /// Transfer point correspondence with known inv. depth to secondary camera
            //  frame.  Points at infinity are supported (rho = 0).
            template<typename T> inline static
                Eigen::Matrix<T,2,1> Transfer(
                        const T* camparam,               //< Input:
                        const Sophus::SE3Group<T>& T_ba, //< Input:
                        const Eigen::Matrix<T,2,1>& pa,  //< Input:
                        const T rho,                     //< Input:
                        bool& in_front                   //< Output:
                        )
                {
                    // rho*P1 (undo distortion, unproject, avoid division by inv depth)
                    const Eigen::Matrix<T,3,1> rhoPa = ProjectionModel::Unproject(pa, camparam);
                    return Transfer3D(camparam, T_ba, rhoPa, rho, in_front);
                }

            /////////////////////////////////////////////////////////////////////////
            // Constructors
            /////////////////////////////////////////////////////////////////////////

        private:
            void ConstructorImpl(
                    size_t w,  size_t h, 
                    const Eigen::Matrix<Scalar,NUM_PARAMS,1>& params
                    )
            {
                m_nWidth = w;
                m_nHeight = h;
                m_params = params;
                m_nVersion = 8;
                m_nSerialNo = 0;
                m_nIndex = 0;
                m_RDF << 1,0,0, 0,1,0, 0,0,1; // vision
                //        m_RDF << 0,0,1, 1,0,0, 0,1,0; // robotics
            }

        public:

            // Most general delegate constructor
            CameraModelT( 
                    size_t w,  size_t h, 
                    const Eigen::Matrix<Scalar,NUM_PARAMS,1>& params
                    )
            {
                ConstructorImpl(w, h, params);
            }

            CameraModelT()
            {
                ConstructorImpl(0,0, Eigen::Matrix<Scalar,NUM_PARAMS,1>::Zero());
            }    

            CameraModelT(size_t w, size_t h)
            {
                ConstructorImpl(w, h, Eigen::Matrix<Scalar,NUM_PARAMS,1>::Zero());
            }    

            CameraModelT( const Eigen::Matrix<Scalar,NUM_PARAMS,1>& params)
            {
                ConstructorImpl(0, 0, params);
            }    

            CameraModelT(Scalar* cam_params)
            {
                ConstructorImpl(0, 0, Eigen::Map<Eigen::Matrix<Scalar,NUM_PARAMS,1> >(cam_params) );
            }

            CameraModelT(int w, int h, Scalar* cam_params)
            {
                ConstructorImpl(w, h, Eigen::Map<Eigen::Matrix<Scalar,NUM_PARAMS,1> >(cam_params) );
            }

            // copy constructor
            CameraModelT( const CameraModelT& rRHS ) 
            {
                m_nWidth    = rRHS.m_nWidth;
                m_nHeight   = rRHS.m_nHeight;
                m_params    = rRHS.m_params;
                m_RDF       = rRHS.m_RDF; 
                m_nIndex    = rRHS.m_nIndex;
                m_nSerialNo = rRHS.m_nSerialNo;
                m_nVersion  = rRHS.m_nVersion;
                m_sName     = rRHS.m_sName;
            }    

            ///////////////////////////////////////////////////////////////////////////
            // Member functions
            ///////////////////////////////////////////////////////////////////////////

            /// Report camera model version number.
            int Version() const
            {
                return m_nVersion;
            }

            /// Set the camera veriona nuber.
            void SetVersion( int nVersion )
            {
                m_nVersion = nVersion;
            }
            
            /// Report camera model 
            static std::string StaticType() 
            {
                return "calibu_" + ProjectionModel::Type();
            }            

            /// Report camera model 
            std::string Type() const 
            {
                return StaticType();
            }

            long int SerialNumber() const
            {
                return m_nSerialNo;
            }

            /// Set the camera serial number.
            void SetSerialNumber( const long int nSerialNo )
            {
                m_nSerialNo = nSerialNo;
            }

            int Index() const
            {
                return m_nIndex;
            }

            /// Set the camera index (for multi-camera rigs).
            void SetIndex( const int nIndex )
            {
                m_nIndex = nIndex;
            }

            /// Return 3x3 RDF matrix, describing the coordinate-frame convention.
            Matrix3t RDF() const
            {
                return m_RDF;
            }

            /// Set the camera model coordinate frame convention
            void SetRDF( const Matrix3t& RDF )
            {
                m_RDF = RDF;
            }

            void PrintInfo() const
            {
                printf("camera model info:\n" );
                printf("    Right        = [%d; %d; %d] unit vector\n", (int)m_RDF(0,0), (int)m_RDF(0,1), (int)m_RDF(0,2) );
                printf("    Down         = [%d; %d; %d] unit vector\n", (int)m_RDF(1,0), (int)m_RDF(1,1), (int)m_RDF(1,2) );
                printf("    Forward      = [%d; %d; %d] unit vector\n", (int)m_RDF(2,0), (int)m_RDF(2,1), (int)m_RDF(2,2) );
                printf("    Width        = %d pixels\n", (int)m_nWidth );
                printf("    Height       = %d pixels\n", (int)m_nHeight );

                // TODO ensure this is right for all models...
                printf("    Horiz Center = %.3f pixels\n", m_params[2] );
                printf("    Vert Center  = %.3f pixels\n", m_params[3] );
                printf("    Horiz FOV    = %.3f degrees\n", 180.0*2.0*atan2( m_nWidth/2, m_params[0] )/M_PI );
                printf("    Vert  FOV    = %.3f degrees\n", 180.0*2.0*atan2( m_nHeight/2, m_params[1] )/M_PI );
            }

            size_t Width() const
            {
                return m_nWidth;
            }

            size_t Height() const
            {
                return m_nHeight;
            }

            void SetImageDimensions(
                    size_t nWidth,  //< Input:
                    size_t nHeight  //< Input:
                    )
            {
                m_nWidth = nWidth;
                m_nHeight = nHeight;
            }

            std::string Name() const
            {
                return m_sName;
            } 

            /// Set the camera model name. e.g., "Left"
            void SetName( const std::string& sName )
            {
                m_sName = sName;
            }
            
            CameraModelT<DistortionFreeModel,Scalar> DistortionFreeCamera() const
            {
                CameraModelT<DistortionFreeModel,Scalar> ret(
                        m_nWidth, m_nHeight, 
                        m_params.template head<DistortionFreeModel::NUM_PARAMS>()
                        );
                return ret;
            }

            VectorXt GenericParams() const
            {
                return m_params;
            }

            void SetGenericParams(const VectorXt& params)
            {
                m_params = params;
            }

            Eigen::Matrix<Scalar,NUM_PARAMS,1>& Params()
            {
                return m_params;
            }

            const Eigen::Matrix<Scalar,NUM_PARAMS,1>& Params() const
            {
                return m_params;
            }
            
            size_t NumParams() const
            {
                return NUM_PARAMS;
            }
                
            const Scalar* data() const
            {
                return m_params.data();
            }

            Scalar* data()
            {
                return m_params.data();
            }
            
            inline Matrix3t K() const
            {
                return ProjectionModel::MakeK(m_params.data());
            }

            inline Matrix3t Kinv() const
            {
                return ProjectionModel::MakeKinv(m_params.data());
            }
            
            inline Vector2t Project(const Vector3t& P) const
            {
                return ProjectionModel::Project(P, m_params.data());
            }

            inline Vector3t Unproject(const Vector2t& p) const
            {
                return ProjectionModel::Unproject(p, m_params.data());
            }            

            inline Vector2t Transfer(const SE3t& T_ba, const Vector2t& pa, Scalar rho) const
            {
                return Transfer<Scalar>(data(), T_ba, pa, rho);
            }

            inline Vector2t Transfer(const SE3t& T_ba, const Vector2t& pa, Scalar rho, bool& in_front) const
            {
                return Transfer<Scalar>(data(), T_ba, pa, rho, in_front);
            }
            
            inline Vector2t Transfer3D(
                    const Sophus::SE3Group<Scalar>& T_ba,
                    const Vector3t& rhoPa, const Scalar rho
                    ) const
            {
                return Transfer3D<Scalar>(m_params.data(), T_ba, rhoPa, rho);
            }
            
            inline Vector2t Transfer3D(
                    const Sophus::SE3Group<Scalar>& T_ba,
                    const Vector3t& rhoPa, const Scalar rho,
                    bool& in_front
                    ) const
            {
                return Transfer3D<Scalar>(m_params.data(), T_ba, rhoPa, rho, in_front);
            }
            
            inline Vector2t Transfer(
                    const CameraModelInterfaceT<Scalar>& cam_a,
                    const SE3t& T_ba,   
                    const Vector2t& pa, 
                    const Scalar rho    
                    ) const
            {
                // rho*Pa (undo distortion, unproject, avoid division by inv depth)
                const Vector3t rhoPa = cam_a.Unproject(pa);
                return Transfer3D(T_ba, rhoPa, rho);
            }
            
            inline Vector2t Transfer(
                    const CameraModelInterfaceT<Scalar>& cam_a,            
                    const SE3t& T_ba,
                    const Vector2t& pa, 
                    const Scalar rho,
                    bool& in_front
                    ) const
            {
                // rho*P1 (undo distortion, unproject, avoid division by inv depth)
                const Vector3t rhoPa = cam_a.Unproject(pa);
                return Transfer3D(T_ba, rhoPa, rho, in_front);
            }
            
            inline Eigen::Matrix<Scalar,2,3> dProject_dP(
                    const Vector3t& P //< Input:
                    ) const
            {
                return ProjectionModel::dProject_dP(P,data());
            }

            inline Eigen::Matrix<Scalar,2,Eigen::Dynamic> dMap_dParams(const Eigen::Matrix<Scalar,3,1>& p,
                                                                        const Eigen::Matrix<Scalar,Eigen::Dynamic,1>& params) const
            {
                return ProjectionModel::dProject_dParams(p,params);
            }

            inline Eigen::Matrix<Scalar,2,4> dTransfer3D_dP(
                    const SE3t& T_ba,                       //< Input:
                    const Eigen::Matrix<Scalar,3,1>& rhoPa, //< Input:
                    const Scalar rho                        //< Input:
                    ) const
            {
                // Inverse= depth point in a transformed to b (homogeneous 2D)
                const Eigen::Matrix<Scalar,3,1> Pb =
                    T_ba.rotationMatrix() * rhoPa + rho * T_ba.translation();

                Eigen::Matrix<Scalar,2,3>dMap =
                    ProjectionModel::dProject_dP(Pb,data());

                Eigen::Matrix<Scalar,2,4> J;
                // Using operator= to get around clang bug.
                J.template block<2,3>(0,0).operator=( dMap*T_ba.rotationMatrix() ); // dTransfer_dXYZ
                J.template block<2,1>(0,3).operator=( dMap*T_ba.translation() );    // dTransfer_dRho

                return J;
            }
            
            inline void Scale( Scalar scale) {
                m_nWidth  *= scale;
                m_nHeight *= scale;
                ProjectionModel::Scale(scale, m_params.data() );
            }            

        protected:

            size_t           m_nWidth;    //< Camera width, in pixels
            size_t           m_nHeight;   //< Camera height, in pixels
            Eigen::Matrix<Scalar,NUM_PARAMS,1> m_params;
            std::string      m_sName;     //< Model name, e.g., "left"
            int              m_nVersion;  //< Calibu or MVL camera model version.
            long int         m_nSerialNo; //< Camera serial number, if appropriate.
            int              m_nIndex;    //< Camera index, for multi-camera systems.
            Matrix3t         m_RDF;       //< Define coordinate-frame convention from Right, Down, Forward vectors.
    };

}
