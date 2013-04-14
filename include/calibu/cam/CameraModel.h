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

#include "CameraModelBase.h"
#include "ProjectionModel.h"

namespace calibu
{

    //////////////////////////////////////////////////////////////////////////////
    // Linear Projection and Distortion
    //////////////////////////////////////////////////////////////////////////////

    template<typename ProjectionModel>
        class CameraModel : public CameraModelBase
    {
        public:
            static const unsigned NUM_PARAMS = ProjectionModel::NUM_PARAMS;

            /////////////////////////////////////////////////////////////////////////
            // Static Utilities
            /////////////////////////////////////////////////////////////////////////

            inline static std::string Name() 
            { 
                return ProjectionModel::Name(); 
            }

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
            //  frame.  Points at infinity are supported (rho = 0)
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

            CameraModel()
                : CameraModelBase(0,0), params(Eigen::Matrix<double,NUM_PARAMS,1>::Zero())
            {
            }    

            CameraModel(int w, int h)
                : CameraModelBase(w,h), params(Eigen::Matrix<double,NUM_PARAMS,1>::Zero())
            {
            }    

            CameraModel(const Eigen::Matrix<double,NUM_PARAMS,1>& params)
                : CameraModelBase(0,0), params(params)
            {
            }    

            CameraModel(int w, int h, const Eigen::Matrix<double,NUM_PARAMS,1>& params)
                : CameraModelBase(w,h), params(params)
            {
            }        

            CameraModel(double* cam_params)
                : CameraModelBase(0,0), params(Eigen::Map<Eigen::Matrix<double,NUM_PARAMS,1> >(cam_params))
            {
            }    

            CameraModel(int w, int h, double* cam_params)
                : CameraModelBase(w,h), params(Eigen::Map<Eigen::Matrix<double,NUM_PARAMS,1> >(cam_params))
            {
            }    

            CameraModel(const CameraModel& other)
                : CameraModelBase(other), params(other.params)
            {
            }    

            ///////////////////////////////////////////////////////////////////////////
            // Member functions
            ///////////////////////////////////////////////////////////////////////////

            Eigen::Matrix<double,NUM_PARAMS,1>& Params() {
                return params;
            }

            const Eigen::Matrix<double,NUM_PARAMS,1>& Params() const {
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

        protected:
            Eigen::Matrix<double,NUM_PARAMS,1> params;
    };







    ///////////////////////////////////////////////////////////////////////////
    /// All cameras must implement this interface
    class CameraModelInterface
    {
        public:
            CameraModelInterface() : m_nWidth(0), m_nHeight(0) {}
            CameraModelInterface( int nWidth, int nHeight) :
                m_nWidth(nWidth), m_nHeight(nHeight) {}
            virtual ~CameraModelInterface(){}

            virtual bool Init() = 0;

            /// Return the perspective projection camera model "K" matrix    
            virtual Eigen::Matrix3d K() = 0;

            /// Return the perspective projection camera model inverse "K" matrix
            virtual Eigen::Matrix3d Kinv() = 0;

            virtual Eigen::Vector2d Map( 
                    const Eigen::Vector2d& proj  //< Input:
                    ) = 0;

            virtual Eigen::Vector2d Unmap( 
                    const Eigen::Vector2d& img //< Input:
                    ) = 0;

            /// getters/setters to implement:
            /*
            virtual int Width() = 0;
            virtual int Height() = 0;
            virtual void SetImageDimensions( int nWidth, int nHeight ) = 0;

            virtual const char* Type() = 0;
            virtual void SetType( const std::string& sType ) = 0;
 
            virtual int Version() = 0;
            virtual void SetVersion( int nVersion ) = 0;
            */
            
            //            virtual Eigen::Matrix3d RDF() = 0;

            //////////////////////////////////////////////////////////////////
            /// Image dimensions
            int Width()
            {
                return m_nWidth;
            }

            //////////////////////////////////////////////////////////////////
            /// Image dimensions
            int Height()
            {
                return m_nHeight;
            }

            //////////////////////////////////////////////////////////////////
            /// Set image dimensions
            void SetImageDimensions(
                    int nWidth,  //< Input:
                    int nHeight  //< Input:
                    )
            {
                m_nWidth = nWidth;
                m_nHeight = nHeight;
            }

            //////////////////////////////////////////////////////////////////
            const char* Name()
            {
                return m_sName.c_str();
            }

            //////////////////////////////////////////////////////////////////
            void SetName( const std::string& sName )
            {
                m_sName = sName;
            }

            //////////////////////////////////////////////////////////////////
            /// Report camera model 
            const char* Type()
            {
                return m_sType.c_str();
            }

            //////////////////////////////////////////////////////////////////
            /// Set the camera model type.
            void SetType( const std::string& sType )
            {
                m_sType = sType;
            }

            //////////////////////////////////////////////////////////////////
            /// Report camera model version number.
            int Version()
            {
                return m_nVersion;
            }

            //////////////////////////////////////////////////////////////////
            /// Set the camera veriona nuber.
            void SetVersion( int nVersion )
            {
                m_nVersion = nVersion;
            }

            //////////////////////////////////////////////////////////////////
            /// Get the camera serial number.
            long int SerialNumber()
            {
                return m_nSerialNo;
            }

            //////////////////////////////////////////////////////////////////
            /// Set the camera serial number.
            void SetSerialNumber( const long int nSerialNo )
            {
                m_nSerialNo = nSerialNo;
            }

            //////////////////////////////////////////////////////////////////
            /// Get the camera index (for multi-camera rigs).
            int Index()
            {
                return m_nIndex;
            }

            //////////////////////////////////////////////////////////////////
            /// Set the camera index (for multi-camera rigs).
            void SetIndex( const int nIndex )
            {
                m_nIndex = nIndex;
            }

        private:
            int              m_nWidth;    //< Camera width, in pixels
            int              m_nHeight;   //< Camera height, in pixels
            std::string      m_sType;     //< Model type name.
            std::string      m_sName;     //< particular camera name, e.g., "Left"
            int              m_nVersion;  //< Calibu or MVL camera model version.
            long int         m_nSerialNo; //< Camera serial number, if appropriate.
            int              m_nIndex;    //< Camera index, for multi-camera systems.
            Eigen::Matrix3d  m_RDF;       //< Define coordinate-frame convention from Right, Down, Forward vectors.


    };


    ///////////////////////////////////////////////////////////////////////////
    /// This class allows templated specialized implementations.
    template <typename ProjectionModel>
    class CameraModelSpecialization : public CameraModelInterface
    {
        public:
            static const unsigned NUM_PARAMS = ProjectionModel::NUM_PARAMS;

            /////////////////////////////////////////////////////////////////////////
            // Static Utilities
            /////////////////////////////////////////////////////////////////////////

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
            /// Transfer point correspondence with known inv. depth to secondary 
            //  camera frame.  Points at infinity are supported (rho = 0) rhoPa =
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
            /// Transfer point correspondence with known inv. depth to secondary 
            //  camera frame.  Points at infinity are supported (rho = 0) rhoPa =
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
            //  frame.  Points at infinity are supported (rho = 0)
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
                CameraModelInterface(0,0),
                vParams( Eigen::Matrix<double,NUM_PARAMS,1>::Zero() )
            {
            }

            CameraModelSpecialization( int nWidth, int nHeight ) :
                CameraModelInterface(nWidth,nHeight),
                vParams( Eigen::Matrix<double,NUM_PARAMS,1>::Zero() )
            {
            }


            /*
            CameraModelSpecialization(int w, int h)
                : CameraModelBase(w,h), params(Eigen::Matrix<double,NUM_PARAMS,1>::Zero())
            {
            }    

            CameraModelSpecialization(const Eigen::Matrix<double,NUM_PARAMS,1>& params)
                : CameraModelBase(0,0), params(params)
            {
            }    

            CameraModelSpecialization(int w, int h, const Eigen::Matrix<double,NUM_PARAMS,1>& params)
                : CameraModelBase(w,h), params(params)
            {
            }        

            CameraModelSpecialization( double* cam_params)
                : CameraModelBase(0,0), params(Eigen::Map<Eigen::Matrix<double,NUM_PARAMS,1> >(cam_params))
            {
            }    

            CameraModelSpecialization(int w, int h, double* cam_params)
                : CameraModelBase(w,h), params(Eigen::Map<Eigen::Matrix<double,NUM_PARAMS,1> >(cam_params))
            {
            }    

            CameraModelSpecialization(const CameraModelSpecialization& other)
                : CameraModelBase(other), params(other.params)
            {
            }    
            */


            //////////////////////////////////////////////////////////////////
            //////////////////////////////////////////////////////////////////
            //////////////////////////////////////////////////////////////////
            bool Init() { return true; }
            ~CameraModelSpecialization(){}


            //////////////////////////////////////////////////////////////////
            /// Return the perspective projection camera model "K" matrix    
            Eigen::Matrix3d K() 
            {
                return Eigen::Matrix3d();
            }

            //////////////////////////////////////////////////////////////////
            /// Return the perspective projection camera model inverse "K" matrix
            Eigen::Matrix3d Kinv()
            {
                return Eigen::Matrix3d();
            }

            //////////////////////////////////////////////////////////////////
            Eigen::Vector2d Map( 
                    const Eigen::Vector2d& proj  //< Input:
                    )
            {
                return Eigen::Vector2d();
            }

            //////////////////////////////////////////////////////////////////
            Eigen::Vector2d Unmap( 
                    const Eigen::Vector2d& img //< Input:
                    )
            {
                return Eigen::Vector2d();
            }

            //////////////////////////////////////////////////////////////////
            /// Set the camera index (for multi-camera rigs).
            /// Return 3x3 RDF matrix, describing the coordinate-frame convention.
            Eigen::Matrix3d RDF() const
            {
                return m_RDF;
            }

        protected:
            Eigen::Matrix<double,NUM_PARAMS,1> vParams;
    };








    typedef CameraModelSpecialization<Pinhole> PinholeCam;

    ///////////////////////////////////////////////////////////////////////////
    CameraModelInterface* CameraModelFactory( const std::string sModelName )
    {
        if ( sModelName == "Pinhole" ){
            return new PinholeCam();
        }
        return NULL;
    }

    ///////////////////////////////////////////////////////////////////////////
    /// Simple pass-through to the specified camera model.
    class MyCameraModel : public CameraModelInterface
    {
        public:
            ///////////////////////////////////////////////////////////////////
            MyCameraModel()
            {
            }

            ///////////////////////////////////////////////////////////////////
            MyCameraModel( const std::string& sType )
            {
                Init( sType );
            }

            ///////////////////////////////////////////////////////////////////
            bool Init()
            {
                return false; // never called
            }

            /// Report camera model 
            const char* Type()
            {
                return m_pCam->Type();
            }

            bool Init( const std::string& sModelType )
            {
                if(m_pCam) {
                    delete m_pCam;
                    m_pCam = 0;
                }
                m_pCam = CameraModelFactory( sModelType );
                if( m_pCam ){
                    return m_pCam->Init();
                }
                return false; 
            }


            ///////////////////////////////////////////////////////////////////
            /// Return the perspective projection camera model "K" matrix    
            Eigen::Matrix3d K() 
            {
                return Eigen::Matrix3d();
            }

            ///////////////////////////////////////////////////////////////////
            /// Return the perspective projection camera model inverse "K" matrix
            Eigen::Matrix3d Kinv()
            {
                return Eigen::Matrix3d();
            }

            ///////////////////////////////////////////////////////////////////
            Eigen::Vector2d Map( 
                    const Eigen::Vector2d& proj  //< Input:
                    )
            {
                return Eigen::Vector2d();
            }

            ///////////////////////////////////////////////////////////////////
            Eigen::Vector2d Unmap( 
                    const Eigen::Vector2d& img //< Input:
                    )
            {
                return Eigen::Vector2d();
            }
/*
            //////////////////////////////////////////////////////////////////
            /// Image dimensions
            int Width()
            {
                return m_pCam->Width();
            }

            //////////////////////////////////////////////////////////////////
            /// Image dimensions
            int Height()
            {
                return m_pCam->Height();
            }

            //////////////////////////////////////////////////////////////////
            /// Set image dimensions
            void SetImageDimensions(
                    int nWidth,  //< Input:
                    int nHeight  //< Input:
                    )
            {
                m_pCam->SetImageDimensions( nWidth, nHeight);
            }

            ///////////////////////////////////////////////////////////////////
            /// Report camera model 
            const char* Type() const
            {
                return m_pCam->Type();
            }

            ///////////////////////////////////////////////////////////////////
            void SetType( const std::string& sType )
            {
                m_pCam->SetType( sType );
            }

            //////////////////////////////////////////////////////////////////
            /// Report camera model version number.
            int Version()
            {
                return m_pCam->Version();
            }

            //////////////////////////////////////////////////////////////////
            /// Set the camera veriona nuber.
            void SetVersion( int nVersion )
            {
                m_pCam->SetVersion( nVersion );
            }
            */

            ///////////////////////////////////////////////////////////////////
            CameraModelInterface* InterfacePtr()
            {
                return m_pCam;
            }

        private:
            CameraModelInterface*   m_pCam;
    };

}



