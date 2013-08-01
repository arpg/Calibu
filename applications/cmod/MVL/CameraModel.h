/*
 *  \file MvlCameraModel.h
 *
 *  This class simply wraps the C API camera model struct.
 *
 *  $Id$
 */



#ifndef _MVLPP_CAMERA_MODEL_H_
#define _MVLPP_CAMERA_MODEL_H_

#include "camera.h"

#include <boost/shared_ptr.hpp>

namespace mvl
{

    class CameraModel
    {
        public:

            /// Default constructor.
            CameraModel()
            {
                m_dPose = Eigen::Matrix4d::Identity();
            }

//            CameraModel( boost::shared_ptr<mvl_camera_t> pCam )
//            {
//                m_pCameraModel = pCam;
//                m_dPose = Eigen::Matrix4d::Identity();
//            }

            /// Utility constructor will read camera model from xml file.
            CameraModel(
                    const std::string& sFile
                    )
            {
                if( !Read( sFile ) ){
                    fprintf( stderr, "ERROR: failed to read '\%s'\n", sFile.c_str() );
                }
            }

            /// Deep Copy.
//            void DeepCopy( const CameraModel& rhs )
//            {
//                m_dPose        = rhs.m_dPose;
//                m_sFileName    = rhs.m_sFileName;
//                m_pCameraModel.reset( mvl_alloc_and_copy_camera( rhs.m_pCameraModel.get() ) );
//            }

            /// Wrap C API to read camera model xml file.
            bool Read( const std::string& sFile )
            {
                m_sFileName = sFile;
                double dPose[16];
                mvl_camera_t* cmod = mvl_read_camera( sFile.c_str(), dPose );
                m_pCameraModel.reset( cmod );
                if( !m_pCameraModel ){
                    return false;
                }
                m_dPose = Eigen::Matrix<double,4,4,Eigen::RowMajor>( dPose );
                return true;
            }

//            /// Wrap C API to write camera model xml file.
//            bool Write( const std::string& sFile )
//            {
//                if( mvl_write_camera( sFile.c_str(), m_dPose.data(), m_pCameraModel.get() ) == false ){
//                    fprintf( stderr, "ERROR: failed to write '\%s'\n", sFile.c_str() );
//                    return false;
//                }
//                return true;
//            }

//            /// Wrap C API to write camera model xml string.
//            std::string WriteToString()
//            {
//                std::string sRes;
//                mvl_write_camera_to_string( m_dPose.data(), m_pCameraModel.get(), sRes );
//                return sRes;
//            }

            /// Return K projection matrix.
            Eigen::Matrix3d K() const
            {
                double dK[9];
                if( mvl_camera_model_to_projmat( m_pCameraModel.get(), dK ) < 0 ){
                    fprintf( stderr, "ERROR: failed to build K matrix" );
                }
                return Eigen::Matrix<double,3,3,Eigen::RowMajor>( dK );
            }

            /// return pointer to mvl c camera model.
            mvl_camera_t* GetModel() const
            {
                return m_pCameraModel.get();
            }

            /// return pointer to pose matrix
            Eigen::Matrix4d GetPose() const
            {
                return m_dPose;
            }

            /// Return camera screen width in pixels.
            double Width() const
            {
                return m_pCameraModel->width;
            }

            /// Return camera screen height in pixels.
            double Height() const
            {
                return m_pCameraModel->height;
            }

//            /// report camera model version number
//            int Version() const
//            {
//                return m_pCameraModel->version;
//            }

            /// report camera model version number
            int Type() const
            {
                return m_pCameraModel->type;
            }

//            /// set the mvl camera model name
//            void SetName( const std::string& sName )
//            {
//                strcpy( m_pCameraModel->name, sName.c_str() );
//            }

//            /// set the mvl camera model name
//            void SetSerialNumber( const long int nSerialNo )
//            {
//                m_pCameraModel->serialno = nSerialNo;
//            }

//            /// set the camera index (for multi-camera rigs)
//            void SetIndex( const int nIndex )
//            {
//                m_pCameraModel->index = nIndex;
//            }

            /// Return 3x3 RDF matrix
            Eigen::Matrix3d RDF() const
            {
                Eigen::Matrix<double,3,3,Eigen::RowMajor> M( m_pCameraModel->RDF );
                return M;
            }

            /// Project point into image using c api.
//            Eigen::Vector2d Project3dTo2d( const Eigen::Matrix4d& T, const Eigen::Vector3d& p )
//            {
//                Eigen::Vector2d px;
//                mvl_camera_3d_to_2d( m_pCameraModel.get(), T.data(),  p.data(),px.data(), NULL );
//                return px;
//            }

        protected:
            CameraModel& operator=( CameraModel& c ); // to dangerous to use, at it can result in copying entire LUTs

        private:
            Eigen::Matrix4d                   m_dPose;
            boost::shared_ptr<mvl_camera_t>   m_pCameraModel;
            std::string                       m_sFileName;
    };
} // mvl namespace

#endif

