#pragma once
#include <calibu/cam/camera_models_crtp.h>
#include <calibu/cam/camera_models_crtp.h>
#include <calibu/cam/camera_crtp_interop.h>

/////////////////////////////////////////////////////////////////////////
namespace calibu
{

  template<typename Scalar>
  class Cam : CameraInterface<Scalar>
  {
    public:
      /////////////////////////////////////////////////////////////////////////
      Vec3t<Scalar> Unproject(const Vec2t<Scalar>& pix) const 
      {
        return m_pCam->Unproject( pix );
      }

      /////////////////////////////////////////////////////////////////////////
      Vec2t<Scalar> Project(const Vec3t<Scalar>& ray) const
      {
        return m_pCam->Project( ray );
      }

      /////////////////////////////////////////////////////////////////////////
      Eigen::Matrix<Scalar, 2, 3> dProject_dray( const Vec3t<Scalar>& ray ) const
      {
        return m_pCam->dProject_dray( ray );
      }

      /////////////////////////////////////////////////////////////////////////
      Vec2t<Scalar> Transfer3d(const SE3t<Scalar>& t_ba,
          const Vec3t<Scalar>& ray,
          const Scalar rho) const
      {
        return m_pCam->Transfer3d( t_ba, ray, rho );
      }

      /////////////////////////////////////////////////////////////////////////
      Eigen::Matrix<Scalar, 2, 4> dTransfer3d_dray(
          const SE3t<Scalar>& t_ba,
          const Vec3t<Scalar>& ray,
          const Scalar rho) const
      {
        return m_pCam->dTransfer3d_dray( t_ba, ray, rho );
      }

      /////////////////////////////////////////////////////////////////////////
      Scalar* GetParams()
      {
        return m_pCam->GetParams();
      } 

      /////////////////////////////////////////////////////////////////////////
      Cam( const std::string& filename )
      {
        LoadCamera( filename );
      }

      /////////////////////////////////////////////////////////////////////////
      void LoadCamera( const std::string& filename )
      {
        CameraModelGeneric<double> old_cam = ReadXmlCameraModel( filename );
        // convert to new camera
        m_pCam = calibu::CreateFromOldCamera( old_cam );
      }

      /////////////////////////////////////////////////////////////////////////
      // pointer to implementation
      std::shared_ptr<CameraInterface<Scalar> >   m_pCam;
  };

}
