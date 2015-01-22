#pragma once

#include <mat.h>

#include <unistd.h>
#include <cstdlib>
#include <string>
#include <vector>

#include <calibu/cam/camera_crtp.h>
#include <calibu/cam/camera_crtp_interop.h>

class CalibuRig {
 public:

  ///////////////////////////////////////////////////////////////////////////
  /// Constructor.
  CalibuRig(const std::string& filename)
  {
    calibu::LoadRig(filename, &rig_);
  }

  ///////////////////////////////////////////////////////////////////////////
  /// Project.
  Eigen::Vector2d Project(unsigned int camera_id, double* ray_ptr)
  {
    Eigen::Vector3d ray;
    ray << ray_ptr[0], ray_ptr[1], ray_ptr[2];

    return rig_.cameras_[camera_id]->Project(ray);
  }

  ///////////////////////////////////////////////////////////////////////////
  /// Unproject.
  Eigen::Vector3d Unproject(unsigned int camera_id, double* pix_coord_ptr)
  {
    Eigen::Vector2d pix_coord;
    pix_coord << pix_coord_ptr[0], pix_coord_ptr[1];

    return rig_.cameras_[camera_id]->Unproject(pix_coord);
  }


  ///////////////////////////////////////////////////////////////////////////
  /// Transfer 3d.
  Eigen::Vector2d Transfer3d(unsigned int camera_id, double* t_ba_ptr,
                            double* ray_ptr, double* rho_ptr)
  {
    // Matlab is column major.
    Eigen::Matrix4d t_ba;
    t_ba << t_ba_ptr[0], t_ba_ptr[4], t_ba_ptr[8], t_ba_ptr[12],
            t_ba_ptr[1], t_ba_ptr[5], t_ba_ptr[9], t_ba_ptr[13],
            t_ba_ptr[2], t_ba_ptr[6], t_ba_ptr[10], t_ba_ptr[14],
            t_ba_ptr[3], t_ba_ptr[7], t_ba_ptr[11], t_ba_ptr[15];
    Sophus::SE3d t_ba_sophus(t_ba);

    Eigen::Vector3d ray;
    ray << ray_ptr[0], ray_ptr[1], ray_ptr[2];

    return rig_.cameras_[camera_id]->Transfer3d(t_ba_sophus, ray, *rho_ptr);
  }


  ///////////////////////////////////////////////////////////////////////////
  /// Return number of cameras.
  unsigned int NumCams()
  {
    return rig_.cameras_.size();
  }


  ///////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////

  private:
    calibu::Rig<double>     rig_;
};
