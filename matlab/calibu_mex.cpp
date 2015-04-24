#include "mex.h"
#include "class_handle.hpp"
#include "calibu/cam/camera_crtp.h"
#include "calibu/cam/camera_xml.h"

std::shared_ptr<calibu::Rig<double>> calibu_wrap;

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  // Command string.
  char cmd[64];

  // Check function string.
  if (nrhs < 1 || mxGetString(prhs[0], cmd, sizeof(cmd))) {
    mexErrMsgTxt("First input should be a string less than 64 characters long.");
  }

  /// Constructor.
  if (!strcmp("new", cmd)) {
    // Check parameters.
    if (nlhs != 1) {
      mexErrMsgTxt("New: One output expected.");
    }
    if (nrhs != 2) {
      mexErrMsgTxt("New: Filename path is expected as second argument.");
    }

    // Get filename from args.
    char filename[1024];
    mxGetString(prhs[1], filename, sizeof(filename));
    std::string sfilename(filename);

    // Check if file exists.
    if (FILE *file = fopen(sfilename.c_str(), "r")) {
      fclose(file);
    } else {
      mexErrMsgTxt("New: Error opening camera model file. Does it exist?");
    }

    calibu_wrap = calibu::ReadXmlRig(sfilename);
    plhs[0] = convertPtr2Mat<calibu::Rig<double>>(calibu_wrap.get());

    return;
  }


  /// Delete pointer.
  if (!strcmp("delete", cmd)) {
    // Destroy the C++ object.
    destroyObject<calibu::Rig<double>>(prhs[1]);

    // Warn if other commands were ignored.
    if (nlhs != 0 || nrhs != 2) {
      mexWarnMsgTxt("Delete: Unexpected arguments ignored.");
    }

    return;
  }


  // Get the class instance pointer from the second input.
  calibu::Rig<double>* calibu_cam_ptr = convertMat2Ptr<calibu::Rig<double>>(prhs[1]);


  /// Project.
  if (!strcmp("project", cmd)) {
    unsigned int camera_id = static_cast<unsigned int>(*mxGetPr(prhs[2]));

    if (camera_id == 0 || camera_id > calibu_cam_ptr->cameras_.size()) {
      mexErrMsgTxt("Camera ID is out of bounds.");
      return;
    }

    double* ray_ptr = mxGetPr(prhs[3]);
    Eigen::Vector3d ray;
    ray << ray_ptr[0], ray_ptr[1], ray_ptr[2];
    plhs[0] = mxCreateDoubleMatrix(2, 1, mxREAL);
    double* pixel_coordinate_ptr = mxGetPr(plhs[0]);

    Eigen::Vector2d pixel_coordinate;
    pixel_coordinate = calibu_cam_ptr->cameras_[camera_id-1]->Project(ray);

    pixel_coordinate_ptr[0] = pixel_coordinate[0];
    pixel_coordinate_ptr[1] = pixel_coordinate[1];

    return;
  }


  /// Project Points.
  if (!strcmp("project_points", cmd)) {
    unsigned int camera_id = static_cast<unsigned int>(*mxGetPr(prhs[2]));

    if (camera_id == 0 || camera_id > calibu_cam_ptr->cameras_.size()) {
      mexErrMsgTxt("Camera ID is out of bounds.");
      return;
    }

    unsigned int num_pts = static_cast<unsigned int>(*mxGetPr(prhs[3]));

    plhs[0] = mxCreateDoubleMatrix(2, num_pts, mxREAL);
    double* pixel_coordinate_ptr = mxGetPr(plhs[0]);

    for (size_t ii = 0; ii < num_pts; ++ii) {
      double* ray_ptr = mxGetPr(prhs[4]);
      int in_i = ii*3;
      Eigen::Vector3d ray;
      ray << ray_ptr[in_i], ray_ptr[in_i+1], ray_ptr[in_i+2];

      Eigen::Vector2d pixel_coordinate;
      pixel_coordinate = calibu_cam_ptr->cameras_[camera_id-1]->Project(ray);

      int out_i = ii*2;
      pixel_coordinate_ptr[out_i] = pixel_coordinate[0];
      pixel_coordinate_ptr[out_i+1] = pixel_coordinate[1];
    }

    return;
  }


  /// Unproject.
  if (!strcmp("unproject", cmd)) {
    unsigned int camera_id = static_cast<unsigned int>(*mxGetPr(prhs[2]));

    if (camera_id == 0 || camera_id > calibu_cam_ptr->cameras_.size()) {
      mexErrMsgTxt("Camera ID is out of bounds.");
      return;
    }

    double* pixel_coordinate_ptr = mxGetPr(prhs[3]);
    Eigen::Vector2d pixel_coordinate;
    pixel_coordinate << pixel_coordinate_ptr[0], pixel_coordinate_ptr[1];
    plhs[0] = mxCreateDoubleMatrix(3, 1, mxREAL);
    double* ray_ptr = mxGetPr(plhs[0]);

    Eigen::Vector3d ray;
    ray = calibu_cam_ptr->cameras_[camera_id-1]->Unproject(pixel_coordinate);

    ray_ptr[0] = ray[0];
    ray_ptr[1] = ray[1];
    ray_ptr[2] = ray[2];

    return;
  }

  /// Unproject Points.
  if (!strcmp("unproject_pixels", cmd)) {
    unsigned int camera_id = static_cast<unsigned int>(*mxGetPr(prhs[2]));

    if (camera_id == 0 || camera_id > calibu_cam_ptr->cameras_.size()) {
      mexErrMsgTxt("Camera ID is out of bounds.");
      return;
    }

    unsigned int num_pixels = static_cast<unsigned int>(*mxGetPr(prhs[3]));

    plhs[0] = mxCreateDoubleMatrix(3, num_pixels, mxREAL);
    double* rays_ptr = mxGetPr(plhs[0]);

    double* pixel_ptr = mxGetPr(prhs[4]);
    for (size_t ii = 0; ii < num_pixels; ++ii) {
      int in_i = ii*2;
      Eigen::Vector2d pixel;
      pixel << pixel_ptr[in_i], pixel_ptr[in_i+1];

      Eigen::Vector3d ray;
      ray = calibu_cam_ptr->cameras_[camera_id-1]->Unproject(pixel);

      int out_i = ii*3;
      rays_ptr[out_i] = ray[0];
      rays_ptr[out_i+1] = ray[1];
      rays_ptr[out_i+2] = ray[2];
    }

    return;
  }



  /// Transfer 3d.
  if (!strcmp("transfer_3d", cmd)) {
    unsigned int camera_id = static_cast<unsigned int>(*mxGetPr(prhs[2]));
    double* t_ba_ptr = mxGetPr(prhs[3]);
    double* ray_ptr = mxGetPr(prhs[4]);
    double* rho = mxGetPr(prhs[5]);

    if (camera_id == 0 || camera_id > calibu_cam_ptr->cameras_.size()) {
      mexErrMsgTxt("Camera ID is out of bounds.");
      return;
    }

    Eigen::Matrix4d t_ba_mat;
    t_ba_mat << t_ba_ptr[0], t_ba_ptr[4], t_ba_ptr[8], t_ba_ptr[12],
        t_ba_ptr[1], t_ba_ptr[5], t_ba_ptr[9], t_ba_ptr[13],
        t_ba_ptr[2], t_ba_ptr[6], t_ba_ptr[10], t_ba_ptr[14],
        t_ba_ptr[3], t_ba_ptr[7], t_ba_ptr[11], t_ba_ptr[15];
    Sophus::SE3d t_ba(t_ba_mat);
    Eigen::Vector3d ray;
    ray << ray_ptr[0], ray_ptr[1], ray_ptr[2];

    plhs[0] = mxCreateDoubleMatrix(2, 1, mxREAL);
    double* pixel_coordinate_ptr = mxGetPr(plhs[0]);

    Eigen::Vector2d pixel_coordinate;
    pixel_coordinate = calibu_cam_ptr->cameras_[camera_id-1]->
        Transfer3d(t_ba, ray, *rho);

    pixel_coordinate_ptr[0] = pixel_coordinate[0];
    pixel_coordinate_ptr[1] = pixel_coordinate[1];

    return;
  }


  /// Get K.
  if (!strcmp("get_K", cmd)) {
    unsigned int camera_id = static_cast<unsigned int>(*mxGetPr(prhs[2]));

    if (camera_id == 0 || camera_id > calibu_cam_ptr->cameras_.size()) {
      mexErrMsgTxt("Camera ID is out of bounds.");
      return;
    }

    plhs[0] = mxCreateDoubleMatrix(3, 3, mxREAL);
    double* k_ptr = mxGetPr(plhs[0]);

    Eigen::Matrix3d K = calibu_cam_ptr->cameras_[camera_id-1]->K();

    k_ptr[0] = K(0,0);
    k_ptr[1] = K(1,0);
    k_ptr[2] = K(2,0);

    k_ptr[3] = K(0,1);
    k_ptr[4] = K(1,1);
    k_ptr[5] = K(2,1);

    k_ptr[6] = K(0,2);
    k_ptr[7] = K(1,2);
    k_ptr[8] = K(2,2);

    return;
  }

  /// Get Trc.
  if (!strcmp("get_Trc", cmd)) {
    unsigned int camera_id = static_cast<unsigned int>(*mxGetPr(prhs[2]));

    if (camera_id == 0 || camera_id > calibu_cam_ptr->cameras_.size()) {
      mexErrMsgTxt("Camera ID is out of bounds.");
      return;
    }

    plhs[0] = mxCreateDoubleMatrix(4, 4, mxREAL);
    double* Trc_ptr = mxGetPr(plhs[0]);

    Eigen::Matrix4d Trc = calibu_cam_ptr->cameras_[camera_id-1]->Pose().matrix();

    Trc_ptr[0] = Trc(0,0);
    Trc_ptr[1] = Trc(1,0);
    Trc_ptr[2] = Trc(2,0);
    Trc_ptr[3] = Trc(3,0);

    Trc_ptr[4] = Trc(0,1);
    Trc_ptr[5] = Trc(1,1);
    Trc_ptr[6] = Trc(2,1);
    Trc_ptr[7] = Trc(3,1);

    Trc_ptr[8] = Trc(0,2);
    Trc_ptr[9] = Trc(1,2);
    Trc_ptr[10] = Trc(2,2);
    Trc_ptr[11] = Trc(3,2);

    Trc_ptr[12] = Trc(0,3);
    Trc_ptr[13] = Trc(1,3);
    Trc_ptr[14] = Trc(2,3);
    Trc_ptr[15] = Trc(3,3);

    return;
  }



  // Got here, so function not recognized.
  mexErrMsgTxt("Function not recognized.");
}
