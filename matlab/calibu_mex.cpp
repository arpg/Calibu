#include "mex.h"
#include "class_handle.hpp"
#include "calibu.h"

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

    CalibuRig* calibu_wrap = new CalibuRig(sfilename);
    plhs[0] = convertPtr2Mat<CalibuRig>(calibu_wrap);

    return;
  }


  /// Delete pointer.
  if (!strcmp("delete", cmd)) {
    // Destroy the C++ object.
    destroyObject<CalibuRig>(prhs[1]);

    // Warn if other commands were ignored.
    if (nlhs != 0 || nrhs != 2) {
      mexWarnMsgTxt("Delete: Unexpected arguments ignored.");
    }

    return;
  }


  // Get the class instance pointer from the second input.
  CalibuRig *calibu_cam_ptr = convertMat2Ptr<CalibuRig>(prhs[1]);


  /// Project.
  if (!strcmp("project", cmd)) {
    unsigned int camera_id = static_cast<unsigned int>(*mxGetPr(prhs[2]));
    double* ray = mxGetPr(prhs[3]);

    if (camera_id == 0 || camera_id > calibu_cam_ptr->NumCams()) {
      mexErrMsgTxt("Camera ID is out of bounds.");
      return;
    }

    plhs[0] = mxCreateDoubleMatrix(1, 2, mxREAL);
    double* pixel_coordinate_ptr = mxGetPr(plhs[0]);

    Eigen::Vector2d pixel_coordinate;
    pixel_coordinate = calibu_cam_ptr->Project(camera_id-1, ray);

    pixel_coordinate_ptr[0] = pixel_coordinate[0];
    pixel_coordinate_ptr[1] = pixel_coordinate[1];

    return;
  }


  /// Unproject.
  if (!strcmp("unproject", cmd)) {
    unsigned int camera_id = static_cast<unsigned int>(*mxGetPr(prhs[2]));
    double* pixel_coordinate = mxGetPr(prhs[3]);

    if (camera_id == 0 || camera_id > calibu_cam_ptr->NumCams()) {
      mexErrMsgTxt("Camera ID is out of bounds.");
      return;
    }

    plhs[0] = mxCreateDoubleMatrix(1, 3, mxREAL);
    double* ray_ptr = mxGetPr(plhs[0]);

    Eigen::Vector3d ray;
    ray = calibu_cam_ptr->Unproject(camera_id-1, pixel_coordinate);

    ray_ptr[0] = ray[0];
    ray_ptr[1] = ray[1];
    ray_ptr[2] = ray[2];

    return;
  }


  /// Transfer 3d.
  if (!strcmp("transfer_3d", cmd)) {
    unsigned int camera_id = static_cast<unsigned int>(*mxGetPr(prhs[2]));
    double* t_ba = mxGetPr(prhs[3]);
    double* ray = mxGetPr(prhs[4]);
    double* rho = mxGetPr(prhs[5]);

    if (camera_id == 0 || camera_id > calibu_cam_ptr->NumCams()) {
      mexErrMsgTxt("Camera ID is out of bounds.");
      return;
    }

    plhs[0] = mxCreateDoubleMatrix(1, 2, mxREAL);
    double* pixel_coordinate_ptr = mxGetPr(plhs[0]);

    Eigen::Vector2d pixel_coordinate;
    pixel_coordinate = calibu_cam_ptr->Transfer3d(camera_id-1, t_ba, ray, rho);

    pixel_coordinate_ptr[0] = pixel_coordinate[0];
    pixel_coordinate_ptr[1] = pixel_coordinate[1];

    return;
  }


  // Got here, so function not recognized.
  mexErrMsgTxt("Function not recognized.");
}
