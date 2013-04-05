#include "pnp.h"

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

using namespace std;
using namespace Eigen;

namespace fiducials {

vector<int> PosePnPRansac(
    const CameraModelBase& cam,
    const std::vector<Vector2d> &img_pts,
    const vector<Vector3d> & ideal_pts,
    const vector<int> & candidate_map,
    int robust_3pt_its,
    float robust_3pt_tol,
    Sophus::SE3d * T
) {
  vector<int> inlier_map(candidate_map.size(), -1);
  std::vector<cv::Point3f> cv_obj;
  std::vector<cv::Point2f> cv_img;
  std::vector<int> idx_vec;
  cv::Mat cv_coeff;
  cv::Mat cv_rot(3,1,CV_64F);
  cv::Mat cv_trans(3,1,CV_64F);
  cv::Mat cv_K(3,3,CV_64F);

//  cv::eigen2cv(cam.K(), cv_K);
  cv::setIdentity(cv_K);

  for (size_t i = 0; i<img_pts.size(); ++i)
  {
    int ideal_point_id = candidate_map[i];
    if (ideal_point_id>=0)
    {
//        const Eigen::Vector2d & center = img_pts[i];
        const Eigen::Vector2d center = cam.Unmap(img_pts[i]);
        const Eigen::Vector3d & c3d = ideal_pts[ideal_point_id];
        cv_img.push_back(cv::Point2f(center.x(), center.y()));
        cv_obj.push_back(cv::Point3f(c3d.x(), c3d.y(), c3d.z()));
        idx_vec.push_back(i);
    }
  }

  std::vector<int> cv_inliers;
  
  if(cv_img.size() < 4)
      return cv_inliers;
  
  if(robust_3pt_its > 0) {
      cv::solvePnPRansac(cv_obj, cv_img, cv_K, cv_coeff, cv_rot, cv_trans,
                         false, robust_3pt_its, robust_3pt_tol / cam.K()(0,0), 60, cv_inliers);
  }else{
      cv::solvePnP(cv_obj, cv_img, cv_K, cv_coeff, cv_rot, cv_trans, false);
  }
  
  Vector3d rot, trans;
  cv::cv2eigen(cv_rot, rot);
  cv::cv2eigen(cv_trans, trans);

  if(std::isnan(rot[0]) || std::isnan(rot[1]) || std::isnan(rot[2]))
    return inlier_map;

  for (size_t i = 0; i<cv_inliers.size(); ++i)
  {
    int idx = cv_inliers[i];
    inlier_map.at(idx_vec.at(idx)) = candidate_map.at(idx_vec.at(idx));
  }

  *T =  Sophus::SE3(Sophus::SO3::exp(rot), trans);
  return inlier_map;
}

int CountInliers(const vector<int> & conics_target_map)
{
  int inliers =0;
  for (size_t i=0; i < conics_target_map.size(); ++i)
  {
    if( conics_target_map[i] >=0 )
    {
      inliers++;
    }
  }
  return inliers;
}

double ReprojectionErrorRMS(const CameraModelBase& cam,
                            const Sophus::SE3d& T_cw,
                            const vector<Vector3d>& pts3d,
                            const vector<Vector2d>& pts2d,
                            const vector<int>& map2d_3d)
{
  int n=0;
  double sse =0;
  for( unsigned i=0; i<pts2d.size(); ++i )
  {
    const int ti = map2d_3d[i];
    if( ti >= 0 )
    {
      const Vector2d t = cam.Map(Project(T_cw * pts3d[ti]));
      Vector2d err = t - pts2d[i].head<2>();
      sse += (err).squaredNorm();
      ++n;
    }
  }
  return sqrt(sse / n);
}

}
