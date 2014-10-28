#include <Eigen/Eigen>
#include <map>
#include <vector>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

#include "math.h"

namespace Eigen{
  typedef Matrix<double, 6, 1> Vector6d;
}

struct tag_t
{
  Eigen::Vector3d tr, tl, br, bl;
  Eigen::Matrix<double, 6, 1> pose;
  unsigned long long data;
  unsigned char color_high, color_low;

  tag_t() {
    tr << 0, 0, 0;
    tl << 0, 0, 0;
    br << 0, 0, 0;
    bl << 0, 0, 0;
  }

  bool CheckZero( Eigen::Vector3d t )
  {
    return (t.norm() == 0);
  }

  void remove_border( void )
  {
    //  The points captured include a black border.  Remove it.
    Eigen::Vector3d dx, dy;
    dx = (tr - tl) / 8;
    dy = (bl - tl) / 8;
    tl = tl + dx + dy;

    dx = (tl - tr) / 8;
    dy = (br - tr) / 8;
    tr = tr + dx + dy;

    dx = (br - bl) / 8;
    dy = (tl - bl) / 8;
    bl = bl + dx + dy;

    dx = (tl - tr) / 8;
    dy = (tl - bl) / 8;
    br = br + dx + dy;
  }

  void CalcTagPose( void )
  {
    if (!CheckZero(tr) && !CheckZero(tl) && !CheckZero(br) && !CheckZero(bl)) {
      Eigen::Vector3d i, j, k, ave;

//      remove_border();
      i = 0.5*((br - bl) + (tr - tl));
      i /= i.norm();
      j = 0.5*((tl - bl) + (tr - br));
      j /= j.norm();
      k = i.cross(j);

      k = k / k.norm();
      j = k.cross(i);

      ave = 0.25*(tr + tl + br + bl);

      Eigen::Matrix4d tran;

      tran << i(0), j(0), k(0),  ave(0),
              i(1), j(1), k(1),  ave(1),
              i(2), j(2), k(2),  ave(2),
                 0,    0,    0,       1;

      pose = _T2Cart(tran);
    }
  }

  void AddPoint(int lm_id, Eigen::Vector3d pt)
  {
    switch(lm_id) {
    case 0: tl = pt; break;
    case 1: bl = pt; break;
    case 2: br = pt; break;
    case 3: tr = pt; break;
    }
    CalcTagPose();
  }

};


/////////////////////////////////////////////////////////////////////////
void Warp_Pts(double &u, double &v, double w)
{
  double ru = sqrtf(u*u + v*v);
  double rd = (1 / w) * atan(2 * ru * tan( w / 2));
  double factor = rd / ru;
//  double rd = sqrtf(u*u + v*v);
//  double ru = tan(rd * w) / (2* tan(w / 2));
//  double factor = ru / rd;
  u *= factor;
  v *= factor;
}

/////////////////////////////////////////////////////////////////////////
Eigen::Vector6d CalcPose(
    double pts_2d[4][2],
const Eigen::Vector3d pts_3d[4],
const Eigen::Matrix3d& K,
const double w
)
{
  std::vector<cv::Point3f> cv_obj;
  std::vector<cv::Point2f> cv_img;

  cv_obj.push_back( cv::Point3f(pts_3d[0][0],pts_3d[0][1],pts_3d[0][2]) );
  cv_obj.push_back( cv::Point3f(pts_3d[3][0],pts_3d[3][1],pts_3d[3][2]) );
  cv_obj.push_back( cv::Point3f(pts_3d[2][0],pts_3d[2][1],pts_3d[2][2]) );
  cv_obj.push_back( cv::Point3f(pts_3d[1][0],pts_3d[1][1],pts_3d[1][2]) );

  cv_img.push_back( cv::Point2f(pts_2d[0][0],pts_2d[0][1]) );
  cv_img.push_back( cv::Point2f(pts_2d[1][0],pts_2d[1][1]) );
  cv_img.push_back( cv::Point2f(pts_2d[2][0],pts_2d[2][1]) );
  cv_img.push_back( cv::Point2f(pts_2d[3][0],pts_2d[3][1]) );

  cv::Mat cv_K(3,3,CV_64F);
  cv::eigen2cv(K, cv_K);

  cv::Mat cv_coeff;
  cv::Mat cv_rot(3,1,CV_64F);
  cv::Mat cv_trans(3,1,CV_64F);

  cv::solvePnP( cv_obj, cv_img, cv_K, cv_coeff, cv_rot, cv_trans, false );

  Eigen::Vector3d rot, trans;
  cv::cv2eigen(cv_rot, rot);
  cv::cv2eigen(cv_trans, trans);

  Eigen::Vector6d pose;
  pose << trans(0), trans(1), trans(2), rot(0), rot(1), rot(2);

  return pose;
}

struct measurement_t
{
  int pose_idx;
  int lm_idx;
  double u;
  double v;
};

struct tag2d{
  Eigen::Vector2d tr;
  Eigen::Vector2d tl;
  Eigen::Vector2d br;
  Eigen::Vector2d bl;
};

struct detection{
  Eigen::Matrix<double,6,1>   pose;
  int                         tag_id;
  tag_t                       tag_data;
  tag2d                       tag_corners;
  cv::Mat                     image;
};


