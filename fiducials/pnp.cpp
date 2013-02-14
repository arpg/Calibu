#include "pnp.h"

#include <opencv/cv.h>
#include "utils.h"

using namespace std;
using namespace Eigen;

//#include <tag/fourpointpose.h>
//#include <tag/ransac.h>
//using namespace tag;

void PoseFromPointsLeastSq(
    const LinearCamera& cam,
    const vector<Vector3d >& pts3d,
    const vector<Vector2d >& pts2d,
    const vector<int>& map2d_3d,
    Sophus::SE3& T_cw,
    bool use_guess
) {
  // Attempt to compute pose
  if( pts2d.size() >= 4 )
  {
    vector<cv::Point2f> cvimg;
    vector<cv::Point3f> cvpts;
    for( unsigned int i=0; i < pts2d.size(); ++ i)
    {
      const int ti = map2d_3d[i];
      if( 0 <= ti)
      {
        assert( ti < (int)pts3d.size() );
        const Vector2d m = cam.unmap(pts2d[i]);
        const Vector3d t = pts3d[ti];
        cvimg.push_back( cv::Point2f( m[0],m[1] ) );
        cvpts.push_back( cv::Point3f( t[0], t[1], t[2] ) );
      }
    }
    if( cvimg.size() >= 4 )
    {
      cv::Mat cv_matched_3d(cvpts);
      cv::Mat cv_matched_obs(cvimg);
      cv::Mat cv_K(3,3,CV_64FC1);
      cv::setIdentity(cv_K);
      cv::Mat cv_dist(4,1,CV_64FC1,0.0);

      Vector3d rot_vec = T_cw.so3().log();
      Vector3d trans = T_cw.translation();
      cv::Mat cv_rot(3,1,CV_64FC1, &(rot_vec(0)) );
      cv::Mat cv_trans(3,1,CV_64FC1, &(trans(0)) );
      cv::solvePnP(
        cv_matched_3d, cv_matched_obs,
        cv_K, cv_dist, cv_rot, cv_trans, use_guess
      );
      T_cw = Sophus::SE3(Sophus::SO3::exp(rot_vec),trans);
    }
  }
}

void PoseFromPointsLeastSqRansac(
    const LinearCamera& cam,
    const vector<Vector3d >& pts3d,
    const vector<Vector2d >& pts2d,
    vector<int>& map2d_3d,
    Sophus::SE3& T_cw,
    float robust_iterations,
    float robust_inlier_tol
) {
    if( pts2d.size() >= 4 )
    {
      vector<cv::Point2f> cvimg;
      vector<cv::Point3f> cvpts;
      for( unsigned int i=0; i < pts2d.size(); ++ i)
      {
        const int ti = map2d_3d[i];
        if( 0 <= ti)
        {
          assert( ti < (int)pts3d.size() );
          const Vector2d m = cam.unmap(pts2d[i]);
          const Vector3d t = pts3d[ti];
          cvimg.push_back( cv::Point2f( m[0],m[1] ) );
          cvpts.push_back( cv::Point3f( t[0], t[1], t[2] ) );
        }
      }
      
      if( cvimg.size() >= 4 )
      {
        cv::Mat cv_matched_3d(cvpts);
        cv::Mat cv_matched_obs(cvimg);
        cv::Mat cv_K(3,3,CV_64FC1);
        cv::setIdentity(cv_K);
        cv::Mat cv_dist(4,1,CV_64FC1,0.0);
  
        Vector3d rot_vec;
        Vector3d trans;
        cv::Mat cv_rot(3,1,CV_64FC1, &(rot_vec(0)) );
        cv::Mat cv_trans(3,1,CV_64FC1, &(trans(0)) );
        
        std::vector<int> inliers;
        cv::solvePnPRansac(
          cv_matched_3d, cv_matched_obs,
          cv_K, cv_dist, cv_rot, cv_trans, false, robust_iterations,
          robust_inlier_tol, 10, inliers
        );
        for( unsigned int i=0; i < pts2d.size(); ++ i)
        {
            bool inlier = false;
            for(unsigned int j=0; j < inliers.size(); ++j) {
                if( inliers[j] = i ) {
                    inlier = true;
                    break;
                }
            }
            if(!inlier) {
                map2d_3d[i] = -1;
            }
        }
        T_cw = Sophus::SE3(Sophus::SO3::exp(rot_vec),trans);
      }
    }
}

//struct Correspondence2D3D
//{
//    TooN::Vector<2> pixel;
//    TooN::Vector<3> position;
//    int pt2d;
//    int pt3d;
//};

Sophus::SE3 FindPose(
    const LinearCamera& cam,
    const vector<Vector3d >& pts3d,
    const vector<Vector2d >& pts2d,
    vector<int>& map2d_3d,
    double robust_inlier_tol,
    size_t robust_iterations
) {
    Sophus::SE3 T;
//    PoseFromPointsLeastSq(cam,pts3d,pts2d,map2d_3d,T,false);
    PoseFromPointsLeastSqRansac(cam,pts3d,pts2d,map2d_3d,T,robust_iterations,robust_inlier_tol);
    return T;

//    TooN::Tag version
//    std::vector<Correspondence2D3D> observations;
//    for( int i=0; i< map2d_3d.size(); ++i )
//    {
//        const int pt3d = map2d_3d[i];
//        if( pt3d >= 0 )
//        {
//            observations.push_back( (Correspondence2D3D){
//              toTooN<double,2>(cam.unmap(pts2d[i])), toTooN<double,3>(pts3d[pt3d]), i, pt3d }
//            );
//        }
//    }

//    if( observations.size() >= Point4SE3Estimation<>::hypothesis_size )
//    {
//        Point4SE3Estimation<> est;

//        std::vector<bool> inliers(observations.size());
//        tag::find_MSAC_inliers<Correspondence2D3D,Point4SE3Estimation<>,double>(
//            observations, robust_inlier_tol, robust_iterations, est, inliers, Point4SE3Estimation<>::hypothesis_size
//        );

//        for( int i=0; i<observations.size(); ++i )
//            if( !inliers[i] ) map2d_3d[observations[i].pt2d] = -1;

//        Sophus::SE3 T = toEigen(est.T);
//        PoseFromPointsLeastSq(cam,pts3d,pts2d,map2d_3d,T,true);
//        return T;
//    }
//    return Sophus::SE3();
}

double ReprojectionErrorRMS(
  const AbstractCamera& cam,
  const Sophus::SE3& T_cw,
  const vector<Vector3d >& pts3d,
  const vector<Vector2d >& pts2d,
  const vector<int>& map2d_3d
) {
  int n=0;
  double sse =0;

  for( unsigned i=0; i<pts2d.size(); ++i ) {
    const int ti = map2d_3d[i];
    if( ti >= 0 ) {
      const Vector2d t = cam.project_map(T_cw * pts3d[ti]);
      sse += (t - pts2d[i]).squaredNorm();
      ++n;
    }
  }

  return sqrt(sse / n);
}
