#include "pnp.h"

#include <opencv/cv.h>

using namespace std;
using namespace TooN;

void PoseFromPoints(
    const LinearCamera& cam,
    const vector<Vector<3> >& pts3d,
    const vector<Vector<2> >& pts2d,
    const vector<int>& map2d_3d,
    SE3<>& T_cw,
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
        const Vector<2> m = cam.unmap(pts2d[i]);
        const Vector<3> t = pts3d[ti];
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

      Vector<3> rot_vec = T_cw.get_rotation().ln();
      Vector<3> trans = T_cw.get_translation();
      cv::Mat cv_rot(3,1,CV_64FC1,rot_vec.my_data);
      cv::Mat cv_trans(3,1,CV_64FC1,trans.my_data);
      cv::solvePnP(
        cv_matched_3d, cv_matched_obs,
        cv_K, cv_dist, cv_rot, cv_trans, use_guess
      );
      T_cw =  SE3<double>(TooN::SO3<double>(rot_vec),trans);
    }
  }
}

double ReprojectionErrorRMS(
  const AbstractCamera& cam,
  const SE3<>& T_cw,
  const vector<Vector<3> >& pts3d,
  const vector<Vector<2> >& pts2d,
  const vector<int>& map2d_3d
) {
  int n=0;
  double sse =0;

  for( unsigned i=0; i<pts2d.size(); ++i ) {
    const int ti = map2d_3d[i];
    if( ti >= 0 ) {
      const Vector<2> t = cam.project_map(T_cw * pts3d[ti]);
      sse += norm_sq(t - pts2d[i]);
      ++n;
    }
  }

  return sqrt(sse / n);
}
