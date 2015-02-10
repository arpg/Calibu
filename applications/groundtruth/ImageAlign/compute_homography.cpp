#include "compute_homography.h"
#include <time.h>
#include <ceres/ceres.h>
#include <sophus/se3.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

extern "C"{
#include <vl/generic.h>
#include <vl/sift.h>
}

struct fd{
  float x, y;
  int d[128];
};

struct CostFunctor{
  CostFunctor( float x0, float y0, float x1, float y1 ) :
    x0_(x0), x1_(x1), y0_(y0), y1_(y1)
  {
  }

  template <typename T>
  bool operator()(const T* const x, T* residual) const
  {
    T A = x[0]*T(x0_) + x[3]*T(y0_) + x[6];
    T B = x[1]*T(x0_) + x[4]*T(y0_) + x[7];
    T z = x[2]*T(x0_) + x[5]*T(y0_) + T(1);
    A /= z;
    B /= z;
    residual[0] = A - T(x1_);
    residual[1] = B - T(y1_);
    return true;
  }

private:
  float x0_, x1_, y0_, y1_,;
};

template <typename Scalar>
inline Eigen::Matrix<Scalar, 4, 4> _Cart2T(
    Scalar x,
    Scalar y,
    Scalar z,
    Scalar r,
    Scalar p,
    Scalar q
    )
{
  Eigen::Matrix< Scalar, 4, 4> T;
  // psi = roll, th = pitch, phi = yaw
  Scalar cq, cp, cr, sq, sp, sr;
  cr = cos(r);
  cp = cos(p);
  cq = cos(q);

  sr = sin(r);
  sp = sin(p);
  sq = sin(q);

  T(0, 0) = cp * cq;
  T(0, 1) = -cr * sq + sr * sp * cq;
  T(0, 2) = sr * sq + cr * sp * cq;
  T(0, 3) = (Scalar) (0.0);

  T(1, 0) = cp * sq;
  T(1, 1) = cr * cq + sr * sp * sq;
  T(1, 2) = -sr * cq + cr * sp * sq;
  T(1, 3) = (Scalar) (0.0);

  T(2, 0) = -sp;
  T(2, 1) = sr * cp;
  T(2, 2) = cr * cp;
  T(2, 3) = (Scalar) (0.0);

  T(0, 3) = x;
  T(1, 3) = y;
  T(2, 3) = z;
  T(3, 3) = (Scalar) (1.0);
  return T;
}

template <typename T>
inline Eigen::Matrix<T, 4, 4> _Cart2T( Eigen::Matrix<T,6,1> x)
{
  return _Cart2T<T>(x(0),x(1),x(2),x(3),x(4),x(5));
}


struct OptimalCostFunctor
{
  OptimalCostFunctor(
      Eigen::Vector2d _x1,
      Eigen::Vector2d _x2,
      Eigen::Matrix3d _K,
      double _d
      ) : x1(_x1),  x2(_x2), K(_K), d(_d)
  {
  }

  template <typename T>
  bool operator()(
      const T* const _t_wi,  // world pose of i'th camera
      T* residuals
      ) const
  {
    const Eigen::Map< const Eigen::Matrix<T,6,1> > temp1(_t_wi);
    Eigen::Matrix< T, 6, 1> temp(temp1);
    const Sophus::SE3Group<T> t_wi = Sophus::SE3Group<T>( _Cart2T<T>(temp) );

    Eigen::Matrix<T,2,1> xij = x2.template cast<T>();
    Eigen::Matrix<T,3,1> pij;
    pij << xij(0), xij(1), T(1);


    Eigen::Matrix<T, 3, 3> k = K.template cast<T>();
    pij = T(d)*(k.inverse()) * pij;

    Eigen::Matrix<T,4,1> p;
    p << pij(0), pij(1), pij(2), T(1);
    p = t_wi.matrix() * p;
    pij << p(0) / p(3), p(1) / p(3), p(2) / p(3);

    pij = k * pij;

    xij << pij(0) / pij(2), pij(1) / pij(2);

    if ((xij(0) < T(0)) || (xij(0) > T(640))) {
      residuals[0] = T(FLT_MAX);
    } else {
      residuals[0] = T(x1(0)) - xij(0);
    }
    if ((xij(1) < T(0)) || (xij(1) > T(480))) {
      residuals[1] = T(FLT_MAX);
    } else {
      residuals[1] = T(x1(1)) - xij(1);
    }

    return true;
  }

  Eigen::Vector2d x1;
  Eigen::Vector2d x2;
  Eigen::Matrix3d K;
  double d;
};


void test_reproject( Eigen::Vector2d x1, Eigen::Vector2d x2,
                     Eigen::Matrix3d K,  float d, double* _t_wi)
{
  const Eigen::Map< const Eigen::Matrix<double,6,1> > temp1(_t_wi);
  Eigen::Matrix< double, 6, 1> temp(temp1);
  const Sophus::SE3d t_wi = Sophus::SE3d( _Cart2T<double>(temp) );

  Eigen::Vector2d xij = x2;
  Eigen::Vector3d pij;
  Eigen::Vector4d p;

  pij << xij(0), xij(1), 1;
  pij = d*(K.inverse() * pij);

  p << pij(0), pij(1), pij(2), 1;
  p = t_wi.matrix() * p;
  pij << p(0) / p(3), p(1) / p(3), p(2) / p(3);

  pij = K * pij;
  xij << pij(0) / pij(2), pij(1) / pij(2);

  fprintf(stderr, "(%f, %f) -> (%f, %f)\n", x1(0), x1(1), xij(0), xij(1));
  fflush(stderr);
}

float distance( cv::Mat d1, cv::Mat d2 )
{
  float sum = 0;
  cv::Mat diff = d1 - d2;
  diff.mul(diff);
  sum = cv::sum(diff)[0];
  return sqrt(sum);
}

float distance( int* d1, int* d2 )
{
  int sum = 0;
  for (int ii = 0; ii < 128; ii++) {
    int c = d1[ii] - d2[ii];
    sum += c*c;
  }
  return sum;
}

std::map<int, int> find_matches_(std::vector< fd > k1,
                                 std::vector< fd > k2 )
{
  std::map<int, int> toRet;

  for (int ii = 0; ii < k1.size(); ii++) {
    int min1 = INT_MAX;
    int min2 = INT_MAX;
    int id = -1;
    for(int jj = 0; jj < k2.size(); jj++) {
      int dist = 0;
      for (int c = 0; c < 128; c++) {
        int s = k1[ii].d[c] - k2[jj].d[c];
        dist += s*s;
      }
      if ((pow(k1[ii].x - k2[jj].x, 2) + pow(k1[ii].y - k2[jj].y, 2)) < 100) {
        if (dist < min1){
          id = jj;
          min2 = min1;
          min1 = dist;
        } else if (dist < min2) {
          min2 = dist;
        }
      }
    }
    if ((1.5*min1 < min2) && (id != -1)) {
      toRet.insert(std::pair<int, int>(ii, id));
    }
  }
  return toRet;
}

std::vector< int > random_vals(int n, int y)
{
  std::vector< int > toRet;
  int count = 0;
  while (count < y) {
    int val = rand() % n;
    if (std::find(toRet.begin(), toRet.end(), val) == toRet.end()) {
      toRet.push_back(val);
      count++;
    }
  }
  return toRet;
}

cv::Mat kron(cv::Mat a, cv::Mat b)
{
  int h = a.rows*b.rows;
  int w = a.cols*b.cols;
  cv::Mat toRet(h, w, CV_32F);
  for (int ii = 0; ii < a.rows; ii++) {
    for (int jj = 0; jj < a.cols; jj++) {
      for(int kk = 0; kk < b.rows; kk++) {
        for (int ll = 0; ll < b.cols; ll++) {
          float val = a.at<float>(ii, jj)*b.at<float>(kk, ll);
          toRet.at<float>(ii*b.rows + kk, jj*b.cols + ll) = val;
        }
      }
    }
  }
  return toRet;
}

cv::Mat hat(cv::Mat in)
{
  cv::Mat toRet(3, 3, CV_32F);
  toRet.at<float>(0, 0) = 0;
  toRet.at<float>(0, 1) = -in.at<float>(0, 2);
  toRet.at<float>(0, 2) = in.at<float>(0, 1);
  toRet.at<float>(1, 0) = in.at<float>(0, 2);
  toRet.at<float>(1, 1) = 0;
  toRet.at<float>(1, 2) = -in.at<float>(0, 0);
  toRet.at<float>(2, 0) = -in.at<float>(0, 1);
  toRet.at<float>(2, 1) = in.at<float>(0, 0);
  toRet.at<float>(2, 2) = 0;
  return toRet;
}

cv::Mat computeSVD(cv::Mat A)
{
  cv::Mat c(A.cols, A.cols, CV_32F);
  c = A.t()*A;
  cv::Mat evs, vals;
  cv::eigen(c, true, vals, evs);
  return evs;
}

cv::Mat H( std::map<int, int> m,
           std::vector< fd > k1,
           std::vector< fd > k2 )
{
  int n = m.size();
  cv::Mat X1(3, n, CV_32F);
  cv::Mat X2(3, n, CV_32F);
  int count = 0;
  std::vector< cv::Mat > hs;
  std::vector< cv::Mat > oks;
  std::vector< int > scores;

  for (std::map<int, int>::iterator it = m.begin(); it != m.end(); it++) {
    X1.at<float>(0, count) = k1[it->first].x;
    X1.at<float>(1, count) = k1[it->first].y;
    X1.at<float>(2, count) = 1;
    X2.at<float>(0, count) = k2[it->second].x;
    X2.at<float>(1, count) = k2[it->second].y;
    X2.at<float>(2, count) = 1;
    count++;
  }

  //  srand(time(NULL));
  for (int t = 0; t < 100; t++){
    std::vector< int > l = random_vals(n, 4);
    cv::Mat A;
    for (int c = 0; c < 4; c++){
      A.push_back(kron(X1.col(l[c]).t(), hat(X2.col(l[c]).t())));
    }
    cv::Mat w, u, v;
    cv::SVD::compute(A, w, u, v);
    v = -v.t();
    cv::Mat V(3, 3, CV_32F);
    for (int ii = 0; ii < 9; ii++) {
      V.at<float>(ii % 3, ii / 3) = v.at<float>(ii, 8);
    }
    hs.push_back(V);
    cv::Mat X2_;
    X2_ = V*X1;

    cv::Mat du = (X2_.row(0) / X2_.row(2)) - (X2.row(0) / X2.row(2));
    cv::Mat dv = (X2_.row(1) / X2_.row(2)) - (X2.row(1) / X2.row(2));
    cv::Mat ok = (du.mul(du) + dv.mul(dv)) < 6*6;
    ok = ok / cv::max(ok, 1);
    double score = cv::sum(ok)[0];
    oks.push_back(ok);
    scores.push_back(score);
  }

  int max_c = 0;
  int id = 0;
  for (int count = 0; count < scores.size(); count++) {
    if (scores[count] > max_c) {
      max_c = scores[count];
      id = count;
    }
  }

  // After RANSAC, perform Ceres minimzation on H to optimize H from X1 to X2
  cv::Mat H = hs[id];
  double h[8];
  for (int ii = 0; ii < 8; ii++) {
    h[ii] = H.at<float>(ii % 3, ii / 3) / H.at<float>(2, 2);
  }
  H.at<float>(2, 2) = 1;

  ceres::Problem problem;
  cv::Mat ok = oks[id];

  std::cout<<ok<<std::endl;
  for (unsigned int ii = 0; ii < X1.cols; ii++) {
    if (ok.data[ii] != 0)  {
      ceres::CostFunction* cost_function =
          new ceres::AutoDiffCostFunction<CostFunctor, 2, 8>(
            new CostFunctor(X1.at<float>(0, ii), X1.at<float>(1, ii),
                            X2.at<float>(0, ii), X2.at<float>(1, ii)));
      problem.AddResidualBlock(cost_function, NULL, h);
    }
  }
  ceres::Solver::Options options;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  for (int ii = 0; ii < 8; ii++) {
    H.at<float>(ii % 3, ii / 3) = (float) h[ii];
  }

  return H;
}

void cv2vl( cv::Mat in, vl_sift_pix* out )
{
  for (int i = 0; i < in.rows; ++i)
    for (int j = 0; j < in.cols; ++j)
      out[j + i*in.cols] = in.at<uchar>(i, j);
}

void process_image(VlSiftFilt* sf, cv::Mat image, std::vector< fd > &fds)
{
  vl_sift_pix* vl_im = new vl_sift_pix[image.rows*image.cols];
  cv2vl(image, vl_im);
  vl_bool first = true;

  while (1) {
    int err;
    VlSiftKeypoint const *kp = 0;

    if (first) {
      err = vl_sift_process_first_octave(sf, vl_im);
      first = 0;
    } else {
      err = vl_sift_process_next_octave(sf);
    }
    if (err) break;
    vl_sift_detect(sf);
    kp = vl_sift_get_keypoints(sf);
    int keys = vl_sift_get_nkeypoints(sf);
    for (int count = 0; count < keys; count++) {
      VlSiftKeypoint const *k = 0;
      k = kp + count;
      double angles[4];
      int nangles = vl_sift_calc_keypoint_orientations(sf, angles, k);
      for (int na = 0; na < nangles; na++) {
        fd temp;
        temp.x = k->x;
        temp.y = k->y;
        float buf[128];
        vl_sift_calc_keypoint_descriptor(sf, buf, k, angles[na]);
        for (int i = 0; i < 128; i++) {
          temp.d[i] = (int)(512*buf[i]);
        }
        fds.push_back(temp);
      }
    }
  }
  delete[] vl_im;
}

cv::Mat find_ok_( std::vector< fd > fds1,
                  std::vector< fd > fds2,
                  std::map<int, int> matches )
{
  int n = matches.size();
  cv::Mat X1(3, n, CV_32F);
  cv::Mat X2(3, n, CV_32F);
  int count = 0;
  std::vector< cv::Mat > hs;
  std::vector< cv::Mat > oks;
  std::vector< int > scores;

  for (std::map<int, int>::iterator it = matches.begin(); it != matches.end(); it++) {
    X1.at<float>(0, count) = fds1[it->first].x;
    X1.at<float>(1, count) = fds1[it->first].y;
    X1.at<float>(2, count) = 1;
    X2.at<float>(0, count) = fds2[it->second].x;
    X2.at<float>(1, count) = fds2[it->second].y;
    X2.at<float>(2, count) = 1;
    count++;
  }

  //  srand(time(NULL));
  for (int t = 0; t < 100; t++){
    std::vector< int > l = random_vals(n, 4);
    cv::Mat A;
    for (int c = 0; c < 4; c++){
      A.push_back(kron(X1.col(l[c]).t(), hat(X2.col(l[c]).t())));
    }
    cv::Mat w, u, v;
    cv::SVD::compute(A, w, u, v);
    v = v.t();
    if (cv::determinant(v) < 0) {
      v = -v;
    }
    cv::Mat V(3, 3, CV_32F);
    for (int ii = 0; ii < 9; ii++) {
      V.at<float>(ii % 3, ii / 3) = v.at<float>(ii, 8);
    }
    hs.push_back(V);
    cv::Mat X2_;
    X2_ = V*X1;

    cv::Mat du = (X2_.row(0) / X2_.row(2)) - (X2.row(0) / X2.row(2));
    cv::Mat dv = (X2_.row(1) / X2_.row(2)) - (X2.row(1) / X2.row(2));
    cv::Mat ok = (du.mul(du) + dv.mul(dv)) < 6*6;
    ok = ok / cv::max(ok, 1);
    double score = cv::sum(ok)[0];
    oks.push_back(ok);
    scores.push_back(score);
  }

  int max_c = 0;
  int id = 0;
  for (int count = 0; count < scores.size(); count++) {
    if (scores[count] > max_c) {
      max_c = scores[count];
      id = count;
    }
  }

  return oks[id];
}

Eigen::Matrix4d estimate_pose_( cv::Mat image1, cv::Mat image2,
                                cv::Mat depth, Eigen::Matrix3d K )
//  image1 = captured, image2 = synthetic, depth = depth of synthetic
{
  int h = image1.rows;
  int w = image1.cols;

  VlSiftFilt* sf = vl_sift_new(w, h, 5, 3, 0);
  sf->peak_thresh = 5;

  std::vector< fd > fds1, fds2;
  process_image(sf, image1, fds1);
  process_image(sf, image2, fds2);

  std::map<int, int> matches, matches_forward, matches_reverse;
  matches_forward = find_matches_(fds1, fds2);
  matches_reverse = find_matches_(fds2, fds1);

  for (std::map<int,int>::iterator it = matches_forward.begin();
       it != matches_forward.end(); it++) {
    int mf = it->first;
    if (matches_reverse.find(it->second)->second == mf) {
      matches.insert( std::pair<int, int>(it->first, it->second) );
    }
  }

  Eigen::Matrix< double, 6, 1> pose;
  pose << 0, 0, 0, 0, 0, 0;
  if (matches.size() < 4) {
    return _Cart2T<double>(pose);
  }

  int n = matches.size();
  cv::Mat X1(3, n, CV_32F);
  cv::Mat X2(3, n, CV_32F);
  int count = 0;
  std::vector< cv::Mat > hs;
  std::vector< cv::Mat > oks;
  std::vector< int > scores;

  for (std::map<int, int>::iterator it = matches.begin(); it != matches.end(); it++) {
    X1.at<float>(0, count) = fds1[it->first].x;
    X1.at<float>(1, count) = fds1[it->first].y;
    X1.at<float>(2, count) = 1;
    X2.at<float>(0, count) = fds2[it->second].x;
    X2.at<float>(1, count) = fds2[it->second].y;
    X2.at<float>(2, count) = 1;
    count++;
  }

  srand(time(NULL));
  int id = 0;
    for (int t = 0; t < 100; t++){
      std::vector< int > l = random_vals(n, 4);
      cv::Mat A;
      for (int c = 0; c < 4; c++){
        A.push_back(kron(X1.col(l[c]).t(), hat(X2.col(l[c]).t())));
      }
      cv::Mat w, u, v;
      cv::SVD::compute(A, w, u, v);

      v = v.t();
      if (cv::determinant(v) < 0) {
        v = -v;
      }
      cv::Mat V(3, 3, CV_32F);
      for (int ii = 0; ii < 9; ii++) {
        V.at<float>(ii % 3, ii / 3) = v.at<float>(ii, 8);
      }
      hs.push_back(V);
      cv::Mat X2_;
      X2_ = V*X1;

      cv::Mat du = (X2_.row(0) / X2_.row(2)) - (X2.row(0) / X2.row(2));
      cv::Mat dv = (X2_.row(1) / X2_.row(2)) - (X2.row(1) / X2.row(2));
      cv::Mat ok = (du.mul(du) + dv.mul(dv)) < 6*6;
      ok = ok / cv::max(ok, 1);
      double score = cv::sum(ok)[0];
      oks.push_back(ok);
      scores.push_back(score);
    }

    int max_c = 0;
    for (int count = 0; count < scores.size(); count++) {
      if (scores[count] > max_c) {
        max_c = scores[count];
        id = count;
      }
    }
//  } else {
//    cv::Mat ok(1, matches.size(), CV_32F);
//    for (int m = 0; m < matches.size(); m++) {
//      ok.data[m] = 1;
//    }
//    oks.push_back(ok);
//    id = 0;
//  }

  cv::Mat ok = oks[id];

//  std::cout<< ok << std::endl;

  ceres::Solver::Options options;
  options.trust_region_strategy_type = ceres::DOGLEG;
  options.num_threads = 1;
  options.max_num_iterations = 50;
  options.minimizer_progress_to_stdout = false;

  ceres::Problem problem;
  count = 0;

  cv::Mat img1, img2;
  cv::cvtColor(image1, img1, CV_GRAY2BGR);
  cv::cvtColor(image2, img2, CV_GRAY2BGR);

  std::vector< cv::Scalar > colors;
  for (int ii = 0; ii < matches.size(); ii++) {
    colors.push_back(cv::Scalar(rand() % 255, rand() % 255, rand() % 255));
  }

  for(std::map<int, int>::iterator it = matches.begin(); it != matches.end(); it++) {
    if (ok.data[count] != 0) {
      Eigen::Vector2d x1, x2;
      x1 << fds1[it->first].x, fds1[it->first].y;
      x2 << fds2[it->second].x, fds2[it->second].y;
      double d = depth.at<float>(x2(1), x2(0));  // X2 should be the synthetic view
      float dist = distance(fds1[it->first].d, fds2[it->second].d);
      if (d != 0) {
        ceres::CostFunction* optimal_cost
            = new ceres::AutoDiffCostFunction<OptimalCostFunctor, 2, 6> (
              new OptimalCostFunctor( x1, x2, K, d)
              );
        problem.AddResidualBlock( optimal_cost, NULL, pose.data());
        cv::circle(img1, cv::Point(fds1[it->first].x, fds1[it->first].y), 3, colors[count]);
        cv::circle(img2, cv::Point(fds2[it->second].x, fds2[it->second].y), 3, colors[count]);
      }
    }
    count++;
  }

//  cv::imshow("image1", img1);
//  cv::imshow("image2", img2);

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  vl_sift_delete(sf);

  return _Cart2T<double>(pose).inverse();
}


cv::Mat compute_homography_( cv::Mat image1, cv::Mat image2 )
{
  //  srand(time(NULL));

  int h = image1.rows;
  int w = image1.cols;

  VlSiftFilt* sf = vl_sift_new(w, h, 5, 3, 0);

  std::vector< fd > fds1, fds2;
  process_image(sf, image1, fds1);
  process_image(sf, image2, fds2);

  std::map<int, int> matches;
  matches = find_matches_(fds1, fds2);

  cv::Mat toRet = H(matches, fds1, fds2);

  vl_sift_delete(sf);

  return toRet;
}
