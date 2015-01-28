#include "compute_homography.h"
#include <time.h>
#include <ceres/ceres.h>

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
      if (dist < min1) {
        id = jj;
        min2 = min1;
        min1 = dist;
      } else if (dist < min2) {
        min2 = dist;
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

  srand(time(NULL));
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
  std::cout << H << std::endl;
  ceres::Problem problem;
  cv::Mat ok = oks[id];
  std::cout << ok << std::endl;
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

cv::Mat compute_homography_( cv::Mat image1, cv::Mat image2 )
{
  srand(time(NULL));

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
