#include <opencv2/opencv.hpp>

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

void pfromh( void )
// Pose estimation from homography: http://vision.ucla.edu//MASKS/MASKS-ch5.pdf
{
  cv::Mat H(3, 3, CV_32F);
  H.at<float>(0, 0) = 5.404;
  H.at<float>(0, 1) = 0;
  H.at<float>(0, 2) = 4.436;
  H.at<float>(1, 0) = 0;
  H.at<float>(1, 1) = 4;
  H.at<float>(1, 2) = 0;
  H.at<float>(2, 0) = -1.236;
  H.at<float>(2, 1) = 0;
  H.at<float>(2, 2) = 3.804;

  cv::Mat R1, R2, R3, R4;
  cv::Mat T1, T2, T3, T4;
  cv::Mat U, S, Vt;
  cv::SVD::compute(H, S, U, Vt);

  H = H / S.at<float>(0, 1);

  cv::Mat A = H.t()*H;
  cv::SVD::compute(A, S, U, Vt);

  float s1, s2, s3;
  if (cv::determinant(U) == -1) {
    U = U*-1;
  }

  s1 = S.at<float>(0, 0);
  s2 = S.at<float>(0, 1);
  s3 = S.at<float>(0, 2);

  cv::Mat v1, v2, v3;
  v1 = U.col(0);
  v2 = U.col(1);
  v3 = U.col(2);

  cv::Mat u1, u2;
  u1 = (sqrt(1 - s3)*v1 + sqrt(s1 - 1)*v3) / (sqrt(s1 - s3));
  u2 = (sqrt(1 - s3)*v1 - sqrt(s1 - 1)*v3) / (sqrt(s1 - s3));

  cv::Mat U1, U2;
  cv::hconcat(v2, u1, U1);
  cv::hconcat(U1, hat(v2.t())*u1, U1);

  cv::hconcat(v2, u2, U2);
  cv::hconcat(U2, hat(v2.t())*u2, U2);

  cv::Mat W1, W2;
  cv::hconcat(H*v2, H*u1, W1);
  cv::hconcat(W1, hat((H*v2).t())*H*u1, W1);

  cv::hconcat(H*v2, H*u2, W2);
  cv::hconcat(W2, hat((H*v2).t())*H*u2, W2);

  R1 = W1*U1.t();
  cv::Mat N1 = hat(v2.t())*u1;
  T1 = (H - R1)*N1;

  R2 = W2*U2.t();
  cv::Mat N2 = hat(v2.t())*u2;
  T2 = (H - R2)*N2;

  R3 = R1;
  T3 = -T1;
  R4 = R2;
  T4 = -T2;

  std::cout << T1 << std::endl;

  std::cout << T2 << std::endl;

  std::cout << T3 << std::endl;

  std::cout << T4 << std::endl;

}
