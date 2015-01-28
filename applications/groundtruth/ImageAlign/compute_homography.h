#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <time.h>
#include <ceres/ceres.h>
#include <Eigen/Eigen>

cv::Mat compute_homography_( cv::Mat image1, cv::Mat image2 );

Eigen::Matrix4d estimate_pose_( cv::Mat image1, cv::Mat image2,
                                cv::Mat depth, Eigen::Matrix3d K );
