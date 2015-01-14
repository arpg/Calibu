#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <time.h>
#include <ceres/ceres.h>

extern "C"{
#include <vl/generic.h>
#include <vl/sift.h>
}

cv::Mat compute_homography_( cv::Mat image1, cv::Mat image2 );
