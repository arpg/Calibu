#pragma once

#include <apriltags/apriltag.h>
#include <apriltags/tag36h11.h>
#include <opencv2/opencv.hpp>

class TagDetector
{
  public:
    TagDetector()
    {
      tf_ = tag36h11_create();
      td_ = april_tag_detector_create(tf_);
//      td_->debug = 1;
      td_->small_tag_refinement = 1;
      td_->nthreads = 8;
    }

    ~TagDetector()
    {
      april_tag_detector_destroy( td_ );
      tag36h11_destroy( tf_ );
    }

    void Detect( const cv::Mat& im, 
        std::vector<april_tag_detection_t>& vDetections )
    {
      image_u8 tmp;
      tmp.width = im.cols;
      tmp.height = im.rows;
      tmp.stride = im.cols;
      tmp.buf = im.data;
      zarray_t *detections = april_tag_detector_detect( td_, &tmp );

      vDetections.clear();
      for( int ii = 0; ii < zarray_size( detections ); ii++ ){
        april_tag_detection_t* p;
        zarray_get( detections, ii, &p );
        if( p->hamming <= 0 ){
          vDetections.push_back( *p );
        }
      }
    }

//  private:
    april_tag_family_t*     tf_;
    april_tag_detector_t*   td_;
};

