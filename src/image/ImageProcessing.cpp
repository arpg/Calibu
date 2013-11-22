/*
  This file is part of the Calibu Project.
  https://github.com/gwu-robotics/Calibu

  Copyright (C) 2013 George Washington University,
  Steven Lovegrove

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

  http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
*/

#include <calibu/image/ImageProcessing.h>
#include <calibu/image/Gradient.h>
#include <calibu/image/AdaptiveThreshold.h>
#include <calibu/image/IntegralImage.h>
#include <calibu/image/Label.h>

namespace calibu {

ImageProcessing::ImageProcessing(int maxWidth, int maxHeight)
    : width(maxWidth), height(maxHeight) {
  AllocateImageData(maxWidth*maxHeight);
}

ImageProcessing::~ImageProcessing() {
  DeallocateImageData();
}

void ImageProcessing::AllocateImageData(int maxPixels) {
  I.resize(maxPixels);
  intI.resize(maxPixels);
  dI.resize(maxPixels);
  lI.resize(maxPixels);
  tI.resize(maxPixels);
}

void ImageProcessing::DeallocateImageData() {}

void ImageProcessing::Process(const unsigned char* greyscale_image,
                              size_t w, size_t h, size_t pitch) {
  width = w;
  height = h;

  size_t img_size = width * height * sizeof(unsigned char);
  if (img_size > I.size()) {
    AllocateImageData(img_size);
  }

  // Copy input image
  if(pitch > width*sizeof(unsigned char) ) {
    // Copy line by line
    for(int y=0; y < height; ++y) {
      memcpy(&I[y*width], greyscale_image+y*pitch, width * sizeof(unsigned char));
    }
  }else{
    memcpy(&I[0], greyscale_image, img_size);
  }

  // Process image
  gradient<>(width, height, &I[0],  &dI[0]);
  integral_image(width, height, &I[0], &intI[0] );

  // Threshold image
  AdaptiveThreshold(
      width, height, &I[0], &intI[0], &tI[0], params.at_threshold,
      width / params.at_window_ratio, 20,
      (unsigned char)0, (unsigned char)255
                    );

  // Label image (connected components)
  labels.clear();
  Label(width, height, &tI[0], &lI[0], labels,
        params.black_on_white ? 0 : 255 );
}

}
