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

#pragma once

#include <Eigen/Eigen>
#include "Label.h"

namespace calibu {

struct ParamsImageProcessing {
  ParamsImageProcessing() : at_threshold(0.7),
                            at_window_ratio(3),
                            black_on_white(true) {}
  float at_threshold;
  int at_window_ratio;
  bool black_on_white;
};

class ImageProcessing {
 public:
  ImageProcessing(int maxWidth, int maxHeight);
  ~ImageProcessing();

  void Process(unsigned char* greyscale_image, size_t w, size_t h, size_t pitch);

  inline int Width()  const { return width; }
  inline int Height() const { return height; }

  inline const unsigned char* Img() const { return &I[0]; }
  inline const Eigen::Vector2f* ImgDeriv() const { return &dI[0]; }
  inline const unsigned char* ImgThresh() const { return &tI[0]; }
  inline const std::vector<PixelClass>& Labels() const { return labels; }

  ParamsImageProcessing& Params() { return params; }

 protected:
  void AllocateImageData(int maxPixels);
  void DeallocateImageData();

  int width, height;

  // Images owned by this class
  std::vector<unsigned char> I;
  std::vector<float> intI;
  std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > dI;
  std::vector<short> lI;
  std::vector<unsigned char> tI;

  std::vector<PixelClass> labels;
  ParamsImageProcessing params;
};

}
