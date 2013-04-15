/* 
   This file is part of the Calibu Project.
   https://robotics.gwu.edu/git/calibu

   Copyright (C) 2013 George Washington University

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

namespace calibu
{

ImageProcessing::ImageProcessing(int width, int height)
{
    AllocateImageData(width, height);
}

ImageProcessing::~ImageProcessing()
{
    DeallocateImageData();
}

void ImageProcessing::AllocateImageData(int w, int h)
{
    width = w;
    height = h;
    
    const int pixels = width*height;
    
    I = new unsigned char[pixels];
    intI = new float[pixels];
    dI = new Eigen::Vector2f[pixels];
    lI = new short[pixels];
    tI = new unsigned char[pixels];
}

void ImageProcessing::DeallocateImageData()
{
    delete[] I;
    delete[] intI;
    delete[] dI;
    delete[] lI;
    delete[] tI;
}

void ImageProcessing::Process(unsigned char* greyscale_image, size_t pitch)
{
    if(pitch > width*sizeof(unsigned char) ) {
        for(int y=0; y < height; ++y) {
            memcpy(I+y*width, greyscale_image+y*pitch, width * sizeof(unsigned char) );
        }
    }else{
        memcpy(I, greyscale_image, width * height * sizeof(unsigned char) );
    }
    
    // Process image
    gradient<>(width, height, I, dI );
    integral_image(width, height, I, intI );   
    
    // Threshold image
    AdaptiveThreshold(
                width, height, I, intI, tI, params.at_threshold,
                width / params.at_window_ratio, 20,
                (unsigned char)0, (unsigned char)255
                );    
    
    // Label image (connected components)
    labels.clear();
    Label(width,height,tI,lI,labels, params.black_on_white ? 0 : 255 );
}

}
