/* This file is part of the fiducials Project.
 * https://github.com/stevenlovegrove/fiducials
 *
 * Copyright (C) 2010-2013 Steven Lovegrove
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

#include "image_processing.h"

#include "gradient.h"
#include "adaptive_threshold.h"
#include "integral_image.h"
#include "label.h"

namespace fiducials
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
    const unsigned char pass = params.black_on_white ? 0 : 255;
    const unsigned char fail = params.black_on_white ? 255 : 0;
    AdaptiveThreshold(
        width, height, I, intI, tI, params.at_threshold,
        width / params.at_window_ratio,
        pass, fail
    );    
    
    // Label image (connected components)
    labels.clear();
    Label(width,height,tI,lI,labels);    
}

}
