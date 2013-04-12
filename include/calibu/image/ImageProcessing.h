/* This file is part of the calibu Project.
 * https://github.com/stevenlovegrove/calibu
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

#pragma once

#include <Eigen/Eigen>
#include "Label.h"

namespace calibu
{

struct ParamsImageProcessing
{
    ParamsImageProcessing() :
        at_threshold(0.7),
        at_window_ratio(3),
        black_on_white(true)
    {
        
    }
    
    float at_threshold;
    int at_window_ratio;
    bool black_on_white;
};

class ImageProcessing
{
public:
    ImageProcessing(int w, int h);
    ~ImageProcessing();
    
    void Process(unsigned char* greyscale_image, size_t pitch);
    
    inline int Width()  const { return width; }
    inline int Height() const { return height; }
    
    inline const unsigned char* Img() const { return I; }    
    inline const Eigen::Vector2f* ImgDeriv() const { return dI; }
    inline const unsigned char* ImgThresh() const { return tI; }    
    inline const std::vector<PixelClass>& Labels() const { return labels; }
    
    ParamsImageProcessing& Params() { return params; }
    
protected:
    void AllocateImageData(int w, int h);
    void DeallocateImageData();
    
    int width, height;
    
    // Images owned by this class
    unsigned char* I;
    float* intI;
    Eigen::Vector2f* dI;
    short* lI;
    unsigned char* tI;
    
    std::vector<PixelClass> labels;
    ParamsImageProcessing params;
};


}
