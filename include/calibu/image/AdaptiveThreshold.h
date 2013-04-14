/* This file is part of the calibu Project.
 * https://github.com/stevenlovegrove/calibu
 *
 * Copyright (c) 2011 Steven Lovegrove
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

#include <algorithm>

namespace calibu {

template<typename TI,typename TintI,typename Tout>
void AdaptiveThreshold( int w, int h, const TI* I, const TintI* intI, Tout* out, float threshold, int rad, Tout pass, Tout fail )
{
    // Adaptive Thresholding Using the Integral Image
    // Derek Bradley, Gerhard Roth
    
    for ( int j=0; j<h; ++j )
    {
        const int y1 = std::max(1,j-rad);
        const int y2 = std::min(h-1,j+rad);
        
        for( int i=0; i<w; ++i )
        {
            const int x1 = std::max(1,i-rad);
            const int x2 = std::min(w-1,i+rad);
            const int count = (x2-x1)*(y2-y1);
            const TintI* intIy2 = intI + y2*w;
            const TintI* intIy1m1 = intI + (y1-1)*w;
            const TintI sum = intIy2[x2] - intIy1m1[x2] - intIy2[x1-1] + intIy1m1[x1-1];
            unsigned id = j*w+i;
            out[id] = (I[id]*count < threshold*sum) ? pass : fail;
        }
    }
}


template<typename TI,typename TintI,typename Tout>
void AdaptiveThreshold( int w, int h, const TI* I, const TintI* intI, Tout* out, float threshold, int rad, int min_diff, Tout pass, Tout fail )
{
    // Adaptive Thresholding Using the Integral Image
    // Derek Bradley, Gerhard Roth
    
    // With min diff trick to make it less sensitive in homogeneous regions:
    // http://homepages.inf.ed.ac.uk/rbf/HIPR2/adpthrsh.htm
    
    for ( int j=0; j<h; ++j )
    {
        const int y1 = std::max(1,j-rad);
        const int y2 = std::min(h-1,j+rad);
        
        for( int i=0; i<w; ++i )
        {
            const int x1 = std::max(1,i-rad);
            const int x2 = std::min(w-1,i+rad);
            const int count = (x2-x1)*(y2-y1);
            const TintI* intIy2 = intI + y2*w;
            const TintI* intIy1m1 = intI + (y1-1)*w;
            const TintI sum = intIy2[x2] - intIy1m1[x2] - intIy2[x1-1] + intIy1m1[x1-1];
            const float avg = sum/count;
            unsigned id = j*w+i;
            out[id] = (I[id] < threshold*(avg-min_diff)) ? pass : fail;
        }
    }
}

}
