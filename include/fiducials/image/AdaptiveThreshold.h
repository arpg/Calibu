/* This file is part of the fiducials Project.
 * https://github.com/stevenlovegrove/fiducials
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

namespace fiducials {

template<typename TI,typename TintI,typename Tout>
void AdaptiveThreshold( int w, int h, const TI* I, const TintI* intI, Tout* out, float threshold, int s, Tout pass, Tout fail )
{
  // Adaptive Thresholding Using the Integral Image
  // Derek Bradley, Gerhard Roth

  for ( int j=0; j<h; ++j )
  {
    const int y1 = std::max(1,j-s);
    const int y2 = std::min(h-1,j+s);

    for( int i=0; i<w; ++i )
    {
      const int x1 = std::max(1,i-s);
      const int x2 = std::min(w-1,i+s);
      const int count = (x2-x1)*(y2-y1);
      const TintI* intIy2 = intI + y2*w;
      const TintI* intIy1m1 = intI + (y1-1)*w;
      const TintI sum = intIy2[x2] - intIy1m1[x2] - intIy2[x1-1] + intIy1m1[x1-1];
      unsigned id = j*w+i;
      out[id] = (I[id]*count <= sum*threshold) ? pass : fail;
//      const TintI sum = intI[y2][x2] - intI[y1-1][x2] - intI[y2][x1-1] + intI[y1-1][x1-1];
//      out[j][i] = (I[j][i]*count <= sum*threshold) ? pass : fail;
    }
  }
}

}
