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

#pragma once

#include <algorithm>

namespace calibu {

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
        }
    }
}

}
