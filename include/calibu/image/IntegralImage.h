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

namespace calibu {

// Influenced by libCVD integral_image
template<typename TI, typename TO>
void integral_image(const int w, const int h, const TI* in, TO* out)
{
    out[0] = in[0];
    
    //Do the first row.
    for(int x=1; x < w; x++)
        out[x] =out[x-1] + in[x];
    
    //Do the first column.
    for(int y=1; y < h; y++)
        out[y*w] =out[(y-1)*w] + in[y*w];
    
    //Do the remainder of the image
    for(int y=1; y < h; y++) {
        TO sum = in[y*w];
        
        for(int x=1; x < w; x++) {
            sum += in[y*w+x];
            out[y*w+x] = sum + out[(y-1)*w+x];
        }
    }
}

}
