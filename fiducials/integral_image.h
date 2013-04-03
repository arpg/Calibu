/* This file is part of the fiducials Project.
 * https://github.com/stevenlovegrove/fiducials
 *
 * Copyright (C) 2010  Steven Lovegrove
 *                     Imperial College London
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

namespace fiducials {

// Based on libCVD integral_image
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
