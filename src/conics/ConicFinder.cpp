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

#include <calibu/conics/ConicFinder.h>
#include <calibu/conics/FindConics.h>
#include <calibu/image/ImageProcessing.h>

namespace calibu {

ConicFinder::ConicFinder()
{
}

void ConicFinder::Find(const ImageProcessing& imgs)
{
    candidates.clear();
    conics.clear();
    
    // Find candidate regions for conics
    FindCandidateConicsFromLabels(
        imgs.Width(), imgs.Height(), imgs.Labels(), candidates,
        params.conic_min_area, params.conic_max_area,
        params.conic_min_density,
        params.conic_min_aspect
    );
    
    // Find conic parameters
    FindConics(imgs.Width(), imgs.Height(), candidates, imgs.ImgDeriv(), conics );
}

}
