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

#pragma once

#include <vector>
#include <Eigen/Eigen>

#include "label.h"
#include "conics.h"
#include "image_processing.h"

namespace fiducials {

struct ParamsConicFinder
{
    ParamsConicFinder() :
        conic_min_area(25),
        conic_max_area(4E4),
        conic_min_density(0.4),
        conic_min_aspect(0.1)
    {
        
    }

    float conic_min_area;
    float conic_max_area;
    float conic_min_density;
    float conic_min_aspect;    
};

class ConicFinder
{
public:
    ConicFinder();
    void Find(const ImageProcessing& imgs);
    
    inline const std::vector<Conic>& Conics() const {
        return conics;
    }
        
    ParamsConicFinder& Params() {
        return params;
    }
    
protected:    
    // Output of this class
    std::vector<PixelClass> candidates;
    std::vector<Conic> conics; 
    
    ParamsConicFinder params;
};

}
