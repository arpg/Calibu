/* This file is part of the calibu Project.
 * https://github.com/stevenlovegrove/calibu
 *
 * Copyright (C) 2010  Steven Lovegrove, Richard Newcombe, Hauke Strasdat
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

#include <Eigen/Dense>

#include <calibu/image/Label.h>
#include <calibu/conics/Conic.h>

namespace calibu {

template<typename TdI>
Eigen::Matrix3d FindEllipse(
        const int w, const int /*h*/,
        const TdI* dI,
        const IRectangle& r,
        double& /*residual*/
        );

void FindCandidateConicsFromLabels(
        unsigned w, unsigned h,
        const std::vector<PixelClass>& labels,
        std::vector<PixelClass>& candidates,
        float min_area,
        float max_area,
        float min_density,
        float min_aspect
        );

template<typename TdI>
void FindConics(
        const int w, const int h,
        const std::vector<PixelClass>& candidates,
        const TdI* dI,
        std::vector<Conic>& conics
        );

}
