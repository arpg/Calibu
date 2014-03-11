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

#include <Eigen/Dense>

#include <calibu/Platform.h>
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

CALIBU_EXPORT
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
        std::vector<Conic, Eigen::aligned_allocator<Conic> >& conics
        );

}
