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

#include <vector>
#include <Eigen/Eigen>

#include <calibu/Platform.h>
#include <calibu/image/ImageProcessing.h>
#include <calibu/conics/Conic.h>

namespace calibu {

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

CALIBU_EXPORT
class ConicFinder
{
public:
    ConicFinder();
    void Find(const ImageProcessing& imgs);

  inline const std::vector<Conic, Eigen::aligned_allocator<Conic> >&
  Conics() const {
        return conics;
    }

    ParamsConicFinder& Params() {
        return params;
    }

protected:
    // Output of this class
  std::vector<PixelClass> candidates;
  std::vector<Conic, Eigen::aligned_allocator<Conic> > conics;

  ParamsConicFinder params;
};

}
