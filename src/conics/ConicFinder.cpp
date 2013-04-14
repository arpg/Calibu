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
