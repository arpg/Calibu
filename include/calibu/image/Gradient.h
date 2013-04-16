/* 
   This file is part of the Calibu Project.
   https://robotics.gwu.edu/git/calibu

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

// Influenced by libCVD gradient method
template<typename TI, typename TD>
void gradient(const int w, const int h, const TI* I, TD* grad)
{
    const TI* pI = I + w + 1;
    const TI* pEnd = I + w*h - w - 1;
    TD* pOut = grad + w + 1;
    
    while (pI != pEnd) {
        (*pOut)[0] = *(pI+1) - *(pI-1);
        (*pOut)[1] = *(pI+w) - *(pI-w);
        pI++;
        pOut++;
    }
}

}
