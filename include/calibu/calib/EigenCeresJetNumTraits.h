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

#include <ceres/jet.h>
#include <Eigen/Core>

namespace ceres {

inline double tan  (double x) { return std::tan(x);      }
inline double atan (double x) { return std::atan(x);     }

template <typename T, int N> inline
ceres::Jet<T, N> fabs(const ceres::Jet<T, N>& f)
{
    return abs(f);
}

// tan(a+h) ~= tan(a) + 1 / cos(a)^2 h
template <typename T, int N> inline
ceres::Jet<T, N> tan(const ceres::Jet<T, N>& f)
{
    const T cosf = cos(f.a);
    Jet<T, N> g;
    g.a = tan(f.a);
    g.v = f.v / (cosf*cosf);    
    return g;
}

// atan(a) ~= atan(a) + 1 / (x^2+1) h
template <typename T, int N> inline
ceres::Jet<T, N> atan(const Jet<T, N>& f)
{
    Jet<T, N> g;
    g.a = atan(f.a);
    g.v = f.v / (f.a*f.a + 1);
    return g;
}

}
