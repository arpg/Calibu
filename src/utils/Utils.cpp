/*
   This file is part of the Calibu Project.
   https://github.com/gwu-robotics/Calibu

   Copyright (C) 2013 George Washington University,
                      Steven Lovegrove,
                      Gabe Sibley

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

#include <calibu/utils/Utils.h>

#include "assert.h"
#include <Eigen/Dense>

using namespace Eigen;

namespace calibu {

Eigen::Matrix3d EstimateH_ba(
        const std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> >& a,
        const std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> >& b
        )
{
    assert(a.size() == b.size());

    // based on estimatehomography.m
    // George Vogiatzis and Carlos Hernández
    // http://george-vogiatzis.org/calib/

    MatrixXd M(a.size()*2,9);

    for( unsigned int i=0; i< a.size(); ++i )
    {
        const double u1 = a[i][0];
        const double v1 = a[i][1];
        const double u2 = b[i][0];
        const double v2 = b[i][1];

        M.block<2,9>(i*2,0) <<
                               u1, v1, 1, 0, 0, 0, -u1 * u2, -v1 * u2, -u2,
                0, 0, 0, u1, v1, 1, -u1 * v2, -v1 * v2, -v2;
    }

    const Matrix<double,9,9> Vt =
            Eigen::JacobiSVD<MatrixXd>(M, ComputeFullV).matrixV().transpose();

    // return last row of svd.get_VT(), reshaped in to 3x3
    Matrix3d H;
    H.block<1,3>(0,0) = Vt.block<1,3>(8,0);
    H.block<1,3>(1,0) = Vt.block<1,3>(8,3);
    H.block<1,3>(2,0) = Vt.block<1,3>(8,6);

    H /= H(2,2);

    return H;
}

}
