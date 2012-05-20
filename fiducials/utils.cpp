/* This file is part of the fiducials Project.
 * https://github.com/stevenlovegrove/fiducials
 *
 * Copyright (c) 2011 Steven Lovegrove
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

#include "utils.h"

#include "assert.h"
#include <Eigen/Dense>

using namespace Eigen;

Eigen::Matrix3d EstimateH_ba(
  const std::vector<Eigen::Vector2d >& a,
  const std::vector<Eigen::Vector2d >& b
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
    Eigen::JacobiSVD(M, ComputeFullV).matrixV().transpose();

  // return last row of svd.get_VT(), reshaped in to 3x3
  Matrix3d H;
  H.block<1,3>(0,0) = Vt.block<1,3>(8,0);
  H.block<1,3>(1,0) = Vt.block<1,3>(8,3);
  H.block<1,3>(2,0) = Vt.block<1,3>(8,6);

  H /= H(2,2);

  return H;
}
