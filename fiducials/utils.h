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

#ifndef UTILS_H
#define UTILS_H

#include <TooN/TooN.h>
#include <TooN/se3.h>

TooN::Matrix<3,3> EstimateH_ba(
  const std::vector<TooN::Vector<2> >& a,
  const std::vector<TooN::Vector<2> >& b
);

inline TooN::SE3<> FromMatrix(TooN::Matrix<4,4> T_wa)
{
  const TooN::SO3<> R(T_wa.slice<0,0,3,3>());
  return TooN::SE3<>(R, T_wa.T()[3].slice<0,3>() );
}

template<typename P>
inline TooN::Matrix<4,4,P> T_4x4(const TooN::SE3<P>& T)
{
  TooN::Matrix<4,4,P> ret = TooN::Identity;
  ret.template slice<0,0,3,3>() = T.get_rotation().get_matrix();
  ret.T()[3].template slice<0,3>() = T.get_translation();
  return ret;
}

inline TooN::Matrix<3,3> SkewSym( const TooN::Vector<3>& A)
{
  return TooN::Data(
    0, -A[2], A[1],
    A[2], 0, -A[0],
    -A[1], A[0], 0
  );
}

inline TooN::Matrix<4,4> SymmetryTransform( TooN::Vector<4> N )
{
  // Compute Symmetry transformation S in ss(3) induced by plane N
  // "The top-left 3 × 3 sub-matrix of any element in ss(3) is always a House-holder matrix"
  // therefore S in ss(3) is involutionary: S^{-1} = S
  // T = S1.S2 where S1,S2 in ss(3) and T in SE(3)

  TooN::Matrix<4,4> S = TooN::Identity;
  const TooN::Vector<3> n = N.slice<0,3>();
  const double d = -N[3];
  S.slice<0,0,3,3>() = ((TooN::Matrix<3,3>)TooN::Identity) - 2 * n.as_col() * n.as_row();
  S.slice<0,3,3,1>() = 2 * d * n.as_col();
  return S;
}

#endif // UTILS_H
