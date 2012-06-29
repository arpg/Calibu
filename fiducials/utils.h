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

#include <Eigen/Dense>
#include <sophus/se3.h>

#ifdef HAVE_TOON
#include <TooN/se3.h>

template<typename T, unsigned s>
TooN::Vector<s,T> toTooN(const Eigen::Matrix<T,s,1>& v)
{
    TooN::Vector<s,T> ret;
    for(int i=0; i<s; ++i)
        ret[i] = v(i);
    return ret;
}

template<typename T, unsigned s>
Eigen::Matrix<T,s,1> toEigen(const TooN::Vector<s,T>& v)
{
    Eigen::Matrix<T,s,1> ret;
    for(int i=0; i<s; ++i)
        ret(i) = v[i];
    return ret;
}

inline TooN::SE3<double> toTooN(const Sophus::SE3& T)
{
    TooN::Vector<6> se3 = toTooN<double,6>(T.log());
    return TooN::SE3<double>(se3);
}

inline Sophus::SE3 toEigen(const TooN::SE3<double>& T)
{
    Sophus::Vector6d se3 = toEigen<double,6>(T.ln());
    return Sophus::SE3::exp(se3);
}

#endif // HAVE_TOON

template<typename Derived>
bool is_nan(const Eigen::MatrixBase<Derived>& x)
{
  return !(x.array() == x.array()).all();
}

template<typename Derived>
bool is_finite(const Eigen::MatrixBase<Derived>& x)
{
  return !is_nan( (x.array() - x.array()).matrix() );
}

Eigen::Matrix3d EstimateH_ba(
  const std::vector<Eigen::Vector2d >& a,
  const std::vector<Eigen::Vector2d >& b
);

//inline Sophus::SE3 FromMatrix(Eigen::Matrix4d T_wa)
//{
//  const Sophus::SO3 R(T_wa.slice<0,0,3,3>());
//  return Sophus::SE3(R, T_wa.T()[3].slice<0,3>() );
//}

//template<typename P>
//inline Eigen::Matrix<P,4,4> T_4x4(const Sophus::SE3<P>& T)
//{
//  Eigen::Matrix<P,4,4> ret = Eigen::Identity;
//  ret.template slice<0,0,3,3>() = T.get_rotation().get_matrix();
//  ret.T()[3].template slice<0,3>() = T.get_translation();
//  return ret;
//}

inline Eigen::Matrix3d SkewSym( const Eigen::Vector3d& A)
{
  Eigen::Matrix3d R;
  R <<
    0, -A[2], A[1],
    A[2], 0, -A[0],
    -A[1], A[0], 0;
  return R;
}

inline Eigen::Matrix4d SymmetryTransform( const Eigen::Vector4d& N )
{
  // Compute Symmetry transformation S in ss(3) induced by plane N
  // "The top-left 3 × 3 sub-matrix of any element in ss(3) is always a House-holder matrix"
  // therefore S in ss(3) is involutionary: S^{-1} = S
  // T = S1.S2 where S1,S2 in ss(3) and T in SE(3)

    Eigen::Matrix4d S = Eigen::Matrix4d::Identity();
  const Eigen::Vector3d n = N.head<3>();
  const double d = -N[3];
  S.block<3,3>(0,0) = (Eigen::Matrix3d::Identity()) - 2 * n * n.transpose();
  S.block<3,1>(0,3) = 2 * d * n;
  return S;
}

#endif // UTILS_H
