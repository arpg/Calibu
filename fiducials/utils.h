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
