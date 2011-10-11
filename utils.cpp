#include "utils.h"

#include "assert.h"
#include <TooN/SVD.h>

using namespace TooN;

TooN::Matrix<3,3> EstimateH_ba(
  const std::vector<TooN::Vector<2> >& a,
  const std::vector<TooN::Vector<2> >& b
)
{
  assert(a.size() == b.size());

  // based on estimatehomography.m
  // George Vogiatzis and Carlos Hernández
  // http://george-vogiatzis.org/calib/

  Matrix<> M(a.size()*2,9);

  for( unsigned int i=0; i< a.size(); ++i )
  {
    const double u1 = a[i][0];
    const double v1 = a[i][1];
    const double u2 = b[i][0];
    const double v2 = b[i][1];

    M.slice(i*2,0,2,9) = (Matrix<2,9>)Data(
      u1, v1, 1, 0, 0, 0, -u1 * u2, -v1 * u2, -u2,
      0, 0, 0, u1, v1, 1, -u1 * v2, -v1 * v2, -v2
    );
  }

  SVD<> svd(M);
  const Matrix<9,9> Vt = svd.get_VT();

  // return last row of svd.get_VT(), reshaped in to 3x3
  Matrix<3,3> H;
  H.slice<0,0,1,3>() = Vt.slice(8,0,1,3);
  H.slice<1,0,1,3>() = Vt.slice(8,3,1,3);
  H.slice<2,0,1,3>() = Vt.slice(8,6,1,3);

  H /= H[2][2];

  return H;
}
