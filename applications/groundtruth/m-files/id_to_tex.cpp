#include "mex.h"

void tex( double* px, unsigned long value )
{
  double t_reverse[36];
  double temp[36];
  int idx = 0;
  for (int count = 0; count < 9; count++) {
    unsigned long long temp = value - ((value >> 4) << 4);
    t_reverse[idx + 0] = temp & 1;
    t_reverse[idx + 1] = temp & 2;
    t_reverse[idx + 2] = temp & 4;
    t_reverse[idx + 3] = temp & 8;
    idx += 4;
    value >>= 4;
  }
  for (int count = 0; count < 36; count++){
    temp[count] = (t_reverse[35 - count] != 0);
  }

  for (int jj = 0; jj < 6; jj++) {
    for (int ii = 0; ii < 6; ii++) {
      px[ii + 1 + (jj + 1)*8] = temp[ii + 6*jj];
    }
  }
}


void mexFunction(int nlhs, mxArray *plhs[],
                 int nrhs, const mxArray *prhs[])
{
    double* out;
    unsigned long in;

    in = mxGetScalar(prhs[0]);

    plhs[0] = mxCreateDoubleMatrix(8, 8, mxREAL);
    out = mxGetPr(plhs[0]);

    tex(out, in);
    return;
}
