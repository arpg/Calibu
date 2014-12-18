#include "mex.h"
#include <Eigen/Eigen>

int color( int dx, int dy )
{
    
}

void project( double* im, int w, int h )
{
    int i = 0;
    int j = 0;
    for (i = 0; i < w; i++){
        for (j = 0; j < h; j++ ){
            im[i + j*w] = (rand() % 255) / 255.0f;
        }
    }
}

void mexFunction(int nlhs, mxArray *plhs[],
                 int nrhs, const mxArray *prhs[])
{
    double* im;
    double* _k;
    int w, h;
    
    _k = mxGetPr(prhs[0]);
    Eigen::Map< Eigen::Matrix< double, 3, 3 > > K(_k);
    
    w = mxGetScalar(prhs[1]);
    h = mxGetScalar(prhs[2]);    

    plhs[0] = mxCreateDoubleMatrix(h, w, mxREAL);    
    im = mxGetPr(plhs[0]);
    
    project(im, w, h);
    return;
}