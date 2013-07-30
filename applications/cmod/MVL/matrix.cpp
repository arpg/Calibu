/**
 *  @file matrix.c
 *
 *  Linear algebra routines.  The goal of this humble library is to provide
 *  fast routines for 3,4 and 6 dimension linear algebra systems.  These are
 *  the dimensions most often encountered, so we should focus on them.  For
 *  larget systems, there is a mix of hand-rolled linear algebra routines and
 *  convince wrappers around LAPACK and BLAS (if it's installed).   Often, this
 *  code does its work in-place, so it should be fast.
 *
 *  $Id: matrix.c 386 2008-03-03 02:10:35Z gsibley $
 */

#include "matrix.h"

#include <stdio.h>
#include <math.h>
#include <strings.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>
#include <float.h>

/*****************************************************************************/
#define swap_double(a,b) {    \
        double tmp;        \
        tmp = b;           \
        b = a;             \
        a = tmp;           \
}
/* inplace transpose of 3x3 matrix */
double* transposem_inplace_3d( double *M )
{
    swap_double( M[1], M[3] );
    swap_double( M[2], M[6] );
    swap_double( M[5], M[7] );

    return M;
}
#undef swap_double

/*****************************************************************************/
#define swap_float(a,b) {  \
        float tmp;         \
        tmp = b;           \
        b = a;             \
        a = tmp;           \
}
/* inplace transpose of 3x3 matrix */
float* transposem_inplace_3f( float *M )
{
    swap_float( M[1], M[3] );
    swap_float( M[2], M[6] );
    swap_float( M[5], M[7] );

    return M;
}
#undef swap_float

/****************************************************************************/
double* transposem_3d( const double *M, double* Res )
{
    if( Res != M ) { // avoid overlap in memory
        memcpy( Res, M, sizeof(double)*9 );
    }
    transposem_inplace_3d( Res );

    return Res;
}

/****************************************************************************/
float* transposem_3f( const float *M, float* Res )
{
    memcpy( Res, M, sizeof(float)*9 );
    transposem_inplace_3f( Res );

    return Res;
}

/****************************************************************************/
double* transposem_4d(
        const double *M, /**< Input: */
        double* Res      /**< Output: */
        )
{
    Res[0] = M[0];
    Res[1] = M[4];
    Res[2] = M[8];
    Res[3] = M[12];

    Res[4] = M[1];
    Res[5] = M[5];
    Res[6] = M[9];
    Res[7] = M[13];

    Res[8] = M[2];
    Res[9] = M[6];
    Res[10] = M[10];
    Res[11] = M[14];

    Res[12] = M[3];
    Res[13] = M[7];
    Res[14] = M[11];
    Res[15] = M[15];

    return Res;
}

/****************************************************************************/
double random_1d( void )
{
    return (double)random()/(double)RAND_MAX;
}

/*****************************************************************************/
double *normalize_nd( const double *v, int n, double *out )
{
    double norm = 0;
    int i;

    for( i = 0; i < n; i++ ){
        norm += v[i];
    }

    for( i = 0; i < n; i++ ){
        out[i] = v[i]/norm;
    }

    return out;
}

/*****************************************************************************/
/* vector length */
float magv_3f( const float *v )
{
    return sqrt( (double)( v[0]*v[0] + v[1]*v[1] + v[2]*v[2]));
}

/*****************************************************************************/
/* vector length */
double magv_3d( const double *v )
{
    return sqrt( (double)( v[0]*v[0] + v[1]*v[1] + v[2]*v[2]));
}

/*****************************************************************************/
float l2normv_4f( const float *v)
{
    return sqrt( (float)( v[0]*v[0] + v[1]*v[1] + v[2]*v[2] + v[3]*v[3]));
}

/*****************************************************************************/
float normv_3f( const float *v)
{
    return magv_3f( v );
}

/*****************************************************************************/
double  l2normv_4d( const double *v )
{
    return sqrt( (double)( v[0]*v[0] + v[1]*v[1] + v[2]*v[2] + v[3]*v[3]));
}

/*****************************************************************************/
double normv_3d( const double *v )
{
    return magv_3d( v);
}

/*****************************************************************************/
/* euclidean distance. same as norm */
float edistv_3f( const float *a, const float *b)
{
    return sqrt( (a[0]-b[0])*(a[0]-b[0])
            + (a[1]-b[1])*(a[1]-b[1]) + (a[2]-b[2])*(a[2]-b[2]));
}

/*****************************************************************************/
/* euclidean distance. same as norm */
double edistv_3d( const double *a, const double *b)
{
    return sqrt( (a[0]-b[0])*(a[0]-b[0])
            + (a[1]-b[1])*(a[1]-b[1]) + (a[2]-b[2])*(a[2]-b[2]));
}

/*****************************************************************************/
/* dot product */
float dot_3f( const float *a, const float *b)
{
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

/*****************************************************************************/
/* dot product */
double dot_3d( const double *a, const double *b)
{
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

/*****************************************************************************/
/* vetcor addition */
float *addvv_3f( const float a, const float *x, const float *y, float *res)
{

    res[0] = a*x[0] + y[0];
    res[1] = a*x[1] + y[1];
    res[2] = a*x[2] + y[2];

    return res;
}

#if 0
TODO
/*****************************************************************************/
/** Fill the diagonal of A with the vector d. */
double *get_diag_nd(
        const double *A,    /**< Input: start address of matrix A */
        const int Arows,    /**< Input: number of rows in A */
        const int Acols,    /**< Input: number of cols in A */
        const int Ainc,     /**< Input: increment between starting rows of A */
        double *d,          /**< Output: */
        const double *dinc  /**< Input: increment between rows of d. */
        )
{



}

/*****************************************************************************/
/** Extract the diagonal of A into the vector d.*/
double *set_diag_nd(
        const double *d,    /**< Input: */
        const double *dinc, /**< Input: increment between rows of d. */
        double *A,          /**< Output: start address of matrix A */
        const int Arows,    /**< Input: number of rows in A */
        const int Acols,    /**< Input: number of cols in A */
        const int Ainc      /**< Input: increment between starting rows of A */
        )
{



}
#endif


/*****************************************************************************/
double* addmm_3d( const double *A, const double *B, double *Res)
{
    Res[0] = A[0] + B[0];
    Res[1] = A[1] + B[1];
    Res[2] = A[2] + B[2];

    Res[3] = A[3] + B[3];
    Res[4] = A[4] + B[4];
    Res[5] = A[5] + B[5];

    Res[6] = A[6] + B[6];
    Res[7] = A[7] + B[7];
    Res[8] = A[8] + B[8];
    return Res;
}

/*****************************************************************************/
/*
 * C = A + B
 */
double *addmm_nd( double *A, const int m, const int n, const int ainc,
        double *B, const int binc, double *C, const int cinc )
{
    int irow, jcol;

    for( irow = 0; irow < m; irow++ ){
        for( jcol = 0; jcol < n; jcol++ ){
            C[ irow*cinc + jcol ]
                = A[ irow*ainc + jcol ] + B[ irow*binc + jcol ];
        }
    }

    return C;
}


/*****************************************************************************/
/*
 * C = A - B
 */
double *submm_nd( double *A, const int m, const int n, const int ainc,
        double *B, const int binc, double *C, const int cinc )
{
    int irow, jcol;

    for( irow = 0; irow < m; irow++ ){
        for( jcol = 0; jcol < n; jcol++ ){
            C[ irow*cinc + jcol ]
                = A[ irow*ainc + jcol ] - B[ irow*binc + jcol ];
        }
    }

    return C;
}

/*****************************************************************************/
/** Zero out the matrix A, A = 0. */
double *zero_nd(
        double *A,          /**< Output: */
        const int m,        /**< Input: */
        const int n,        /**< Input: */
        const int inc       /**< Input: */
        )
{
    int irow;

    for( irow = 0; irow < m; irow++ ){
        memset( &A[ irow*inc ], 0, sizeof(double)*n );
    }

    return A;
}

/*****************************************************************************/
/** Scalar times a matrix, B = alpha*A. */
double *multsm_nd(
        const double alpha, /**< Input: */
        const double *A,    /**< Input: */
        const int m,        /**< Input: */
        const int n,        /**< Input: */
        const int ainc,     /**< Input: */
        double *B,          /**< Output: */
        const int binc      /**< Output: */
        )
{
    int irow, jcol;

    for( irow = 0; irow < m; irow++ ){
        for( jcol = 0; jcol < n; jcol++ ){
            B[ irow*binc + jcol ] = alpha*A[ irow*ainc + jcol ];
        }
    }

    return B;
}

/****************************************************************************/
double* addmmd_inplace( const double *A, const int m, const int n, double *B )
{
    int i;
    for( i = 0; i < m*n; i++ ){
        B[i] = A[i]+B[i];
    }
    return B;
}

/****************************************************************************/
double* submm_3d( const double *A, const double *B, double *Res)
{
    Res[0] = A[0] - B[0];
    Res[1] = A[1] - B[1];
    Res[2] = A[2] - B[2];

    Res[3] = A[3] - B[3];
    Res[4] = A[4] - B[4];
    Res[5] = A[5] - B[5];

    Res[6] = A[6] - B[6];
    Res[7] = A[7] - B[7];
    Res[8] = A[8] - B[8];
    return Res;
}

/*****************************************************************************/
/* vetcor addition */
double *addvv_3d( const double a, const double *x, const double *y, double *res)
{

    res[0] = a*x[0] + y[0];
    res[1] = a*x[1] + y[1];
    res[2] = a*x[2] + y[2];

    return res;
}

/*****************************************************************************/
/* vector subtraction */
float *subvv_3f( const float a, const float *x, const float *y, float *res)
{

    res[0] = a*x[0] - y[0];
    res[1] = a*x[1] - y[1];
    res[2] = a*x[2] - y[2];

    return res;
}

/*****************************************************************************/
/* vector subtraction */
double *subvv_3d( const double a, const double *x, const double *y, double *res)
{

    res[0] = a*x[0] - y[0];
    res[1] = a*x[1] - y[1];
    res[2] = a*x[2] - y[2];

    return res;
}

/*****************************************************************************/
/* vector normalize */
float* normalizev_3f( const float *x, float *nx)
{

    float len = sqrt(x[0]*x[0] + x[1]*x[1] + x[2]*x[2]);

    nx[0] = x[0] / len;
    nx[1] = x[1] / len;
    nx[2] = x[2] / len;

    return nx;
}

/*****************************************************************************/
/* vector normalize */
double* normalizev_3d( const double *x, double *nx)
{

    double len = sqrt(x[0]*x[0] + x[1]*x[1] + x[2]*x[2]);

    nx[0] = x[0] / len;
    nx[1] = x[1] / len;
    nx[2] = x[2] / len;

    return nx;
}

/*****************************************************************************/
float* cross_3f( const float *a, const float *b, float *res)
{
  float local[3];
    local[0] = a[1]*b[2] - a[2]*b[1];
    local[1] = a[2]*b[0] - a[0]*b[2];
    local[2] = a[0]*b[1] - a[1]*b[0];

    memcpy(res, local, 3*sizeof(float));
    return res;
}

/*****************************************************************************/
double* cross_3d( const double *a, const double *b, double *res)
{
  double local[3];
    local[0] = a[1]*b[2] - a[2]*b[1];
    local[1] = a[2]*b[0] - a[0]*b[2];
    local[2] = a[0]*b[1] - a[1]*b[0];

    memcpy(res, local, 3*sizeof(double));
    return res;
}

/*****************************************************************************/
/* c := a + b */
double *addvv_nd( const double *a, const int n, const double *b, double *c )
{
    int i;
    for( i = 0; i < n; i++ ){
        c[i] = a[i] + b[i];
    }

    return c;
}

/*****************************************************************************/
/* c := a - b */
double *subvv_nd( const double *a, const int n, const double *b, double *c )
{
    int i;
    for( i = 0; i < n; i++ ){
        c[i] = a[i] - b[i];
    }

    return c;
}


/*****************************************************************************/
/* given vector a and vector b, compute vector c = diag(a)*b */
double* diagmultvv_nd( const double *a, const int n, const double *b, double *c)
{
    int i;

    for( i = 0; i < n; i++){
        c[i] = a[i]*b[i];
    }
    return c;
}

/*****************************************************************************/
/* divide vector by vector element by element.  */
double *diagdivvv_nd( const double *a, const int n, const double *b, double *c )
{
    int i;

    for( i = 0; i < n; i++ ){
        c[i] = a[i] / b[i];
    }

    return c;
}

/*****************************************************************************/
float *diagdivvv_nf( const float *a, const int n, const float *b, float *c )
{
    int i;

    for( i = 0; i < n; i++ ){
        c[i] = a[i] / b[i];
    }

    return c;
}

/*****************************************************************************/
float* addvv_nf( float const  *a, const int n, const float *b, float *c )
{
    int i;
    for( i = 0; i < n; i++ ){
        c[i] = a[i] + b[i];
    }

    return c;
}

/*****************************************************************************/
float* subvv_nf( const float *a, const int n, const float *b, float *c )
{
    int i;
    for( i = 0; i < n; i++ ){
        c[i] = a[i] - b[i];
    }
    return c;
}


/*****************************************************************************/
float* multmm_3f( const float *A, const float *B, float *Res )
{
    float local[9];

    local[0] = A[0]*B[0] + A[1]*B[3] + A[2]*B[6];
    local[1] = A[0]*B[1] + A[1]*B[4] + A[2]*B[7];
    local[2] = A[0]*B[2] + A[1]*B[5] + A[2]*B[8];

    local[3] = A[3]*B[0] + A[4]*B[3] + A[5]*B[6];
    local[4] = A[3]*B[1] + A[4]*B[4] + A[5]*B[7];
    local[5] = A[3]*B[2] + A[4]*B[5] + A[5]*B[8];

    local[6] = A[6]*B[0] + A[7]*B[3] + A[8]*B[6];
    local[7] = A[6]*B[1] + A[7]*B[4] + A[8]*B[7];
    local[8] = A[6]*B[2] + A[7]*B[5] + A[8]*B[8];

    memcpy(Res, local, sizeof(float)*9);
    return Res;
}

/*****************************************************************************/
double* multmm_3d( const double *A, const double *B, double *Res )
{
    double local[9];
    local[0] = A[0]*B[0] + A[1]*B[3] + A[2]*B[6];
    local[1] = A[0]*B[1] + A[1]*B[4] + A[2]*B[7];
    local[2] = A[0]*B[2] + A[1]*B[5] + A[2]*B[8];

    local[3] = A[3]*B[0] + A[4]*B[3] + A[5]*B[6];
    local[4] = A[3]*B[1] + A[4]*B[4] + A[5]*B[7];
    local[5] = A[3]*B[2] + A[4]*B[5] + A[5]*B[8];

    local[6] = A[6]*B[0] + A[7]*B[3] + A[8]*B[6];
    local[7] = A[6]*B[1] + A[7]*B[4] + A[8]*B[7];
    local[8] = A[6]*B[2] + A[7]*B[5] + A[8]*B[8];

    memcpy(Res, local, 9*sizeof(double));
    return Res;
}

/*****************************************************************************/
float* multmv_3f( const float *A, const float *b, float *res )
{
    float local[3];

    local[0] = b[0]*A[0] + b[1]*A[1] + b[2]*A[2];
    local[1] = b[0]*A[3] + b[1]*A[4] + b[2]*A[5];
    local[2] = b[0]*A[6] + b[1]*A[7] + b[2]*A[8];

    res[0] = local[0];
    res[1] = local[1];
    res[2] = local[2];

    return res;
}

/*****************************************************************************/
float* multmtv_3f( const float *A, const float *b, float *res )
{
    float local[3];

    local[0] = b[0]*A[0] + b[1]*A[3] + b[2]*A[6];
    local[1] = b[0]*A[1] + b[1]*A[4] + b[2]*A[7];
    local[2] = b[0]*A[2] + b[1]*A[5] + b[2]*A[8];

    res[0] = local[0];
    res[1] = local[1];
    res[2] = local[2];

    return res;
}

/*****************************************************************************/
double* multmv_3d( const double *A, const double *b, double *res )
{
    double local[3];

    local[0] = b[0]*A[0] + b[1]*A[1] + b[2]*A[2];
    local[1] = b[0]*A[3] + b[1]*A[4] + b[2]*A[5];
    local[2] = b[0]*A[6] + b[1]*A[7] + b[2]*A[8];

    res[0] = local[0];
    res[1] = local[1];
    res[2] = local[2];

    return res;
}

/*****************************************************************************/
double* multmtv_3d( const double *A, const double *b, double *res )
{
    double local[3];

    local[0] = b[0]*A[0] + b[1]*A[3] + b[2]*A[6];
    local[1] = b[0]*A[1] + b[1]*A[4] + b[2]*A[7];
    local[2] = b[0]*A[2] + b[1]*A[5] + b[2]*A[8];

    res[0] = local[0];
    res[1] = local[1];
    res[2] = local[2];

    return res;
}
/*****************************************************************************/
float *multsv_3f( const float a, const float *x, float *res )
{
    res[0] = a*x[0];
    res[1] = a*x[1];
    res[2] = a*x[2];

    return res;
}

/*****************************************************************************/
double *multsv_3d( const double a, const double *x, double *res )
{
    res[0] = a*x[0];
    res[1] = a*x[1];
    res[2] = a*x[2];

    return res;
}

/*****************************************************************************/
float *multsm_4f( const float a, const float *M, float *Res )
{
    Res[0] = a * M[0];
    Res[1] = a * M[1];
    Res[2] = a * M[2];
    Res[3] = a * M[3];

    Res[4] = a * M[4];
    Res[5] = a * M[5];
    Res[6] = a * M[6];
    Res[7] = a * M[7];

    Res[8] = a * M[8];
    Res[9] = a * M[9];
    Res[10] = a * M[10];
    Res[11] = a * M[11];

    Res[12] = a * M[12];
    Res[13] = a * M[13];
    Res[14] = a * M[14];
    Res[15] = a * M[15];

    return Res;
}

/*****************************************************************************/
double *multsm_4d( const double a, const double *M, double *Res )
{
    Res[0] = a * M[0];
    Res[1] = a * M[1];
    Res[2] = a * M[2];
    Res[3] = a * M[3];

    Res[4] = a * M[4];
    Res[5] = a * M[5];
    Res[6] = a * M[6];
    Res[7] = a * M[7];

    Res[8] = a * M[8];
    Res[9] = a * M[9];
    Res[10] = a * M[10];
    Res[11] = a * M[11];

    Res[12] = a * M[12];
    Res[13] = a * M[13];
    Res[14] = a * M[14];
    Res[15] = a * M[15];

    return Res;
}

/*****************************************************************************/
float *multsm_3f( const float alpha, const float *M, float *Res )
{
    Res[0] = alpha * M[0];
    Res[1] = alpha * M[1];
    Res[2] = alpha * M[2];

    Res[3] = alpha * M[3];
    Res[4] = alpha * M[4];
    Res[5] = alpha * M[5];

    Res[6] = alpha * M[6];
    Res[7] = alpha * M[7];
    Res[8] = alpha * M[8];

    return Res;
}

/*****************************************************************************/
double *multsm_3d( const double a, const double *M, double *Res )
{
    Res[0] = a * M[0];
    Res[1] = a * M[1];
    Res[2] = a * M[2];

    Res[3] = a * M[3];
    Res[4] = a * M[4];
    Res[5] = a * M[5];

    Res[6] = a * M[6];
    Res[7] = a * M[7];
    Res[8] = a * M[8];

    return Res;
}

/*****************************************************************************/
float* multmm_4f( const float *A, const float *B, float *Res )
{
    float local[16];

    local[0] =  A[0]*B[0] + A[1]*B[4] + A[2]*B[8] + A[3]*B[12];
    local[1] =  A[0]*B[1] + A[1]*B[5] + A[2]*B[9] + A[3]*B[13];
    local[2] =  A[0]*B[2] + A[1]*B[6] + A[2]*B[10] + A[3]*B[14];
    local[3] =  A[0]*B[3] + A[1]*B[7] + A[2]*B[11] + A[3]*B[15];

    local[4] =  A[4]*B[0] + A[5]*B[4] + A[6]*B[8] + A[7]*B[12];
    local[5] =  A[4]*B[1] + A[5]*B[5] + A[6]*B[9] + A[7]*B[13];
    local[6] =  A[4]*B[2] + A[5]*B[6] + A[6]*B[10] + A[7]*B[14];
    local[7] =  A[4]*B[3] + A[5]*B[7] + A[6]*B[11] + A[7]*B[15];

    local[8] =  A[8]*B[0] + A[9]*B[4] + A[10]*B[8] + A[11]*B[12];
    local[9] =  A[8]*B[1] + A[9]*B[5] + A[10]*B[9] + A[11]*B[13];
    local[10] = A[8]*B[2] + A[9]*B[6] + A[10]*B[10] + A[11]*B[14];
    local[11] = A[8]*B[3] + A[9]*B[7] + A[10]*B[11] + A[11]*B[15];

    local[12] = A[12]*B[0] + A[13]*B[4] + A[14]*B[8] + A[15]*B[12];
    local[13] = A[12]*B[1] + A[13]*B[5] + A[14]*B[9] + A[15]*B[13];
    local[14] = A[12]*B[2] + A[13]*B[6] + A[14]*B[10] + A[15]*B[14];
    local[15] = A[12]*B[3] + A[13]*B[7] + A[14]*B[11] + A[15]*B[15];

    memcpy(Res, local, 16 * sizeof(float));

    return Res;
}

/*****************************************************************************/
double* multmm_4d( const double *A, const double *B, double *Res )
{
    double local[16];

    local[0] =  A[0]*B[0] + A[1]*B[4] + A[2]*B[8] + A[3]*B[12];
    local[1] =  A[0]*B[1] + A[1]*B[5] + A[2]*B[9] + A[3]*B[13];
    local[2] =  A[0]*B[2] + A[1]*B[6] + A[2]*B[10] + A[3]*B[14];
    local[3] =  A[0]*B[3] + A[1]*B[7] + A[2]*B[11] + A[3]*B[15];

    local[4] =  A[4]*B[0] + A[5]*B[4] + A[6]*B[8] + A[7]*B[12];
    local[5] =  A[4]*B[1] + A[5]*B[5] + A[6]*B[9] + A[7]*B[13];
    local[6] =  A[4]*B[2] + A[5]*B[6] + A[6]*B[10] + A[7]*B[14];
    local[7] =  A[4]*B[3] + A[5]*B[7] + A[6]*B[11] + A[7]*B[15];

    local[8] =  A[8]*B[0] + A[9]*B[4] + A[10]*B[8] + A[11]*B[12];
    local[9] =  A[8]*B[1] + A[9]*B[5] + A[10]*B[9] + A[11]*B[13];
    local[10] = A[8]*B[2] + A[9]*B[6] + A[10]*B[10] + A[11]*B[14];
    local[11] = A[8]*B[3] + A[9]*B[7] + A[10]*B[11] + A[11]*B[15];

    local[12] = A[12]*B[0] + A[13]*B[4] + A[14]*B[8] + A[15]*B[12];
    local[13] = A[12]*B[1] + A[13]*B[5] + A[14]*B[9] + A[15]*B[13];
    local[14] = A[12]*B[2] + A[13]*B[6] + A[14]*B[10] + A[15]*B[14];
    local[15] = A[12]*B[3] + A[13]*B[7] + A[14]*B[11] + A[15]*B[15];

    memcpy( Res, local, 16 * sizeof(double) );

    return Res;
}

/*****************************************************************************/
float* multmv_4f( const float *A, const float *b, float *res)
{
    float local[4];

    local[0] = b[0]*A[0]  + b[1]*A[1]  + b[2]*A[2]  + b[3]*A[3];
    local[1] = b[0]*A[4]  + b[1]*A[5]  + b[2]*A[6]  + b[3]*A[7];
    local[2] = b[0]*A[8]  + b[1]*A[9]  + b[2]*A[10] + b[3]*A[11];
    local[3] = b[0]*A[12] + b[1]*A[13] + b[2]*A[14] + b[3]*A[15];

    res[0] = local[0];
    res[1] = local[1];
    res[2] = local[2];
    res[3] = local[3];

    return res;
}

/*****************************************************************************/
double* multmv_4d( const double *A, const double *b, double *res)
{
    double local[4];

    local[0] = b[0]*A[0]  + b[1]*A[1]  + b[2]*A[2]  + b[3]*A[3];
    local[1] = b[0]*A[4]  + b[1]*A[5]  + b[2]*A[6]  + b[3]*A[7];
    local[2] = b[0]*A[8]  + b[1]*A[9]  + b[2]*A[10] + b[3]*A[11];
    local[3] = b[0]*A[12] + b[1]*A[13] + b[2]*A[14] + b[3]*A[15];

    res[0] = local[0];
    res[1] = local[1];
    res[2] = local[2];
    res[3] = local[3];

    return res;
}

/*****************************************************************************/
/* M = a*b' */
float* multvvt_3f( const float *a, const float *b, float *Res)
{
    Res[0] = a[0]*b[0];
    Res[1] = a[0]*b[1];
    Res[2] = a[0]*b[2];

    Res[3] = a[1]*b[0];
    Res[4] = a[1]*b[1];
    Res[5] = a[1]*b[2];

    Res[6] = a[2]*b[0];
    Res[7] = a[2]*b[1];
    Res[8] = a[2]*b[2];

    return Res;
}

/*****************************************************************************/
/* M = a*b' */
double* multvvt_3d( const double *a, const double *b, double *Res)
{

    Res[0] = a[0]*b[0];
    Res[1] = a[0]*b[1];
    Res[2] = a[0]*b[2];

    Res[3] = a[1]*b[0];
    Res[4] = a[1]*b[1];
    Res[5] = a[1]*b[2];

    Res[6] = a[2]*b[0];
    Res[7] = a[2]*b[1];
    Res[8] = a[2]*b[2];

    return Res;
}

/*****************************************************************************/
float trace_3f( const float *M)
{
    return M[0] + M[4] + M[8];
}

/*****************************************************************************/
double trace_3d( const double *M)
{
    return M[0] + M[4] + M[8];
}

/*****************************************************************************/
float det_3f( const float *M)
{
    /*
     *res  = A[0][0]*A[1][1]*A[2][2] + A[0][1]*A[1][2]*A[2][0]
     + A[0][2]*A[1][0]*A[2][1]
     - A[0][2]*A[1][1]*A[2][0] -  A[0][0]*A[1][2]*A[2][1]
     - A[0][1]*A[1][0]*A[2][2];
     */
    return  M[0]*M[4]*M[8] + M[1]*M[5]*M[6] + M[2]*M[3]*M[7]
        - M[2]*M[4]*M[6] -  M[0]*M[5]*M[7] - M[1]*M[3]*M[8];
}

/*****************************************************************************/
double det_3d( const double *M)
{
    return  M[0]*M[4]*M[8] + M[1]*M[5]*M[6] + M[2]*M[3]*M[7]
        - M[2]*M[4]*M[6] -  M[0]*M[5]*M[7] - M[1]*M[3]*M[8];
}

/*****************************************************************************/
float * copyv_3f( const float *src, float *dst)
{
    dst[0] = src[0];
    dst[1] = src[1];
    dst[2] = src[2];

    return dst;
}

/*****************************************************************************/
double* copyv_3d( const double *src, double *dst)
{
    dst[0] = src[0];
    dst[1] = src[1];
    dst[2] = src[2];

    return dst;
}

/*****************************************************************************/
float* zerov_3f( float *v)
{
    v[0] = v[1] = v[2] = 0.0;
    return v;
}

/*****************************************************************************/
double* zerov_3d( double *v)
{
    v[0] = v[1] = v[2] = 0.0;
    return v;
}

/*****************************************************************************/
float *transposem_nf( const float *m, int mrows, int ncols, float *mt )
{
    int irow, jcol;
    assert(m!=mt);

    for( irow = 0; irow < mrows; irow++ ) {
        for( jcol = 0; jcol < ncols; jcol++ ) {
            mt[ jcol*mrows + irow ] = m[ irow*ncols + jcol ];
        }
    }

    return mt;
}

/*****************************************************************************/
/** Transpose nd matrix. */
double* transposem_nd(
        const double *A,/**< Input: start address of matrix A */
        const int Arows,/**< Input: number of rows in A */
        const int Acols,/**< Input: number of cols in A */
        const int Ainc, /**< Input: increment between starting rows of A */
        const int Atinc,/**< Input: increment between starting rows of At */
        double *At      /**< Output: A transposed */
        )
{
    int irow, jcol;
    assert( A!=At );

    for( irow = 0; irow < Arows; irow++ ) {
        for( jcol = 0; jcol < Acols; jcol++ ) {
            At[ jcol*Atinc + irow ] = A[ irow*Ainc + jcol ];
        }
    }

    return At;
}

#if 0
/*****************************************************************************/
/** Transpose nd matrix.  This funciton only works with contiguous matrices. */
double* transposem_inplace_nd(
        double *A,/**< Input/Output: start address of matrix A */
        const int Arows,/**< Input: number of rows in A */
        const int Acols,/**< Input: number of cols in A */
        const int Ainc  /**< Input: increment between starting rows of A */
        )
{
    int irow, jcol;
    double swap;

    for( irow = 0; irow < Arows; irow++ ) {
        for( jcol = irow; jcol < Acols; jcol++ ) {
            swap = A[ jcol*Acols + irow ];
            A[ jcol*Arows + irow ] = A[ irow*Acols + jcol ];
            A[ irow*Acols + jcol ] = swap;
        }
    }

    return A;
}
#endif

/*****************************************************************************/
float* vdtof( const double *src, int len, int srcinc, float *dst, int dstinc )
{
    int i;
    for(i = 0; i < len; i++){
        dst[i*dstinc] = (float)src[i*srcinc];
    }

    return dst;
}

/*****************************************************************************/
double* mftod( const float *, int , int ,
               int , double *, int  )
{
    printf("write me \n");
    assert(0);
    return NULL;
}

/*****************************************************************************/
double* vftod( const float *src, int len, int srcinc, double *dst, int dstinc )
{
    int i;
    for(i = 0; i < len; i++){
        dst[i*dstinc] = (double)src[i*srcinc];
    }

    return dst;
}

/*****************************************************************************/
float* mdtof( const double *, int , int ,
        int , float *, int  )
{
    printf("write me \n");
    assert(0);
    return NULL;
}


