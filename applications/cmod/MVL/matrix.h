/**
 *  @file matrix.h
 *
 *  Simple linear algebra routines header file.
 *
 *  $Id: matrix.h 386 2008-03-03 02:10:35Z gsibley $
 */

#ifndef _MATRIX_H_
#define _MATRIX_H_

#include <math.h>

//#include <mvl/matrix/slice.h>
//#include <mvl/matrix/printm.h>
//#include <mvl/matrix/resize.h>
//#include <mvl/matrix/eigen.h>
//#include <mvl/matrix/random.h>

/* code that depends on netlib */
//#include <mvl/matrix/inverse.h>
//#include <mvl/matrix/linlsq.h>

/* 3 dimesion double precision routines */


/** Distance between two 3-vectors. */
double edistv_3d(
        const double *a, /**< Input: */
        const double *b  /**< Input: */
        );

/** L2 norm of a 3-vector. */
double magv_3d(
        const double *v/**< Input: */
        );

/** L2 norm of a 3-vector. */
double normv_3d(
        const double *v /**< Input: */
        );

/** Dot product between two 3-vectors. */
double dot_3d(
        const double *a, /**< Input: */
        const double *b  /**< Input: */
        );

/** Trace of a 3x3 matrix. */
double trace_3d(
        const double *M /**< Input: */
        );

/** Determinant of a 3x3 matrix */
double det_3d(
        const double *M /**< Input: */
        );

/** Add two 3-vectors. */
double* addvv_3d(
        const double alpha, /**< Input: */
        const double *x,    /**< Input: */
        const double *y,    /**< Input: */
        double *res         /**< Output: */
        );

/** Subtract two 3-vectors. */
double* subvv_3d(
        const double alpha, /**< Input: */
        const double *x,    /**< Input: */
        const double *y,    /**< Input: */
        double *res         /**< Output: */
        );

/** Normalize a 3-vector. */
double* normalizev_3d(
        const double *x, /**< Input: */
        double *nx       /**< Output: */
        );

/** Cross product between two 3-vectors. */
double* cross_3d(
        const double *a, /**< Input: */
        const double *b, /**< Input: */
        double *res      /**< Output: */
        );

/** Add two 3x3 matrices. */
double* addmm_3d(
        const double *A,  /**< Input: */
        const double *B,  /**< Input: */
        double *Res       /**< Output: */
        );

/** Subtract two 3x3 matrices. */
double* submm_3d(
        const double *A, /**< Input: */
        const double *B, /**< Input: */
        double *Res      /**< Output: */
        );

/** Multiply two 3x3 matrices. */
double* multmm_3d(
        const double *A,  /**< Input: */
        const double *B,  /**< Input: */
        double *Res       /**< Output: */
        );

/** Right multiply a 3x3 matrix by a 3x1 vector. */
double* multmv_3d(
        const double *A,  /**< Input: */
        const double *b,  /**< Input: */
        double *res       /**< Output: */
        );

/** Right multiply the transpose of a 3x3 matrix by a 3x1 vector. */
double* multmtv_3d(
        const double *A, /**< Input: */
        const double *b, /**< Input: */
        double *res      /**< Output: */
        );

/** Multiply a 3-vector by a scalar. */
double* multsv_3d(
        const double alpha, /**< Input: */
        const double *x,    /**< Input: */
        double *res         /**< Output: */
        );

/** Multiply a 3x3 matrix by a scalar. */
double* multsm_3d(
        const double alpha,/**< Input: */
        const double *M,   /**< Input: */
        double *Res        /**< Output: */
        );

/** Multiply a 3x1 vector by a 3x1 vector transposed into a matrix. */
double* multvvt_3d(
        const double *a,  /**< Input: */
        const double *b,  /**< Input: */
        double *Res       /**< Output: */
        );

/** Copy a 3 vector from another 3 vector. */
double* copyv_3d(
        const double *src, /**< Input: */
        double *dst        /**< Output: */
        );

/** Zero out a 3 vector. */
double* zerov_3d(
        double *v       /**< Output: */
        );

/** Inplace transpose a 3x3 matrix. */
double* transposem_inplace_3d(
        double *M       /**< Input/Output: */
        );

/** Transpose a 3x3 matrix. */
double* transposem_3d(
        const double *M, /**< Input: */
        double* Res      /**< Output: */
        );

/* 4 dimesion double precision routines (useful for homogeneous coordinates etc)*/

/** Multiply two 4x4 matrices. */
double* multmm_4d(
        const double *A,  /**< Input: */
        const double *B,  /**< Input: */
        double *Res       /**< Output: */
        );

/** Right multiply a 4x4 matrix by a 4x1 vector. */
double* multmv_4d(
        const double *A,  /**< Input: */
        const double *b,  /**< Input: */
        double *res       /**< Output: */
        );

/** Multiply a 4x4 matrix by a scalar. */
double* multsm_4d(
        const double alpha,/**< Input: */
        const double *M,   /**< Input: */
        double *Res        /**< Output: */
        );

/** L2 norm of a 4x1 vector. */
double l2normv_4d(
        const double *v /**< Input: */
        );

double* transposem_4d(
        const double *M, /**< Input: */
        double* Res      /**< Output: */
        );



/* mn dimensional double precision routines. */

/** Add two n-dimensional vectors. */
double* addvv_nd(
        const double *a, /**< Input: */
        const int n,     /**< Input: */
        const double *b, /**< Input: */
        double *c        /**< Output: */
        );


/** Scalar times a matrix, B = alpha*A. */
double *multsm_nd(
        const double alpha, /**< Input: */
        const double *A,    /**< Input: */
        const int m,        /**< Input: */
        const int n,        /**< Input: */
        const int ainc,     /**< Input: */
        double *B,          /**< Output: */
        const int binc      /**< Output: */
        );

/** Zero out the matrix A, A = 0. */
double *zero_nd(
        double *A,          /**< Output: */
        const int m,        /**< Input: */
        const int n,        /**< Input: */
        const int inc       /**< Input: */
        );

/** Subtract two n-dimensional vectors. */
double* subvv_nd(
        const double *a, /**< Input: */
        const int n,     /**< Input: */
        const double *b, /**< Input: */
        double *c        /**< Output: */
        );

double* divvv_nd(
        const double *a, /**< Input: */
        const int n,     /**< Input: */
        const double *b, /**< Input: */
        double *c        /**< Output: */
        );

/** Add two m by n dimensional matrices. */
double *addmm_nd(
        double *A,     /**< Input: */
        const int m,   /**< Input: */
        const int n,   /**< Input: */
        const int ainc,/**< Input: */
        double *B,     /**< Input: */
        const int binc,/**< Input: */
        double *C,     /**< Output: */
        const int cinc /**< Input: */
        );

/** Subtract two m by n dimensional matrices. */
double *submm_nd(
        double *A,     /**< Input: */
        const int m,   /**< Input: */
        const int n,   /**< Input: */
        const int ainc,/**< Input: */
        double *B,     /**< Input: */
        const int binc,/**< Input: */
        double *C,     /**< Output: */
        const int cinc /**< Input: */
        );


/** Add two m by n dimensional matrices inplace. */
double* addmm_inplace_nd(
        const double *A, /**< Input: */
        const int m,     /**< Input: */
        const int n,     /**< Input: */
        double *B        /**< Input/Output: */
        );

/** Multiply two vectors, element by element. */
double* diagmultvv_nd(
        const double *a, /**< Input: */
        const int n,     /**< Input: */
        const double *b, /**< Input: */
        double *c        /**< Output: */
        );

/** Transpose nd matrix. */
double* transposem_nd(
        const double *A,/**< Input: start address of matrix A */
        const int Arows,/**< Input: number of rows in A */
        const int Acols,/**< Input: number of cols in A */
        const int Ainc, /**< Input: increment between starting rows of A */
        const int Atinc,/**< Input: increment between starting rows of At */
        double *At      /**< Output: A transposed */
        );

/** Normalize an n-dimensional vector. */
double* normalize_nd(
        const double *vec,/**< Input: */
        int n,            /**< Input: */
        double *out       /**< Output: */
        );
#if 0
TODO
/** Fill the diagonal of A with the vector d. */
double *get_diag_nd(
        const double *A,    /**< Input: start address of matrix A */
        const int Arows,    /**< Input: number of rows in A */
        const int Acols,    /**< Input: number of cols in A */
        const int Ainc,     /**< Input: increment between starting rows of A */
        double *d,          /**< Output: */
        const double *dinc  /**< Input: increment between rows of d. */
        );

/** Extract the diagonal of A into the vector d.*/
double *set_diag_nd(
        const double *d,    /**< Input: */
        const double *dinc, /**< Input: increment between rows of d. */
        double *A,          /**< Output: start address of matrix A */
        const int Arows,    /**< Input: number of rows in A */
        const int Acols,    /**< Input: number of cols in A */
        const int Ainc      /**< Input: increment between starting rows of A */
        );
#endif

/* conversion routines */
/** Convert float vector to double vector. */
double* vftod(
        const float *src, /* Input: address of start of vector. */
        int len,          /* Input: length of vector. */
        int srcinc,       /* Input: memory increment between vector elements. */
        double *dst,      /* Output: output vector memory address. */
        int dstinc        /* Input: memory increment between vector elements. */
        );

/** Convert float matrix to double matrix. */
double* mftod(
        const float *src,/* Input: address of start of matrix */
        int srcm,        /* Input: number of rows in matrix */
        int srcn,        /* Input: number of cols in matirx */
        int srcinc,      /* Input: memory increment between input matrix rows */
        double *dst,     /* Output: output matrix memory address. */
        int dstinc       /* Input: memory increment between output matrix rows */
        );

float random_1f( void );   /* betwen 0 and 1 */

/* 3 dimension single precision routines */
float  edistv_3f( const float *a, const float *b );
float  magv_3f( const float *v );
float  normv_3f( const float *v );
float  dot_3f( const float *a, const float *b );
float  trace_3f( const float *M );
float  det_3f( const float *M );
float* addvv_3f( const float alpha, const float *x, const float *y, float *res );
float* subvv_3f( const float alpha, const float *x, const float *y, float *res );
float* normalizev_3f( const float *x, float *nx );
float* transposem_3f( const float *M, float* Res );
float* cross_3f( const float *a, const float *b, float *res );
float* multmm_3f( const float *A, const float *B, float *Res );
float* multmv_3f( const float *A, const float *b, float *res );
float* multmtv_3f( const float *A, const float *b, float *res );
float* multsv_3f( const float alpha, const float *x, float *res );
float* multsm_3f( const float alpha, const float *M, float *Res );
float* multvvt_3f( const float *a, const float *b, float *Res );
float* copyv_3f( const float *src, float *dst );
float* zerov_3f( float *v );

/* 4 dimension single precision routines */
float* multsm_4f( const float alpha, const float *M, float *Res );
float* multmm_4f( const float *A, const float *B, float *Res );
float* multmv_4f( const float *A, const float *b, float *res );

/* n dimensional single precision routines */
float* addvv_nf( const float *a, const int n, const float *b, float *c );
float* subvv_nf( const float *a, const int n, const float *b, float *c );
float* divvv_nf( const float *a, const int n, const float *b, float *c );

float* transposem_nf( const float *m, int mrows, int ncols, float *mt );
float* normalize_nf( const float *vec, float* out, int n );

float l2normv_4f( const float *v);

/* conversion routines */
float* vdtof( const double *src, int len, int srcinc, float *dst, int dstinc );
float* mdtof( const double *src, int srcm, int srcn,
        int srcinc, float *dst, int dstinc );

#endif
