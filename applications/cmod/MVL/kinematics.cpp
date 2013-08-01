/**
 *  @file kinematics.c
 *
 *  Kinematics routines
 *
 *  $Id: kinematicsf.c 73 2007-01-15 04:49:05Z gsibley $
 */

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <float.h>

#include "kinematics.h"
#include "matrix.h"

/****************************************************************************/
double *quat_compound_point_op_d( const double *xij,
        const double *xjk, double *xik)
{
    double rotmat[9], tmp[3];

    quat_to_rotmat3_d( &xij[3], rotmat );
    multmv_3d( rotmat, xjk, tmp ); /* R*xjk */

    xik[0] = tmp[0] + xij[0];
    xik[1] = tmp[1] + xij[1];
    xik[2] = tmp[2] + xij[2];

    return xik;
}

/****************************************************************************/
double *quat_compound_op_d( const double *xij, const double *xjk, double *xik)
{
    double rotmat[9], tmp[7], res[7];

    quat_to_rotmat3_d( &xij[3], rotmat);

    multmv_3d(rotmat, xjk, tmp);

    quat_mult_d( &xij[3], &xjk[3], &res[3] );
    quat_normalize_d( &res[3] );

    addvv_3d(1.0, tmp, xij, res);
    memcpy(xik, res, sizeof(double)*7);

    return xik;
}

/****************************************************************************/
double *quat_inverse_op_d( const double *xij, double *xji)
{
    double rotmat[9], trotmat[9], pos[4];

    /* TODO: clean this up */
    quat_to_rotmat3_d(&xij[3], rotmat);    /* use quat to define a rotation matrix */
    transposem_3d(rotmat, trotmat); /* transpose rot mat */
    multsm_3d(-1.0, trotmat, rotmat);     /* negate transposed rot mat */
    copyv_3d(xij, pos);
    pos[3] = 1.0;
    multmv_3d(rotmat, pos, xji);    /* multiply transposed dcm by position
                                       vector to get new position */

    /* new orientation (new quat) is the conjugate of the old quat */
    xji[3] =  xij[3];
    xji[4] = -xij[4];
    xji[5] = -xij[5];
    xji[6] = -xij[6];

    return xji;
}

double *epose_compound_op_d( const double *xij, const double *xjk, double *xik )
{
    double R1[9], R2[9], R12[9];

    eulerRPY_to_rotmat3_d( &xij[3], R1 );
    eulerRPY_to_rotmat3_d( &xjk[3], R2 );

    /* xij[0:2] = R1*xjk[0:2] + xij[0:2] */
    xik[0] = R1[0]*xjk[0] + R1[1]*xjk[1] +R1[2]*xjk[2] + xij[0];
    xik[1] = R1[3]*xjk[0] + R1[4]*xjk[1] +R1[5]*xjk[2] + xij[1];
    xik[2] = R1[6]*xjk[0] + R1[7]*xjk[1] +R1[8]*xjk[2] + xij[2];

    /* rotmat_to_eulerRPY(R1*R2)]; */
    rotmat3_to_eulerRPY_d( multmm_3d(R1, R2, R12), &xik[3] );

    return xik;
}

/****************************************************************************/
double *epose_compound_point_op_d( const double *xij, const double *xjk, double *xik )
{
    double R1[9];
    eulerRPY_to_rotmat3_d( &xij[3], R1 );

    /* xij[0:2] = R1*xjk[0:2] + xij[0:2] */
    xik[0] = R1[0]*xjk[0] + R1[1]*xjk[1] +R1[2]*xjk[2] + xij[0];
    xik[1] = R1[3]*xjk[0] + R1[4]*xjk[1] +R1[5]*xjk[2] + xij[1];
    xik[2] = R1[6]*xjk[0] + R1[7]*xjk[1] +R1[8]*xjk[2] + xij[2];

    return xik;
}

/****************************************************************************/
double *epose_inverse_op_d( const double *a, double *b)
{
    double qpose[7];
    double aq[7];

    /* TODO:  do these directly, it's not that hard */
    quat_inverse_op_d( epose_to_qpose_d(a, aq), qpose);
    qpose_to_epose_d(qpose, b);

    return b;
}

/****************************************************************************/
double *hpose_compound_op_d( const double *Hij, const double *Hjk, double *Hik )
{
    // Tij*Tjk = [ Rij*Rjk,   Rij*tjk + tij   ; ...
    //                   0,               1  ];
    Hik[3]  = Hij[0]*Hjk[3] + Hij[1]*Hjk[7] + Hij[2]*Hjk[11] + Hij[3];
    Hik[7]  = Hij[4]*Hjk[3] + Hij[5]*Hjk[7] + Hij[6]*Hjk[11] + Hij[7];
    Hik[11] = Hij[8]*Hjk[3] + Hij[9]*Hjk[7] + Hij[10]*Hjk[11] + Hij[11];
    Hik[15] = 1;

    double RijRjk[9];
    RijRjk[0] = Hij[0]*Hjk[0] + Hij[1]*Hjk[4] + Hij[2]*Hjk[8];
    RijRjk[1] = Hij[0]*Hjk[1] + Hij[1]*Hjk[5] + Hij[2]*Hjk[9];
    RijRjk[2] = Hij[0]*Hjk[2] + Hij[1]*Hjk[6] + Hij[2]*Hjk[10];

    RijRjk[3] = Hij[4]*Hjk[0] + Hij[5]*Hjk[4] + Hij[6]*Hjk[8];
    RijRjk[4] = Hij[4]*Hjk[1] + Hij[5]*Hjk[5] + Hij[6]*Hjk[9];
    RijRjk[5] = Hij[4]*Hjk[2] + Hij[5]*Hjk[6] + Hij[6]*Hjk[10];

    RijRjk[6]  = Hij[8]*Hjk[0] + Hij[9]*Hjk[4] + Hij[10]*Hjk[8];
    RijRjk[7]  = Hij[8]*Hjk[1] + Hij[9]*Hjk[5] + Hij[10]*Hjk[9];
    RijRjk[8] = Hij[8]*Hjk[2] + Hij[9]*Hjk[6] + Hij[10]*Hjk[10];

    Hik[0] = RijRjk[0];
    Hik[1] = RijRjk[1];
    Hik[2] = RijRjk[2];

    Hik[4] = RijRjk[3];
    Hik[5] = RijRjk[4];
    Hik[6] = RijRjk[5];

    Hik[8] = RijRjk[6];
    Hik[9] = RijRjk[7];
    Hik[10] = RijRjk[8];

    Hik[12] = 0;
    Hik[13] = 0;
    Hik[14] = 0;

    return Hik;
}

/****************************************************************************/
double *hpose_compound_point_op_d(
        const double *Hij,   /**< Input: object j in coordinate frame i */
        const double *pjk,   /**< Input: point k in coordinate frame j */
        double *pik          /**< Output: point k in coordinate frame i */
        )
{
    double local[3];

    local[0] = pjk[0]*Hij[0]  + pjk[1]*Hij[1]  + pjk[2]*Hij[2]  + Hij[3];
    local[1] = pjk[0]*Hij[4]  + pjk[1]*Hij[5]  + pjk[2]*Hij[6]  + Hij[7];
    local[2] = pjk[0]*Hij[8]  + pjk[1]*Hij[9]  + pjk[2]*Hij[10] + Hij[11];

    pik[0] = local[0];
    pik[1] = local[1];
    pik[2] = local[2];

    return pik;
//    return multmv_4d( Hij, Pjk, Pik ); /* not worth a function call, really */
}

/****************************************************************************/
double *hpose_inverse_op_d( const double *Hij, double *Hji )
{
    /* calc Hji = [ Hij(1:3,1:3).' -Hij(1:3,1:3).'*Hij(1:3,4); 0 0 0 1 ]; */
    Hji[0]  = Hij[0];
    Hji[1]  = Hij[4];
    Hji[2]  = Hij[8];

    Hji[4]  = Hij[1];
    Hji[5]  = Hij[5];
    Hji[6]  = Hij[9];

    Hji[8]  = Hij[2];
    Hji[9]  = Hij[6];
    Hji[10] = Hij[10];

    Hji[3]  = -Hij[0]*Hij[3] - Hij[4]*Hij[7] - Hij[8]*Hij[11];
    Hji[7]  = -Hij[1]*Hij[3] - Hij[5]*Hij[7] - Hij[9]*Hij[11];
    Hji[11] = -Hij[2]*Hij[3] - Hij[6]*Hij[7] - Hij[10]*Hij[11];

    Hji[12] = Hji[13] = Hji[14] = 0.0;
    Hji[15] = 1.0;

    return Hji;
}

/****************************************************************************/
double *quat_add_d( const double *q0, const double *q1, double *dest )
{

    dest[0] = q0[0] + q1[0];
    dest[1] = q0[1] + q1[1];
    dest[2] = q0[2] + q1[2];
    dest[3] = q0[3] + q1[3];

    quat_normalize_d(dest);

    return dest;
}

/****************************************************************************/
double *quat_mult_d( const double *q0, const double *q1, double *dest )
{

    dest[0] = q0[0]*q1[0] - q0[1]*q1[1] - q0[2]*q1[2] - q0[3]*q1[3];
    dest[1] = q0[0]*q1[1] + q0[1]*q1[0] + q0[2]*q1[3] - q0[3]*q1[2];
    dest[2] = q0[0]*q1[2] - q0[1]*q1[3] + q0[2]*q1[0] + q0[3]*q1[1];
    dest[3] = q0[0]*q1[3] + q0[1]*q1[2] - q0[2]*q1[1] + q0[3]*q1[0];

    return dest;
}

/****************************************************************************/
double* quat_normalize_d( double *q )
{
    int i;
    double mag;

    mag = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if(mag){
        for (i = 0; i < 4; i++){
            q[i] /= mag;
        }
    }

    return q;
}

/****************************************************************************/
double anglewrap_d( double a )
{
    while(a < -M_PI){
        a += 2.0*M_PI;
    }
    while(a > M_PI){
        a -= 2.0*M_PI;
    }
    return a;
}

/****************************************************************************/
double *eulerRPY_anglewrap_d( double *euler )
{
    euler[0] = anglewrap_d( euler[0] );
    euler[1] = anglewrap_d( euler[1] );
    euler[2] = anglewrap_d( euler[2] );

    return euler;
}

/****************************************************************************/
double *zero_quat_d( double *q )
{
    q[0] = 1.0;
    q[1] = 0.0;
    q[2] = 0.0;
    q[3] = 0.0;

    return (double*)q;
}

/****************************************************************************/
double *line_plane_interesect_d(
        const double *p1,
        const double *p2,
        const double *p3,
        const double *n,
        double *res )
{
    double slope[3], denom, u;

    subvv_3d(1.0, p2, p1, slope); /* p1 = up, p2 = target */
    denom = dot_3d( n, slope); /* zero if updir = viewdir */

    if( fabs(denom) > 0.000000000000001 ){
        double numer;
        numer = dot_3d( n, subvv_3d(1.0, p3, p1, res));
        u = numer/denom;

        /* res = p1 + u(p2-p1) = u*slope + p1 */
        addvv_3d( u, slope, p1, res);
    }
    else{
        return NULL;
    }
    return res;
}


/****************************************************************************/
double point_line_dist_d(
        const double *p1,
        const double *p2,
        const double *p3 )
{
    double slope[3], hyp[3], p4[3];

    normalizev_3d( subvv_3d(1.0, p2, p1, slope), slope);
    subvv_3d(1.0, p3, p1, hyp);
    addvv_3d(dot_3d(hyp, slope), slope, p1, p4);
    return normv_3d(subvv_3d(1.0, p3, p4, hyp));
}


/****************************************************************************/
double* orthonormal_basis_d(
       const double *up,
       const double *at,
       const double *target,
       double *R )
{
    double forward[3], right[3], tmpup[3];

    /* get the orthonormal basis out of target and up */
    /* the forward part */
    forward[0] = target[0] - at[0];
    forward[1] = target[1] - at[1];
    forward[2] = target[2] - at[2];

    normalizev_3d( forward, forward );
    R[0] = forward[0];
    R[3] = forward[1];
    R[6] = forward[2];

    /* get right, then get the real up */
    cross_3d( forward, up, right);
    normalizev_3d(right, right);
    R[1] = right[0];
    R[4] = right[1];
    R[7] = right[2];

    cross_3d( right, forward, tmpup);
    normalizev_3d( tmpup, tmpup );
    R[2] = tmpup[0];
    R[5] = tmpup[1];
    R[8] = tmpup[2];

    return R;
}

/****************************************************************************/
double* calc_qpose_d(
        const double *pos,
        const double *target,
        const double *up,
        double *qpose )
{
    double normal[3], angle, axis[3];
    double newup[3], p1[3], p2[3],  z[3] = {0.0, 0.0, -1.0};

    /* line between points will be the plane normal */
    normalizev_3d( subvv_3d(1.0, target, pos, normal), normal);

    /* line from "up" to "target" will intersect plane defined by
     *    * axis and pos.  That point will define our new up */
    addvv_3d(1.0, up, pos, p1);
    addvv_3d(1.0, normal, target, p2);

    if( line_plane_interesect_d( p1, p2, target, normal, newup) == NULL){
        newup[0] = up[0];
        newup[1] = up[1];
        newup[2] = up[2];
        /* TODO: get newup out out of the quaternion */
        angle = 0.0;
    }
    else{
        /* dot of new up with "old" up gives new angle */
        angle = acos(dot_3d( normalizev_3d(newup,newup), z));
    }

    /* cross between up and normal will be what we rotate about */
    cross_3d(newup, normal, axis);

    /* now we have an angle and an axis - so get a quat */
    axis_angle_to_quat_d( axis, angle, &qpose[3]);
    qpose[0] = pos[0];
    qpose[1] = pos[1];
    qpose[2] = pos[2];
    return qpose;
}

/****************************************************************************/
/* TODO: change all up's to down's */
double* calc_target_from_epose_d(
        const double *epose,
        double *pos,
        double *target,
        double *up )
{
    double xaxis[3] = { 1.0, 0.0,  0.0 };
    double zaxis[3] = { 0.0, 0.0, -1.0}; /* assume -z is up */
    double tmp[3];

    epose_compound_point_op_d( epose, xaxis, tmp );
    target[0] = tmp[0];
    target[1] = tmp[1];
    target[2] = tmp[2];

    epose_compound_point_op_d( epose, zaxis, tmp );
    up[0] = tmp[0];
    up[1] = tmp[1];
    up[2] = tmp[2];

    pos[0] = epose[0];
    pos[1] = epose[1];
    pos[2] = epose[2];

    return target;
}

/****************************************************************************/
/** Given a  position, an up dir, and a target, update the homogeneous pose.
*/
double* calc_hpose_from_target_d(
        const double *pos,    /**< Input: observers position */
        const double *target, /**< Input: target observer is looking at */
        const double *up,     /**< Input: direction considered "up" */
        double *hpose         /**< Output: 4x4 homogeneous pose matrix */
        )
{
    // line from pos to target defines forward
    double forward[3];
    normalizev_3d( subvv_3d(1.0, target, pos, forward), forward );

    // cross forward with up to get right
    double right[3];
    normalizev_3d( cross_3d( forward, up, right ), right );

    // and cross right and forward again to get down
    double down[3];
    normalizev_3d( cross_3d( forward, right, down ), down );

    // stuff results in hpose
    hpose[0] = forward[0];
    hpose[4] = forward[1];
    hpose[8] = forward[2];

    hpose[1] = right[0];
    hpose[5] = right[1];
    hpose[9] = right[2];

    hpose[2]  = down[0];
    hpose[6]  = down[1];
    hpose[10] = down[2];

    hpose[3]  = pos[0];
    hpose[7]  = pos[1];
    hpose[11] = pos[2];

    hpose[12] = 0.0;
    hpose[13] = 0.0;
    hpose[14] = 0.0;
    hpose[15] = 1.0;

    return hpose;
#if 0
    double normal[3], angle, axis[3];
    double newup[3], p1[3], p2[3],  z[3] = {0.0, 0.0, -1.0};

    /* line between points will be the plane normal */
    normalizev_3d( subvv_3d(1.0, target, pos, normal), normal);

    /* line from "up" to "target" will intersect plane defined by
     * axis and pos.  That point will define our new up */
    addvv_3d(1.0, up, pos, p1);
    addvv_3d(1.0, normal, target, p2);

    if( line_plane_interesect_d( p1, p2, target, normal, newup ) == NULL ){
        newup[0] = up[0];
        newup[1] = up[1];
        newup[2] = up[2];
        angle = 0.0;
    }
    else{
        /* dot of new up with "old" up gives new angle */
        angle = acos(dot_3d( normalizev_3d(newup,newup), z));
    }

    hpose[3] = pos[0];
    hpose[7] = pos[1];
    hpose[11] = pos[2];

    /* cross between up and normal will be what we rotate about */
    cross_3d( newup, normal, axis );

    printf( " angle %f\n", angle );
    printf( " axis %f %f %f\n", axis[0], axis[1], axis[2] );

    /* now we have an angle and an axis - so get an hpose */
    return axis_angle_to_hpose_d( axis, angle, hpose );
#endif
}

/****************************************************************************/
/** Check the validity of a 4x4 homogeneous pose matrix. */
//int isvalid_hpose_d(
//        const double *hpose /**< 4x4 homogeneous pose matrix */
//        )
//{
//    double R[9];
//    double d;

//    if( hpose[12] != 0 || hpose[13] != 0 || hpose[14] != 0 || hpose[15] != 1 ){
//        return 0;
//    }

//    /* get R out of hpose, then check the determinant */
//    d = det_3d( get_slice_nd( hpose, 4, 4, 0, 3, 0, 3, R ) );
//    if( fabs( d - 1.0 ) < 1e-12 ){ /* TODO: find better check */
//        return 1; /* good, it's an orthonormal right handed bases */
//    } else if( fabs( d + 1.0 ) < 1e-12 ){ /* TODO: find better check */
//        printf("Warning: basis is left handed!\n");
//        return 1;
//    }
//    printf("Warning: matrix determinant is%e \n", d );
//    return 0;
//}

/****************************************************************************/
/** Check the validity of a 6x1 roll-pitch-yaw Euler pose vector.  The vector
 * elements are [x,y,z,r,p,q]' where x,y and z are 3D position and r=roll,
 * p=pitch and q=yaw. */
int isvalid_epose_d(
        const double * /**< 4x4 homogeneous pose matrix */
        )
{
    /* it's implossible to have an invalid epose */
    return 1;
}

/****************************************************************************/
/** Check the validity of a 7x1 quaternion pose vector. The vector
 * elements are [x,y,z,qw,qx,qy,qz]' where x,y and z are 3D position and
 * q=[qw,qx,qy,qz] is a unit quaternion.  (Recall that qw=cos( angle/2), and
 * qx,qy,qz define an axis of rotation). */
int check_qpose_d(
        const double *qpose /**< 4x4 homogeneous pose matrix */
        )
{
    /** TODO \todo Come up with a better test */
    double normq = l2normv_4d( qpose );
    if( normq - 1.0 > 10.0*DBL_MIN ){
        return 0;
    }
    return 1;
}


/****************************************************************************



  Single precision routines



 ****************************************************************************/

float *quat_compound_point_op_f( const float *xij,
        const float *xjk, float *xik)
{
    float rotmat[9], tmp[3];

    quat_to_rotmat3_f( &xij[3], rotmat );
    multmv_3f( rotmat, xjk, tmp ); /* R*xjk */

    xik[0] = tmp[0] + xij[0];
    xik[1] = tmp[1] + xij[1];
    xik[2] = tmp[2] + xij[2];

    return xik;
}

/****************************************************************************/
float *quat_compound_op_f( const float *xij, const float *xjk, float *xik)
{
    float rotmat[9], tmp[7], res[7];

    quat_to_rotmat3_f( &xij[3], rotmat);

    multmv_3f(rotmat, xjk, tmp);

    quat_mult_f( &xij[3], &xjk[3], &res[3] );
    quat_normalize_f( &res[3] );

    addvv_3f(1.0, tmp, xij, res);
    memcpy(xik, res, sizeof(float)*7);

    return xik;
}

/****************************************************************************/
float *quat_inverse_op_f( const float *xij, float *xji)
{
    float rotmat[9], trotmat[9], pos[4];

    /* TODO: clean this up */
    quat_to_rotmat3_f(&xij[3], rotmat);    /* use quat to define a rotation matrix */
    transposem_3f(rotmat, trotmat); /* transpose rot mat */
    multsm_3f(-1.0, trotmat, rotmat);     /* negate transposed rot mat */
    copyv_3f(xij, pos);
    pos[3] = 1.0;
    multmv_3f(rotmat, pos, xji);    /* multiply transposed dcm by position
                                       vector to get new position */

    /* new orientation (new quat) is the conjugate of the old quat */
    xji[3] =  xij[3];
    xji[4] = -xij[4];
    xji[5] = -xij[5];
    xji[6] = -xij[6];

    return xji;
}

float *epose_compound_op_f( const float *xij, const float *xjk, float *xik )
{
    float R1[9], R2[9], R12[9];

    eulerRPY_to_rotmat3_f( &xij[3], R1 );
    eulerRPY_to_rotmat3_f( &xjk[3], R2 );

    /* xij[0:2] = R1*xjk[0:2] + xij[0:2] */
    xik[0] = R1[0]*xjk[0] + R1[1]*xjk[1] +R1[2]*xjk[2] + xij[0];
    xik[1] = R1[3]*xjk[0] + R1[4]*xjk[1] +R1[5]*xjk[2] + xij[1];
    xik[2] = R1[6]*xjk[0] + R1[7]*xjk[1] +R1[8]*xjk[2] + xij[2];

    /* rotmat_to_eulerRPY(R1*R2)]; */
    rotmat3_to_eulerRPY_f( multmm_3f(R1, R2, R12), &xik[3] );

    return xik;
}

/****************************************************************************/
float *epose_compound_point_op_f( const float *xij, const float *xjk, float *xik )
{
    float R1[9];
    eulerRPY_to_rotmat3_f( &xij[3], R1 );

    /* xij[0:2] = R1*xjk[0:2] + xij[0:2] */
    xik[0] = R1[0]*xjk[0] + R1[1]*xjk[1] +R1[2]*xjk[2] + xij[0];
    xik[1] = R1[3]*xjk[0] + R1[4]*xjk[1] +R1[5]*xjk[2] + xij[1];
    xik[2] = R1[6]*xjk[0] + R1[7]*xjk[1] +R1[8]*xjk[2] + xij[2];

    return xik;
}

/****************************************************************************/
float *epose_inverse_op_f( const float *a, float *b)
{
    float qpose[7];
    float aq[7];

    /* TODO:  do these directly, it's not that hard */
    quat_inverse_op_f( epose_to_qpose_f(a, aq), qpose);
    qpose_to_epose_f(qpose, b);

    return b;
}

/****************************************************************************/
float *hpose_compound_op_f( const float *Hij, const float *Hjk, float *Hik )
{
    return multmm_4f( Hij, Hjk, Hik ); /* not worth a function call, really */
}

/****************************************************************************/
float *hpose_compound_point_op_f(
        const float *Hij,   /**< Input: object j in coordinate frame i */
        const float *Pjk,   /**< Input: point k in coordinate frame j */
        float *Pik          /**< Output: point k in coordinate frame i */
        )
{
    return multmv_4f( Hij, Pjk, Pik ); /* not worth a function call, really */
}

/****************************************************************************/
float *hpose_inverse_op_f( const float *Hij, float *Hji )
{
    /* calc Hji = [ Hij(1:3,1:3).' -Hij(1:3,1:3).'*Hij(1:3,4); 0 0 0 1 ]; */
    Hji[0]  = Hij[0];
    Hji[1]  = Hij[4];
    Hji[2]  = Hij[8];

    Hji[4]  = Hij[1];
    Hji[5]  = Hij[5];
    Hji[6]  = Hij[9];

    Hji[8]  = Hij[2];
    Hji[9]  = Hij[6];
    Hji[10] = Hij[10];

    Hji[3]  = -Hij[0]*Hij[3] - Hij[4]*Hij[7] - Hij[8]*Hij[11];
    Hji[7]  = -Hij[1]*Hij[3] - Hij[5]*Hij[7] - Hij[9]*Hij[11];
    Hji[11] = -Hij[2]*Hij[3] - Hij[6]*Hij[7] - Hij[10]*Hij[11];

    Hji[12] = Hji[13] = Hji[14] = 0.0;
    Hji[15] = 1.0;

    return Hji;
}

/****************************************************************************/
float *quat_add_f( const float *q0, const float *q1, float *dest )
{

    dest[0] = q0[0] + q1[0];
    dest[1] = q0[1] + q1[1];
    dest[2] = q0[2] + q1[2];
    dest[3] = q0[3] + q1[3];

    quat_normalize_f(dest);

    return dest;
}

/****************************************************************************/
float *quat_mult_f( const float *q0, const float *q1, float *dest )
{

    dest[0] = q0[0]*q1[0] - q0[1]*q1[1] - q0[2]*q1[2] - q0[3]*q1[3];
    dest[1] = q0[0]*q1[1] + q0[1]*q1[0] + q0[2]*q1[3] - q0[3]*q1[2];
    dest[2] = q0[0]*q1[2] - q0[1]*q1[3] + q0[2]*q1[0] + q0[3]*q1[1];
    dest[3] = q0[0]*q1[3] + q0[1]*q1[2] - q0[2]*q1[1] + q0[3]*q1[0];

    return dest;
}

/****************************************************************************/
float* quat_normalize_f( float *q )
{
    int i;
    float mag;

    mag = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if(mag){
        for (i = 0; i < 4; i++){
            q[i] /= mag;
        }
    }

    return q;
}

/****************************************************************************/
float anglewrap_f( float a )
{
    while(a < -M_PI){
        a += 2.0*M_PI;
    }
    while(a > M_PI){
        a -= 2.0*M_PI;
    }
    return a;
}

/****************************************************************************/
float *eulerRPY_anglewrap_f( float *euler )
{
    euler[0] = anglewrap_f( euler[0] );
    euler[1] = anglewrap_f( euler[1] );
    euler[2] = anglewrap_f( euler[2] );

    return euler;
}

/****************************************************************************/
float *zero_quat_f( float *q )
{
    q[0] = 1.0;
    q[1] = 0.0;
    q[2] = 0.0;
    q[3] = 0.0;

    return (float*)q;
}

/****************************************************************************/
float *line_plane_interesect_f(
        const float *p1,
        const float *p2,
        const float *p3,
        const float *n,
        float *res )
{
    float slope[3], denom, u;

    subvv_3f(1.0, p2, p1, slope); /* p1 = up, p2 = target */

    denom = dot_3f( n, slope); /* zero if updir = viewdir */
    if( fabs(denom) > 0.000000000000001 ){
        float numer;
        numer = dot_3f( n, subvv_3f(1.0, p3, p1, res));
        u = numer/denom;

        /* res = p1 + u(p2-p1) = u*slope + p1 */
        addvv_3f( u, slope, p1, res);
    }
    else{
        return NULL;
    }
    return res;
}


/****************************************************************************/
float point_line_dist_f(
        const float *p1,
        const float *p2,
        const float *p3 )
{
    float slope[3], hyp[3], p4[3];

    normalizev_3f( subvv_3f(1.0, p2, p1, slope), slope);
    subvv_3f(1.0, p3, p1, hyp);
    addvv_3f(dot_3f(hyp, slope), slope, p1, p4);
    return normv_3f(subvv_3f(1.0, p3, p4, hyp));
}


/****************************************************************************/
float* orthonormal_basis_f(
       const float *up,
       const float *at,
       const float *target,
       float *R )
{
    float forward[3], right[3], tmpup[3];

    /* get the orthonormal basis out of target and up */
    /* the forward part */
    forward[0] = target[0] - at[0];
    forward[1] = target[1] - at[1];
    forward[2] = target[2] - at[2];

    normalizev_3f( forward, forward );
    R[0] = forward[0];
    R[3] = forward[1];
    R[6] = forward[2];

    /* get right, then get the real up */
    cross_3f( forward, up, right);
    normalizev_3f(right, right);
    R[1] = right[0];
    R[4] = right[1];
    R[7] = right[2];

    cross_3f( right, forward, tmpup);
    normalizev_3f( tmpup, tmpup );
    R[2] = tmpup[0];
    R[5] = tmpup[1];
    R[8] = tmpup[2];

    return R;
}

/****************************************************************************/
float* calc_qpose_f(
        const float *pos,
        const float *target,
        const float *up,
        float *qpose )
{
    float normal[3], angle, axis[3];
    float newup[3], p1[3], p2[3],  z[3] = {0.0, 0.0, -1.0};

    /* line between points will be the plane normal */
    normalizev_3f( subvv_3f(1.0, target, pos, normal), normal);

    /* line from "up" to "target" will intersect plane defined by
     *    * axis and pos.  That point will define our new up */
    addvv_3f(1.0, up, pos, p1);
    addvv_3f(1.0, normal, target, p2);

    if( line_plane_interesect_f( p1, p2, target, normal, newup) == NULL){
        newup[0] = up[0];
        newup[1] = up[1];
        newup[2] = up[2];
        /* TODO: get newup out out of the quaternion */
        angle = 0.0;
    }
    else{
        /* dot of new up with "old" up gives new angle */
        angle = acos(dot_3f( normalizev_3f(newup,newup), z));
    }

    /* cross between up and normal will be what we rotate about */
    cross_3f(newup, normal, axis);

    /* now we have an angle and an axis - so get a quat */
    axis_angle_to_quat_f( axis, &angle, &qpose[3]);
    qpose[0] = pos[0];
    qpose[1] = pos[1];
    qpose[2] = pos[2];
    return qpose;
}

/****************************************************************************/
/* TODO: change all up's to down's */
float* calc_target_from_epose_f(
        const float *epose,
        float *pos,
        float *target,
        float *up )
{
    float xaxis[3] = { 1.0, 0.0,  0.0 };
    float zaxis[3] = { 0.0, 0.0, -1.0}; /* assume -z is up */
    float tmp[3];

    epose_compound_point_op_f( epose, xaxis, tmp );
    target[0] = tmp[0];
    target[1] = tmp[1];
    target[2] = tmp[2];

    epose_compound_point_op_f( epose, zaxis, tmp );
    up[0] = tmp[0];
    up[1] = tmp[1];
    up[2] = tmp[2];

    pos[0] = epose[0];
    pos[1] = epose[1];
    pos[2] = epose[2];

    return target;
}

/****************************************************************************/
/** Check the validity of a 4x4 homogeneous pose matrix. */
//int isvalid_hpose_f(
//        const float *hpose /**< 4x4 homogeneous pose matrix */
//        )
//{
//    float R[9];
//    float d;

//    if( hpose[12] != 0 || hpose[13] != 0 || hpose[14] != 0 || hpose[15] != 1 ){
//        return 0;
//    }

//    /* get R out of hpose, then check the determinant */
//    d = det_3f( get_slice_nf( hpose, 4, 4, 0, 3, 0, 3, R ) );
//    if( fabs( d - 1.0 ) < 1e-12 ){ /* TODO: find better check */
//        return 1; /* good, it's an orthonormal right handed bases */
//    } else if( fabs( d + 1.0 ) < 1e-12 ){ /* TODO: find better check */
//        printf("Warning: basis is left handed!\n");
//        return 1;
//    }
//    printf("Warning: matrix determinant is%e \n", d );
//    return 0;
//}

/****************************************************************************/
/** Check the validity of a 6x1 roll-pitch-yaw Euler pose vector.  The vector
 * elements are [x,y,z,r,p,q]' where x,y and z are 3D position and r=roll,
 * p=pitch and q=yaw. */
int isvalid_epose_f(
        const float * /**< 4x4 homogeneous pose matrix */
        )
{
    /* it's implossible to have an invalid epose */
    return 1;
}

/****************************************************************************/
/** Check the validity of a 7x1 quaternion pose vector. The vector
 * elements are [x,y,z,qw,qx,qy,qz]' where x,y and z are 3D position and
 * q=[qw,qx,qy,qz] is a unit quaternion.  (Recall that qw=cos( angle/2), and
 * qx,qy,qz define an axis of rotation). */
int check_qpose_f(
        const float *qpose /**< 4x4 homogeneous pose matrix */
        )
{
    /** TODO \todo Come up with a better test */
    float normq = l2normv_4f( qpose );
    if( normq - 1.0 > 10.0*DBL_MIN ){
        return 0;
    }
    return 1;
}

