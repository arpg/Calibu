/**
 *  @file conversions.c
 *
 *  Useful routines for converting between pose/orientation representations.
 *
 *  $Id$
 */

#include <math.h>

#include "kinematics.h"
#include "matrix.h"


/****************************************************************************/
double* axis_angle_to_eulerRPY_d(
        const double* axis, /**< Input: */
        const double angle, /**< Input: */
        double *rpy         /**< Output: */
        )
{
    double R[9];
    axis_angle_to_rotmat_d( axis, angle, R );
    rotmat3_to_eulerRPY_d( R, rpy );
    return rpy;
}

/****************************************************************************/
double* axis_angle_to_rotmat_d(
        const double* axis,
        const double  angle,
        double* R
        )
{

    /*
       NOTE: There are three sources of redundancy with the axis-angle rotation
       specification. 1). if angle is 0, axis is undefined 2) using 4 parameters to
       define 3 degrees of freedom, and 3) [-axis,-angle] = [axis,angle] =
       [axis,2pi*angle]

       These issues will also effect quaternions.  Converting from an axis
       angle to a quaternion will be undefined for angle=0
       function rot = axis_angle_to_rotmat( axis, angle )
    */

    double quat[4];
    axis_angle_to_quat_d( axis, angle, quat );
    quat_to_rotmat3_d( quat, R );

    return R;
}

/****************************************************************************/
double *eulerRPY_to_axis_angle_d(
        const double *euler, /**< Input: 3x1 roll-pitch-yaw Euler angle. */
        double *axis,         /**< Output: 3x1 axis to rotate about. */
        double *angle        /**< Output: angle to rotate about axis. */
        )
{
    double quat[4];
//    eulerRPY_anglewrap_d( euler );
    eulerRPY_to_quat_d( euler, quat );
    quat_to_axis_angle_d( quat, axis, angle );
    return axis;
}

/****************************************************************************/
double *axis_angle_to_quat_d( const double *axis, const double angle, double *q)
{
    normalizev_3d( axis, &q[1] );
    multsv_3d( sin( angle/2.0), &q[1], &q[1]);

    q[0] = cos( angle/2.0);

    return q;
}


/****************************************************************************/
/** Convert axis angle to a rotmat stored in top left of a  4x4 block.
    NOTE: we leave the non-rotation part of the hpose alone.
 */
double *axis_angle_to_hpose_d(
        const double *axis,  /**< Input: 3x1 axis */
        const double angle,  /**< Input: angle in radians */
        double *hpose        /**< Output: 4x4 homogeneous pose matrix */
        )
{
    double q[4];
    normalizev_3d( axis, &q[1] );
    multsv_3d( sin( angle/2.0), &q[1], &q[1]);
    q[0] = cos( angle/2.0);

    hpose[0] = q[0]*q[0] + q[1]*q[1] - q[2]*q[2] - q[3]*q[3];
    hpose[1] = 2.0*( q[1]*q[2] - q[0]*q[3]);
    hpose[2] = 2.0*( q[1]*q[3] + q[0]*q[2]);

    hpose[4] = 2.0*( q[2]*q[1] + q[0]*q[3]);
    hpose[5] = q[0]*q[0] - q[1]*q[1] + q[2]*q[2] - q[3]*q[3];
    hpose[6] = 2.0*( q[2]*q[3] - q[0]*q[1]);

    hpose[8] = 2.0*( q[3]*q[1] - q[0]*q[2]);
    hpose[9] = 2.0*( q[3]*q[2] + q[0]*q[1]);
    hpose[10] = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];
    return hpose;
}

/****************************************************************************/
double *quat_to_rotmat4_d( const double *q, double *mat )
{
    mat[0] = q[0]*q[0] + q[1]*q[1] - q[2]*q[2] - q[3]*q[3];
    mat[1] = 2.0*( q[1]*q[2] - q[0]*q[3]);
    mat[2] = 2.0*( q[1]*q[3] + q[0]*q[2]);
    mat[3] = 0.0;

    mat[4] = 2.0*( q[2]*q[1] + q[0]*q[3]);
    mat[5] = q[0]*q[0] - q[1]*q[1] + q[2]*q[2] - q[3]*q[3];
    mat[6] = 2.0*( q[2]*q[3] - q[0]*q[1]);
    mat[7] = 0.0;

    mat[8] = 2.0*( q[3]*q[1] - q[0]*q[2]);
    mat[9] = 2.0*( q[3]*q[2] + q[0]*q[1]);
    mat[10] = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];
    mat[11] = 0.0;

    mat[12] = 0.0;
    mat[13] = 0.0;
    mat[14] = 0.0;
    mat[15] = 1.0;

    return mat;
}

/****************************************************************************/
double *quat_to_rotmat3_d( const double *q, double *mat )
{

    mat[0] = q[0]*q[0] + q[1]*q[1] - q[2]*q[2] - q[3]*q[3];
    mat[1] = 2.0*( q[1]*q[2] - q[0]*q[3]);
    mat[2] = 2.0*( q[1]*q[3] + q[0]*q[2]);

    mat[3] = 2.0*( q[2]*q[1] + q[0]*q[3]);
    mat[4] = q[0]*q[0] - q[1]*q[1] + q[2]*q[2] - q[3]*q[3];
    mat[5] = 2.0*( q[2]*q[3] - q[0]*q[1]);

    mat[6] = 2.0*( q[3]*q[1] - q[0]*q[2]);
    mat[7] = 2.0*( q[3]*q[2] + q[0]*q[1]);
    mat[8] = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];

    return mat;
}

/****************************************************************************/
double *eulerRPY_to_quat_d( const double *euler, double *quat )
{
    double cosX, cosY, cosZ, sinX, sinY, sinZ;
    double rpy[3];

    rpy[0] = euler[0];
    rpy[1] = euler[1];
    rpy[2] = euler[2];

    /* make sure our angles are in the desired range */
    eulerRPY_anglewrap_d( rpy );

    cosX = cos( rpy[0]/2.0 );
    cosY = cos( rpy[1]/2.0 );
    cosZ = cos( rpy[2]/2.0 );

    sinX = sin( rpy[0]/2.0 );
    sinY = sin( rpy[1]/2.0 );
    sinZ = sin( rpy[2]/2.0 );

    quat[0] = cosX*cosY*cosZ + sinX*sinY*sinZ;
    quat[1] = sinX*cosY*cosZ - cosX*sinY*sinZ;
    quat[2] = cosX*sinY*cosZ + sinX*cosY*sinZ;
    quat[3] = cosX*cosY*sinZ - sinX*sinY*cosZ;

    return quat;
}

/****************************************************************************/
double *rotmat4_to_quat_d( const double *mat, double *quat )
{
    double rpy[3];

    rotmat4_to_eulerRPY_d( mat, rpy );
    eulerRPY_to_quat_d( rpy, quat );

    return quat;
}

/****************************************************************************/
double *rotmat3_to_quat_d( const double *mat, double *quat )
{
    double rpy[3];

    rotmat3_to_eulerRPY_d( mat, rpy );
    eulerRPY_to_quat_d( rpy, quat );

    return quat;
}

/****************************************************************************/
double *rotmat4_to_eulerRPY_d( const double *mat, double *euler )
{

    double det;
    /* roll */
    euler[0] = atan2(mat[9], mat[10]);

    /* pitch */
    det = -mat[8] * mat[8] + 1.0;
    if (det <= 0) {
        if (mat[8] > 0){
            euler[1] = -M_PI / 2;
        }
        else{
            euler[1] = M_PI / 2;
        }
    }
    else{
        euler[1] = -asin(mat[8]);
    }

    /* yaw */
    euler[2] = atan2(mat[4], mat[0]);

    return euler;
}

/****************************************************************************/
double *rotmat3_to_eulerRPY_d( const double *mat, double *euler )
{

    double det;
    /* roll */
    euler[0] = atan2(mat[7], mat[8]);

    /* pitch */
    det = -mat[6] * mat[6] + 1.0;
    if (det <= 0) {
        if (mat[6] > 0){
            euler[1] = -M_PI / 2.0;
        }
        else{
            euler[1] = M_PI / 2.0;
        }
    }
    else{
        euler[1] = -asin(mat[6]);
    }

    /* yaw */
    euler[2] = atan2(mat[3], mat[0]);

    return euler;
}

/****************************************************************************/
double *eulerRPY_to_rotmat3_d( const double *rpy, double *R )
{
    double cq, cp, cr, sq, sp, sr;

    /* psi = roll, th = pitch, phi = yaw */
    cq = cos( rpy[2] );
    cp = cos( rpy[1] );
    cr = cos( rpy[0] );

    sq = sin( rpy[2] );
    sp = sin( rpy[1] );
    sr = sin( rpy[0] );

    R[0] = cp*cq;
    R[1] = -cr*sq+sr*sp*cq;
    R[2] = sr*sq+cr*sp*cq;

    R[3] = cp*sq;
    R[4] = cr*cq+sr*sp*sq;
    R[5] = -sr*cq+cr*sp*sq;

    R[6] = -sp;
    R[7] = sr*cp;
    R[8] = cr*cp;

    return R;
}

/****************************************************************************/
double *eulerRPY_to_rotmat4_d( const double *rpy, double *R )
{
    double cq, cp, cr, sq, sp, sr;

    cq = cos( rpy[2] );
    cp = cos( rpy[1] );
    cr = cos( rpy[0] );

    sq = sin( rpy[2] );
    sp = sin( rpy[1] );
    sr = sin( rpy[0] );

    R[0] = cp*cq;
    R[1] = -cr*sq+sr*sp*cq;
    R[2] = sr*sq+cr*sp*cq;
    R[3] = 0.0;

    R[4] = cp*sq;
    R[5] = cr*cq+sr*sp*sq;
    R[6] = -sr*cq+cr*sp*sq;
    R[7] = 0.0;

    R[8] = -sp;
    R[9] = sr*cp;
    R[10] = cr*cp;
    R[11] = 0.0;

    R[12] = 0.0;
    R[13] = 0.0;
    R[14] = 0.0;
    R[15] = 1.0;

    return R;
}

/****************************************************************************/
double *quat_to_axis_angle_d(
        const double *q,
        double *axis,
        double *angle  )
{
    /* get angle first */
    *angle = 2 * acos(q[0]);

    /* now get the axis */
    double norm = sqrt( q[1]*q[1] + q[2]*q[2] + q[3]*q[3] );

    axis[0] = q[1] / norm;
    axis[1] = q[2] / norm;
    axis[2] = q[3] / norm;

    return axis;
}

/****************************************************************************/
double *quat_to_eulerRPY_d( const double *quat, double *euler )
{
    double R[9];

    quat_to_rotmat3_d( quat, R );
    rotmat3_to_eulerRPY_d( R, euler );

    return euler;
}

/****************************************************************************/
double *qpose_to_hpose_d( const double *qpose, double *hpose )
{
    quat_to_rotmat4_d( &qpose[3], hpose );
    hpose[3]  = qpose[0];
    hpose[7]  = qpose[1];
    hpose[11] = qpose[2];

    return hpose;
}

/****************************************************************************/
double *qpose_to_epose_d( const double *qpose, double *epose )
{

    epose[0] = qpose[0];
    epose[1] = qpose[1];
    epose[2] = qpose[2];
    quat_to_eulerRPY_d( &qpose[3], &epose[3] );

    return epose;
}


/****************************************************************************/
double *epose_to_qpose_d( const double *epose, double *qpose )
{
    qpose[0] = epose[0];
    qpose[1] = epose[1];
    qpose[2] = epose[2];
    eulerRPY_to_quat_d( &epose[3], &qpose[3] );

    return qpose;
}


/****************************************************************************/
double *epose_to_hpose_d( const double *epose, double *hpose )
{
    eulerRPY_to_rotmat4_d( &epose[3], hpose );

    /* position */
    hpose[3] = epose[0];
    hpose[7] = epose[1];
    hpose[11] = epose[2];

    return hpose;
}

/****************************************************************************/
double *epose_to_hpose_with_trig_terms_d( const double *epose, double *hpose, double *trigterms )
{

    trigterms[0] = sin( epose[3] );
    trigterms[1] = cos( epose[3] );

    trigterms[2] = sin( epose[4] );
    trigterms[3] = cos( epose[4] );

    trigterms[4] = sin( epose[5] );
    trigterms[5] = cos( epose[5] );

    hpose[0] = trigterms[3]*trigterms[5];
    hpose[1] = -trigterms[1]*trigterms[4]+trigterms[0]*trigterms[2]*trigterms[5];
    hpose[2] = trigterms[0]*trigterms[4]+trigterms[1]*trigterms[2]*trigterms[5];
    hpose[3] = 0.0;

    hpose[4] = trigterms[3]*trigterms[4];
    hpose[5] = trigterms[1]*trigterms[5]+trigterms[0]*trigterms[2]*trigterms[4];
    hpose[6] = -trigterms[0]*trigterms[5]+trigterms[1]*trigterms[2]*trigterms[4];
    hpose[7] = 0.0;

    hpose[8] = -trigterms[2];
    hpose[9] = trigterms[0]*trigterms[3];
    hpose[10] = trigterms[1]*trigterms[3];
    hpose[11] = 0.0;

    hpose[12] = 0.0;
    hpose[13] = 0.0;
    hpose[14] = 0.0;
    hpose[15] = 1.0;

    /* position */
    hpose[3] = epose[0];
    hpose[7] = epose[1];
    hpose[11] = epose[2];

    return hpose;
}

/****************************************************************************/
double *hpose_to_qpose_d( const double *hpose, double *qpose )
{
    rotmat4_to_quat_d( hpose, &qpose[3] );
    qpose[0] = hpose[3];
    qpose[1] = hpose[7];
    qpose[2] = hpose[11];

    return qpose;
}

/****************************************************************************/
double *hpose_to_epose_d( const double *hpose, double *epose )
{
    double R[9];
    R[0] = hpose[0];
    R[1] = hpose[1];
    R[2] = hpose[2];

    R[3] = hpose[4];
    R[4] = hpose[5];
    R[5] = hpose[6];

    R[6] = hpose[8];
    R[7] = hpose[9];
    R[8] = hpose[10];

    rotmat3_to_eulerRPY_d( R, &epose[3] );

    epose[0] = hpose[3];
    epose[1] = hpose[7];
    epose[2] = hpose[11];

    return epose;
}

/****************************************************************************/
/** Convert from a 3x3 rotaion matrix and a 3x1 position vector to a 4x4
 * homogeheous pose matrix. */
double *Rt_to_hpose_d(
        const double *R, /**< Input: 3x3 rotation matrix. */
        const double *t, /**< Input: 3xt position vector. */
        double *hpose    /**< Output: 4x4 Homogeneous pose matrix.*/
        )
{
    hpose[0] = R[0];
    hpose[1] = R[1];
    hpose[2] = R[2];
    hpose[3] = t[0];

    hpose[4] = R[3];
    hpose[5] = R[4];
    hpose[6] = R[5];
    hpose[7] = t[1];

    hpose[8] = R[6];
    hpose[9] = R[7];
    hpose[10] = R[8];
    hpose[11] = t[2];

    hpose[12] = 0.0;
    hpose[13] = 0.0;
    hpose[14] = 0.0;
    hpose[15] = 1.0;

    return hpose;
}


/** Extract a 3x3 rotation matrix out of a 4x4 homogeneous pose matrix. */
double *hpose_to_rotmat_d(
        const double *hpose, /**< Input: 4x4 homogeneous pose matrix. */
        double *R            /**< Output: 3x3 rotation matrix. */
        )
{
    R[0] = hpose[0];
    R[1] = hpose[1];
    R[2] = hpose[2];

    R[3] = hpose[4];
    R[4] = hpose[5];
    R[5] = hpose[6];

    R[6] = hpose[8];
    R[7] = hpose[9];
    R[8] = hpose[10];

    return R;
}




/* Single Precision Routines */



/****************************************************************************/
float *axis_angle_to_quat_f( const float *axis, const float *angle, float *q)
{
    normalizev_3f( axis, &q[1] );
    multsv_3f( sin( *angle/2.0), &q[1], &q[1]);

    q[0] = cos( *angle/2.0);

    return q;
}

/****************************************************************************/
float *quat_to_rotmat4_f( const float *q, float *mat )
{

    mat[0] = q[0]*q[0] + q[1]*q[1] - q[2]*q[2] - q[3]*q[3];
    mat[1] = 2.0*( q[1]*q[2] - q[0]*q[3]);
    mat[2] = 2.0*( q[1]*q[3] + q[0]*q[2]);
    mat[3] = 0.0;

    mat[4] = 2.0*( q[2]*q[1] + q[0]*q[3]);
    mat[5] = q[0]*q[0] - q[1]*q[1] + q[2]*q[2] - q[3]*q[3];
    mat[6] = 2.0*( q[2]*q[3] - q[0]*q[1]);
    mat[7] = 0.0;

    mat[8] = 2.0*( q[3]*q[1] - q[0]*q[2]);
    mat[9] = 2.0*( q[3]*q[2] + q[0]*q[1]);
    mat[10] = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];
    mat[11] = 0.0;

    mat[12] = 0.0;
    mat[13] = 0.0;
    mat[14] = 0.0;
    mat[15] = 1.0;

    return (float*)mat;
}

/****************************************************************************/
float *quat_to_rotmat3_f( const float *q, float *mat )
{

    mat[0] = q[0]*q[0] + q[1]*q[1] - q[2]*q[2] - q[3]*q[3];
    mat[1] = 2.0*( q[1]*q[2] - q[0]*q[3]);
    mat[2] = 2.0*( q[1]*q[3] + q[0]*q[2]);

    mat[3] = 2.0*( q[2]*q[1] + q[0]*q[3]);
    mat[4] = q[0]*q[0] - q[1]*q[1] + q[2]*q[2] - q[3]*q[3];
    mat[5] = 2.0*( q[2]*q[3] - q[0]*q[1]);

    mat[6] = 2.0*( q[3]*q[1] - q[0]*q[2]);
    mat[7] = 2.0*( q[3]*q[2] + q[0]*q[1]);
    mat[8] = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];

    return mat;
}

/****************************************************************************/
float *eulerRPY_to_quat_f( const float *euler, float *quat )
{
    float cosX, cosY, cosZ, sinX, sinY, sinZ;
    float rpy[3];

    rpy[0] = euler[0];
    rpy[1] = euler[1];
    rpy[2] = euler[2];

    /* make sure our angles are in the desired range */
    eulerRPY_anglewrap_f( rpy );

    cosX = cos( rpy[0]/2.0 );
    cosY = cos( rpy[1]/2.0 );
    cosZ = cos( rpy[2]/2.0 );

    sinX = sin( rpy[0]/2.0 );
    sinY = sin( rpy[1]/2.0 );
    sinZ = sin( rpy[2]/2.0 );

    quat[0] = cosX*cosY*cosZ + sinX*sinY*sinZ;
    quat[1] = sinX*cosY*cosZ - cosX*sinY*sinZ;
    quat[2] = cosX*sinY*cosZ + sinX*cosY*sinZ;
    quat[3] = cosX*cosY*sinZ - sinX*sinY*cosZ;

    return quat;
}

/****************************************************************************/
float *rotmat4_to_quat_f( const float *mat, float *quat )
{
    float rpy[3];

    rotmat4_to_eulerRPY_f( mat, rpy );
    eulerRPY_to_quat_f( rpy, quat );

    return quat;
}

/****************************************************************************/
float *rotmat3_to_quat_f( const float *mat, float *quat )
{
    float rpy[3];

    rotmat3_to_eulerRPY_f( mat, rpy );
    eulerRPY_to_quat_f( rpy, quat );

    return quat;
}

/****************************************************************************/
float *rotmat4_to_eulerRPY_f( const float *mat, float *euler )
{

    float det;
    /* roll */
    euler[0] = atan2(mat[9], mat[10]);

    /* pitch */
    det = -mat[8] * mat[8] + 1.0;
    if (det <= 0) {
        if (mat[8] > 0){
            euler[1] = -M_PI / 2;
        }
        else{
            euler[1] = M_PI / 2;
        }
    }
    else{
        euler[1] = -atan(mat[8]);
    }

    /* yaw */
    euler[2] = atan2(mat[4], mat[0]);

    return euler;
}

/****************************************************************************/
float *rotmat3_to_eulerRPY_f( const float *mat, float *euler )
{

    float det;
    /* roll */
    euler[0] = atan2(mat[7], mat[8]);

    /* pitch */
    det = -mat[6] * mat[6] + 1.0;
    if (det <= 0) {
        if (mat[6] > 0){
            euler[1] = -M_PI / 2.0;
        }
        else{
            euler[1] = M_PI / 2.0;
        }
    }
    else{
        euler[1] = -asin(mat[6]);
    }

    /* yaw */
    euler[2] = atan2(mat[3], mat[0]);

    return euler;
}

/****************************************************************************/
float *eulerRPY_to_rotmat3_f( const float *rpy, float *R )
{
    float cq, cp, cr, sq, sp, sr;

    /* psi = roll, th = pitch, phi = yaw */
    cq = cos( rpy[2] );
    cp = cos( rpy[1] );
    cr = cos( rpy[0] );

    sq = sin( rpy[2] );
    sp = sin( rpy[1] );
    sr = sin( rpy[0] );

    R[0] = cp*cq;
    R[1] = -cr*sq+sr*sp*cq;
    R[2] = sr*sq+cr*sp*cq;

    R[3] = cp*sq;
    R[4] = cr*cq+sr*sp*sq;
    R[5] = -sr*cq+cr*sp*sq;

    R[6] = -sp;
    R[7] = sr*cp;
    R[8] = cr*cp;

    return R;
}

/****************************************************************************/
float *eulerRPY_to_rotmat4_f( const float *rpy, float *R )
{
    float cq, cp, cr, sq, sp, sr;

    cq = cos( rpy[2] );
    cp = cos( rpy[1] );
    cr = cos( rpy[0] );

    sq = sin( rpy[2] );
    sp = sin( rpy[1] );
    sr = sin( rpy[0] );

    R[0] = cp*cq;
    R[1] = -cr*sq+sr*sp*cq;
    R[2] = sr*sq+cr*sp*cq;
    R[3] = 0.0;

    R[4] = cp*sq;
    R[5] = cr*cq+sr*sp*sq;
    R[6] = -sr*cq+cr*sp*sq;
    R[7] = 0.0;

    R[8] = -sp;
    R[9] = sr*cp;
    R[10] = cr*cp;
    R[11] = 0.0;

    R[12] = 0.0;
    R[13] = 0.0;
    R[14] = 0.0;
    R[15] = 1.0;

    return R;
}

/****************************************************************************/
float *quat_to_axis_angle_f( const float *q, float *axis, float* angle )
{
    float  sp;

    /* get angle first */
    *angle = 2 * acos(q[0]);

    /* now get the axis */
    sp = sqrt(1-q[0]*q[0]);
    if ( fabs(sp) < 0.001){
        sp=1;
    }

    axis[1] = q[1] / sp;
    axis[2] = q[2] / sp;
    axis[3] = q[3] / sp;

    return axis;
}

/****************************************************************************/
float *quat_to_eulerRPY_f( const float *quat, float *euler )
{
    float R[9];

    quat_to_rotmat3_f( quat, R );
    rotmat3_to_eulerRPY_f( R, euler );

    return euler;
}

/****************************************************************************/
float *qpose_to_hpose_f( const float *qpose, float *hpose )
{
    quat_to_rotmat4_f( &qpose[3], hpose );
    hpose[3]  = qpose[0];
    hpose[7]  = qpose[1];
    hpose[11] = qpose[2];


    return hpose;
}

/****************************************************************************/
float *qpose_to_epose_f( const float *qpose, float *epose )
{

    epose[0] = qpose[0];
    epose[1] = qpose[1];
    epose[2] = qpose[2];
    quat_to_eulerRPY_f( &qpose[3], &epose[3] );

    return epose;
}


/****************************************************************************/
float *epose_to_qpose_f( const float *epose, float *qpose )
{
    qpose[0] = epose[0];
    qpose[1] = epose[1];
    qpose[2] = epose[2];
    eulerRPY_to_quat_f( &epose[3], &qpose[3] );

    return qpose;
}


/****************************************************************************/
float *epose_to_hpose_f( const float *epose, float *hpose )
{
    eulerRPY_to_rotmat4_f( &epose[3], hpose );

    /* position */
    hpose[3] = epose[0];
    hpose[7] = epose[1];
    hpose[11] = epose[2];

    return hpose;
}

/****************************************************************************/
float *hpose_to_qpose_f( const float *hpose, float *qpose )
{
    rotmat4_to_quat_f( hpose, &qpose[3] );
    qpose[0] = hpose[3];
    qpose[1] = hpose[7];
    qpose[2] = hpose[11];

    return qpose;
}

/****************************************************************************/
float *hpose_to_epose_f( const float *hpose, float *epose )
{
    float R[9];
    R[0] = hpose[0];
    R[1] = hpose[1];
    R[2] = hpose[2];

    R[3] = hpose[4];
    R[4] = hpose[5];
    R[5] = hpose[6];

    R[6] = hpose[8];
    R[7] = hpose[9];
    R[8] = hpose[10];

    rotmat3_to_eulerRPY_f( R, &epose[3] );

    epose[0] = hpose[3];
    epose[1] = hpose[7];
    epose[2] = hpose[11];

    return epose;
}

/****************************************************************************/
/** Convert from a 3x3 rotaion matrix and a 3x1 position vector to a 4x4
 * homogeheous pose matrix. */
float *Rt_to_hpose_f(
        const float *R, /**< Input: 3x3 rotation matrix. */
        const float *t, /**< Input: 3xt position vector. */
        float *hpose    /**< Output: 4x4 Homogeneous pose matrix.*/
        )
{
    hpose[0] = R[0];
    hpose[1] = R[1];
    hpose[2] = R[2];
    hpose[3] = t[0];

    hpose[4] = R[3];
    hpose[5] = R[4];
    hpose[6] = R[5];
    hpose[7] = t[1];

    hpose[8] = R[6];
    hpose[9] = R[7];
    hpose[10] = R[8];
    hpose[11] = t[2];

    hpose[12] = 0.0;
    hpose[13] = 0.0;
    hpose[14] = 0.0;
    hpose[15] = 1.0;

    return hpose;
}


/****************************************************************************/
/** Extract a 3x3 rotation matrix out of a 4x4 homogeneous pose matrix. */
float *hpose_to_rotmat_f(
        const float *hpose, /**< Input: 4x4 homogeneous pose matrix. */
        float *R            /**< Output: 3x3 rotation matrix. */
        )
{
    R[0] = hpose[0];
    R[1] = hpose[1];
    R[2] = hpose[2];

    R[3] = hpose[4];
    R[4] = hpose[5];
    R[5] = hpose[6];

    R[6] = hpose[8];
    R[7] = hpose[9];
    R[8] = hpose[10];

    return R;
}

