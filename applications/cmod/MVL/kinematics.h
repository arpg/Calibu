/**
    @file kinematics.h

    Double and single precision kinematics routines.

    These are simple routines for preforming "spatial operations" on 3D
    objects-e.g. moving, rotating, combinging transforms, "inverting"
    transfroms etc.  It supports arbitraty composition of spatial relations,
    alowing easy determination of the pose of an object in a coordinate frame
    of interest.  This is useful in particular in many robotics applications -
    e.g.  manipulatior kinematics, localization, mapping, SLAM, target
    tracking, simulation etc. etc.

    An object, i, in coordinate frame, j, has pose xij. An object  pose is an
    entity comprised of a 3D Cartesian position, xyz, and an orientation.
    Orientation can be represented in a number of ways, so we have a variety of
    kinds of "poses", depending on how the orientation is stored in memory,
    these are:

    - epose: 6x1, x,y,z,roll,pitch,yaw, pose vector.
    - hpose: 4x4 homogeneous pose matrix
    - qpose: 7x1 x,y,z, quaternion pose vector

    \b Note that the convention used throughout this library is the standard
    aeroplane convention:  +x is forward, +y is right, and +z is down.

    Composition of poses is supported via the "compound" and "inverse"
    operators. The primary benefit of thinking in terms of operations on
    spatial relations is the intuitive appeal, and the fact that the underlying
    linear-algebra does not need to be considered over and over again.

    Many plant and sensor models can be written in terms of compound and
    inverse  operators.  These functions will often need to be linearized (e.g.
    in linear error propagation, Kalman filtering, least squares normal
    equations, non-linear optimization, etc.)  Hence, the Jacobian of the
    compound and inverse operators are also  implemented.

    $Id: kinematics.h 73 2007-01-15 04:49:05Z gsibley $
 */

#ifndef __KINEMATICS_H__
#define __KINEMATICS_H__

#include "matrix.h"

/* double precision routines */

/**
 * Compound op for Euler Roll-Pitch-Yaw poses for finding position of 3d points
 * as opposed to poses of objets.
 *
 * Given xij, (j's pose in frame i) and xjk (k's 3d position - not pose - in
 * frame j), compute the 3d position point k in frame i
 */
double *epose_compound_point_op_d(
        const double *xij,   /**< input: object j in coordinate frame i */
        const double *xjk,   /**< input: point k in coordinate frame j */
        double *xik          /**< output: k in coordinate frame i */
        );

/**  Compound opperator for Euler Roll-Pitch-Yaw poses.
 * Given xij, (j's pose in frame i) and xjk (k's 3d pose in frame j), compute
 * the 3d position point k in frame i
 */
double *epose_compound_op_d(
        const double *xij,   /**< Input: object j in coordinate frame i */
        const double *xjk,   /**< Input: object k in coordinate frame j */
        double *xik          /**< Output: k in coordinate frame i */
        );

/**  Inverse for Euler Roll-Pitch-Yaw poses. */
double *epose_inverse_op_d(
        const double *xij,   /* Input: object j in coordinate frame i */
        double *xji          /* Output: object i in coordinate frame j */
        );

/**  Compound opperator for Homogeneous poses.
 * Given xij, (j's pose in frame i) and xjk (k's 3d pose in frame j), compute
 * the 3d position point k in frame i
 */
double *hpose_compound_op_d(
        const double *Hij,   /**< Input: object j in coordinate frame i */
        const double *Hjk,   /**< Input: object k in coordinate frame j */
        double *Hik          /**< Output: k in coordinate frame i */
        );

/**  Compound opperator for Homogeneous poses and points.
 * Given xij, (j's pose in frame i) and xjk (k's 3d position - not pose - in
 * frame j), compute the 3d position point k in frame i
 */
double *hpose_compound_point_op_d(
        const double *Hij,   /**< Input: object j in coordinate frame i */
        const double *Pjk,   /**< Input: point k in coordinate frame j */
        double *Hik          /**< Output: k in coordinate frame i */
        );

/**  Inverse for Homogeneous poses. */
double *hpose_inverse_op_d(
        const double *Hij,   /* Input: j in coordinate frame i */
        double *Hji          /* Output: i in coordinate frame j */
        );

/** Quaternion compound operations.  The compound operation for quaternion
 * poses where xjk is just a point. */
double *quat_compound_point_op_d(
        const double *xij,   /**< Input: entity j in coordinate frame i */
        const double *xjk,   /**< Input: entity k in coordinate frame j */
        double *xik          /**< Output: k in coordinate frame i */
        );

/** The compound operation for quaternion poses. */
double *quat_compound_op_d(
        const double *xij,   /**< Input: entity j in coordinate frame i */
        const double *xjk,   /**< Input: entity k in coordinate frame j */
        double *xik          /**< Output: k in coordinate frame i */
        );

/** The inverse operation for quaternion poses. */
double *quat_inverse_op_d(
        const double *xij,   /**< Input: entity j in coordinate frame i */
        double *xji          /**< Output: i in coordinate frame j */
        );

double *eulerRPY_anglewrap_d(
        double *euler        /**< Input/output: 3x1 roll-pitch-yaw Euler angle */
        );

double *quat_add_d(
        const double *q1,    /**< Input: 4x1 unit quaternion */
        const double *q2,    /**< Input: 4x1 unit quaternion */
        double *dest         /**< Output: 4x1 unit quaternion */
        );

/** Quaternion multiplication.  Quaternion multiplication is NOT vector
 * multiplication */
double *quat_mult_d(
        const double *q1,    /**< Input: 4x1 unit quaternion */
        const double *q2,    /**< Input: 4x1 unit quaternion */
        double *dest         /**< Output: 4x1 unit quaternion */
        );

double *quat_normalize_d(
        double *q            /**< output: 4x1 unit quaternion */
        );


/** Check the validity of a 4x4 homogeneous pose matrix. */
int isvalid_hpose_d(
        const double *hpose /**< 4x4 homogeneous pose matrix */
        );

/** Check the validity of a 6x1 roll-pitch-yaw Euler pose vector.  The vector
 * elements are [x,y,z,r,p,q]' where x,y and z are 3D position and r=roll,
 * p=pitch and q=yaw. */
int isvalid_epose_d(
        const double *hpose /**< 4x4 homogeneous pose matrix */
        );

/** Check the validity of a 7x1 quaternion pose vector. The vector
 * elements are [x,y,z,qw,qx,qy,qz]' where x,y and z are 3D position and
 * q=[qw,qx,qy,qz] is a unit quaternion.  (Recall that qw=cos( angle/2), and
 * qx,qy,qz define an axis of rotation). */
int isvalid_qpose_d(
        const double *hpose /**< 4x4 homogeneous pose matrix */
        );

/** Find the intersection of a line with a plane. */
double *line_plane_interesect_d(
        const double *p1,  /**  Input: first point on a line. */
        const double *p2,  /**  Input: second point on a line, closer to plane */
        const double *planept,  /**  Input: point on a plane with normal n. */
        const double *n,   /**  Input: plane normal.    */
        double *res  /**  Output: res, the point if it exsists, else NULL */
        );

/** p1 and p2 define the line.  return shortest distance to p3 */
double point_line_dist_d(
        const double *p1,    /**< Input: First point on the line */
        const double *p2,    /**< Input: Second point on the line */
        const double *p3     /**< Input: Point to find distance to */
        );
/** Given a position, an up dir, and a target, calc the orthonormal basis.
 * Degenerate solutions should preserve the current up dir in R col 3.
 *
 * Notice that we don't assume that up and forward are orthogonal to start.
 */
double* orthonormal_basis_d(
        const double *up,    /**< Input: 3x1 up vector. */
        const double *at,    /**< Input: 3x1 observer position. */
        const double *target,/**< Input: 3x1 target observer is looking at. */
        double *R            /**< Output: 3x3 orientation matrix */
        );

/**  Given an Euler pose, calculate pos, target and up. */
double* calc_target_from_epose_d(
        const double *epose, /**< Input: 6x1 ovserver Euler pose */
        double *pos,        /**< Output: 3x1 observer position */
        double *target,     /**< Output: 3x1 target position */
        double *up          /**< Output: 3x1 up direction */
        );

/** Given a  position, an up dir, and a target, calc the quat pose.
 * Degenerate solutions should preserve the current up dir in pose. */
double* calc_qpose_d(
        const double *pos,    /**< Input: observers position */
        const double *target, /**< Input: target observer is looking at */
        const double *up,     /**< Input: direction considered "up" */
        double *qpose         /**< Output: 7x1 quaternion pose */
        );

/** Given a  position, an up dir, and a target, calc the quat pose.
 * Degenerate solutions should preserve the current up dir in pose. */
double* calc_qpose_d(
        const double *pos,    /**< Input: observers position */
        const double *target, /**< Input: target observer is looking at */
        const double *up,     /**< Input: direction considered "up" */
        double *qpose         /**< Output: 7x1 quaternion pose */
        );

/** Given a  position, an up dir, and a target, update the homogeneous pose.
*/
double* calc_hpose_from_target_d(
        const double *pos,    /**< Input: observers position */
        const double *target, /**< Input: target observer is looking at */
        const double *up,     /**< Input: direction considered "up" */
        double *hpose         /**< Output: 4x4 homogeneous pose matrix */
        );

/* single precision routines */

/**
 * Compound op for finding position of 3d points as opposed to poses of objets.
 *
 * Given xij, (j's pose in frame i) and xjk (k's 3d position - not pose - in
 * frame j), compute the 3d position point k in frame i
 */
float *epose_compound_point_op_f(
        const float *xij,   /**< Input: object j in coordinate frame i */
        const float *xjk,   /**< Input: point k in coordinate frame j */
        float *xik          /**< Output: k in coordinate frame i */
        );

 /**
 * Compound op.
 */
float *epose_compound_op_f(
        const float *xij,   /**< Input: object j in coordinate frame i */
        const float *xjk,   /**< Input: object k in coordinate frame j */
        float *xik          /**< Output: k in coordinate frame i */
        );

float *epose_inverse_op_f(
        const float *xij,   /* Input: object j in coordinate frame i */
        float *xji          /* Output: object i in coordinate frame j */
        );

 /**
 * Compound op.
 */
float *hpose_compound_op_f(
        const float *Hij,   /**< Input: object j in coordinate frame i */
        const float *Hjk,   /**< Input: object k in coordinate frame j */
        float *Hik          /**< Output: k in coordinate frame i */
        );

float *hpose_compound_point_op_f(
        const float *Hij,   /**< Input: object j in coordinate frame i */
        const float *Pjk,   /**< Input: point k in coordinate frame j */
        float *Hik          /**< Output: k in coordinate frame i */
        );

float *hpose_inverse_op_f(
        const float *Hij,   /* Input: j in coordinate frame i */
        float *Hji          /* Output: i in coordinate frame j */
        );

/** Quaternion compound operations.  The compound operation for quaternion
 * poses where xjk is just a point. */
float *quat_compound_point_op_f(
        const float *xij,   /**< Input: entity j in coordinate frame i */
        const float *xjk,   /**< Input: entity k in coordinate frame j */
        float *xik          /**< Output: k in coordinate frame i */
        );

/** The compound operation for quaternion poses. */
float *quat_compound_op_f(
        const float *xij,   /**< Input: entity j in coordinate frame i */
        const float *xjk,   /**< Input: entity k in coordinate frame j */
        float *xik          /**< Output: k in coordinate frame i */
        );

/** The inverse operation for quaternion poses. */
float *quat_inverse_op_f(
        const float *xij,   /**< Input: entity j in coordinate frame i */
        float *xji          /**< Output: i in coordinate frame j */
        );

float *eulerRPY_anglewrap_f(
        float *euler        /**< Input/Output: 3x1 roll-pitch-yaw Euler angle */
        );

float *quat_add_f(
        const float *q1,    /**< Input: 4x1 unit quaternion */
        const float *q2,    /**< Input: 4x1 unit quaternion */
        float *dest         /**< Output: 4x1 unit quaternion */
        );

/** Quaternion multiplication.  Quaternion multiplication is NOT vector
 * multiplication */
float *quat_mult_f(
        const float *q1,    /**< Input: 4x1 unit quaternion */
        const float *q2,    /**< Input: 4x1 unit quaternion */
        float *dest         /**< Output: 4x1 unit quaternion */
        );

float *quat_normalize_f(
        float *q            /**< Output: 4x1 unit quaternion */
        );


/** Check the validity of a 4x4 homogeneous pose matrix. */
int isvalid_hpose_f(
        const float *hpose /**< Input: 4x4 homogeneous pose matrix */
        );

/** Check the validity of a 6x1 roll-pitch-yaw Euler pose vector.  The vector
 * elements are [x,y,z,r,p,q]' where x,y and z are 3D position and r=roll,
 * p=pitch and q=yaw. */
int isvalid_epose_f(
        const float *hpose /**< Input: 4x4 homogeneous pose matrix */
        );

/** Check the validity of a 7x1 quaternion pose vector. The vector
 * elements are [x,y,z,qw,qx,qy,qz]' where x,y and z are 3D position and
 * q=[qw,qx,qy,qz] is a unit quaternion.  (Recall that qw=cos( angle/2), and
 * qx,qy,qz define an axis of rotation). */
int isvalid_qpose_f(
        const float *hpose /**< Input: 4x4 homogeneous pose matrix */
        );

/** Find the intersection of a line with a plane. */
float *line_plane_interesect_f(
        const float *p1,  /**  Input: first point on a line. */
        const float *p2,  /**  Input: second point on a line. */
        const float *p3,  /**  Input: point on a plane with normal n. */
        const float *n,   /**  Input: plane normal.    */
        float *res  /**  Output: res, the point if it exsists, else NULL */
        );

/** p1 and p2 define the line.  return shortest distance to p3 */
float point_line_dist_f(
        const float *p1,    /**< Input: First point on the line */
        const float *p2,    /**< Input: Second point on the line */
        const float *p3     /**< Input: Point to find distance to */
        );
/** Given a position, an up dir, and a target, calc the orthonormal basis.
 * Degenerate solutions should preserve the current up dir in R col 3.
 *
 * Notice that we don't assume that up and forward are orthogonal to start.
 */
float* orthonormal_basis_f(
        const float *up,    /**< Input: 3x1 up vector. */
        const float *at,    /**< Input: 3x1 observer position. */
        const float *target,/**< Input: 3x1 target observer is looking at. */
        float *R            /**< Output: 3x3 orientation matrix */
        );

/**  Given an Euler pose, calculate pos, target and up. */
float* calc_target_from_epose_f(
        const float *epose, /**< Input: 6x1 ovserver Euler pose */
        float *pos,        /**< Output: 3x1 observer position */
        float *target,     /**< Output: 3x1 target position */
        float *up          /**< Output: 3x1 up direction */
        );

/** Given a  position, an up dir, and a target, calc the quat pose.
 * Degenerate solutions should preserve the current up dir in pose. */
float* calc_qpose_f(
        const float *pos,    /**< Input: observers position */
        const float *target, /**< Input: target observer is looking at */
        const float *up,     /**< Input: direction considered "up" */
        float *qpose         /**< Output: 7x1 quaternion pose */
        );

#include "conversions.h"
//#include <mvl/kinematics/jacobians.h>
//#include <mvl/kinematics/homogeneous_transforms.h>

#endif
