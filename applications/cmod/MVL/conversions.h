/**
 *  @file conversions.h 
 *  
 *  Useful routines for converting between pose/orientation representations.
 *
 *  $Id$
 */

#ifndef __KINEMATIC_CONVERSIONS_H__
#define __KINEMATIC_CONVERSIONS_H__

/** Convert axis angle to a quaternion. */
double *axis_angle_to_quat_d(
	   	const double *axis,  /**< Input: 3x1 axis */
	   	const double angle, /**< Input: angle in radians */
	   	double *q            /**< Output: unit quaternion */
		);

/** Convert axis angle to a rotmat stored in top left of a  4x4 block. */
double *axis_angle_to_hpose_d(
	   	const double *axis,  /**< Input: 3x1 axis */
	   	const double angle,  /**< Input: angle in radians */
	   	double *hpose        /**< Output: 4x4 homogeneous pose matrix */
		);


/** Convert axis angle to a euler RPY angle. */
double* axis_angle_to_eulerRPY_d( 
        const double* axis, /**< Input: */
        const double angle, /**< Input: */
        double *rpy         /**< Output: */
        );

/** Convert axis angle to a rotation matrix. */
double* axis_angle_to_rotmat_d( 
        const double* axis,  /**< Input: */
        const double  angle, /**< Input: */  
        double* R            /**< Output: */
        );


/**  Convert Euler angle to quaternion.  Note that Euler angles are
 * roll,pitch,yaw */
double *eulerRPY_to_quat_d(
	   	const double *euler, /**< Input: 3x1 roll-pitch-yaw Euler angle */
	   	double *quat         /**< Output: 4x1 unit quaternion */
		);

/**  Convert Euler angle to axis angle.  Note that Euler angles are
 * roll,pitch,yaw */
double *eulerRPY_to_axis_angle_d(
	   	const double *euler, /**< Input: 3x1 roll-pitch-yaw Euler angle. */
	   	double *axis,        /**< Output: 3x1 axis to rotate about. */
	   	double *angle        /**< Output: angle to rotate about axis. */
		);

/** Convert a quaternion to a roll-pitch-yaw Euler angle. */
double *quat_to_eulerRPY_d(
	   	const double *quat,  /**< Input: 4x1 unit quaternion */
	   	double *euler        /**< Output: 3x1 roll-pitch-yaw Euler angle */
		);

/** Convert a quaternion to an angle axis. */
double *quat_to_axis_angle_d(
	   	const double *q,      /**< Input: 4x1 unit quaternion */
	   	double *axis,          /**< Output: 3x1 axis */
	   	double *angle        /**< Output: angle */
		);

/** Build a homogenous rotation matrix, given a quaternion.  */
double *quat_to_rotmat4_d(
	   	const double *q,     /**< Input: 4x1 unit quaternion */
	   	double *mat          /**< Output: 4x4 homogeneous orientation matirx */
		);

/** Build a 3x3 rotation matrix, given a quaternion rotation. */
double *quat_to_rotmat3_d( 
		const double *q,     /**< Input: 4x1 unit quaternion */
	   	double *mat          /**< Output: 3x3 rotation matirx */
		);

/**  Convert a 3x1 roll-pitch-yaw Euler angle to a 4x4 homogeneous matrix. */
double *eulerRPY_to_rotmat4_d(
	   	const double *rpy,   /**< Input: 3x1 roll-pitch-yaw Euler angle */
	   	double *mat          /**< Output: 4x4 homogeneous rotation matirx */
		);

/**  Convert a roll-pitch-yaw Euler angle to a 3x3 rotation matirx */
double *eulerRPY_to_rotmat3_d(
	   	const double *rpy,   /**< Input: 3x1 roll-pitch-yaw Euler angle */
		double *mat          /**< Output: 3x3 rotation matirx */
		);

/** Convert a 4x4 homogeneous rotation matrix to a 3x1 Euler angle. */
double *rotmat4_to_eulerRPY_d(
	   	const double *mat,   /**< Input: 4x4 homogeneous orientation matirx */
		double *euler        /**< Output: 3x1 roll-pitch-yaw Euler angle */
		);

/** Convert a 3x3 rotation matrix to a 3x1 Euler angle. */
double *rotmat3_to_eulerRPY_d(
	   	const double *mat,   /**< Input: 3x3 rotation matirx */
	   	double *euler        /**< Output: 3x1 roll-pitch-yaw Euler angle */
		);

/** Convert a 4x4 homogeneous matrix a 4x1 unit quaternion. */
double *rotmat4_to_quat_d(
	   	const double *mat,   /**< Input: 4x4 homogeneous orientation matirx */
	   	double *quat         /**< Output: 4x1 unit quaternion */
		);

/** Convert a 3x3 rotation matrix a 4x1 unit quaternion. */
double *rotmat3_to_quat_d( 
		const double *mat,   /**< Input: 3x3 rotation matirx */
		double *quat         /**< Output: 4x1 unit quaternion */
		);


/** Convert from 4x4 homogeneous pose to 6x1 Euler pose. */
double *hpose_to_epose_d( 
		const double *hpose,  /**< Input: 4x4 homogeneous pose matrix. */
	   	double *epose         /**< Output: 6x1 Euler pose vector.*/ 
		);

/** Convert from 4x4 homogeneous pose to 7x1 Quaternion pose. */
double *hpose_to_qpose_d(
	   	const double *hpose, /**< Input: 4x4 homogeneous pose matrix. */
	   	double *qpose        /**< Output: 7x1 Quaternion pose vector. */
	   	);

/** Convert from 6x1 Euler pose to 4x4 Homogeneous pose. */
double *epose_to_hpose_d( 
		const double *epose, /**< Input: 6x1 roll-pitch-yaw Euler pose vector. */
	   	double *hpose        /**< Output: 4x4 Homogeneous pose matrix. */
	   	);

/** Convert from 6x1 Euler pose to 4x4 Homogeneous pose. */
double *epose_to_hpose_with_trig_terms_d( 
        const double *epose, /**< Input: 6x1 roll-pitch-yaw Euler pose vector. */
        double *hpose,       /**< Output: 4x4 Homogeneous pose matrix. */
        double *trigterms    /**< Outout: 6x1 vector; sin and cos of r,p,q */
        );

/** Convert from 6x1 Euler pose to 7x1 Quaternion pose. */
double *epose_to_qpose_d(
	   	const double *epose, /**< Input: 6x1 roll-pitch-yaw Euler pose vector. */
		double *qpose        /**< Output: 7x1 Quaternion pose vector. */
	   	);

/** Convert from 7x1 Quaternion pose to a 6x1 Euler pose. */
double *qpose_to_epose_d( 
		const double *qpose, /**< Input: 7x1 Quaternion pose vector. */
	   	double *epose        /**< Output: 6x1 Euler pose vector.*/ 
		);

/** Convert from 7x1 Quaternion pose to a 4x4 Homogeneous pose. */
double *qpose_to_hpose_d( 
		const double *qpose, /**< Input: 7x1 Quaternion pose vector. */
	   	double *hpose        /**< Output: 4x4 Homogeneous pose matrix. */
		);

/** Convert from a 3x3 rotaion matrix and a 3x1 position vector to a 4x4
 * homogeheous pose matrix. */
double *Rt_to_hpose_d(
		const double *R, /**< Input: 3x3 rotation matrix. */
		const double *t, /**< Input: 3xt position vector. */
	   	double *hpose    /**< Output: 4x4 Homogeneous pose matrix.*/ 
		);

/** Convert a 4x4 homogeneous pose matrix into a 3x3 rotation matrix and a 3x1
 * position vector.  */
double *hpose_to_Rt_d(
	   	const double *hpose, /**< Input: 4x4 Homogeneous pose matrix.*/
		double *R,           /**< Output: 3x3 rotation matrix. */
		double *t            /**< Output: 3x1 position vector. */
		);

/** Extract a 3x3 rotation matrix out of a 4x4 homogeneous pose matrix. */
double *hpose_to_rotmat_d( 
        const double *hpose, /**< Input: 4x4 homogeneous pose matrix. */
        double *R            /**< Output: 3x3 rotation matrix. */
        );


/* TODO: remove the duplication of supporting two floating point types.. maybe
 * use the pre-processor to make both the float and single precision routines
 * from one file that has a real_t type? */

/** Convert axis angle to a quaternion. */
float *axis_angle_to_quat_f(
	   	const float *axis,  /**< Input: 3x1 axis */
	   	const float *angle, /**< Input: angle in radians */
	   	float *q            /**< Output: unit quaternion */
		);

/**  Convert Euler angle to quaternion.  Note that Euler angles are
 * roll,pitch,yaw */
float *eulerRPY_to_quat_f(
	   	const float *euler, /**< Input: 3x1 roll-pitch-yaw Euler angle */
	   	float *quat         /**< Output: 4x1 unit quaternion */
		);

/** Convert a quaternion to a roll-pitch-yaw Euler angle. */
float *quat_to_eulerRPY_f(
	   	const float *quat,  /**< Input: 4x1 unit quaternion */
	   	float *euler        /**< Output: 3x1 roll-pitch-yaw Euler angle */
		);

/** Convert a quaternion to an axis angle. */
float *quat_to_axis_angle_f( 
        const float *q,      /**< Input: 4x1 unit quaternion */  
        float* axis,         /**< Output: 3x1 axis */ 
        float* angle         /**< Output: 3x1 angle */
        );

/** Build a homogenous rotation matrix, given a quaternion.  */
float *quat_to_rotmat4_f(
	   	const float *q,     /**< Input: 4x1 unit quaternion */
	   	float *mat          /**< Output: 4x4 homogeneous orientation matirx */
		);

/** Build a 3x3 rotation matrix, given a quaternion rotation. */
float *quat_to_rotmat3_f( 
		const float *q,     /**< Input: 4x1 unit quaternion */
	   	float *mat          /**< Output: 3x3 rotation matirx */
		);

/**  Convert a 3x1 roll-pitch-yaw Euler angle to a 4x4 homogeneous matrix. */
float *eulerRPY_to_rotmat4_f(
	   	const float *rpy,   /**< Input: 3x1 roll-pitch-yaw Euler angle */
	   	float *mat          /**< Output: 4x4 homogeneous rotation matirx */
		);

/**  Convert a roll-pitch-yaw Euler angle to a 3x3 rotation matirx */
float *eulerRPY_to_rotmat3_f(
	   	const float *rpy,   /**< Input: 3x1 roll-pitch-yaw Euler angle */
		float *mat          /**< Output: 3x3 rotation matirx */
		);

/** Convert a 4x4 homogeneous rotation matrix to a 3x1 Euler angle. */
float *rotmat4_to_eulerRPY_f(
	   	const float *mat,   /**< Input: 4x4 homogeneous orientation matirx */
		float *euler        /**< Output: 3x1 roll-pitch-yaw Euler angle */
		);

/** Convert a 3x3 rotation matrix to a 3x1 Euler angle. */
float *rotmat3_to_eulerRPY_f(
	   	const float *mat,   /**< Input: 3x3 rotation matirx */
	   	float *euler        /**< Output: 3x1 roll-pitch-yaw Euler angle */
		);

/** Convert a 4x4 homogeneous matrix a 4x1 unit quaternion. */
float *rotmat4_to_quat_f(
	   	const float *mat,   /**< Input: 4x4 homogeneous orientation matirx */
	   	float *quat         /**< Output: 4x1 unit quaternion */
		);

/** Convert a 3x3 rotation matrix a 4x1 unit quaternion. */
float *rotmat3_to_quat_f( 
		const float *mat,   /**< Input: 3x3 rotation matirx */
		float *quat         /**< Output: 4x1 unit quaternion */
		);


/** Convert from 4x4 homogeneous pose to 6x1 Euler pose. */
float *hpose_to_epose_f( 
		const float *hpose,  /**< Input: 4x4 homogeneous pose matrix. */
	   	float *epose         /**< Output: 6x1 Euler pose vector.*/ 
		);

/** Convert from 4x4 homogeneous pose to 7x1 Quaternion pose. */
float *hpose_to_qpose_f(
	   	const float *hpose, /**< Input: 4x4 homogeneous pose matrix. */
	   	float *qpose        /**< Output: 7x1 Quaternion pose vector. */
	   	);

/** Convert from 6x1 Euler pose to 4x4 Homogeneous pose. */
float *epose_to_hpose_f( 
		const float *epose, /**< Input: 6x1 roll-pitch-yaw Euler pose vector. */
	   	float *hpose        /**< Output: 4x4 Homogeneous pose matrix. */
	   	);

/** Convert from 6x1 Euler pose to 7x1 Quaternion pose. */
float *epose_to_qpose_f(
	   	const float *epose, /**< Input: 6x1 roll-pitch-yaw Euler pose vector. */
		float *qpose        /**< Output: 7x1 Quaternion pose vector. */
	   	);

/** Convert from 7x1 Quaternion pose to a 6x1 Euler pose. */
float *qpose_to_epose_f( 
		const float *qpose, /**< Input: 7x1 Quaternion pose vector. */
	   	float *epose        /**< Output: 6x1 Euler pose vector.*/ 
		);

/** Convert from 7x1 Quaternion pose to a 4x4 Homogeneous pose. */
float *qpose_to_hpose_f( 
		const float *qpose, /**< Input: 7x1 Quaternion pose vector. */
	   	float *hpose        /**< Output: 4x4 Homogeneous pose matrix. */
		);

/** Convert from a 3x3 rotaion matrix and a 3x1 position vector to a 4x4
 * homogeheous pose matrix. */
float *Rt_to_hpose_f(
		const float *R, /**< Input: 3x3 rotation matrix. */
		const float *t, /**< Input: 3xt position vector. */
	   	float *hpose    /**< Output: 4x4 Homogeneous pose matrix.*/ 
		);

/** Convert a 4x4 homogeneous pose matrix into a 3x3 rotation matrix and a 3x1
 * position vector.  */
float *hpose_to_Rt_f(
	   	const float *hpose, /**< Input: 4x4 Homogeneous pose matrix.*/
		float *R,           /**< Output: 3x3 rotation matrix. */
		float *t            /**< Output: 3x1 position vector. */
		);

/** Extract a 3x3 rotation matrix out of a 4x4 homogeneous pose matrix. */
float *hpose_to_rotmat_f( 
        const float *hpose, /**< Input: 4x4 homogeneous pose matrix. */
        float *R            /**< Output: 3x3 rotation matrix. */
        );

#endif

