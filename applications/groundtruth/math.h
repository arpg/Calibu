#pragma once

#include <Eigen/Eigen>

namespace Eigen
{
typedef Matrix<double,6,1>  Vector6d;
}


template <typename Scalar=double>
inline Eigen::Matrix<Scalar, 4, 4> _Cart2T(
                                    Scalar x,
                                    Scalar y,
                                    Scalar z,
                                    Scalar r,
                                    Scalar p,
                                    Scalar q
                                    )
{
    Eigen::Matrix<Scalar, 4, 4> T;
    // psi = roll, th = pitch, phi = yaw
    Scalar cq, cp, cr, sq, sp, sr;
    cr = cos( r );
    cp = cos( p );
    cq = cos( q );

    sr = sin( r );
    sp = sin( p );
    sq = sin( q );

    T(0,0) = cp*cq;
    T(0,1) = -cr*sq+sr*sp*cq;
    T(0,2) = sr*sq+cr*sp*cq;
    T(0,3) = (Scalar) 0;

    T(1,0) = cp*sq;
    T(1,1) = cr*cq+sr*sp*sq;
    T(1,2) = -sr*cq+cr*sp*sq;
    T(1,3) = (Scalar) 0;

    T(2,0) = -sp;
    T(2,1) = sr*cp;
    T(2,2) = cr*cp;
    T(2,3) = (Scalar) 0;

    T(0,3) = x;
    T(1,3) = y;
    T(2,3) = z;
    T(3,3) = (Scalar) 1;
    return T;
}

template <typename T>
inline Eigen::Matrix<T, 4, 4> _Cart2T( Eigen::Matrix<T,6,1> x)
{
    return _Cart2T<T>(x(0),x(1),x(2),x(3),x(4),x(5));
}

inline Eigen::Vector3d _R2Cart(
        const Eigen::Matrix3d& R
        )
{
    Eigen::Vector3d rpq;
    // roll
    rpq[0] = atan2( R(2,1), R(2,2) );

    // pitch
    double det = -R(2,0) * R(2,0) + 1.0;
    if (det <= 0) {
        if (R(2,0) > 0){
            rpq[1] = -M_PI / 2.0;
        }
        else{
            rpq[1] = M_PI / 2.0;
        }
    }
    else{
        rpq[1] = -asin(R(2,0));
    }

    // yaw
    rpq[2] = atan2(R(1,0), R(0,0));

    return rpq;
}

inline Eigen::Matrix<double,6,1> _T2Cart(
        const Eigen::Matrix4d& T
        )
{
    Eigen::Matrix<double,6,1> Cart;
    Eigen::Vector3d rpq = _R2Cart( T.block<3,3>(0,0) );
    Cart[0] = T(0,3);
    Cart[1] = T(1,3);
    Cart[2] = T(2,3);
    Cart[3] = rpq[0];
    Cart[4] = rpq[1];
    Cart[5] = rpq[2];

    return Cart;
}
