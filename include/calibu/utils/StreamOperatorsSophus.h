#pragma once

#include <calibu/Platform.h>
#include <calibu/utils/StreamOperatorsEigen.h>
#include <sophus/se3.hpp>

////////////////////////////////////////////////////////////////////////////
// Overloading stream operators for Sophus
namespace calibu
{

////////////////////////////////////////////////////////////////////////////
inline std::ostream& operator<<( std::ostream& Stream, const Sophus::SO3d& R )
{
    Stream << R.unit_quaternion().coeffs();
    return Stream;
}

////////////////////////////////////////////////////////////////////////////
inline std::ostream& operator<<( std::ostream& Stream, const Sophus::SE3d& T )
{
    Stream << "[" << T.so3() << "," << T.translation() << "]";
    return Stream;
}

////////////////////////////////////////////////////////////////////////////
inline std::istream& operator>>( std::istream& Stream, Sophus::SO3d& R )
{
    Eigen::Matrix<double,4,1> coeffs;
    Stream >> coeffs;
    R.setQuaternion(Eigen::Quaterniond(coeffs));
    return Stream;
}

////////////////////////////////////////////////////////////////////////////
inline std::istream& operator>>( std::istream& Stream, Sophus::SE3d& T )
{
    char str[256];
    
    Stream.getline(str, 255, '[');
    if( Stream.gcount() > 1 ) {
        return Stream;
    }
    Stream >> T.so3();
    Stream.getline(str, 255, ',');
    Stream >> T.translation();
    Stream.getline(str, 255, ']');
    return Stream;
}

}
