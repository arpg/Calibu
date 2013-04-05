#include <Sophus/se3.hpp>

namespace fiducials
{

//////////////////////////////////////////////////////////////////////////////
// Simple utilities
//////////////////////////////////////////////////////////////////////////////

template<typename T> inline
Eigen::Matrix<T,3,1> Unproject(const Eigen::Matrix<T,2,1>& P)
{
    Eigen::Matrix<T,3,1> ret;
    ret.template head<2>() = P;
    ret[2] = (T)1.0;
    return ret;
}

template<typename T> inline
Eigen::Matrix<T,2,1> Project(const Eigen::Matrix<T,3,1>& P)
{
    return Eigen::Matrix<T,2,1>(P(0)/P(2), P(1)/P(2));
}

inline
Eigen::Matrix<double,1,2> dNorm_dx(const Eigen::Vector2d& x)
{
    const double normx = x.norm();
    return Eigen::Matrix<double,1,2>(x(0)/normx, x(1)/normx);
}

//////////////////////////////////////////////////////////////////////////////
// Polymorphic camera base class
//////////////////////////////////////////////////////////////////////////////

class CameraModelBase
{
public:
    //////////////////////////////////////////////////////////////////////////////    
    // Virtual member functions
    //////////////////////////////////////////////////////////////////////////////    

    virtual Eigen::Vector2d Map(const Eigen::Vector2d& proj) const = 0;
    virtual Eigen::Vector2d Unmap(const Eigen::Vector2d& img) const = 0;
    
    virtual Eigen::Matrix3d MakeK() const = 0;
    virtual Eigen::Matrix3d MakeKinv() const = 0;    
        
    //////////////////////////////////////////////////////////////////////////////
    // Project point in 3d camera coordinates to image coordinates
    Eigen::Vector2d ProjectMap(const Eigen::Vector3d& P) const
    {
        return Map( Project(P) );
    }    
    
    //////////////////////////////////////////////////////////////////////////////
    // Create 3D camera coordinates ray from image space coordinates
    Eigen::Vector3d UnmapUnproject(const Eigen::Vector2d& p) const
    {
        return Unproject( Unmap(p) );
    }        
    
    //////////////////////////////////////////////////////////////////////////////
    // Transfer point correspondence with known inv. depth to secondary camera frame.
    // Points at infinity are supported (rho = 0)
    // rhoPa = unproject(unmap(pa))
    Eigen::Vector2d Transfer3D(const Sophus::SE3d& T_ba, const Eigen::Vector3d& rhoPa, const double rho) const
    {            
        // Inverse depth point in a transformed to b (homogeneous 2D)
        const Eigen::Vector3d Pb = T_ba.rotationMatrix() * rhoPa + rho * T_ba.translation();
    
        // to non-homogeneous 2D
        const Eigen::Vector2d proj( Pb(0)/Pb(2), Pb(1)/Pb(2) );
                
        // apply distortion and linear cam
        return Map(proj); 
    }
    
    //////////////////////////////////////////////////////////////////////////////
    // Transfer point correspondence with known inv. depth to secondary camera frame.
    // Points at infinity are supported (rho = 0)
    // rhoPa = unproject(unmap(pa))
    Eigen::Vector2d Transfer3D(const Sophus::SE3d& T_ba, const Eigen::Vector3d& rhoPa, const double rho, bool& in_front) const
    {            
        // Inverse depth point in a transformed to b (homogeneous 2D)
        const Eigen::Vector3d Pb = T_ba.rotationMatrix() * rhoPa + rho * T_ba.translation();
    
        // to non-homogeneous 2D
        const Eigen::Vector2d proj( Pb(0)/Pb(2), Pb(1)/Pb(2) );
        in_front = Pb(2) > 0;
                
        // apply distortion and linear cam
        return Map(proj); 
    }
    
    //////////////////////////////////////////////////////////////////////////////
    // Transfer point correspondence with known inv. depth to secondary camera frame.
    // Points at infinity are supported (rho = 0)
    Eigen::Vector2d Transfer(const Sophus::SE3d& T_ba, const Eigen::Vector2d& pa, const double rho) const
    {
        // rho*Pa (undo distortion, unproject, avoid division by inv depth)
        const Eigen::Vector3d rhoPa = UnmapUnproject(pa);
        return Transfer3D(T_ba, rhoPa, rho);
    }
    
    //////////////////////////////////////////////////////////////////////////////
    // Transfer point correspondence with known inv. depth to secondary camera frame.
    // Points at infinity are supported (rho = 0)
    Eigen::Vector2d Transfer(const Sophus::SE3d& T_ba, const Eigen::Vector2d& pa, const double rho, bool& in_front) const
    {
        // rho*P1 (undo distortion, unproject, avoid division by inv depth)
        const Eigen::Vector3d rhoPa = UnmapUnproject(pa);
        return Transfer3D(T_ba, rhoPa, rho, in_front);
    }    
};

}
