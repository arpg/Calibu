#pragma once

#include <Eigen/Eigen>
#include <Sophus/se3.hpp>

#define CAM_EPS 1E-5

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
// Linear Projection parametrizations
//////////////////////////////////////////////////////////////////////////////

struct ProjectionLinear
{
    static const unsigned NUM_PARAMS = 4;
    inline static std::string Name() { return "fu_fv_u0_v0"; }        
    template<typename T> inline static T fu(T const* params) { return params[0]; }
    template<typename T> inline static T fv(T const* params) { return params[1]; }
    template<typename T> inline static T u0(T const* params) { return params[2]; }
    template<typename T> inline static T v0(T const* params) { return params[3]; }
};

struct ProjectionLinearEqualF
{
    static const unsigned NUM_PARAMS = 3;
    inline static std::string Name() { return "f_u0_v0"; }        
    template<typename T> inline static T fu(T const* params) { return params[0]; }
    template<typename T> inline static T fv(T const* params) { return params[0]; }
    template<typename T> inline static T u0(T const* params) { return params[1]; }
    template<typename T> inline static T v0(T const* params) { return params[2]; }
};

template<unsigned w, unsigned h>
struct ProjectionLinearMinimal
{
    static const unsigned NUM_PARAMS = 1;
    inline static std::string Name() { return "f"; }
    template<typename T> inline static T fu(T const* params) { return params[0]; }
    template<typename T> inline static T fv(T const* params) { return params[0]; }
    template<typename T> inline static T u0(T const*) { return w / 2.0; }
    template<typename T> inline static T v0(T const*) { return h / 2.0; }
};

//////////////////////////////////////////////////////////////////////////////
// Distortion Parametrizations
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
struct DistortionPinhole
{
    static const unsigned NUM_PARAMS = 0;
    inline static std::string Name() { return ""; }
    template<typename T> inline static double RFactor(double r, T const*) { return 1; }    
    template<typename T> inline static double RinvFactor(double dr, T const*) { return 1; }    
};

//////////////////////////////////////////////////////////////////////////////
struct DistortionPoly
{
    static const unsigned NUM_PARAMS = 3;
    
    inline static std::string Name() { return "k1_k2_k3"; }    
    
    template<typename T> inline
    static T RFactor(T r, const T* params)
    {
        const T r2 = r*r;
        const T r4 = r2*r2;
        return (T)1.0 + params[0]*r2 + params[1]*r4 + params[2]*r4*r2;
    }

    template<typename T> inline
    static T RinvFactor(T dr, const T* params)
    {
        // TODO: imeplement
        throw std::exception();
    }    
};

//////////////////////////////////////////////////////////////////////////////
struct DistortionFov
{
    static const unsigned NUM_PARAMS = 1;
    
    inline static std::string Name() { return "w"; }
    
    template<typename T> inline
    static T RFactor(T r, const T* params)
    {
        if(params[0]*params[0] < CAM_EPS) {
            // limit w->0
            return (T)1;            
        }else{
            const T mul2tanwby2 = (T)2.0 * tan(params[0]/2.0);
            const T mul2tanwby2byw = mul2tanwby2 / params[0];
            
            if(r*r < CAM_EPS) {
                // limit r->0
                return mul2tanwby2byw;
            }else{
                return atan(r*mul2tanwby2) / (r*params[0]);
            }
        }
    }
    
    template<typename T> inline
    static T RinvFactor(T dr, const T* params)
    {
        if(params[0]*params[0] < CAM_EPS) {
            // limit w->0
            return (T)1.0;
        }else{
            const T wby2 = params[0] / 2.0;
            const T mul2tanwby2 = tan(wby2) * 2.0;
            
            if(dr*dr < CAM_EPS) {
                // limit r->0
                return params[0] / mul2tanwby2;
            }else{
                return tan(dr*params[0]) / (dr*mul2tanwby2);
            }
        }
    }
    
    inline static
    double dRFactor_dr(double r, const double* params)
    {
        if(params[0]*params[0] < CAM_EPS) {
            return 0;
        }else{
            if(r*r < CAM_EPS) {
                return 0;
            }else{
                const double wby2 = params[0]/2.0;
                const double tanwby2 = tan(wby2);
                const double sq_tanwby2 = tanwby2 * tanwby2;
                const double rr=r*r;
                return (2*tanwby2)/(r*params[0]*(4*rr*sq_tanwby2+1))-atan(2*r*tanwby2)/(rr*params[0]);
            }
        }
    }
};

//////////////////////////////////////////////////////////////////////////////
// Linear Projection and Distortion
//////////////////////////////////////////////////////////////////////////////

template<typename ProjModel, typename DistortionModel>
class Camera
{
public:
    static const unsigned NUM_PARAMS = ProjModel::NUM_PARAMS + DistortionModel::NUM_PARAMS;
            
    ///////////////////////////////////////////////////////////////////////////
    // Static Utilities
    ///////////////////////////////////////////////////////////////////////////

    inline static std::string Name() { return ProjModel::Name() + "_" + DistortionModel::Name(); }
    
    template<typename T> inline
    static Eigen::Matrix<T,3,3> MakeK(T const* camparam)
    {
        Eigen::Matrix<T,3,3> K;
        K << ProjModel::fu(camparam), 0, ProjModel::u0(camparam),
                0, ProjModel::fv(camparam), ProjModel::v0(camparam),
                0,0,1;
        return K;
    }

    template<typename T> inline
    static Eigen::Matrix<T,3,3> MakeKinv(T const* camparam)
    {
        Eigen::Matrix<T,3,3> K;
        K << 1.0/ProjModel::fu(camparam), 0,    -ProjModel::u0(camparam) / ProjModel::fu(camparam),
                0, 1.0/ProjModel::fv(camparam), -ProjModel::v0(camparam) / ProjModel::fv(camparam),
                0,0,1;
        return K;
    }
    
    template<typename T> inline
    static Eigen::Matrix<T,2,1> Map(const Eigen::Matrix<T,2,1>& proj, T const* camparam)
    {
        const T fac = DistortionModel::template RFactor<T>(proj.norm(), camparam + ProjModel::NUM_PARAMS);
        
        // apply distortion and linear cam
        return Eigen::Matrix<T,2,1>(
            fac*ProjModel::fu(camparam)*proj(0) + ProjModel::u0(camparam),
            fac*ProjModel::fv(camparam)*proj(1) + ProjModel::v0(camparam)
        );
    }
    
    template<typename T> inline
    static Eigen::Matrix<T,2,1> Unmap(const Eigen::Matrix<T,2,1>& img, T const* camparam)
    {    
        const Eigen::Matrix<T,2,1> dproj = Eigen::Matrix<T,2,1>(
            (img[0] - ProjModel::u0(camparam) ) / ProjModel::fu(camparam),
            (img[1] - ProjModel::v0(camparam) ) / ProjModel::fv(camparam)
        );
        const T fac = DistortionModel::RinvFactor(dproj.norm(), camparam + ProjModel::NUM_PARAMS);
        return fac*dproj;
    }
    
    // Transfer point correspondence with known inv. depth to secondary camera frame.
    // Points at infinity are supported (rho = 0)
    // rhoPa = unproject(unmap(pa))
    template<typename T> inline
    static Eigen::Matrix<T,2,1> Transfer3D(const T* camparam, const Sophus::SE3Group<T>& T_ba, const Eigen::Matrix<T,3,1>& rhoPa, const T rho)
    {            
        // Inverse depth point in a transformed to b (homogeneous 2D)
        const Eigen::Matrix<T,3,1> Pb =
                T_ba.rotationMatrix() * rhoPa + rho * T_ba.translation();
    
        // to non-homogeneous 2D
        const Eigen::Matrix<T,2,1> proj( Pb(0)/Pb(2), Pb(1)/Pb(2) );
                
        // apply distortion and linear cam
        return Map(proj, camparam); 
    }
    
    // Transfer point correspondence with known inv. depth to secondary camera frame.
    // Points at infinity are supported (rho = 0)
    // rhoPa = unproject(unmap(pa))
    template<typename T> inline
    static Eigen::Matrix<T,2,1> Transfer3D(const T* camparam, const Sophus::SE3Group<T>& T_ba, const Eigen::Matrix<T,3,1>& rhoPa, const T rho, bool& in_front)
    {            
        // Inverse depth point in a transformed to b (homogeneous 2D)
        const Eigen::Matrix<T,3,1> Pb =
                T_ba.rotationMatrix() * rhoPa + rho * T_ba.translation();
    
        // to non-homogeneous 2D
        const Eigen::Matrix<T,2,1> proj( Pb(0)/Pb(2), Pb(1)/Pb(2) );
        in_front = Pb(2) > 0;
                
        // apply distortion and linear cam
        return Map(proj, camparam); 
    }
    
    // Transfer point correspondence with known inv. depth to secondary camera frame.
    // Points at infinity are supported (rho = 0)
    template<typename T> inline
    static Eigen::Matrix<T,2,1> Transfer(const T* camparam, const Sophus::SE3Group<T>& T_ba, const Eigen::Matrix<T,2,1>& pa, const T rho)
    {
        // rho*Pa (undo distortion, unproject, avoid division by inv depth)
        const Eigen::Matrix<T,3,1> rhoPa = Unproject<T>(
            Unmap<T>(pa, camparam)
        );
        
        return Transfer3D(camparam, T_ba, rhoPa, rho);
    }
    
    // Transfer point correspondence with known inv. depth to secondary camera frame.
    // Points at infinity are supported (rho = 0)
    template<typename T> inline
    static Eigen::Matrix<T,2,1> Transfer(const T* camparam, const Sophus::SE3Group<T>& T_ba, const Eigen::Matrix<T,2,1>& pa, const T rho, bool& in_front)
    {
        // rho*P1 (undo distortion, unproject, avoid division by inv depth)
        const Eigen::Matrix<T,3,1> rhoPa = Unproject<T>(
            Unmap<T>(pa, camparam)
        );
            
        return Transfer3D(camparam, T_ba, rhoPa, rho, in_front);
    }
    
    static inline
    Eigen::Matrix<double,2,2> dMap_dp(const Eigen::Vector2d& p, const double* camparam)
    {
        const double r = p.norm();    
        const Eigen::Vector2d dNorm_dp(p(0)/r, p(1)/r);

        const double fac = DistortionModel::RFactor(r, camparam + ProjModel::NUM_PARAMS );
        const Eigen::Vector2d dfac_dp = DistortionModel::dRFactor_dr(r,camparam + ProjModel::NUM_PARAMS) * dNorm_dp;
                
        Eigen::Matrix<double,2,2> J;
        J.col(0) = Eigen::Matrix<double,2,1>(
            dfac_dp(0) *ProjModel::fu(camparam)*p(0) + fac*ProjModel::fu(camparam),
            dfac_dp(0) *ProjModel::fv(camparam)*p(1)
        );
        J.col(1) = Eigen::Matrix<double,2,1>(
            dfac_dp(1) *ProjModel::fu(camparam)*p(0),
            dfac_dp(1) *ProjModel::fv(camparam)*p(1) + fac*ProjModel::fv(camparam)
        );
        
        return J;
    }
    
    static inline
    Eigen::Matrix<double,2,3> dMap_dP(const Eigen::Vector3d& P, const double* camparam)
    {
        const Eigen::Vector2d p(P(0) / P(2), P(1) / P(2));
        const Eigen::Matrix<double,2,2> _dMap_dp = dMap_dp(p,camparam);
        
        Eigen::Matrix<double,2,3> _dp_dP;
        _dp_dP << 
          1.0/P(2), 0, -P(0)/(P(2)*P(2)),
          0, 1.0/P(2), -P(1)/(P(2)*P(2));
                
        return _dMap_dp * _dp_dP;
    }    
    
    ///////////////////////////////////////////////////////////////////////////
    // Member functions
    ///////////////////////////////////////////////////////////////////////////

    Camera()
        : params(Eigen::Matrix<double,NUM_PARAMS,1>::Zero())
    {
    }    

    Camera(const Eigen::Matrix<double,NUM_PARAMS,1>& params)
        : params(params)
    {
    }    

    Camera(double* cam_params)
        : params(Eigen::Map<Eigen::Matrix<double,NUM_PARAMS,1> >(cam_params))
    {
    }    
    
    Camera(const Camera& other)
        : params(other.params)
    {
    }    
    
    Eigen::Matrix<double,NUM_PARAMS,1>& Params() {
        return params;
    }

    const Eigen::Matrix<double,NUM_PARAMS,1>& Params() const {
        return params;
    }
    
    const double* data() const {
        return params.data();
    }

    double* data() {
        return params.data();
    }
    
    inline Eigen::Vector2d Map(const Eigen::Vector2d& proj) const
    {
        return Map(proj, params.data());
    }

    inline Eigen::Vector2d Unmap(const Eigen::Vector2d& img) const
    {
        return Unmap(img, params.data());
    }
    
    inline Eigen::Vector2d ProjectMap(const Eigen::Vector3d& P) const
    {
        return Map( Project(P) , params.data() );
    }    
    
    inline Eigen::Vector3d UnmapUnproject(const Eigen::Vector2d& p) const
    {
        return Unproject( Unmap( p, params.data()) );
    }        
    
    inline Eigen::Vector2d Transfer(const Sophus::SE3d& T_ba, const Eigen::Vector2d& pa, double rho) const
    {
        return Transfer<double>(data(), T_ba, pa, rho);
    }

    inline Eigen::Vector2d Transfer(const Sophus::SE3d& T_ba, const Eigen::Vector2d& pa, double rho, bool& in_front) const
    {
        return Transfer<double>(data(), T_ba, pa, rho, in_front);
    }
    
    inline Eigen::Matrix3d K()
    {
        return MakeK(params.data());
    }

    inline Eigen::Matrix3d Kinv()
    {
        return MakeKinv(params.data());
    }
    
    int& Width() { return width; }
    int Width() const { return width; }

    int& Height() { return height; }
    int Height() const { return height; }
    
protected:
    int width;
    int height;
    Eigen::Matrix<double,NUM_PARAMS,1> params;
};
