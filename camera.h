#ifndef CAMERA_H
#define CAMERA_H

#include <TooN/TooN.h>

class AbstractCamera
{
public:
  AbstractCamera(int width, int height)
    : size( TooN::makeVector(width,height) )
  {
  }

  //applies camera intrinsics including distortion
  virtual TooN::Vector<2> map(const TooN::Vector<2>& camframe) const = 0;

  //undo camera intrinsics including undistortion
  virtual TooN::Vector<2> unmap(const TooN::Vector<2>& imframe) const = 0;

  // Project p_cam in to the camera, applying intrinsics and distortion
  inline TooN::Vector<2> project_map(const TooN::Vector<3>& p_cam) const
  {
    return map( project(p_cam) );
  }

  // Take image coordinates into 3D camera coordinates on image plane
  inline TooN::Vector<3> unmap_unproject(const TooN::Vector<2>& img) const
  {
    return unproject( unmap( img) );
  }

  inline int width() const { return size[0]; }
  inline int height() const { return size[1]; }
  inline int pixel_area() const { return size[0] * size[1]; }
  inline double aspect() const { return (double)size[0] / (double)size[1]; }

  inline bool in_image(const TooN::Vector<2> & obs) const
  {
    return (obs[0]>=0 && obs[0]<width() && obs[1]>=1 && obs[1]<height());
  }

protected:
  TooN::Vector<2,unsigned> size;
};

class LinearCamera : public AbstractCamera
{
public:

  LinearCamera(int w, int h, double fu, double fv, double u0, double v0)
    :AbstractCamera(w,h), _pp(TooN::makeVector(u0,v0)), _f(TooN::makeVector(fu,fv)),
      _K(TooN::Identity), _Kinv(TooN::Identity)
  {
    _K(0,0) = _f[0];
    _K(1,1) = _f[1];
    _K(0,2) = _pp[0];
    _K(1,2) = _pp[1];
    _Kinv(0,0) = 1.0/_f[0];
    _Kinv(1,1) = 1.0/_f[1];
    _Kinv(0,2) = -_pp[0] / _f[0];
    _Kinv(1,2) = -_pp[1] / _f[1];
  }

  inline virtual TooN::Vector<2> map(const TooN::Vector<2>& cam) const
  {
    return TooN::makeVector(
      _f[0] * cam[0] + _pp[0],
      _f[1] * cam[1] + _pp[1]
    );
  }

  inline virtual TooN::Vector<2> unmap(const TooN::Vector<2>& img) const
  {
    return TooN::makeVector(
      (img[0] - _pp[0]) / _f[0],
      (img[1] - _pp[1]) / _f[1]
    );
  }

  inline const TooN::Matrix<3,3>& K() const
  {
    return _K;
  }

  inline const TooN::Matrix<3,3>& Kinv() const
  {
    return _Kinv;
  }

protected:
  TooN::Vector<2> _pp;
  TooN::Vector<2> _f;
  TooN::Matrix<3,3> _K;
  TooN::Matrix<3,3> _Kinv;
};

class FovCamera : public LinearCamera
{
public:

  FovCamera(int w, int h, double fu, double fv, double u0, double v0, double W)
      :LinearCamera(w,h,fu,fv,u0,v0), _W(W)
  {
      if( _W != 0.0 )
      {
        _2tan = 2.0 * tan(_W / 2.0 );
        _1over2tan = 1.0 / _2tan;
        _Winv = 1.0 / _W;
      }else{
        _Winv = 0.0;
        _2tan = 0.0;
      }
  }

  inline double rtrans_factor(double r) const
  {
    if(r < 0.001 || _W == 0.0)
      return 1.0;
    else
      return (_Winv* atan(r * _2tan) / r);
  }

  inline double invrtrans(double r) const
  {
    if(_W == 0.0)
      return r;
    return(tan(r * _W) * _1over2tan);
  }

  inline TooN::Vector<2> map(const TooN::Vector<2>& cam) const
  {
    const double fac = rtrans_factor(norm(cam));

    return TooN::makeVector(
      fac * _f[0] * cam[0] + _pp[0],
      fac * _f[1] * cam[1] + _pp[1]
    );

  }

  inline TooN::Vector<2> unmap(const TooN::Vector<2>& img) const
  {
      const TooN::Vector<2> distCam = TooN::makeVector(
        (img[0] - _pp[0]) / _f[0],
        (img[1] - _pp[1]) / _f[1]
      );
      const double distR = norm(distCam);
      const double R = invrtrans(distR);
      if( distR > 0.01 ) {
          return (R / distR) * distCam;
      }else{
          return distCam;
      }
  }

protected:
  double _W;
  double _Winv;
  double _2tan;
  double _1over2tan;
};

#endif // CAMERA_H
