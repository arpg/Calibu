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

  LinearCamera(int w, int h, double u0, double v0, double fu, double fv)
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

  inline TooN::Vector<2> map(const TooN::Vector<2>& cam) const
  {
    return TooN::makeVector(
      _f[0] * cam[0] + _pp[0],
      _f[1] * cam[1] + _pp[1]
    );
  }

  inline TooN::Vector<2> unmap(const TooN::Vector<2>& img) const
  {
    return TooN::makeVector(
      (img[0] - _pp[0]) / _f[0],
      (img[1] - _pp[1]) / _f[1]
    );
  }

  inline virtual const TooN::Matrix<3,3>& K() const
  {
    return _K;
  }

  inline virtual const TooN::Matrix<3,3>& Kinv() const
  {
    return _Kinv;
  }

protected:
  TooN::Vector<2> _pp;
  TooN::Vector<2> _f;
  TooN::Matrix<3,3> _K;
  TooN::Matrix<3,3> _Kinv;
};

#endif // CAMERA_H
