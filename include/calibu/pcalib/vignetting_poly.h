#pragma once

#include <calibu/pcalib/vignetting_impl.h>

namespace calibu
{

template <typename Scalar>
class EvenPoly6Vignetting :
    public VignettingImpl<Scalar, EvenPoly6Vignetting<Scalar>>
{
  public:

    static constexpr const char* Type = "epoly6";

  public:

    EvenPoly6Vignetting(int width, int height) :
      VignettingImpl<Scalar, EvenPoly6Vignetting<Scalar>>(width, height)
    {
    }

    virtual ~EvenPoly6Vignetting()
    {
    }

    template <typename T>
    static inline T GetVignetting(const T* params, double u, double v, int, int)
    {
      T result = T(1);
      const T rr = u * u + v * v;
      T pow = rr;

      for (int i = 0; i < 3; ++i)
      {
        result += pow * params[i];
        pow *= rr;
      }

      return result;
    }

    static inline void ResetParameters(double* params, int, int)
    {
      Eigen::Map<Eigen::Vector3d> x(params);
      x = Eigen::Vector3d(1, 0, 0);
    }

    static inline int GetNumParams(int, int)
    {
      return 3;
    }
};

} // namespace calibu