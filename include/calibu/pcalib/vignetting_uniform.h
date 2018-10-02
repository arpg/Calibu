#pragma once

#include <calibu/pcalib/vignetting_impl.h>

namespace calibu
{

template <typename Scalar>
class UniformVignetting :
    public VignettingImpl<Scalar, UniformVignetting<Scalar>>
{
  public:

    static constexpr const char* Type = "uniform";

  public:

    UniformVignetting(int width, int height) :
      VignettingImpl<Scalar, UniformVignetting<Scalar>>(width, height)
    {
    }

    virtual ~UniformVignetting()
    {
    }

    template <typename T>
    static inline T GetVignetting(const T*, double, double, int, int)
    {
      return T(1);
    }

    static inline void ResetParameters(double*, int, int)
    {
    }

    static inline int GetNumParams(int, int)
    {
      return 0;
    }
};

} // namespace calibu