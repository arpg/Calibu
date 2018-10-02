#pragma once

#include <calibu/pcalib/vignetting_impl.h>

namespace calibu
{

template <typename Scalar>
class DenseVignetting : public VignettingImpl<Scalar, DenseVignetting<Scalar>>
{
  public:

    static constexpr const char* Type = "dense";

  public:

    DenseVignetting(int width, int height) :
      VignettingImpl<Scalar, DenseVignetting<Scalar>>(width, height)
    {
    }

    virtual ~DenseVignetting()
    {
    }

    template <typename T>
    static inline T GetVignetting(const T* params, double u, double v,
        int width, int height)
    {
      T result = T(0);

      if (u < 0.5 || u > width - 0.5 || v < 0.5 || v > height - 0.5)
      {
        return result;
      }

      const int x0 = u - 0.5;
      const int y0 = v - 0.5;
      const T wx1 = T(u - x0 - 0.5);
      const T wy1 = T(v - y0 - 0.5);
      const T wx0 = T(1) - wx1;
      const T wy0 = T(1) - wy1;

      result += wy0 * wx0 * params[(y0 + 0) * width + (x0 + 0)];
      result += wy0 * wx1 * params[(y0 + 0) * width + (x0 + 1)];
      result += wy1 * wx0 * params[(y0 + 1) * width + (x0 + 0)];
      result += wy1 * wx1 * params[(y0 + 1) * width + (x0 + 1)];

      return result;
    }

    static inline void ResetParameters(double* params, int width, int height)
    {
      const int count = GetNumParams(width, height);
      Eigen::Map<Eigen::VectorXd> x(params, count);
      x.setOnes();
    }

    static inline int GetNumParams(int width, int height)
    {
      return width * height;
    }
};

} // namespace calibu