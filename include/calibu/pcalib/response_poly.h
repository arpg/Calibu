#pragma once

#include <calibu/pcalib/response_impl.h>

namespace calibu
{

template <typename Scalar>
class Poly3Response : public ResponseImpl<Scalar, Poly3Response<Scalar>>
{
  public:

    static const int NumParams = 3;

    static constexpr const char* Type = "poly3";

  public:

    Poly3Response()
    {
    }

    virtual ~Poly3Response()
    {
    }

    template <typename T>
    static inline T GetResponse(const T* params, T value)
    {
      T pow = value;
      T result = T(0);

      for (int i = 0; i < NumParams; ++i)
      {
        result += pow * params[i];
        pow *= value;
      }

      return result;
    }

    template <typename T>
    static inline void ResetParameters(T* params)
    {
      Eigen::Map<Eigen::Vector3d> x(params);
      x = Eigen::Vector3d(1, 0, 0);
    }
};

template <typename Scalar>
class Poly4Response : public ResponseImpl<Scalar, Poly4Response<Scalar>>
{
  public:

    static const int NumParams = 4;

    static constexpr const char* Type = "poly4";

  public:

    Poly4Response()
    {
    }

    virtual ~Poly4Response()
    {
    }

    template <typename T>
    static inline T GetResponse(const T* params, T value)
    {
      T pow = value;
      T result = T(0);

      for (int i = 0; i < NumParams; ++i)
      {
        result += pow * params[i];
        pow *= value;
      }

      return result;
    }

    template <typename T>
    static inline void ResetParameters(T* params)
    {
      Eigen::Map<Eigen::Vector4d> x(params);
      x = Eigen::Vector4d(1, 0, 0, 0);
    }
};

} // namespace calibu