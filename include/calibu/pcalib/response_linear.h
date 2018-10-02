#pragma once

#include <calibu/pcalib/response_impl.h>

namespace calibu
{

template <typename Scalar>
class LinearResponse : public ResponseImpl<Scalar, LinearResponse<Scalar>>
{
  public:

    static const int NumParams = 0;

    static constexpr const char* Type = "linear";

  public:

    LinearResponse()
    {
    }

    virtual ~LinearResponse()
    {
    }

    template <typename T>
    static inline T GetResponse(const T*, T value)
    {
      return value;
    }

    template <typename T>
    static inline void ResetParameters(T*)
    {
    }
};

} // namespace calibu