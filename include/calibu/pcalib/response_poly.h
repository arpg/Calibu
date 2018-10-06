#pragma once

#include <calibu/pcalib/response_impl.h>

namespace calibu
{

/**
 * 3rd order polynomial inverse-response model.
 */
template <typename Scalar>
class Poly3Response : public ResponseImpl<Scalar, Poly3Response<Scalar>>
{
  public:

    /** Number of parameters used for model */
    static const int param_count = 3;

    /** Unique inverse-response type name */
    static constexpr const char* type = "poly3";

  public:

    /**
     * Create inverse-response model
     */
    Poly3Response()
    {
    }

    virtual ~Poly3Response()
    {
    }

    /**
     * Evaluates the inverse-response for the given pixel intensity.
     * @param params model parameters used for evaluation
     * @param value pixel intensity
     * @return evaluated inverse-response
     */
    template <typename T>
    static inline T GetResponse(const T* params, T value)
    {
      // initialize attenuation

      T pow = value;
      T result = T(0);

      // add each term of the polynomial

      for (int i = 0; i < param_count; ++i)
      {
        result += pow * params[i];
        pow *= value;
      }

      return result;
    }

    /**
     * Resets the model parameters, which results in a linear response.
     * @param params parameter vector to be reset
     * @param range min and max range of image intensities
     */
    template <typename T>
    static inline void ResetParameters(T* params)
    {
      Eigen::Map<Eigen::Vector3d> x(params);
      x = Eigen::Vector3d(1, 0, 0);
    }
};

/**
 * 4th order polynomial inverse-response model.
 */
template <typename Scalar>
class Poly4Response : public ResponseImpl<Scalar, Poly4Response<Scalar>>
{
  public:

    /** Number of parameters used for model */
    static const int param_count = 4;

    /** Unique inverse-response type name */
    static constexpr const char* type = "poly4";

  public:

    /**
     * Create inverse-response model
     */
    Poly4Response()
    {
    }

    virtual ~Poly4Response()
    {
    }

    /**
     * Evaluates the inverse-response for the given pixel intensity.
     * @param params model parameters used for evaluation
     * @param value pixel intensity
     * @return evaluated inverse-response
     */
    template <typename T>
    static inline T GetResponse(const T* params, T value)
    {
      // initialize attenuation

      T pow = value;
      T result = T(0);

      // add each term of the polynomial

      for (int i = 0; i < param_count; ++i)
      {
        result += pow * params[i];
        pow *= value;
      }

      return result;
    }

    /**
     * Resets the model parameters, which results in a linear response.
     * @param params parameter vector to be reset
     * @param range min and max range of image intensities
     */
    template <typename T>
    static inline void ResetParameters(T* params)
    {
      Eigen::Map<Eigen::Vector4d> x(params);
      x = Eigen::Vector4d(1, 0, 0, 0);
    }
};

} // namespace calibu