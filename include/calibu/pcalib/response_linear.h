#pragma once

#include <calibu/pcalib/response_impl.h>

namespace calibu
{

/**
 * Linear camera response model, used in the trivial case where the
 * image intensities are always proportional to scene irradiance.
 */
template <typename Scalar>
class LinearResponse : public ResponseImpl<Scalar, LinearResponse<Scalar>>
{
  public:

    /** Number of parameters used for model */
    static const int NumParams = 0;

    /** Unique vignetting type name */
    static constexpr const char* Type = "linear";

  public:

    /**
     * Create inverse-response model
     */
    LinearResponse()
    {
    }

    virtual ~LinearResponse()
    {
    }

    /**
     * Evaluates the inverse-response for the given pixel intensity.
     * As this model served in the trivial case where the image is uniformly
     * attenuated, an attenuation factor of one is always returned, regardless
     * of the privided arguments.
     * @param params model parameters used for evaluation
     * @param value pixel intensity
     * @return evaluated inverse-response
     */
    template <typename T>
    static inline T GetResponse(const T*, T value)
    {
      return value;
    }

    /**
     * Resets the model parameters, which results in a linear response.
     * For this specific model, this function does nothing
     * @param params parameter vector to be reset
     * @param range min and max range of image intensities
     */
    template <typename T>
    static inline void ResetParameters(T*)
    {
    }
};

} // namespace calibu