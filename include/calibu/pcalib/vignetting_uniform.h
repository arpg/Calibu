#pragma once

#include <calibu/pcalib/vignetting_impl.h>

namespace calibu
{

/**
* Uniform vignetting model, used in the trivial case where a camera has
* effectively no attenuation
*/
template <typename Scalar>
class UniformVignetting :
    public VignettingImpl<Scalar, UniformVignetting<Scalar>>
{
  public:

    /** Unique vignetting type name */
    static constexpr const char* type = "uniform";

  public:

    /**
     * Create vignetting model for given image resolution
     * @param width image width
     * @param height image height
     */
    UniformVignetting(int width, int height) :
      VignettingImpl<Scalar, UniformVignetting<Scalar>>(width, height)
    {
    }

    virtual ~UniformVignetting()
    {
    }

    /**
     * Evaluates the attenuation at the specified point in the image.
     * As this model served in the trivial case where the image is uniformly
     * attenuated, an attenuation factor of one is always returned, regardless
     * of the privided arguments.
     * @param params model parameters used for evaluation
     * @param u horizontal image coordinate for point being evaluated
     * @param v vertical image coordinate for point being evaluated
     * @param width image width of model
     * @param height image height of model
     * @return evaluated attenuation factor at image position
     */
    template <typename T>
    static inline T GetAttenuation(const T*, double, double, int, int)
    {
      return T(1);
    }

    /**
     * Resets the model parameters, which results in uniform attenuation.
     * For this specific model, this function does nothing
     * @param params parameter vector to be reset
     * @param width image width of model
     * @param height image height of model
     */
    static inline void ResetParameters(double*, int, int)
    {
    }

    /**
     * Returns the number of parameters needed for the specified image size.
     * This specific model has no parameters, and will always return zero.
     * @param width image width of model
     * @param height image height of model
     * @return number of parameters of the model
     */
    static inline int GetNumParams(int, int)
    {
      return 0;
    }
};

} // namespace calibu