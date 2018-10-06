#pragma once

#include <calibu/pcalib/vignetting_impl.h>

namespace calibu
{

/**
 * Dense vignetting model, with one attenuation factor specified per pixel
 */
template <typename Scalar>
class DenseVignetting : public VignettingImpl<Scalar, DenseVignetting<Scalar>>
{
  public:

    /** Unique vignetting type name */
    static constexpr const char* Type = "dense";

  public:

    /**
     * Create vignetting model for given image resolution
     * @param width image width
     * @param height image height
     */
    DenseVignetting(int width, int height) :
      VignettingImpl<Scalar, DenseVignetting<Scalar>>(width, height)
    {
    }

    virtual ~DenseVignetting()
    {
    }

    /**
     * Evaluates the attenuation at the specified point in the image.
     * The attuentation factor is retrieved from a look up table, using bilinear
     * interpolation. If the point is outside the image an attenuation factor of
     * zero is returned.
     * @param params model parameters used for evaluation
     * @param u horizontal image coordinate for point being evaluated
     * @param v vertical image coordinate for point being evaluated
     * @param width image width of model
     * @param height image height of model
     * @return evaluated attenuation factor at image position
     */
    template <typename T>
    static inline T GetVignetting(const T* params, double u, double v,
        int width, int height)
    {
      // initialize attenuation

      T result = T(0);

      // check if given position outside of image

      if (u < 0.5 || u > width - 0.5 || v < 0.5 || v > height - 0.5)
      {
        return result;
      }

      // compute values and weigths for bilinear interpolation

      const int x0 = u - 0.5;
      const int y0 = v - 0.5;
      const T wx1 = T(u - x0 - 0.5);
      const T wy1 = T(v - y0 - 0.5);
      const T wx0 = T(1) - wx1;
      const T wy0 = T(1) - wy1;

      // perform final bilinear interpolation of all attenuation factors

      result += wy0 * wx0 * params[(y0 + 0) * width + (x0 + 0)];
      result += wy0 * wx1 * params[(y0 + 0) * width + (x0 + 1)];
      result += wy1 * wx0 * params[(y0 + 1) * width + (x0 + 0)];
      result += wy1 * wx1 * params[(y0 + 1) * width + (x0 + 1)];

      return result;
    }

    /**
     * Resets the model parameters, which results in uniform attenuation
     * @param params parameter vector to be reset
     * @param width image width of model
     * @param height image height of model
     */
    static inline void ResetParameters(double* params, int width, int height)
    {
      const int count = GetNumParams(width, height);
      Eigen::Map<Eigen::VectorXd> x(params, count);
      x.setOnes();
    }

    /**
     * Returns the number of parameters needed for the specified image size
     * @param width image width of model
     * @param height image height of model
     * @return number of parameters of the model
     */
    static inline int GetNumParams(int width, int height)
    {
      return width * height;
    }
};

} // namespace calibu