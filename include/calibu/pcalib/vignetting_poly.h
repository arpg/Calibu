#pragma once

#include <calibu/pcalib/vignetting_impl.h>

namespace calibu
{

/**
 * 6th order even polynomial vignetting model as described in the work:
 * Alexandrov, S. V., Prankl, J., Zillich, M., & Vincze, M. (2016).
 * Calibration and correction of vignetting effects with an application to 3D
 * mapping. International Conference on Intelligent Robots and Systems
 */
template <typename Scalar>
class EvenPoly6Vignetting :
    public VignettingImpl<Scalar, EvenPoly6Vignetting<Scalar>>
{
  public:

    /** Unique vignetting type name */
    static constexpr const char* type = "epoly6";

  public:

    /**
     * Create vignetting model for given image resolution
     * @param width image width
     * @param height image height
     */
    EvenPoly6Vignetting(int width, int height) :
      VignettingImpl<Scalar, EvenPoly6Vignetting<Scalar>>(width, height)
    {
    }

    virtual ~EvenPoly6Vignetting()
    {
    }

    /**
     * Evaluates the attenuation at the specified point in the image.
     * The evaluated point radius is computed relative to the image diagonal,
     * so a point at any corner of the image would have a radius of one.
     * @param params model parameters used for evaluation
     * @param u horizontal image coordinate for point being evaluated
     * @param v vertical image coordinate for point being evaluated
     * @param width image width of model
     * @param height image height of model
     * @return evaluated attenuation factor at image position
     */
    template <typename T>
    static inline T GetAttenuation(const T* params, double u, double v,
        int width, int height)
    {
      // initialize attenuation

      T result = T(1);
      const Eigen::Vector2d point(u, v);
      const Eigen::Vector2d size(width, height);
      const Eigen::Vector2d center = 0.5 * size;
      const double radius = (point - center).norm();
      const double max_radius = center.norm();
      const double ratio = radius / max_radius;
      const T rr = ratio * ratio;
      T pow = rr;

      // add each term of the polynomial

      for (int i = 0; i < 3; ++i)
      {
        result += pow * params[i];
        pow *= rr;
      }

      return result;
    }

    /**
     * Resets the model parameters, which results in uniform attenuation
     * @param params parameter vector to be reset
     * @param width image width of model
     * @param height image height of model
     */
    static inline void ResetParameters(double* params, int, int)
    {
      Eigen::Map<Eigen::Vector3d> x(params);
      x = Eigen::Vector3d(0, 0, 0);
    }

    /**
     * Returns the number of parameters needed for the specified image size
     * @param width image width of model
     * @param height image height of model
     * @return number of parameters of the model
     */
    static inline int GetNumParams(int, int)
    {
      return 3;
    }
};

} // namespace calibu