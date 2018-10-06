#pragma once

#include <iostream>
#include <Eigen/Eigen>
#include <calibu/calib/exception.h>

namespace calibu
{

/**
 * Abstract vignetting interface, which models image attenuation.
 */
template <typename Scalar>
class Vignetting
{
  public:

    /**
     * Create vignetting model for given image resolution
     * @param width image width
     * @param height image height
     */
    Vignetting(int width, int height) :
      width_(width),
      height_(height)
    {
    }

    virtual ~Vignetting()
    {
    }

    /**
     * Returns the image width of vignetting model
     * @return image width
     */
    inline int Width() const
    {
      return width_;
    }

    /**
     * Returns the image height of vignetting model
     * @return image height
     */
    inline int Height() const
    {
      return height_;
    }

    /**
     * Returns name of the vignetting model type. This will be a unique name for
     * each vignetting model implementation.
     * @return model type name
     */
    inline const std::string& Type() const
    {
      return type_;
    }

    /**
     * Returns the number of parameters in the vignetting model
     * @return number of model parameters
     */
    inline int NumParams() const
    {
      return params_.size();
    }

    /**
     * Returns a constant reference to the model parameters
     * @return current model parameters
     */
    inline const Eigen::VectorXd& GetParams() const
    {
      return params_;
    }

    /**
     * Assigns a new set of model parameters
     * @param params new model parameters to be assigned
     */
    inline void SetParams(const Eigen::VectorXd& params)
    {
      CALIBU_ASSERT_MSG(params.size() == params_.size(), "invalid param count");
      params_ = params;
    }

    /**
     * Evaluates the attenuation at the specified point in the image.
     * @param u horizontal image coordinate for point being evaluated
     * @param v vertical image coordinate for point being evaluated
     * @return evaluated attenuation factor at image position
     */
    virtual Scalar operator()(Scalar u, Scalar v) const = 0;

    /**
     * Resets the model parameters, which results in uniform attenuation.
     */
    virtual void Reset() = 0;

  protected:

    /** Image width of vignetting model */
    int width_;

    /** Image height of vignetting model */
    int height_;

    /** Vignetting type name */
    std::string type_;

    /** Vignetting model parameter vector */
    Eigen::VectorXd params_;
};

} // namespace calibu