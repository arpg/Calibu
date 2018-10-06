#pragma once

#include <iostream>
#include <Eigen/Eigen>
#include <calibu/calib/exception.h>

namespace calibu
{

template <typename Scalar>
class Response
{
  public:

    /**
     * Create inverse-response model
     */
    Response()
    {
    }

    virtual ~Response()
    {
    }

    /**
     * Returns name of the response model type. This will be a unique name for
     * each response model implementation.
     * @return model type name
     */
    inline const std::string& Type() const
    {
      return type_;
    }

    /**
     * Returns the number of parameters in the response model
     * @return number of model parameters
     */
    inline int NumParams() const
    {
      return params_.size();
    }

    /**
     * Determines if given intensity value is within value response range
     * @param value intensity value in question
     * @return true if within range
     */
    inline bool InRange(double value) const
    {
      return value >= range_[0] && value <= range_[1];
    }

    /**
     * Returns the modeled pixel intensity range. The minimum intensity is
     * stored at index-0 and the maximum at index-1. These values can be used to
     * adapt the current model, when processing images with different formats.
     * @return pixel intensity range
     */
    inline const Eigen::Vector2d& GetRange() const
    {
      return range_;
    }

    /**
     * Sets the minimum and maximum pixel intensities. For example, the standard
     * unsigned, 8-bit color, a range of [0..255] would be used. In the case of
     * 32-bit floating point color, a range of [0..1] could be used.
     * @param min minimum intensity value
     * @param max maximum intensity value
     */
    inline void SetRange(double min, double max)
    {
      SetRange(Eigen::Vector2d(min, max));
    }

    /**
     * Sets the minimum and maximum pixel intensities. For example, the standard
     * unsigned, 8-bit color, a range of [0..255] would be used. In the case of
     * 32-bit floating point color, a range of [0..1] could be used. The minimum
     * intensity is stored at index-0 and the maximum at index-1.
     * @param range 2D vector specifying min and max intensities, respectively
     */
    inline void SetRange(const Eigen::Vector2d& range)
    {
      range_ = range;
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
     * Evaluates the inverse-response for the given pixel intensity.
     * @param value pixel intensity
     * @return evaluated inverse-response
     */
    virtual Scalar operator()(Scalar value) const = 0;

    /**
     * Resets the model parameters, which results in a linear response.
     */
    virtual void Reset() = 0;

  protected:

    /** Response type name */
    std::string type_;

    /** Pixel intensity range */
    Eigen::Vector2d range_;

    /** Response model parameter vector */
    Eigen::VectorXd params_;
};

} // namespace calibu