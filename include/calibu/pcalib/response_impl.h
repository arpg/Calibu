#pragma once

#include <calibu/pcalib/response.h>

namespace calibu
{

/**
 * Inverse camera response implementation template, which implements the
 * Response interface by calling static functions provided by derived classes.
 */
template <typename Scalar, typename Derived>
class ResponseImpl : public Response<Scalar>
{
  public:

    /**
     * Create inverse-response model
     */
    ResponseImpl()
    {
      Initialize();
    }

    virtual ~ResponseImpl()
    {
    }

    /**
     * Evaluates the inverse-response for the given pixel intensity.
     * @param value pixel intensity
     * @return evaluated inverse-response
     */
    Scalar operator()(Scalar value) const override
    {
      return Derived::GetResponse(this->params_.data(), value);
    }

    /**
     * Resets the model parameters, which results in a linear response.
     */
    virtual void Reset() override
    {
      Derived::ResetParameters(this->params_.data());
    }

  private:

    /**
     * Initializes member variables shared by all derived classes
     */
    void Initialize()
    {
      this->range_ = Eigen::Vector2d(0, 1);
      this->type_ = std::string(Derived::Type);
      this->params_.resize(Derived::NumParams);
      Derived::ResetParameters(this->params_.data());
    }
};

} // namespace calibu