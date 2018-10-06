#pragma once

#include <calibu/pcalib/vignetting.h>

namespace calibu
{

/**
 * Vignetting implementation template, which implements the Vignetting interface
 * by calling static functions provided by derived classes.
 */
template <typename Scalar, typename Derived>
class VignettingImpl : public Vignetting<Scalar>
{
  public:

    /**
     * Create vignetting model for given image resolution
     * @param width image width
     * @param height image height
     */
    VignettingImpl(int width, int height) :
      Vignetting<Scalar>(width, height)
    {
      Initialize();
    }

    virtual ~VignettingImpl()
    {
    }

    /**
     * Evaluates the attenuation at the specified point in the image.
     * @param u horizontal image coordinate for point being evaluated
     * @param v vertical image coordinate for point being evaluated
     * @return evaluated attenuation factor at image position
     */
    Scalar operator()(Scalar u, Scalar v) const override
    {
      return Derived::GetVignetting(this->params_.data(), u, v,
          this->width_, this->height_);
    }

    /**
     * Resets the model parameters, which results in uniform attenuation.
     */
    virtual void Reset() override
    {
      Derived::ResetParameters(this->params_.data(),
          this->width_, this->height_);
    }

  private:

    /**
     * Initializes member variables shared by all derived classes
     */
    void Initialize()
    {
      const int w = this->width_;
      const int h = this->height_;
      this->type_ = std::string(Derived::Type);
      this->params_.resize(Derived::GetNumParams(w, h));
      Derived::ResetParameters(this->params_.data(), w, h);
    }
};

} // namespace calibu