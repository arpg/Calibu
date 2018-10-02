#pragma once

#include <calibu/pcalib/vignetting.h>

namespace calibu
{

template <typename Scalar, typename Derived>
class VignettingImpl : public Vignetting<Scalar>
{
  public:

    VignettingImpl(int width, int height) :
      Vignetting<Scalar>(width, height)
    {
      Initialize();
    }

    virtual ~VignettingImpl()
    {
    }

    Scalar operator()(Scalar u, Scalar v) const override
    {
      return Derived::GetVignetting(this->params_.data(), u, v,
          this->width_, this->height_);
    }

    virtual void Reset() override
    {
      Derived::ResetParameters(this->params_.data(),
          this->width_, this->height_);
    }

  private:

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