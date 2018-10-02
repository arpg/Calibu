#pragma once

#include <calibu/pcalib/response.h>

namespace calibu
{

template <typename Scalar, typename Derived>
class ResponseImpl : public Response<Scalar>
{
  public:

    ResponseImpl()
    {
      Initialize();
    }

    virtual ~ResponseImpl()
    {
    }

    Scalar operator()(Scalar value) const override
    {
      return Derived::GetResponse(this->params_.data(), value);
    }

    virtual void Reset() override
    {
      Derived::ResetParameters(this->params_.data());
    }

  private:

    void Initialize()
    {
      this->range_ = Eigen::Vector2d(0, 1);
      this->type_ = std::string(Derived::Type);
      this->params_.resize(Derived::NumParams);
      Derived::ResetParameters(this->params_.data());
    }
};

} // namespace calibu