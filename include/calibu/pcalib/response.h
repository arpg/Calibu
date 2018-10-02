#pragma once

#include <iostream>
#include <Eigen/Eigen>

namespace calibu
{

template <typename Scalar>
class Response
{
  public:

    Response()
    {
    }

    virtual ~Response()
    {
    }

    inline const std::string& Type() const
    {
      return type_;
    }

    inline int NumParams() const
    {
      return params_.size();
    }

    inline const Eigen::Vector2d& GetRange() const
    {
      return range_;
    }

    inline void SetRange(double min, double max)
    {
      SetRange(Eigen::Vector2d(min, max));
    }

    inline void SetRange(const Eigen::Vector2d& range)
    {
      range_ = range;
    }

    inline const Eigen::VectorXd& GetParams() const
    {
      return params_;
    }

    inline void SetParams(const Eigen::VectorXd& params)
    {
      if (params.size() != params_.size())
      {
        std::cerr << "invalid parameter count" << std::endl;
        throw 0;
      }

      params_ = params;
    }

    virtual Scalar operator()(Scalar value) const = 0;

    virtual void Reset() = 0;

  protected:

    std::string type_;

    Eigen::Vector2d range_;

    Eigen::VectorXd params_;
};

} // namespace calibu