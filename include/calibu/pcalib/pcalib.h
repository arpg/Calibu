#pragma once

#include <memory>
#include <vector>
#include <calibu/pcalib/response.h>
#include <calibu/pcalib/vignetting.h>

namespace calibu
{

template <typename Scalar>
struct PhotoCalib
{
  inline void Clear()
  {
    responses.clear();
    vignettings.clear();
  }

  std::vector<std::shared_ptr<Response<Scalar>>> responses;

  std::vector<std::shared_ptr<Vignetting<Scalar>>> vignettings;
};

typedef PhotoCalib<double> PhotoCalibd;

} // namespace calibu