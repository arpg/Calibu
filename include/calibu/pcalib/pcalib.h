#pragma once

#include <memory>
#include <vector>
#include <calibu/pcalib/response.h>
#include <calibu/pcalib/vignetting.h>

namespace calibu
{

/**
 * Represents the photometric calibration of a single camera.
 * It is a collection of camera responses and attenuation factors (vignetting).
 * If multiple response models are provided, it is assumed each response
 * pertains to each color channel. If only one response is given, then it
 * pertains to all color channels. The same is true for vignetting.
 */
template <typename Scalar>
struct PhotoCalib
{
  /**
   * Clears the response and vignetting vectors
   */
  inline void Clear()
  {
    responses.clear();
    vignettings.clear();
  }

  /** Ordered list of response models */
  std::vector<std::shared_ptr<Response<Scalar>>> responses;

  /** Ordered list of vignetting models */
  std::vector<std::shared_ptr<Vignetting<Scalar>>> vignettings;
};

typedef PhotoCalib<double> PhotoCalibd;

} // namespace calibu