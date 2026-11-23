// Copyright 2026 rm_vision contributors
// Licensed under the Apache-2.0 License.

#ifndef ARMOR_TRACKER__MEASUREMENT_NOISE_HPP_
#define ARMOR_TRACKER__MEASUREMENT_NOISE_HPP_

#include <algorithm>
#include <cmath>

namespace rm_auto_aim
{

inline double confidenceNoiseScale(
  double confidence, double reference_confidence,
  double min_scale, double max_scale)
{
  if (
    !std::isfinite(confidence) || confidence <= 0.0 ||
    !std::isfinite(reference_confidence) || reference_confidence <= 0.0)
  {
    return 1.0;
  }
  const double ratio = reference_confidence / std::min(confidence, 1.0);
  return std::max(min_scale, std::min(max_scale, ratio * ratio));
}

}  // namespace rm_auto_aim

#endif  // ARMOR_TRACKER__MEASUREMENT_NOISE_HPP_
