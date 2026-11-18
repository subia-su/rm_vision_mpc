// Copyright (c) 2024 rm_vision contributors
// Licensed under the Apache-2.0 License.

#include "armor_detector/yolo_detector.hpp"

#include <stdexcept>

namespace rm_auto_aim
{

YoloDetector::YoloDetector(
  const std::string &, float, float, const std::vector<std::string> &)
: impl_(nullptr)
{
  throw std::runtime_error(
          "YoloDetector is unavailable because this build has no TensorRT/CUDA support");
}

YoloDetector::~YoloDetector() = default;

std::vector<Armor> YoloDetector::detect(const cv::Mat &, int)
{
  return {};
}

}  // namespace rm_auto_aim
