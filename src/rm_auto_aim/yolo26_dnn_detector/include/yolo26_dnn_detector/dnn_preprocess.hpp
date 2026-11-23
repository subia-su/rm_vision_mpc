// Copyright 2026 rm_vision contributors
// Licensed under the Apache-2.0 License.

#ifndef YOLO26_DNN_DETECTOR__DNN_PREPROCESS_HPP_
#define YOLO26_DNN_DETECTOR__DNN_PREPROCESS_HPP_

#include <opencv2/core.hpp>

namespace rm_auto_aim
{

struct DnnInput
{
  cv::Mat blob;
  float scale = 1.0F;
  int pad_left = 0;
  int pad_top = 0;
  bool fast_path = false;
};

DnnInput prepareDnnInput(const cv::Mat & image, bool input_is_rgb);

}  // namespace rm_auto_aim

#endif  // YOLO26_DNN_DETECTOR__DNN_PREPROCESS_HPP_
