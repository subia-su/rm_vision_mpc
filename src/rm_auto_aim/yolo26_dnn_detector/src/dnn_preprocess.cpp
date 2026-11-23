// Copyright 2026 rm_vision contributors
// Licensed under the Apache-2.0 License.

#include "yolo26_dnn_detector/dnn_preprocess.hpp"

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>

namespace rm_auto_aim
{
namespace
{
constexpr int INPUT_SIZE = 640;
}

DnnInput prepareDnnInput(const cv::Mat & image, bool input_is_rgb)
{
  DnnInput result;
  if (
    image.cols == INPUT_SIZE && image.rows == INPUT_SIZE &&
    image.type() == CV_8UC3 && image.isContinuous())
  {
    result.blob = cv::dnn::blobFromImage(
      image, 1.0 / 255.0, cv::Size(INPUT_SIZE, INPUT_SIZE), cv::Scalar(),
      !input_is_rgb, false, CV_32F);
    result.fast_path = true;
    return result;
  }

  result.scale = std::min(
    static_cast<float>(INPUT_SIZE) / image.cols,
    static_cast<float>(INPUT_SIZE) / image.rows);
  const int resized_width = std::lround(image.cols * result.scale);
  const int resized_height = std::lround(image.rows * result.scale);
  result.pad_left = (INPUT_SIZE - resized_width) / 2;
  result.pad_top = (INPUT_SIZE - resized_height) / 2;

  cv::Mat resized;
  cv::resize(image, resized, cv::Size(resized_width, resized_height));
  cv::Mat canvas(INPUT_SIZE, INPUT_SIZE, CV_8UC3, cv::Scalar(114, 114, 114));
  resized.copyTo(
    canvas(cv::Rect(result.pad_left, result.pad_top, resized_width, resized_height)));
  result.blob = cv::dnn::blobFromImage(
    canvas, 1.0 / 255.0, cv::Size(INPUT_SIZE, INPUT_SIZE), cv::Scalar(),
    !input_is_rgb, false, CV_32F);
  return result;
}

}  // namespace rm_auto_aim
