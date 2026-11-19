// Copyright 2026 rm_vision contributors
// Licensed under the Apache-2.0 License.

#ifndef YOLO26_DNN_DETECTOR__YOLO_DETECTOR_HPP_
#define YOLO26_DNN_DETECTOR__YOLO_DETECTOR_HPP_

#include <opencv2/core.hpp>

#include <array>
#include <string>
#include <vector>

namespace rm_auto_aim
{

struct DnnDetection
{
  int class_id;
  float score;
  float center_x;
  float center_y;
  float width;
  float height;
  float angle;
  std::array<cv::Point2f, 4> corners;
};

class YoloDetectorBase
{
public:
  virtual ~YoloDetectorBase() = default;

  virtual std::vector<DnnDetection> detect(
    const cv::Mat & image, bool input_is_rgb, int detect_color) = 0;
  virtual const std::string & className(int class_id) const = 0;
};

}  // namespace rm_auto_aim

#endif  // YOLO26_DNN_DETECTOR__YOLO_DETECTOR_HPP_
