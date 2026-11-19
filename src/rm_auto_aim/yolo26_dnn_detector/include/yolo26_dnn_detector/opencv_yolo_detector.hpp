// Copyright 2026 rm_vision contributors
// Licensed under the Apache-2.0 License.

#ifndef YOLO26_DNN_DETECTOR__OPENCV_YOLO_DETECTOR_HPP_
#define YOLO26_DNN_DETECTOR__OPENCV_YOLO_DETECTOR_HPP_

#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>

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

class OpenCvYoloDetector
{
public:
  OpenCvYoloDetector(
    const std::string & model_path, const std::string & classes_path,
    float confidence_threshold, float nms_threshold, int threads);

  std::vector<DnnDetection> detect(const cv::Mat & image, bool input_is_rgb, int detect_color);

  const std::string & className(int class_id) const;

private:
  cv::dnn::Net net_;
  std::vector<std::string> classes_;
  float confidence_threshold_;
  float nms_threshold_;
};

}  // namespace rm_auto_aim

#endif  // YOLO26_DNN_DETECTOR__OPENCV_YOLO_DETECTOR_HPP_
