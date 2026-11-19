// Copyright 2026 rm_vision contributors
// Licensed under the Apache-2.0 License.

#ifndef YOLO26_DNN_DETECTOR__OPENCV_YOLO_DETECTOR_HPP_
#define YOLO26_DNN_DETECTOR__OPENCV_YOLO_DETECTOR_HPP_

#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>

#include <string>
#include <vector>

#include "yolo26_dnn_detector/yolo_detector.hpp"

namespace rm_auto_aim
{

class OpenCvYoloDetector : public YoloDetectorBase
{
public:
  OpenCvYoloDetector(
    const std::string & model_path, const std::string & classes_path,
    float confidence_threshold, float nms_threshold, int threads,
    const std::string & model_bin = "", bool use_openvino = false);

  std::vector<DnnDetection> detect(
    const cv::Mat & image, bool input_is_rgb, int detect_color) override;

  const std::string & className(int class_id) const override;

private:
  cv::dnn::Net net_;
  std::vector<std::string> classes_;
  float confidence_threshold_;
  float nms_threshold_;
  bool use_openvino_;
};

}  // namespace rm_auto_aim

#endif  // YOLO26_DNN_DETECTOR__OPENCV_YOLO_DETECTOR_HPP_
