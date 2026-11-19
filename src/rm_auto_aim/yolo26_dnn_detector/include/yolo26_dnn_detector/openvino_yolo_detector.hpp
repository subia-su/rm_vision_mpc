// Copyright 2026 rm_vision contributors
// Licensed under the Apache-2.0 License.

#ifndef YOLO26_DNN_DETECTOR__OPENVINO_YOLO_DETECTOR_HPP_
#define YOLO26_DNN_DETECTOR__OPENVINO_YOLO_DETECTOR_HPP_

#include <openvino/openvino.hpp>
#include <opencv2/core.hpp>

#include <string>
#include <vector>

#include "yolo26_dnn_detector/yolo_detector.hpp"

namespace rm_auto_aim
{

class OpenVinoYoloDetector : public YoloDetectorBase
{
public:
  OpenVinoYoloDetector(
    const std::string & model_path, const std::string & classes_path,
    float confidence_threshold, float nms_threshold, int threads);

  std::vector<DnnDetection> detect(
    const cv::Mat & image, bool input_is_rgb, int detect_color) override;

  const std::string & className(int class_id) const override;

private:
  ov::Core core_;
  ov::CompiledModel compiled_model_;
  ov::InferRequest infer_request_;
  std::vector<std::string> classes_;
  float confidence_threshold_;
  float nms_threshold_;
};

}  // namespace rm_auto_aim

#endif  // YOLO26_DNN_DETECTOR__OPENVINO_YOLO_DETECTOR_HPP_
