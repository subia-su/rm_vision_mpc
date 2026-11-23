// Copyright 2026 rm_vision contributors
// Licensed under the Apache-2.0 License.

#include "yolo26_dnn_detector/openvino_yolo_detector.hpp"

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>

#include "yolo26_dnn_detector/dnn_preprocess.hpp"

namespace rm_auto_aim
{
namespace
{
constexpr int INPUT_SIZE = 640;
constexpr int NUM_CLASSES = 36;
constexpr int NUM_FEATURES = 41;
constexpr int NUM_ANCHORS = 8400;

std::string colorPrefix(const std::string & class_name)
{
  const auto separator = class_name.find('_');
  return separator == std::string::npos ? "" : class_name.substr(0, separator);
}

std::array<cv::Point2f, 4> orderCorners(const cv::Point2f input[4])
{
  std::array<cv::Point2f, 4> points{input[0], input[1], input[2], input[3]};
  std::sort(points.begin(), points.end(), [](const cv::Point2f & a, const cv::Point2f & b) {
    return a.x < b.x;
  });
  const cv::Point2f left_top = points[0].y < points[1].y ? points[0] : points[1];
  const cv::Point2f left_bottom = points[0].y < points[1].y ? points[1] : points[0];
  const cv::Point2f right_top = points[2].y < points[3].y ? points[2] : points[3];
  const cv::Point2f right_bottom = points[2].y < points[3].y ? points[3] : points[2];
  return {left_bottom, left_top, right_top, right_bottom};
}

float feature(const float * output, int row, int anchor)
{
  return output[row * NUM_ANCHORS + anchor];
}

}  // namespace

OpenVinoYoloDetector::OpenVinoYoloDetector(
  const std::string & model_path, const std::string & classes_path,
  float confidence_threshold, float nms_threshold, int threads)
: confidence_threshold_(confidence_threshold), nms_threshold_(nms_threshold)
{
  const ov::AnyMap properties{
    ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY),
    ov::inference_num_threads(threads),
    ov::num_streams(1),
    ov::hint::enable_cpu_pinning(true),
  };
  compiled_model_ = core_.compile_model(model_path, "CPU", properties);
  infer_request_ = compiled_model_.create_infer_request();

  std::ifstream stream(classes_path);
  if (!stream.is_open()) {
    throw std::runtime_error("Cannot open YOLO26 classes file: " + classes_path);
  }
  for (std::string line; std::getline(stream, line);) {
    if (!line.empty()) {
      classes_.push_back(line);
    }
  }
  if (classes_.size() != NUM_CLASSES) {
    throw std::runtime_error("Expected 36 YOLO26 classes");
  }
}

std::vector<DnnDetection> OpenVinoYoloDetector::detect(
  const cv::Mat & image, bool input_is_rgb, int detect_color)
{
  const DnnInput prepared = prepareDnnInput(image, input_is_rgb);

  ov::Tensor input = infer_request_.get_input_tensor();
  if (input.get_size() != static_cast<size_t>(3 * INPUT_SIZE * INPUT_SIZE)) {
    throw std::runtime_error("Unexpected OpenVINO input tensor size");
  }
  std::copy_n(prepared.blob.ptr<float>(), input.get_size(), input.data<float>());
  infer_request_.infer();

  const ov::Tensor output_tensor = infer_request_.get_output_tensor(0);
  const auto shape = output_tensor.get_shape();
  if (
    shape.size() != 3 || shape[1] != NUM_FEATURES ||
    shape[2] != NUM_ANCHORS)
  {
    throw std::runtime_error("Unexpected YOLO26 OpenVINO output shape");
  }
  const float * output = output_tensor.data<const float>();
  const std::string wanted_color = detect_color == 0 ? "red" : "blue";

  std::vector<std::vector<int>> candidates_by_class(NUM_CLASSES);
  for (int anchor = 0; anchor < NUM_ANCHORS; ++anchor) {
    float best_score = confidence_threshold_;
    int best_class = -1;
    for (int class_id = 0; class_id < NUM_CLASSES; ++class_id) {
      if (colorPrefix(classes_[class_id]) != wanted_color) {
        continue;
      }
      const float score = feature(output, 4 + class_id, anchor);
      if (score > best_score) {
        best_score = score;
        best_class = class_id;
      }
    }
    if (best_class >= 0) {
      candidates_by_class[best_class].push_back(anchor);
    }
  }

  std::vector<int> keep;
  for (int class_id = 0; class_id < NUM_CLASSES; ++class_id) {
    const auto & candidates = candidates_by_class[class_id];
    if (candidates.empty()) {
      continue;
    }
    std::vector<cv::RotatedRect> boxes;
    std::vector<float> scores;
    for (int anchor : candidates) {
      boxes.emplace_back(
        cv::Point2f(feature(output, 0, anchor), feature(output, 1, anchor)),
        cv::Size2f(feature(output, 2, anchor), feature(output, 3, anchor)),
        feature(output, 40, anchor) * 180.0F / CV_PI);
      scores.push_back(feature(output, 4 + class_id, anchor));
    }
    std::vector<int> class_keep;
    cv::dnn::NMSBoxes(
      boxes, scores, confidence_threshold_, nms_threshold_, class_keep);
    for (int index : class_keep) {
      keep.push_back(candidates[index]);
    }
  }

  std::vector<DnnDetection> detections;
  for (int anchor : keep) {
    float best_score = 0.0F;
    int best_class = -1;
    for (int class_id = 0; class_id < NUM_CLASSES; ++class_id) {
      if (colorPrefix(classes_[class_id]) != wanted_color) {
        continue;
      }
      const float score = feature(output, 4 + class_id, anchor);
      if (score > best_score) {
        best_score = score;
        best_class = class_id;
      }
    }
    if (best_class < 0) {
      continue;
    }

    DnnDetection detection;
    detection.class_id = best_class;
    detection.score = best_score;
    detection.center_x =
      (feature(output, 0, anchor) - prepared.pad_left) / prepared.scale;
    detection.center_y =
      (feature(output, 1, anchor) - prepared.pad_top) / prepared.scale;
    detection.width = feature(output, 2, anchor) / prepared.scale;
    detection.height = feature(output, 3, anchor) / prepared.scale;
    detection.angle = feature(output, 40, anchor);
    const cv::RotatedRect rect(
      cv::Point2f(detection.center_x, detection.center_y),
      cv::Size2f(detection.width, detection.height),
      detection.angle * 180.0F / CV_PI);
    cv::Point2f points[4];
    rect.points(points);
    detection.corners = orderCorners(points);
    detections.push_back(detection);
  }
  return detections;
}

const std::string & OpenVinoYoloDetector::className(int class_id) const
{
  return classes_.at(class_id);
}

}  // namespace rm_auto_aim
