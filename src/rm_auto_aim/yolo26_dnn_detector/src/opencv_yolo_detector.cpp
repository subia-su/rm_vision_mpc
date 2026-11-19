// Copyright 2026 rm_vision contributors
// Licensed under the Apache-2.0 License.

#include "yolo26_dnn_detector/opencv_yolo_detector.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <numeric>
#include <stdexcept>

namespace rm_auto_aim
{
namespace
{
constexpr int INPUT_SIZE = 640;
constexpr int NUM_CLASSES = 36;
constexpr int NUM_FEATURES = 41;
constexpr int NUM_ANCHORS = 8400;

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

std::string colorPrefix(const std::string & class_name)
{
  const auto separator = class_name.find('_');
  return separator == std::string::npos ? "" : class_name.substr(0, separator);
}

}  // namespace

OpenCvYoloDetector::OpenCvYoloDetector(
  const std::string & model_path, const std::string & classes_path,
  float confidence_threshold, float nms_threshold, int threads)
: confidence_threshold_(confidence_threshold), nms_threshold_(nms_threshold)
{
  cv::setNumThreads(threads);
  net_ = cv::dnn::readNetFromONNX(model_path);
  net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
  net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

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
    throw std::runtime_error(
            "Expected " + std::to_string(NUM_CLASSES) + " classes, got " +
            std::to_string(classes_.size()));
  }
}

std::vector<DnnDetection> OpenCvYoloDetector::detect(
  const cv::Mat & image, bool input_is_rgb, int detect_color)
{
  const float scale = std::min(
    static_cast<float>(INPUT_SIZE) / image.cols,
    static_cast<float>(INPUT_SIZE) / image.rows);
  const int resized_width = std::lround(image.cols * scale);
  const int resized_height = std::lround(image.rows * scale);
  const int left = (INPUT_SIZE - resized_width) / 2;
  const int top = (INPUT_SIZE - resized_height) / 2;

  cv::Mat resized;
  cv::resize(image, resized, cv::Size(resized_width, resized_height));
  cv::Mat canvas(INPUT_SIZE, INPUT_SIZE, CV_8UC3, cv::Scalar(114, 114, 114));
  resized.copyTo(canvas(cv::Rect(left, top, resized_width, resized_height)));

  cv::Mat blob = cv::dnn::blobFromImage(
    canvas, 1.0 / 255.0, cv::Size(INPUT_SIZE, INPUT_SIZE), cv::Scalar(), !input_is_rgb,
    false, CV_32F);
  net_.setInput(blob);
  const cv::Mat output = net_.forward();
  if (
    output.dims != 3 || output.size[1] != NUM_FEATURES ||
    output.size[2] != NUM_ANCHORS)
  {
    throw std::runtime_error("Unexpected YOLO26 output shape");
  }

  const std::string wanted_color = detect_color == 0 ? "red" : "blue";
  std::vector<std::vector<int>> candidates_by_class(NUM_CLASSES);
  for (int anchor = 0; anchor < NUM_ANCHORS; ++anchor) {
    float best_score = confidence_threshold_;
    int best_class = -1;
    for (int class_id = 0; class_id < NUM_CLASSES; ++class_id) {
      if (colorPrefix(classes_[class_id]) != wanted_color) {
        continue;
      }
      const float score = output.ptr<float>(0, 4 + class_id)[anchor];
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
    boxes.reserve(candidates.size());
    scores.reserve(candidates.size());
    for (int anchor : candidates) {
      const float cx = output.ptr<float>(0, 0)[anchor];
      const float cy = output.ptr<float>(0, 1)[anchor];
      const float width = output.ptr<float>(0, 2)[anchor];
      const float height = output.ptr<float>(0, 3)[anchor];
      const float angle = output.ptr<float>(0, 40)[anchor] * 180.0F / CV_PI;
      boxes.emplace_back(cv::Point2f(cx, cy), cv::Size2f(width, height), angle);
      scores.push_back(output.ptr<float>(0, 4 + class_id)[anchor]);
    }
    std::vector<int> class_keep;
    cv::dnn::NMSBoxes(
      boxes, scores, confidence_threshold_, nms_threshold_, class_keep);
    for (int index : class_keep) {
      keep.push_back(candidates[index]);
    }
  }

  std::vector<DnnDetection> detections;
  detections.reserve(keep.size());
  for (int anchor : keep) {
    float best_score = 0.0F;
    int best_class = -1;
    for (int class_id = 0; class_id < NUM_CLASSES; ++class_id) {
      if (colorPrefix(classes_[class_id]) != wanted_color) {
        continue;
      }
      const float score = output.ptr<float>(0, 4 + class_id)[anchor];
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
      (output.ptr<float>(0, 0)[anchor] - left) / scale;
    detection.center_y =
      (output.ptr<float>(0, 1)[anchor] - top) / scale;
    detection.width = output.ptr<float>(0, 2)[anchor] / scale;
    detection.height = output.ptr<float>(0, 3)[anchor] / scale;
    detection.angle = output.ptr<float>(0, 40)[anchor];
    const cv::RotatedRect rotated(
      cv::Point2f(detection.center_x, detection.center_y),
      cv::Size2f(detection.width, detection.height),
      detection.angle * 180.0F / CV_PI);
    cv::Point2f points[4];
    rotated.points(points);
    detection.corners = orderCorners(points);
    detections.push_back(detection);
  }
  return detections;
}

const std::string & OpenCvYoloDetector::className(int class_id) const
{
  return classes_.at(class_id);
}

}  // namespace rm_auto_aim
