// Copyright 2026 rm_vision contributors
// Licensed under the Apache-2.0 License.

#include "yolo26_dnn_detector/opencv_yolo_detector.hpp"
#include "yolo26_dnn_detector/openvino_yolo_detector.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <numeric>
#include <string>
#include <vector>

int main(int argc, char ** argv)
{
  if (argc < 5) {
    std::cerr <<
      "Usage: yolo26_cpu_benchmark IMAGE MODEL CLASSES REPEAT "
      "[THREADS] [OUTPUT] [COLOR: red|blue]\n";
    return 2;
  }
  const std::string image_path = argv[1];
  const std::string model_path = argv[2];
  const std::string classes_path = argv[3];
  const int repeat = std::max(1, std::stoi(argv[4]));
  const int threads = argc > 5 ? std::stoi(argv[5]) : 16;
  const std::string output_path = argc > 6 ? argv[6] : "";
  const int detect_color = argc > 7 && std::string(argv[7]) == "blue" ? 1 : 0;

  const cv::Mat image = cv::imread(image_path);
  if (image.empty()) {
    std::cerr << "Cannot read image: " << image_path << "\n";
    return 1;
  }

  const bool use_openvino =
    model_path.size() >= 4 && model_path.substr(model_path.size() - 4) == ".xml";
  std::unique_ptr<rm_auto_aim::YoloDetectorBase> detector;
  if (use_openvino) {
    detector = std::make_unique<rm_auto_aim::OpenVinoYoloDetector>(
      model_path, classes_path, 0.25F, 0.5F, threads);
  } else {
    detector = std::make_unique<rm_auto_aim::OpenCvYoloDetector>(
      model_path, classes_path, 0.25F, 0.5F, threads);
  }
  for (int index = 0; index < 3; ++index) {
    detector->detect(image, false, detect_color);
  }

  std::vector<double> elapsed;
  std::vector<rm_auto_aim::DnnDetection> detections;
  for (int index = 0; index < repeat; ++index) {
    const auto started = std::chrono::steady_clock::now();
    detections = detector->detect(image, false, detect_color);
    const auto finished = std::chrono::steady_clock::now();
    elapsed.push_back(
      std::chrono::duration<double, std::milli>(finished - started).count());
  }
  std::sort(elapsed.begin(), elapsed.end());
  const double mean = std::accumulate(elapsed.begin(), elapsed.end(), 0.0) / elapsed.size();
  const double p95 =
    elapsed[std::min(elapsed.size() - 1, static_cast<size_t>(elapsed.size() * 0.95))];

  std::cout << "opencv=" << CV_VERSION << " backend=" <<
    (use_openvino ? "openvino" : "opencv") << " threads=" << threads <<
    " mean_ms=" << mean << " p50_ms=" << elapsed[elapsed.size() / 2] <<
    " p95_ms=" << p95 << " fps=" << 1000.0 / mean <<
    " detections=" << detections.size() << "\n";
  for (const auto & detection : detections) {
    std::cout << detector->className(detection.class_id) <<
      " confidence=" << detection.score << "\n";
  }

  if (!output_path.empty()) {
    cv::Mat rendered = image.clone();
    for (const auto & detection : detections) {
      std::vector<cv::Point> points;
      for (const auto & corner : detection.corners) {
        points.emplace_back(cvRound(corner.x), cvRound(corner.y));
      }
      cv::polylines(rendered, points, true, cv::Scalar(0, 0, 255), 2);
    }
    cv::imwrite(output_path, rendered);
  }
  return 0;
}
