// Copyright 2026 rm_vision contributors
// Licensed under the Apache-2.0 License.

#ifndef YOLO26_DNN_DETECTOR__PNP_FILTER_HPP_
#define YOLO26_DNN_DETECTOR__PNP_FILTER_HPP_

#include <opencv2/core.hpp>

#include <cstddef>
#include <string>
#include <unordered_map>
#include <vector>

namespace rm_auto_aim
{

struct PnpFilterConfig
{
  double max_reprojection_error_px = 8.0;
  double min_box_aspect = 1.2;
  double max_box_aspect = 6.5;
  double min_depth_m = 0.05;
  double max_depth_m = 12.0;
  double temporal_gate_seconds = 0.15;
  double max_position_jump_m = 1.5;
  double max_image_jump_px = 220.0;
  double max_angle_jump_rad = 1.4;
};

struct PnpFilterResult
{
  bool accepted = false;
  double reprojection_error_px = 0.0;
  double position_jump_m = 0.0;
  double image_jump_px = 0.0;
  double angle_jump_rad = 0.0;
  std::string reason;
};

class PnpFilter
{
public:
  explicit PnpFilter(const PnpFilterConfig & config);

  PnpFilterResult evaluate(
    const std::string & key, double stamp_seconds,
    double box_width, double box_height,
    const cv::Point2f & image_center, double obb_angle,
    const cv::Mat & rotation_vector, const cv::Mat & translation_vector,
    const cv::Mat & camera_matrix, const cv::Mat & distortion,
    const std::vector<cv::Point3f> & object_points,
    const std::vector<cv::Point2f> & image_points);

  std::size_t rejectedCount() const { return rejected_count_; }

private:
  struct PreviousState
  {
    double stamp_seconds;
    cv::Point3d position;
    cv::Point2d image_center;
    double angle;
  };

  PnpFilterConfig config_;
  std::unordered_map<std::string, PreviousState> previous_;
  std::size_t rejected_count_ = 0;
};

}  // namespace rm_auto_aim

#endif  // YOLO26_DNN_DETECTOR__PNP_FILTER_HPP_
