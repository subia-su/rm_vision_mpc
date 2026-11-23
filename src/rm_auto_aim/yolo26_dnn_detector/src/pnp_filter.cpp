// Copyright 2026 rm_vision contributors
// Licensed under the Apache-2.0 License.

#include "yolo26_dnn_detector/pnp_filter.hpp"

#include <opencv2/calib3d.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace rm_auto_aim
{
namespace
{
double longAxisAngle(double width, double height, double angle)
{
  const double adjusted = width < height ? angle + CV_PI / 2.0 : angle;
  return std::remainder(adjusted, CV_PI);
}
}

PnpFilter::PnpFilter(const PnpFilterConfig & config)
: config_(config)
{
  if (
    config_.min_box_aspect <= 0.0 || config_.max_box_aspect <= config_.min_box_aspect ||
    config_.max_reprojection_error_px <= 0.0 || config_.min_depth_m <= 0.0 ||
    config_.max_depth_m <= config_.min_depth_m)
  {
    throw std::invalid_argument("Invalid PnP filter geometry thresholds");
  }
}

PnpFilterResult PnpFilter::evaluate(
  const std::string & key, double stamp_seconds,
  double box_width, double box_height,
  const cv::Point2f & image_center, double obb_angle,
  const cv::Mat & rotation_vector, const cv::Mat & translation_vector,
  const cv::Mat & camera_matrix, const cv::Mat & distortion,
  const std::vector<cv::Point3f> & object_points,
  const std::vector<cv::Point2f> & image_points)
{
  PnpFilterResult result;
  const double short_edge = std::min(box_width, box_height);
  const double long_edge = std::max(box_width, box_height);
  if (short_edge <= 1.0) {
    result.reason = "box_too_small";
    ++rejected_count_;
    return result;
  }
  const double aspect = long_edge / short_edge;
  if (aspect < config_.min_box_aspect || aspect > config_.max_box_aspect) {
    result.reason = "box_aspect";
    ++rejected_count_;
    return result;
  }

  const double x = translation_vector.at<double>(0);
  const double y = translation_vector.at<double>(1);
  const double z = translation_vector.at<double>(2);
  if (
    !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
    z < config_.min_depth_m || z > config_.max_depth_m)
  {
    result.reason = "invalid_depth";
    ++rejected_count_;
    return result;
  }

  std::vector<cv::Point2f> projected;
  cv::projectPoints(
    object_points, rotation_vector, translation_vector, camera_matrix, distortion,
    projected);
  if (projected.size() != image_points.size()) {
    result.reason = "reprojection_size";
    ++rejected_count_;
    return result;
  }
  for (std::size_t index = 0; index < projected.size(); ++index) {
    result.reprojection_error_px = std::max(
      result.reprojection_error_px, cv::norm(projected[index] - image_points[index]));
  }
  if (
    !std::isfinite(result.reprojection_error_px) ||
    result.reprojection_error_px > config_.max_reprojection_error_px)
  {
    result.reason = "reprojection_error";
    ++rejected_count_;
    return result;
  }

  const auto found = previous_.find(key);
  if (found != previous_.end() && config_.temporal_gate_seconds > 0.0) {
    const auto & previous = found->second;
    const double elapsed = stamp_seconds - previous.stamp_seconds;
    if (elapsed >= 0.0 && elapsed <= config_.temporal_gate_seconds) {
      const cv::Point3d current_position(x, y, z);
      const cv::Point3d delta = current_position - previous.position;
      result.position_jump_m = std::sqrt(delta.dot(delta));
      result.image_jump_px = cv::norm(cv::Point2d(image_center) - previous.image_center);
      const double current_angle = longAxisAngle(box_width, box_height, obb_angle);
      result.angle_jump_rad = std::abs(
        std::remainder(current_angle - previous.angle, CV_PI));

      if (
        result.position_jump_m > config_.max_position_jump_m ||
        result.image_jump_px > config_.max_image_jump_px ||
        result.angle_jump_rad > config_.max_angle_jump_rad)
      {
        result.reason = "temporal_jump";
        ++rejected_count_;
        return result;
      }
    }
  }

  previous_[key] = PreviousState{
    stamp_seconds,
    cv::Point3d(x, y, z),
    cv::Point2d(image_center.x, image_center.y),
    longAxisAngle(box_width, box_height, obb_angle),
  };
  result.accepted = true;
  return result;
}

}  // namespace rm_auto_aim
