// Copyright 2026 rm_vision contributors
// Licensed under the Apache-2.0 License.

#include <gtest/gtest.h>

#include <opencv2/calib3d.hpp>

#include <vector>

#include "yolo26_dnn_detector/pnp_filter.hpp"

namespace
{
std::vector<cv::Point3f> objectPoints()
{
  return {
    {0.0F, 0.0675F, -0.0275F},
    {0.0F, 0.0675F, 0.0275F},
    {0.0F, -0.0675F, 0.0275F},
    {0.0F, -0.0675F, -0.0275F},
  };
}

cv::Mat cameraMatrix()
{
  return (cv::Mat_<double>(3, 3) <<
    500.0, 0.0, 320.0,
    0.0, 500.0, 320.0,
    0.0, 0.0, 1.0);
}

std::vector<cv::Point2f> project(
  const std::vector<cv::Point3f> & object_points,
  const cv::Mat & rvec, const cv::Mat & tvec)
{
  std::vector<cv::Point2f> points;
  cv::projectPoints(object_points, rvec, tvec, cameraMatrix(), cv::Mat(), points);
  return points;
}
}  // namespace

TEST(PnpFilter, AcceptsValidProjectionAndImage)
{
  rm_auto_aim::PnpFilter filter(rm_auto_aim::PnpFilterConfig{});
  const auto objects = objectPoints();
  cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
  cv::Mat tvec = (cv::Mat_<double>(3, 1) << 0.0, 0.0, 2.0);
  const auto image_points = project(objects, rvec, tvec);

  const auto result = filter.evaluate(
    "1", 10.0, 70.0, 28.0, cv::Point2f(320.0F, 320.0F), 0.0,
    rvec, tvec, cameraMatrix(), cv::Mat(), objects, image_points);

  EXPECT_TRUE(result.accepted);
  EXPECT_LT(result.reprojection_error_px, 1e-6);
}

TEST(PnpFilter, RejectsBadAspectAndDepth)
{
  rm_auto_aim::PnpFilter filter(rm_auto_aim::PnpFilterConfig{});
  const auto objects = objectPoints();
  cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
  cv::Mat tvec = (cv::Mat_<double>(3, 1) << 0.0, 0.0, 2.0);
  const auto image_points = project(objects, rvec, tvec);

  const auto aspect = filter.evaluate(
    "1", 10.0, 20.0, 18.0, cv::Point2f(320.0F, 320.0F), 0.0,
    rvec, tvec, cameraMatrix(), cv::Mat(), objects, image_points);
  EXPECT_FALSE(aspect.accepted);
  EXPECT_EQ(aspect.reason, "box_aspect");

  cv::Mat negative_depth = (cv::Mat_<double>(3, 1) << 0.0, 0.0, -2.0);
  const auto depth = filter.evaluate(
    "2", 10.0, 70.0, 28.0, cv::Point2f(320.0F, 320.0F), 0.0,
    rvec, negative_depth, cameraMatrix(), cv::Mat(), objects, image_points);
  EXPECT_FALSE(depth.accepted);
  EXPECT_EQ(depth.reason, "invalid_depth");
}

TEST(PnpFilter, RejectsTemporalPositionAndAngleJumps)
{
  rm_auto_aim::PnpFilter filter(rm_auto_aim::PnpFilterConfig{});
  const auto objects = objectPoints();
  cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
  cv::Mat near_tvec = (cv::Mat_<double>(3, 1) << 0.0, 0.0, 2.0);
  const auto near_points = project(objects, rvec, near_tvec);
  ASSERT_TRUE(
    filter.evaluate(
      "1", 10.0, 70.0, 28.0, cv::Point2f(320.0F, 320.0F), 0.0,
      rvec, near_tvec, cameraMatrix(), cv::Mat(), objects, near_points).accepted);

  cv::Mat far_tvec = (cv::Mat_<double>(3, 1) << 0.0, 0.0, 3.8);
  const auto far_points = project(objects, rvec, far_tvec);
  const auto position_jump = filter.evaluate(
    "1", 10.02, 70.0, 28.0, cv::Point2f(320.0F, 320.0F), 0.0,
    rvec, far_tvec, cameraMatrix(), cv::Mat(), objects, far_points);
  EXPECT_FALSE(position_jump.accepted);
  EXPECT_EQ(position_jump.reason, "temporal_jump");
  EXPECT_GT(position_jump.position_jump_m, 1.5);

  ASSERT_TRUE(
    filter.evaluate(
      "2", 10.0, 70.0, 28.0, cv::Point2f(320.0F, 320.0F), 0.0,
      rvec, near_tvec, cameraMatrix(), cv::Mat(), objects, near_points).accepted);
  const auto angle_jump = filter.evaluate(
    "2", 10.02, 70.0, 28.0, cv::Point2f(320.0F, 320.0F), 1.5,
    rvec, near_tvec, cameraMatrix(), cv::Mat(), objects, near_points);
  EXPECT_FALSE(angle_jump.accepted);
  EXPECT_EQ(angle_jump.reason, "temporal_jump");
  EXPECT_NEAR(angle_jump.angle_jump_rad, 1.5, 1e-3);
}

TEST(PnpFilter, TreatsSwappedLongEdgeAsSameOrientation)
{
  rm_auto_aim::PnpFilter filter(rm_auto_aim::PnpFilterConfig{});
  const auto objects = objectPoints();
  cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
  cv::Mat tvec = (cv::Mat_<double>(3, 1) << 0.0, 0.0, 2.0);
  const auto image_points = project(objects, rvec, tvec);

  ASSERT_TRUE(
    filter.evaluate(
      "1", 10.0, 70.0, 28.0, cv::Point2f(320.0F, 320.0F), 0.0,
      rvec, tvec, cameraMatrix(), cv::Mat(), objects, image_points).accepted);
  const auto swapped = filter.evaluate(
    "1", 10.02, 28.0, 70.0, cv::Point2f(320.0F, 320.0F), CV_PI / 2.0,
    rvec, tvec, cameraMatrix(), cv::Mat(), objects, image_points);

  EXPECT_TRUE(swapped.accepted);
  EXPECT_NEAR(swapped.angle_jump_rad, 0.0, 1e-6);
}
