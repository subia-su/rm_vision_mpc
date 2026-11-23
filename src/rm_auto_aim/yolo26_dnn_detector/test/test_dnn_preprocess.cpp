// Copyright 2026 rm_vision contributors
// Licensed under the Apache-2.0 License.

#include <gtest/gtest.h>

#include <opencv2/core.hpp>

#include "yolo26_dnn_detector/dnn_preprocess.hpp"

TEST(DnnPreprocess, UsesFastPathForContinuous640Rgb)
{
  const cv::Mat image(640, 640, CV_8UC3, cv::Scalar(30, 20, 10));
  const auto prepared = rm_auto_aim::prepareDnnInput(image, true);

  EXPECT_TRUE(prepared.fast_path);
  EXPECT_FLOAT_EQ(prepared.scale, 1.0F);
  EXPECT_EQ(prepared.pad_left, 0);
  EXPECT_EQ(prepared.pad_top, 0);
  EXPECT_EQ(prepared.blob.size[0], 1);
  EXPECT_EQ(prepared.blob.size[1], 3);
  EXPECT_EQ(prepared.blob.size[2], 640);
  EXPECT_EQ(prepared.blob.size[3], 640);
  EXPECT_NEAR(prepared.blob.ptr<float>()[0], 30.0 / 255.0, 1e-6);
}

TEST(DnnPreprocess, UsesLetterboxForNonSquareInput)
{
  const cv::Mat image(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
  const auto prepared = rm_auto_aim::prepareDnnInput(image, false);

  EXPECT_FALSE(prepared.fast_path);
  EXPECT_FLOAT_EQ(prepared.scale, 1.0F);
  EXPECT_EQ(prepared.pad_left, 0);
  EXPECT_EQ(prepared.pad_top, 80);
  EXPECT_EQ(prepared.blob.size[2], 640);
  EXPECT_EQ(prepared.blob.size[3], 640);
}
