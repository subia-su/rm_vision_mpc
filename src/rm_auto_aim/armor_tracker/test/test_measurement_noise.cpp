// Copyright 2026 rm_vision contributors
// Licensed under the Apache-2.0 License.

#include <gtest/gtest.h>

#include "armor_tracker/measurement_noise.hpp"

TEST(MeasurementNoise, ScalesWithConfidence)
{
  const double reference = 0.7;
  EXPECT_DOUBLE_EQ(
    rm_auto_aim::confidenceNoiseScale(reference, reference, 0.25, 4.0), 1.0);
  EXPECT_NEAR(
    rm_auto_aim::confidenceNoiseScale(0.35, reference, 0.25, 4.0), 4.0, 1e-9);
  EXPECT_NEAR(
    rm_auto_aim::confidenceNoiseScale(0.95, reference, 0.25, 4.0),
    (0.7 / 0.95) * (0.7 / 0.95), 1e-9);
}

TEST(MeasurementNoise, MissingConfidenceFallsBackToNeutral)
{
  EXPECT_DOUBLE_EQ(
    rm_auto_aim::confidenceNoiseScale(0.0, 0.7, 0.25, 4.0), 1.0);
  EXPECT_DOUBLE_EQ(
    rm_auto_aim::confidenceNoiseScale(-1.0, 0.7, 0.25, 4.0), 1.0);
}
