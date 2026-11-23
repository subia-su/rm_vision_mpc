// Copyright 2026 rm_vision contributors
// Licensed under the Apache-2.0 License.

#include <gtest/gtest.h>

#include <Eigen/Dense>

#include "armor_tracker/extended_kalman_filter.hpp"

namespace
{
Eigen::VectorXd identityVector(const Eigen::VectorXd & value)
{
  return value;
}

Eigen::MatrixXd identityMatrix(const Eigen::VectorXd & value)
{
  return Eigen::MatrixXd::Identity(value.size(), value.size());
}

Eigen::MatrixXd zeroProcessNoise()
{
  return Eigen::MatrixXd::Zero(4, 4);
}

Eigen::MatrixXd measurementNoise(const Eigen::VectorXd &)
{
  return Eigen::MatrixXd::Identity(4, 4);
}
}

TEST(ConfidenceEkf, LargerMeasurementScaleReducesUpdateGain)
{
  rm_auto_aim::ExtendedKalmanFilter high_confidence(
    identityVector, identityVector, identityMatrix, identityMatrix,
    zeroProcessNoise, measurementNoise, Eigen::MatrixXd::Identity(4, 4));
  rm_auto_aim::ExtendedKalmanFilter low_confidence(
    identityVector, identityVector, identityMatrix, identityMatrix,
    zeroProcessNoise, measurementNoise, Eigen::MatrixXd::Identity(4, 4));
  const Eigen::VectorXd initial = Eigen::VectorXd::Zero(4);
  const Eigen::VectorXd measurement = Eigen::VectorXd::Ones(4);
  high_confidence.setState(initial);
  low_confidence.setState(initial);

  high_confidence.predict();
  low_confidence.predict();
  const Eigen::VectorXd high_result =
    high_confidence.update(measurement, 1.0);
  const Eigen::VectorXd low_result =
    low_confidence.update(measurement, 4.0);

  EXPECT_NEAR(high_result(0), 0.5, 1e-9);
  EXPECT_NEAR(low_result(0), 0.2, 1e-9);
  EXPECT_LT(low_result.norm(), high_result.norm());
}
