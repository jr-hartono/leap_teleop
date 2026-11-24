#include <gtest/gtest.h>

#include <cmath>

#include "leap_teleop_core/filters.hpp"

using leap_teleop::axis_angle;
using leap_teleop::degrees_to_radians;
using leap_teleop::OneEuroFilter;
using leap_teleop::OrientationFilter;
using leap_teleop::RotationDeadband;
using leap_teleop::rotation_angle;
using leap_teleop::TranslationDeadband;

TEST(Filters, OneEuroConvergesToAStep) {
  OneEuroFilter filter(100.0, 0.0);
  double value = filter.filter(0.0, 0.0);
  for (int i = 1; i <= 100; ++i) {
    value = filter.filter(1.0, i * 0.01);
  }
  EXPECT_NEAR(value, 1.0, 1e-6);
}

TEST(Filters, OneEuroHandlesZeroAndNegativeDt) {
  OneEuroFilter filter(1.0, 0.05);
  EXPECT_TRUE(std::isfinite(filter.filter(1.0, 1.0)));
  EXPECT_TRUE(std::isfinite(filter.filter(2.0, 1.0)));
  EXPECT_TRUE(std::isfinite(filter.filter(2.0, 0.5)));
  EXPECT_TRUE(std::isfinite(filter.filter(std::nan(""), 2.0)));
}

TEST(Filters, OrientationFilterSlerpsTowardsTheTarget) {
  OrientationFilter filter(100.0);
  const Eigen::Quaterniond target =
      axis_angle(0.0, 0.0, 1.0, degrees_to_radians(90.0));
  EXPECT_NEAR(rotation_angle(filter.filter(target, 0.0)), degrees_to_radians(90.0), 1e-12);

  OrientationFilter slow(1.0);
  const Eigen::Quaterniond first = slow.filter(Eigen::Quaterniond::Identity(), 0.0);
  const Eigen::Quaterniond second = slow.filter(target, 0.01);
  EXPECT_LT(rotation_angle(first), rotation_angle(second));
  EXPECT_LT(rotation_angle(second), degrees_to_radians(90.0));
  EXPECT_TRUE(std::isfinite(second.w()));
}

TEST(Filters, TranslationDeadbandSuppressesSmallMotion) {
  TranslationDeadband deadband(0.002, 1.5);
  EXPECT_DOUBLE_EQ(deadband.filter(Eigen::Vector3d::Zero(), 0.0).norm(), 0.0);
  EXPECT_DOUBLE_EQ(deadband.filter(Eigen::Vector3d(0.001, 0.0, 0.0), 0.01).norm(), 0.0);
  const Eigen::Vector3d moved = deadband.filter(Eigen::Vector3d(0.005, 0.0, 0.0), 0.02);
  EXPECT_GT(moved.x(), 0.0);
  EXPECT_NEAR(moved.x(), 0.003, 1e-12);
}

TEST(Filters, TranslationDeadbandRebasesAfterTheHandStops) {
  TranslationDeadband deadband(0.002, 1.5);
  deadband.filter(Eigen::Vector3d::Zero(), 0.0);
  deadband.filter(Eigen::Vector3d(0.005, 0.0, 0.0), 0.01);
  double time = 0.01;
  for (int i = 0; i < 20; ++i) {
    time += 0.01;
    deadband.filter(Eigen::Vector3d(0.005, 0.0, 0.0), time);
  }
  const Eigen::Vector3d rebased = deadband.filter(Eigen::Vector3d(0.005, 0.0, 0.0), time + 0.01);
  EXPECT_NEAR(rebased.x(), 0.005, 1e-12);
}

TEST(Filters, RotationDeadbandSuppressesSmallMotion) {
  RotationDeadband deadband(degrees_to_radians(0.5), 1.5);
  const Eigen::Quaterniond identity = Eigen::Quaterniond::Identity();
  const Eigen::Quaterniond small =
      axis_angle(0.0, 0.0, 1.0, degrees_to_radians(0.4));
  const Eigen::Quaterniond large =
      axis_angle(0.0, 0.0, 1.0, degrees_to_radians(3.0));

  EXPECT_NEAR(rotation_angle(deadband.filter(identity, 0.0)), 0.0, 1e-12);
  EXPECT_NEAR(rotation_angle(deadband.filter(small, 0.01)), 0.0, 1e-12);
  const Eigen::Quaterniond output = deadband.filter(large, 0.02);
  EXPECT_NEAR(rotation_angle(output), degrees_to_radians(2.5), 1e-9);
}
