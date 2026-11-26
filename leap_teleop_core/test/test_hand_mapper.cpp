#include <gtest/gtest.h>

#include <cmath>
#include <optional>

#include "leap_teleop_core/frames.hpp"
#include "leap_teleop_core/hand_mapper.hpp"

using leap_teleop::axis_angle;
using leap_teleop::clamp_unit_strength;
using leap_teleop::degrees_to_radians;
using leap_teleop::HandMapper;
using leap_teleop::HandMapperConfig;
using leap_teleop::HandPoseInput;
using leap_teleop::non_negative_distance;
using leap_teleop::point_from_msg;
using leap_teleop::rotation_angle;

namespace {

HandMapperConfig fast_config() {
  HandMapperConfig config;
  config.filter_min_cutoff_hz = 100.0;
  config.filter_beta = 0.0;
  config.deadband_translation_m = 0.0;
  config.deadband_rotation_rad = 0.0;
  config.deadband_hysteresis = 1.0;
  return config;
}

}  // namespace

TEST(HandMapper, AwayFromOperatorBecomesForward) {
  HandMapper mapper(fast_config());
  HandPoseInput input;
  input.position_m = Eigen::Vector3d(0.0, 0.0, -0.1);
  input.time_sec = 0.0;
  const auto pose = mapper.update(input);
  EXPECT_TRUE(point_from_msg(pose.position).isApprox(Eigen::Vector3d(0.1, 0.0, 0.0), 1e-12));
}

TEST(HandMapper, HandDownBecomesToolDown) {
  HandMapper mapper(fast_config());
  HandPoseInput input;
  input.position_m = Eigen::Vector3d(0.0, -0.1, 0.0);
  input.time_sec = 0.0;
  const auto pose = mapper.update(input);
  EXPECT_TRUE(point_from_msg(pose.position).isApprox(Eigen::Vector3d(0.0, 0.0, -0.1), 1e-12));
}

TEST(HandMapper, HeadMountedForwardBecomesForward) {
  HandMapperConfig config = fast_config();
  config.mount_mode = leap_teleop::MountMode::kHeadMounted;
  HandMapper mapper(config);
  HandPoseInput input;
  input.position_m = Eigen::Vector3d(0.0, 0.1, 0.0);
  input.time_sec = 0.0;
  EXPECT_TRUE(point_from_msg(mapper.update(input).position)
                  .isApprox(Eigen::Vector3d(0.1, 0.0, 0.0), 1e-12));
}

TEST(HandMapper, ScalesNeitherPositionNorOrientation) {
  HandMapper mapper(fast_config());
  HandPoseInput input;
  input.position_m = Eigen::Vector3d(0.0, 0.0, -0.2);
  input.orientation = axis_angle(0.0, 0.0, 1.0, degrees_to_radians(30.0));
  input.time_sec = 0.0;
  const auto pose = mapper.update(input);
  EXPECT_TRUE(point_from_msg(pose.position).isApprox(Eigen::Vector3d(0.2, 0.0, 0.0), 1e-12));
  // Only mapped into operator axes, not scaled.
  const Eigen::Quaterniond expected = leap_teleop::device_orientation_to_operator(
      input.orientation, leap_teleop::MountMode::kDesktop);
  EXPECT_NEAR(std::abs(leap_teleop::quaternion_from_msg(pose.orientation).dot(expected)), 1.0,
              1e-12);
}

TEST(HandMapper, SetTuningKeepsFilterState) {
  HandMapperConfig config = fast_config();
  HandMapper mapper(config);
  HandPoseInput input;
  input.position_m = Eigen::Vector3d(0.0, 0.0, -0.1);
  input.time_sec = 0.0;
  EXPECT_NEAR(point_from_msg(mapper.update(input).position).x(), 0.1, 1e-12);

  config.filter_min_cutoff_hz = 1.0;
  mapper.set_tuning(config);
  input.position_m = Eigen::Vector3d(0.0, 0.0, -0.2);
  input.time_sec = 0.01;
  const double x = point_from_msg(mapper.update(input).position).x();
  EXPECT_GT(x, 0.1);
  EXPECT_LT(x, 0.2);
}

TEST(HandMapper, OrientationIsNotScaled) {
  HandMapper mapper(fast_config());
  HandPoseInput input;
  input.orientation = axis_angle(0.0, 0.0, 1.0, degrees_to_radians(30.0));
  input.time_sec = 0.0;
  const auto pose = mapper.update(input);
  // Only mapped into operator axes, not scaled.
  const Eigen::Quaterniond expected = leap_teleop::device_orientation_to_operator(
      input.orientation, leap_teleop::MountMode::kDesktop);
  EXPECT_NEAR(std::abs(leap_teleop::quaternion_from_msg(pose.orientation).dot(expected)), 1.0,
              1e-12);
}

TEST(PerSideMapper, LosingTheLeftHandKeepsTheRightHandsFilterState) {
  using leap_teleop::HandSide;
  HandMapperConfig config = fast_config();
  config.filter_min_cutoff_hz = 1.0;
  leap_teleop::PerSideMapper mapper(config);

  HandPoseInput input;
  input.position_m = Eigen::Vector3d(0.0, 0.0, -0.1);
  input.time_sec = 0.0;
  ASSERT_TRUE(mapper.update(HandSide::kLeft, input));
  ASSERT_TRUE(mapper.update(HandSide::kRight, input));

  // The left hand disappears; the right hand keeps moving through its smoothing filter.
  EXPECT_FALSE(mapper.update(HandSide::kLeft, std::nullopt));
  input.position_m = Eigen::Vector3d(0.0, 0.0, -0.2);
  input.time_sec = 0.01;
  const auto right = mapper.update(HandSide::kRight, input);
  ASSERT_TRUE(right);
  const double right_x = point_from_msg(right->position).x();
  EXPECT_GT(right_x, 0.1);
  EXPECT_LT(right_x, 0.2);

  // The left hand returns with fresh state and passes its first sample through unfiltered.
  const auto left = mapper.update(HandSide::kLeft, input);
  ASSERT_TRUE(left);
  EXPECT_NEAR(point_from_msg(left->position).x(), 0.2, 1e-12);
}

TEST(PerSideMapper, ResetAllClearsBothSides) {
  using leap_teleop::HandSide;
  HandMapperConfig config = fast_config();
  config.filter_min_cutoff_hz = 1.0;
  leap_teleop::PerSideMapper mapper(config);
  HandPoseInput input;
  input.position_m = Eigen::Vector3d(0.0, 0.0, -0.1);
  input.time_sec = 0.0;
  mapper.update(HandSide::kLeft, input);
  mapper.update(HandSide::kRight, input);
  mapper.reset_all();
  input.position_m = Eigen::Vector3d(0.0, 0.0, -0.2);
  input.time_sec = 0.01;
  EXPECT_NEAR(point_from_msg(mapper.update(HandSide::kLeft, input)->position).x(), 0.2, 1e-12);
  EXPECT_NEAR(point_from_msg(mapper.update(HandSide::kRight, input)->position).x(), 0.2, 1e-12);
}

TEST(PinchSignals, StrengthIsClampedToUnitRange) {
  EXPECT_FLOAT_EQ(clamp_unit_strength(0.25F), 0.25F);
  EXPECT_FLOAT_EQ(clamp_unit_strength(-0.1F), 0.0F);
  EXPECT_FLOAT_EQ(clamp_unit_strength(1.5F), 1.0F);
  EXPECT_FLOAT_EQ(clamp_unit_strength(std::nanf("")), 0.0F);
}

TEST(PinchSignals, DistanceIsNonNegative) {
  EXPECT_FLOAT_EQ(non_negative_distance(42.0F), 42.0F);
  EXPECT_FLOAT_EQ(non_negative_distance(-3.0F), 0.0F);
  EXPECT_FLOAT_EQ(non_negative_distance(std::nanf("")), 0.0F);
}
