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

TEST(HandMapper, ChangingTrackingModeChangesAxesAndRestartsFilters) {
  leap_teleop::HandMapper mapper(leap_teleop::HandMapperConfig{});
  leap_teleop::HandPoseInput input;
  input.position_m = Eigen::Vector3d(0.0, 0.0, -0.2);  // desktop: toward the operator's forward
  input.time_sec = 0.0;
  const auto desktop = mapper.update(input);
  mapper.set_tracking_mode(leap_teleop::TrackingMode::kHeadMounted);
  input.time_sec = 0.01;
  const auto head = mapper.update(input);
  EXPECT_FALSE(desktop.position.x == head.position.x && desktop.position.y == head.position.y &&
               desktop.position.z == head.position.z);
}

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
  config.tracking_mode = leap_teleop::TrackingMode::kHeadMounted;
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
      input.orientation, leap_teleop::TrackingMode::kDesktop);
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
      input.orientation, leap_teleop::TrackingMode::kDesktop);
  EXPECT_NEAR(std::abs(leap_teleop::quaternion_from_msg(pose.orientation).dot(expected)), 1.0,
              1e-12);
}

TEST(HandMapper, ResetMakesTheNextSamplePassUnfiltered) {
  HandMapperConfig config = fast_config();
  config.filter_min_cutoff_hz = 1.0;
  leap_teleop::HandMapper mapper(config);
  HandPoseInput input;
  input.position_m = Eigen::Vector3d(0.0, 0.0, -0.1);
  input.time_sec = 0.0;
  (void)mapper.update(input);
  input.position_m = Eigen::Vector3d(0.0, 0.0, -0.2);
  input.time_sec = 0.01;
  const double smoothed = point_from_msg(mapper.update(input).position).x();
  EXPECT_GT(smoothed, 0.1);
  EXPECT_LT(smoothed, 0.2);

  mapper.reset();
  input.time_sec = 0.02;
  EXPECT_NEAR(point_from_msg(mapper.update(input).position).x(), 0.2, 1e-12);
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
