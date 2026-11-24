#include <gtest/gtest.h>

#include "leap_teleop_core/frames.hpp"
#include "leap_teleop_core/hand_mapper.hpp"

using leap_teleop::axis_angle;
using leap_teleop::degrees_to_radians;
using leap_teleop::HandMapper;
using leap_teleop::HandMapperConfig;
using leap_teleop::HandPoseInput;
using leap_teleop::rotation_angle;
using leap_teleop::point_from_msg;

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

TEST(HandMapper, YawRotatesTheTranslationMapping) {
  HandMapperConfig config = fast_config();
  config.yaw_deg = 180.0;
  HandMapper mapper(config);
  HandPoseInput input;
  input.position_m = Eigen::Vector3d(0.0, 0.0, -0.1);
  input.time_sec = 0.0;
  EXPECT_TRUE(point_from_msg(mapper.update(input).position)
                  .isApprox(Eigen::Vector3d(-0.1, 0.0, 0.0), 1e-12));
}

TEST(HandMapper, TranslationGainScalesTheHandMotion) {
  HandMapperConfig config = fast_config();
  config.translation_scale = 0.5;
  HandMapper mapper(config);
  HandPoseInput input;
  input.position_m = Eigen::Vector3d(0.0, 0.0, -0.2);
  input.time_sec = 0.0;
  EXPECT_TRUE(point_from_msg(mapper.update(input).position)
                  .isApprox(Eigen::Vector3d(0.1, 0.0, 0.0), 1e-12));
}

TEST(HandMapper, RotationGainScalesTheWristMotion) {
  for (const double gain : {1.0, 2.0, 0.5}) {
    HandMapperConfig config = fast_config();
    config.rotation_scale = gain;
    HandMapper mapper(config);
    HandPoseInput input;
    input.orientation = axis_angle(0.0, 0.0, 1.0, degrees_to_radians(30.0));
    input.time_sec = 0.0;
    const auto pose = mapper.update(input);
    EXPECT_NEAR(rotation_angle(leap_teleop::quaternion_from_msg(pose.orientation)),
                degrees_to_radians(30.0 * gain), 1e-12);
  }
}
