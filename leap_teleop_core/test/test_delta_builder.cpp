#include <gtest/gtest.h>

#include "leap_teleop_core/delta_builder.hpp"
#include "leap_teleop_core/frames.hpp"
#include "leap_teleop_core/hand_mapper.hpp"

using leap_teleop::apply_delta;
using leap_teleop::axis_angle;
using leap_teleop::degrees_to_radians;
using leap_teleop::DeltaBuilder;
using leap_teleop::HandMapper;
using leap_teleop::HandMapperConfig;
using leap_teleop::HandPoseInput;
using leap_teleop::point_from_msg;
using leap_teleop::quaternion_from_msg;
using leap_teleop::quaternion_to_msg;
using leap_teleop::rotation_angle;
using leap_teleop::to_transform_msg;
using leap_teleop::vector_from_msg;

namespace {

geometry_msgs::msg::Pose make_pose(const Eigen::Vector3d& position,
                                   const Eigen::Quaterniond& orientation) {
  geometry_msgs::msg::Pose pose;
  pose.position = leap_teleop::point_to_msg(position);
  pose.orientation = quaternion_to_msg(orientation);
  return pose;
}

// Maps one sample through a fresh mapper, so filters and deadbands pass it through unchanged.
geometry_msgs::msg::Pose map_hand(const Eigen::Quaterniond& orientation) {
  HandMapperConfig config;
  config.deadband_translation_m = 0.0;
  config.deadband_rotation_rad = 0.0;
  HandMapper mapper(config);
  HandPoseInput input;
  input.orientation = orientation;
  return mapper.update(input);
}

}  // namespace

TEST(DeltaBuilder, IdentityAtEngagement) {
  const auto latched = make_pose(Eigen::Vector3d(0.5, 0.0, 0.3), Eigen::Quaterniond::Identity());
  DeltaBuilder builder;
  builder.latch(latched);
  const auto delta = builder.compute(latched);
  EXPECT_TRUE(delta.translation.isApprox(Eigen::Vector3d::Zero(), 1e-12));
  EXPECT_NEAR(rotation_angle(delta.rotation), 0.0, 1e-12);
}

TEST(DeltaBuilder, PureRotationLeavesTheLatchedPositionUnchanged) {
  const Eigen::Vector3d latched_position(0.5, 0.0, 0.3);
  const auto latched = make_pose(latched_position, Eigen::Quaterniond::Identity());
  const auto rotated =
      make_pose(latched_position, axis_angle(0.0, 0.0, 1.0, degrees_to_radians(90.0)));

  DeltaBuilder builder;
  builder.latch(latched);
  const auto delta = builder.compute(rotated);

  EXPECT_TRUE(delta.translation.isApprox(Eigen::Vector3d::Zero(), 1e-12));
  EXPECT_NEAR(rotation_angle(delta.rotation), degrees_to_radians(90.0), 1e-12);

  const auto target = apply_delta(latched, delta);
  EXPECT_TRUE(point_from_msg(target.position).isApprox(latched_position, 1e-12));
  EXPECT_NEAR(rotation_angle(quaternion_from_msg(target.orientation)), degrees_to_radians(90.0),
              1e-12);
}

TEST(DeltaBuilder, TranslationIsAddedInBaseAxes) {
  const auto latched = make_pose(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity());
  const auto current = make_pose(Eigen::Vector3d(0.1, -0.2, 0.05), Eigen::Quaterniond::Identity());
  DeltaBuilder builder;
  builder.latch(latched);
  EXPECT_TRUE(
      builder.compute(current).translation.isApprox(Eigen::Vector3d(0.1, -0.2, 0.05), 1e-12));
}

TEST(DeltaBuilder, RotationIsComposedInBaseAxes) {
  const Eigen::Quaterniond hand_reference = axis_angle(1.0, 0.0, 0.0, degrees_to_radians(90.0));
  const Eigen::Quaterniond yaw = axis_angle(0.0, 0.0, 1.0, degrees_to_radians(90.0));
  const Eigen::Quaterniond tool_latched = axis_angle(0.0, 1.0, 0.0, degrees_to_radians(45.0));
  const auto latched_hand = make_pose(Eigen::Vector3d::Zero(), hand_reference);
  const auto current_hand = make_pose(Eigen::Vector3d::Zero(), yaw * hand_reference);
  const auto latched_tool = make_pose(Eigen::Vector3d::Zero(), tool_latched);

  DeltaBuilder builder;
  builder.latch(latched_hand);
  const auto delta = builder.compute(current_hand);
  EXPECT_TRUE(delta.rotation.isApprox(yaw, 1e-12));

  const auto target = apply_delta(latched_tool, delta);
  EXPECT_TRUE(quaternion_from_msg(target.orientation).isApprox(yaw * tool_latched, 1e-12));
}

TEST(DeltaBuilder, TransformMessageCarriesTheDelta) {
  const auto latched = make_pose(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity());
  const auto current = make_pose(Eigen::Vector3d(0.02, 0.0, 0.0), Eigen::Quaterniond::Identity());
  DeltaBuilder builder;
  builder.latch(latched);
  const auto msg = to_transform_msg(builder.compute(current));
  EXPECT_TRUE(vector_from_msg(msg.translation).isApprox(Eigen::Vector3d(0.02, 0.0, 0.0), 1e-12));
}

TEST(DeltaBuilder, RotationScaleMultipliesTheTurnSinceLatch) {
  const double rotation_scale = 2.0;
  const Eigen::Quaterniond start = axis_angle(1.0, 0.0, 0.0, degrees_to_radians(90.0));
  const Eigen::Quaterniond turn = axis_angle(0.0, 0.0, 1.0, degrees_to_radians(30.0));

  DeltaBuilder builder;
  builder.latch(map_hand(start));
  const auto delta = builder.compute(map_hand(turn * start), rotation_scale);

  EXPECT_NEAR(rotation_angle(delta.rotation), degrees_to_radians(60.0), 1e-9);
  // Same as an unscaled 60 degree hand turn, which the mapper expresses in base axes.
  DeltaBuilder unscaled;
  unscaled.latch(map_hand(start));
  const auto expected = unscaled.compute(map_hand(turn * turn * start));
  EXPECT_TRUE(delta.rotation.isApprox(expected.rotation, 1e-9) ||
              delta.rotation.coeffs().isApprox(-expected.rotation.coeffs(), 1e-9));
}

TEST(DeltaBuilder, RotationScaleKeepsTheAxisAndLeavesTranslation) {
  const Eigen::Quaterniond start = axis_angle(0.3, -0.5, 0.8, degrees_to_radians(70.0));
  const Eigen::Quaterniond turn = axis_angle(1.0, 1.0, 0.0, degrees_to_radians(20.0));
  const auto latched = make_pose(Eigen::Vector3d::Zero(), start);
  const auto current = make_pose(Eigen::Vector3d(0.1, 0.0, 0.0), turn * start);

  DeltaBuilder builder;
  builder.latch(latched);
  for (const double rotation_scale : {0.5, 1.0, 3.0}) {
    const auto delta = builder.compute(current, rotation_scale);
    EXPECT_TRUE(delta.rotation.isApprox(
        axis_angle(1.0, 1.0, 0.0, degrees_to_radians(20.0 * rotation_scale)), 1e-9));
    EXPECT_TRUE(delta.translation.isApprox(Eigen::Vector3d(0.1, 0.0, 0.0), 1e-12));
  }
}
