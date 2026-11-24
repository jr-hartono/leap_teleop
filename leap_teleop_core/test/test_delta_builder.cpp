#include <gtest/gtest.h>

#include "leap_teleop_core/delta_builder.hpp"
#include "leap_teleop_core/frames.hpp"

using leap_teleop::apply_delta;
using leap_teleop::axis_angle;
using leap_teleop::DeltaBuilder;
using leap_teleop::degrees_to_radians;
using leap_teleop::quaternion_from_msg;
using leap_teleop::quaternion_to_msg;
using leap_teleop::RotationFrame;
using leap_teleop::rotation_angle;
using leap_teleop::to_transform_msg;
using leap_teleop::point_from_msg;
using leap_teleop::vector_from_msg;

namespace {

geometry_msgs::msg::Pose make_pose(const Eigen::Vector3d& position,
                                   const Eigen::Quaterniond& orientation) {
  geometry_msgs::msg::Pose pose;
  pose.position = leap_teleop::point_to_msg(position);
  pose.orientation = quaternion_to_msg(orientation);
  return pose;
}

}  // namespace

TEST(DeltaBuilder, IdentityAtEngagement) {
  const auto latched = make_pose(Eigen::Vector3d(0.5, 0.0, 0.3),
                                 Eigen::Quaterniond::Identity());
  DeltaBuilder builder;
  builder.latch(latched);
  const auto delta = builder.compute(latched);
  EXPECT_TRUE(delta.translation.isApprox(Eigen::Vector3d::Zero(), 1e-12));
  EXPECT_NEAR(rotation_angle(delta.rotation), 0.0, 1e-12);
}

TEST(DeltaBuilder, PureRotationLeavesTheLatchedPositionUnchanged) {
  const Eigen::Vector3d latched_position(0.5, 0.0, 0.3);
  const auto latched = make_pose(latched_position, Eigen::Quaterniond::Identity());
  const auto rotated = make_pose(
      latched_position, axis_angle(0.0, 0.0, 1.0, degrees_to_radians(90.0)));

  DeltaBuilder builder(RotationFrame::kWorld);
  builder.latch(latched);
  const auto delta = builder.compute(rotated);

  EXPECT_TRUE(delta.translation.isApprox(Eigen::Vector3d::Zero(), 1e-12));
  EXPECT_NEAR(rotation_angle(delta.rotation), degrees_to_radians(90.0), 1e-12);

  const auto target = apply_delta(latched, delta);
  EXPECT_TRUE(point_from_msg(target.position).isApprox(latched_position, 1e-12));
  EXPECT_NEAR(rotation_angle(quaternion_from_msg(target.orientation)),
              degrees_to_radians(90.0), 1e-12);
}

TEST(DeltaBuilder, TranslationIsAddedInBaseAxes) {
  const auto latched = make_pose(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity());
  const auto current = make_pose(Eigen::Vector3d(0.1, -0.2, 0.05),
                                 Eigen::Quaterniond::Identity());
  DeltaBuilder builder;
  builder.latch(latched);
  EXPECT_TRUE(builder.compute(current).translation.isApprox(
      Eigen::Vector3d(0.1, -0.2, 0.05), 1e-12));
}

TEST(DeltaBuilder, WorldAndToolFramesComposeInDifferentOrders) {
  const Eigen::Quaterniond hand_reference =
      axis_angle(1.0, 0.0, 0.0, degrees_to_radians(90.0));
  const Eigen::Quaterniond yaw =
      axis_angle(0.0, 0.0, 1.0, degrees_to_radians(90.0));
  const Eigen::Quaterniond tool_latched =
      axis_angle(0.0, 1.0, 0.0, degrees_to_radians(45.0));
  const auto latched_hand = make_pose(Eigen::Vector3d::Zero(), hand_reference);
  const auto current_hand = make_pose(Eigen::Vector3d::Zero(), yaw * hand_reference);
  const auto latched_tool = make_pose(Eigen::Vector3d::Zero(), tool_latched);

  DeltaBuilder world(RotationFrame::kWorld);
  world.latch(latched_hand);
  DeltaBuilder tool(RotationFrame::kTool);
  tool.latch(latched_hand);

  const auto world_delta = world.compute(current_hand);
  const auto tool_delta = tool.compute(current_hand);
  EXPECT_TRUE(world_delta.rotation.isApprox(yaw, 1e-12));

  const auto world_target = apply_delta(latched_tool, world_delta);
  const auto tool_target = apply_delta(latched_tool, tool_delta);
  EXPECT_TRUE(quaternion_from_msg(world_target.orientation)
                  .isApprox(world_delta.rotation * tool_latched, 1e-12));
  EXPECT_TRUE(quaternion_from_msg(tool_target.orientation)
                  .isApprox(tool_latched * tool_delta.rotation, 1e-12));
  EXPECT_FALSE(quaternion_from_msg(tool_target.orientation)
                   .isApprox(quaternion_from_msg(world_target.orientation), 1e-9));
}

TEST(DeltaBuilder, TransformMessageCarriesTheDelta) {
  const auto latched = make_pose(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity());
  const auto current = make_pose(Eigen::Vector3d(0.02, 0.0, 0.0),
                                 Eigen::Quaterniond::Identity());
  DeltaBuilder builder;
  builder.latch(latched);
  const auto msg = to_transform_msg(builder.compute(current));
  EXPECT_TRUE(vector_from_msg(msg.translation)
                  .isApprox(Eigen::Vector3d(0.02, 0.0, 0.0), 1e-12));
}
