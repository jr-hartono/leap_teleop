#include <gtest/gtest.h>

#include "leap_teleop_bridge/gripper_command.hpp"

namespace leap_teleop {
namespace {

TEST(GripperPosition, EndpointsMapToOpenAndClosed) {
  EXPECT_DOUBLE_EQ(gripper_position(0.0, 0.0, 1.0), 0.0);
  EXPECT_DOUBLE_EQ(gripper_position(1.0, 0.0, 1.0), 1.0);
}

TEST(GripperPosition, IsLinear) {
  EXPECT_DOUBLE_EQ(gripper_position(0.5, 0.0, 1.0), 0.5);
  EXPECT_DOUBLE_EQ(gripper_position(0.25, 0.2, 1.0), 0.4);
}

TEST(GripperPosition, ClampsClosure) {
  EXPECT_DOUBLE_EQ(gripper_position(-0.5, 0.0, 1.0), 0.0);
  EXPECT_DOUBLE_EQ(gripper_position(1.5, 0.0, 1.0), 1.0);
}

TEST(GripperPosition, HandlesClosedBelowOpen) {
  EXPECT_DOUBLE_EQ(gripper_position(0.0, 0.08, 0.0), 0.08);
  EXPECT_DOUBLE_EQ(gripper_position(1.0, 0.08, 0.0), 0.0);
  EXPECT_DOUBLE_EQ(gripper_position(0.5, 0.08, 0.0), 0.04);
  EXPECT_DOUBLE_EQ(gripper_position(2.0, 0.08, 0.0), 0.0);
}

}  // namespace
}  // namespace leap_teleop
