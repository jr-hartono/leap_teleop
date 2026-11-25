#include <gtest/gtest.h>

#include "leap_teleop_core/frames.hpp"

using leap_teleop::axis_angle;
using leap_teleop::degrees_to_radians;
using leap_teleop::device_to_base_rotation;
using leap_teleop::leap_position_to_base;
using leap_teleop::quaternion_power;
using leap_teleop::rotation_angle;
using leap_teleop::slerp;
using leap_teleop::world_delta;

TEST(Frames, RotationIsOrthonormalWithDeterminantOne) {
  for (const double yaw : {0.0, 37.0, 180.0, -90.0}) {
    const Eigen::Matrix3d rotation = device_to_base_rotation(yaw);
    EXPECT_TRUE((rotation.transpose() * rotation).isApprox(Eigen::Matrix3d::Identity(), 1e-12));
    EXPECT_NEAR(rotation.determinant(), 1.0, 1e-12);
  }
}

TEST(Frames, DeviceAxesMapToBaseAxesAtZeroYaw) {
  EXPECT_TRUE(leap_position_to_base(Eigen::Vector3d(0.0, 0.0, -1.0), 0.0)
                  .isApprox(Eigen::Vector3d(1.0, 0.0, 0.0), 1e-12));
  EXPECT_TRUE(leap_position_to_base(Eigen::Vector3d(0.0, 1.0, 0.0), 0.0)
                  .isApprox(Eigen::Vector3d(0.0, 0.0, 1.0), 1e-12));
  EXPECT_TRUE(leap_position_to_base(Eigen::Vector3d(1.0, 0.0, 0.0), 0.0)
                  .isApprox(Eigen::Vector3d(0.0, -1.0, 0.0), 1e-12));
}

TEST(Frames, YawRotatesTheMappingAboutBaseZ) {
  EXPECT_TRUE(leap_position_to_base(Eigen::Vector3d(0.0, 0.0, -1.0), 180.0)
                  .isApprox(Eigen::Vector3d(-1.0, 0.0, 0.0), 1e-12));
}

TEST(Frames, QuaternionPowerScalesTheAngle) {
  const Eigen::Quaterniond thirty = axis_angle(0.0, 0.0, 1.0, degrees_to_radians(30.0));
  EXPECT_NEAR(rotation_angle(quaternion_power(thirty, 2.0)), degrees_to_radians(60.0), 1e-12);
  EXPECT_NEAR(rotation_angle(quaternion_power(thirty, 0.0)), 0.0, 1e-12);
}

TEST(Frames, SlerpStaysOnTheShortestArc) {
  const Eigen::Quaterniond start = Eigen::Quaterniond::Identity();
  const Eigen::Quaterniond end = axis_angle(0.0, 0.0, 1.0, degrees_to_radians(90.0));
  EXPECT_NEAR(rotation_angle(slerp(start, end, 0.5)), degrees_to_radians(45.0), 1e-12);
  EXPECT_NEAR(rotation_angle(slerp(start, end, 0.0)), 0.0, 1e-12);
  EXPECT_NEAR(rotation_angle(slerp(start, end, 1.0)), degrees_to_radians(90.0), 1e-12);
}

TEST(Frames, WorldDeltaIsInBaseAxes) {
  const Eigen::Quaterniond reference = axis_angle(1.0, 0.0, 0.0, degrees_to_radians(90.0));
  const Eigen::Quaterniond yaw = axis_angle(0.0, 0.0, 1.0, degrees_to_radians(90.0));
  const Eigen::Quaterniond current = yaw * reference;

  EXPECT_TRUE(world_delta(current, reference).isApprox(yaw, 1e-12));
}

TEST(Frames, MessageConversionRoundTrip) {
  const Eigen::Quaterniond orientation = axis_angle(0.0, 1.0, 0.0, degrees_to_radians(30.0));
  const Eigen::Vector3d position(0.1, -0.2, 0.3);
  EXPECT_TRUE(leap_teleop::quaternion_from_msg(leap_teleop::quaternion_to_msg(orientation))
                  .isApprox(orientation, 1e-12));
  EXPECT_TRUE(
      leap_teleop::vector_from_msg(leap_teleop::vector_to_msg(position)).isApprox(position, 1e-12));
}
