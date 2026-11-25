#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include "leap_teleop_core/frames.hpp"

using leap_teleop::axis_angle;
using leap_teleop::degrees_to_radians;
using leap_teleop::device_orientation_to_operator;
using leap_teleop::device_position_to_operator;
using leap_teleop::device_to_base_rotation;
using leap_teleop::leap_orientation_to_base;
using leap_teleop::leap_position_to_base;
using leap_teleop::mount_mode_from_string;
using leap_teleop::MountMode;
using leap_teleop::operator_orientation_to_base;
using leap_teleop::operator_position_to_base;
using leap_teleop::quaternion_power;
using leap_teleop::rotation_angle;
using leap_teleop::slerp;
using leap_teleop::world_delta;

namespace {

// Equal as rotations, so q and -q match.
bool same_rotation(const Eigen::Quaterniond& a, const Eigen::Quaterniond& b) {
  return std::abs(a.normalized().dot(b.normalized())) > 1.0 - 1e-12;
}

// The turn in base axes produced by turning the wrist by `turn` (in device axes) from `start`.
Eigen::Quaterniond turn_in_base(const Eigen::Quaterniond& turn, double yaw, MountMode mode) {
  const Eigen::Quaterniond start = axis_angle(0.4, -0.7, 0.2, 1.1);
  return world_delta(leap_orientation_to_base(turn * start, yaw, mode),
                     leap_orientation_to_base(start, yaw, mode));
}

}  // namespace

TEST(Frames, RotationIsOrthonormalWithDeterminantOne) {
  for (const MountMode mode : {MountMode::kDesktop, MountMode::kHeadMounted}) {
    for (const double yaw : {0.0, 37.0, 180.0, -90.0}) {
      const Eigen::Matrix3d rotation = device_to_base_rotation(yaw, mode);
      EXPECT_TRUE((rotation.transpose() * rotation).isApprox(Eigen::Matrix3d::Identity(), 1e-12));
      EXPECT_NEAR(rotation.determinant(), 1.0, 1e-12);
    }
  }
}

TEST(Frames, ParsesMountMode) {
  EXPECT_EQ(mount_mode_from_string("desktop"), MountMode::kDesktop);
  EXPECT_EQ(mount_mode_from_string("head_mounted"), MountMode::kHeadMounted);
  EXPECT_THROW(mount_mode_from_string("screentop"), std::invalid_argument);
}

TEST(Frames, DeviceAxesMapToBaseAxesAtZeroYaw) {
  EXPECT_TRUE(leap_position_to_base(Eigen::Vector3d(0.0, 0.0, -1.0), 0.0, MountMode::kDesktop)
                  .isApprox(Eigen::Vector3d(1.0, 0.0, 0.0), 1e-12));
  EXPECT_TRUE(leap_position_to_base(Eigen::Vector3d(0.0, 1.0, 0.0), 0.0, MountMode::kDesktop)
                  .isApprox(Eigen::Vector3d(0.0, 0.0, 1.0), 1e-12));
  EXPECT_TRUE(leap_position_to_base(Eigen::Vector3d(1.0, 0.0, 0.0), 0.0, MountMode::kDesktop)
                  .isApprox(Eigen::Vector3d(0.0, -1.0, 0.0), 1e-12));
}

TEST(Frames, YawRotatesTheMappingAboutBaseZ) {
  EXPECT_TRUE(leap_position_to_base(Eigen::Vector3d(0.0, 0.0, -1.0), 180.0, MountMode::kDesktop)
                  .isApprox(Eigen::Vector3d(-1.0, 0.0, 0.0), 1e-12));
}

// Head-mounted device axes: +y out of the cameras (forward), +x to the operator's left, +z down.
TEST(Frames, HeadMountedHandMotionMapsToOperatorAxes) {
  const MountMode mode = MountMode::kHeadMounted;
  const Eigen::Vector3d forward(0.0, 1.0, 0.0);
  const Eigen::Vector3d up(0.0, 0.0, -1.0);
  const Eigen::Vector3d right(-1.0, 0.0, 0.0);
  EXPECT_TRUE(leap_position_to_base(forward, 0.0, mode).isApprox(Eigen::Vector3d::UnitX(), 1e-12));
  EXPECT_TRUE(leap_position_to_base(up, 0.0, mode).isApprox(Eigen::Vector3d::UnitZ(), 1e-12));
  EXPECT_TRUE(leap_position_to_base(right, 0.0, mode).isApprox(-Eigen::Vector3d::UnitY(), 1e-12));
}

TEST(Frames, HeadMountedAppliesYawAfterTheMountMapping) {
  const Eigen::Vector3d forward(0.0, 1.0, 0.0);
  EXPECT_TRUE(leap_position_to_base(forward, 90.0, MountMode::kHeadMounted)
                  .isApprox(Eigen::Vector3d::UnitY(), 1e-12));
  EXPECT_TRUE(leap_position_to_base(forward, 180.0, MountMode::kHeadMounted)
                  .isApprox(-Eigen::Vector3d::UnitX(), 1e-12));
}

TEST(Frames, HeadMountedWristRotationMapsToBaseAxes) {
  const MountMode mode = MountMode::kHeadMounted;
  const double angle = degrees_to_radians(30.0);
  // Roll about the forward-pointing device +y is a roll about base +x.
  EXPECT_TRUE(same_rotation(turn_in_base(axis_angle(0.0, 1.0, 0.0, angle), 0.0, mode),
                            axis_angle(1.0, 0.0, 0.0, angle)));
  // Turning about the downward-pointing device +z is a turn about base -z.
  EXPECT_TRUE(same_rotation(turn_in_base(axis_angle(0.0, 0.0, 1.0, angle), 0.0, mode),
                            axis_angle(0.0, 0.0, 1.0, -angle)));
  // Pitch about the left-pointing device +x is a pitch about base +y.
  EXPECT_TRUE(same_rotation(turn_in_base(axis_angle(1.0, 0.0, 0.0, angle), 0.0, mode),
                            axis_angle(0.0, 1.0, 0.0, angle)));
  // With yaw 90, the forward roll axis becomes base +y.
  EXPECT_TRUE(same_rotation(turn_in_base(axis_angle(0.0, 1.0, 0.0, angle), 90.0, mode),
                            axis_angle(0.0, 1.0, 0.0, angle)));
}

TEST(Frames, BothMountsAgreeInOperatorAxes) {
  // The same physical hand motion, seen by each mount, gives the same operator-frame motion.
  const Eigen::Vector3d desktop_forward(0.0, 0.0, -1.0);
  const Eigen::Vector3d head_forward(0.0, 1.0, 0.0);
  EXPECT_TRUE(
      device_position_to_operator(desktop_forward, MountMode::kDesktop)
          .isApprox(device_position_to_operator(head_forward, MountMode::kHeadMounted), 1e-12));
  // The same physical palm orientation, as each mount reports it, maps to one operator orientation.
  const Eigen::Quaterniond palm = axis_angle(0.2, 0.9, -0.4, 0.8);
  for (const MountMode mode : {MountMode::kDesktop, MountMode::kHeadMounted}) {
    const Eigen::Quaterniond seen_by_device =
        Eigen::Quaterniond(leap_teleop::device_to_operator_rotation(mode).transpose()) * palm;
    EXPECT_TRUE(same_rotation(device_orientation_to_operator(seen_by_device, mode), palm));
  }
}

TEST(Frames, PalmAxesMapLikePositions) {
  // An orientation is the palm's axes, so each axis maps like a direction does.
  const Eigen::Quaterniond palm = axis_angle(-0.3, 0.5, 1.0, 1.3);
  for (const MountMode mode : {MountMode::kDesktop, MountMode::kHeadMounted}) {
    const Eigen::Quaterniond in_base = leap_orientation_to_base(palm, 37.0, mode);
    for (const Eigen::Vector3d axis :
         {Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitZ()}) {
      EXPECT_TRUE((in_base * axis).isApprox(leap_position_to_base(palm * axis, 37.0, mode), 1e-12));
    }
  }
}

TEST(Frames, CombinedMappingIsTheTwoStepsInOrder) {
  const Eigen::Vector3d position(0.1, -0.2, 0.3);
  const Eigen::Quaterniond orientation = axis_angle(1.0, 2.0, -0.5, 0.7);
  for (const MountMode mode : {MountMode::kDesktop, MountMode::kHeadMounted}) {
    EXPECT_TRUE(
        leap_position_to_base(position, 37.0, mode)
            .isApprox(operator_position_to_base(device_position_to_operator(position, mode), 37.0),
                      1e-12));
    EXPECT_TRUE(same_rotation(
        leap_orientation_to_base(orientation, 37.0, mode),
        operator_orientation_to_base(device_orientation_to_operator(orientation, mode), 37.0)));
  }
}

TEST(Frames, OperatorToBaseIsOnlyYaw) {
  EXPECT_TRUE(operator_position_to_base(Eigen::Vector3d::UnitX(), 0.0)
                  .isApprox(Eigen::Vector3d::UnitX(), 1e-12));
  EXPECT_TRUE(operator_position_to_base(Eigen::Vector3d::UnitX(), 90.0)
                  .isApprox(Eigen::Vector3d::UnitY(), 1e-12));
  EXPECT_TRUE(operator_position_to_base(Eigen::Vector3d::UnitZ(), 90.0)
                  .isApprox(Eigen::Vector3d::UnitZ(), 1e-12));
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
