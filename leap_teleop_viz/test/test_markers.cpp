#include <gtest/gtest.h>

#include "leap_teleop_core/delta_builder.hpp"
#include "leap_teleop_core/frames.hpp"
#include "leap_teleop_viz/markers.hpp"

using leap_teleop::apply_delta;
using leap_teleop::axis_angle;
using leap_teleop::degrees_to_radians;
using leap_teleop::point_from_msg;
using leap_teleop::quaternion_from_msg;
using leap_teleop::quaternion_power;
using leap_teleop::rotation_angle;
using leap_teleop::TransformDelta;
using leap_teleop::viz::unscale_delta;

namespace {

TransformDelta make_delta(const Eigen::Vector3d& translation, const Eigen::Quaterniond& rotation) {
  TransformDelta delta;
  delta.translation = translation;
  delta.rotation = rotation;
  return delta;
}

// The mapper's scaling, as in DeltaBuilder::compute.
TransformDelta scale(const TransformDelta& delta, double translation_scale, double rotation_scale) {
  return make_delta(translation_scale * delta.translation,
                    quaternion_power(delta.rotation, rotation_scale));
}

bool same_rotation(const Eigen::Quaterniond& a, const Eigen::Quaterniond& b) {
  return std::abs(a.dot(b)) > 1.0 - 1e-9;
}

geometry_msgs::msg::Pose make_pose(const Eigen::Vector3d& position,
                                   const Eigen::Quaterniond& orientation) {
  geometry_msgs::msg::Pose pose;
  pose.position = leap_teleop::point_to_msg(position);
  pose.orientation = leap_teleop::quaternion_to_msg(orientation);
  return pose;
}

}  // namespace

TEST(UnscaleDelta, UndoesTheMappersScaling) {
  const TransformDelta hand = make_delta(Eigen::Vector3d(0.1, -0.05, 0.02),
                                         axis_angle(1.0, 2.0, -0.5, degrees_to_radians(40.0)));
  const TransformDelta unscaled = unscale_delta(scale(hand, 2.5, 0.5), 2.5, 0.5);
  EXPECT_TRUE(unscaled.translation.isApprox(hand.translation, 1e-12));
  EXPECT_TRUE(same_rotation(unscaled.rotation, hand.rotation));
}

TEST(UnscaleDelta, RescalingReturnsTheOriginal) {
  const TransformDelta scaled = make_delta(Eigen::Vector3d(0.3, 0.0, -0.1),
                                           axis_angle(0.0, 1.0, 1.0, degrees_to_radians(60.0)));
  const TransformDelta round_trip = scale(unscale_delta(scaled, 3.0, 2.0), 3.0, 2.0);
  EXPECT_TRUE(round_trip.translation.isApprox(scaled.translation, 1e-12));
  EXPECT_TRUE(same_rotation(round_trip.rotation, scaled.rotation));
}

TEST(UnscaleDelta, ScaleOneIsIdentity) {
  const TransformDelta delta = make_delta(Eigen::Vector3d(0.1, 0.2, 0.3),
                                          axis_angle(1.0, 0.0, 0.0, degrees_to_radians(25.0)));
  const TransformDelta unscaled = unscale_delta(delta, 1.0, 1.0);
  EXPECT_TRUE(unscaled.translation.isApprox(delta.translation, 1e-12));
  EXPECT_TRUE(same_rotation(unscaled.rotation, delta.rotation));
}

TEST(UnscaleDelta, DividesTheAngleAndKeepsTheAxis) {
  const Eigen::Vector3d axis = Eigen::Vector3d(1.0, -1.0, 2.0).normalized();
  const TransformDelta delta =
      make_delta(Eigen::Vector3d::Zero(), axis_angle(axis.x(), axis.y(), axis.z(), 1.2));
  const TransformDelta unscaled = unscale_delta(delta, 1.0, 3.0);
  EXPECT_NEAR(rotation_angle(unscaled.rotation), 0.4, 1e-12);
  const Eigen::AngleAxisd angle_axis(unscaled.rotation);
  EXPECT_TRUE(angle_axis.axis().isApprox(axis, 1e-9));
}

TEST(UnscaleDelta, IdentityRotationStaysIdentity) {
  const TransformDelta unscaled =
      unscale_delta(make_delta(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity()), 2.0, 0.5);
  EXPECT_NEAR(rotation_angle(unscaled.rotation), 0.0, 1e-12);
}

// The viz draws apply_delta(latched tool, delta), the bridge's target.
TEST(Target, TranslationInBaseAxesAndRotationAboutTheToolOrigin) {
  const auto latched = make_pose(Eigen::Vector3d(0.4, 0.1, 0.5),
                                 axis_angle(1.0, 0.0, 0.0, degrees_to_radians(180.0)));
  const TransformDelta delta = make_delta(Eigen::Vector3d(0.0, 0.0, 0.1),
                                          axis_angle(0.0, 0.0, 1.0, degrees_to_radians(90.0)));
  const auto target = apply_delta(latched, delta);
  EXPECT_TRUE(point_from_msg(target.position).isApprox(Eigen::Vector3d(0.4, 0.1, 0.6), 1e-12));
  const Eigen::Matrix3d rotation = quaternion_from_msg(target.orientation).toRotationMatrix();
  // Tool x was base +x; after +90 deg about base z it is base +y. Tool z was base -z and stays.
  EXPECT_TRUE(rotation.col(0).isApprox(Eigen::Vector3d::UnitY(), 1e-12));
  EXPECT_TRUE(rotation.col(2).isApprox(-Eigen::Vector3d::UnitZ(), 1e-12));
}

TEST(Axes, ArrowsAlongThePoseAxes) {
  const Eigen::Vector3d origin(1.0, 2.0, 3.0);
  const Eigen::Quaterniond orientation = axis_angle(0.0, 0.0, 1.0, degrees_to_radians(90.0));
  const auto markers = leap_teleop::viz::axes(make_pose(origin, orientation), 0.2, 0.01, 0.5F);
  const Eigen::Vector3d expected[3] = {Eigen::Vector3d(0.0, 0.2, 0.0),
                                       Eigen::Vector3d(-0.2, 0.0, 0.0),
                                       Eigen::Vector3d(0.0, 0.0, 0.2)};
  for (int axis = 0; axis < 3; ++axis) {
    const auto& marker = markers[axis];
    EXPECT_EQ(marker.type, visualization_msgs::msg::Marker::ARROW);
    ASSERT_EQ(marker.points.size(), 2U);
    EXPECT_TRUE(point_from_msg(marker.points[0]).isApprox(origin, 1e-12));
    EXPECT_TRUE((point_from_msg(marker.points[1]) - origin).isApprox(expected[axis], 1e-12));
    EXPECT_FLOAT_EQ(marker.color.a, 0.5F);
  }
  EXPECT_FLOAT_EQ(markers[0].color.r, 1.0F);
  EXPECT_FLOAT_EQ(markers[1].color.g, 1.0F);
  EXPECT_FLOAT_EQ(markers[2].color.b, 1.0F);
}

namespace {

Eigen::Isometry3d isometry(const geometry_msgs::msg::Transform& transform) {
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translate(leap_teleop::vector_from_msg(transform.translation));
  result.rotate(quaternion_from_msg(transform.rotation));
  return result;
}

}  // namespace

// A point in the device frame, through the published transforms, lands where the mapper
// puts it in base axes, shifted by the display offset.
TEST(HandFrames, DeviceToBaseMatchesTheTeleopMapping) {
  using leap_teleop::MountMode;
  const Eigen::Vector3d offset(0.5, -0.1, 0.3);
  const Eigen::Vector3d point(0.03, 0.2, -0.05);
  for (const MountMode mode : {MountMode::kDesktop, MountMode::kHeadMounted}) {
    for (const double yaw : {0.0, 90.0, 180.0}) {
      const Eigen::Isometry3d base_from_device =
          isometry(leap_teleop::viz::base_to_operator(yaw, offset)) *
          isometry(leap_teleop::viz::operator_to_device(mode));
      const Eigen::Vector3d expected =
          leap_teleop::leap_position_to_base(point, yaw, mode) + offset;
      EXPECT_TRUE((base_from_device * point).isApprox(expected, 1e-12))
          << "yaw " << yaw << " mode " << static_cast<int>(mode);
    }
  }
}

TEST(HandFrames, FilteredPalmAxesMatchTheRawPalmAxes) {
  using leap_teleop::MountMode;
  const Eigen::Quaterniond device_orientation = axis_angle(0.3, -1.0, 0.5, 0.8);
  for (const MountMode mode : {MountMode::kDesktop, MountMode::kHeadMounted}) {
    const Eigen::Quaterniond filtered =
        leap_teleop::device_orientation_to_operator(device_orientation, mode);
    const Eigen::Quaterniond raw_in_operator =
        quaternion_from_msg(leap_teleop::viz::operator_to_device(mode).rotation) *
        device_orientation;
    EXPECT_TRUE(same_rotation(filtered, raw_in_operator));
  }
}
