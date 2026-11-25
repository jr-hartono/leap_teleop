#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform.hpp>

#include "leap_teleop_core/frames.hpp"

namespace leap_teleop {

struct TransformDelta {
  Eigen::Vector3d translation = Eigen::Vector3d::Zero();
  Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
};

class DeltaBuilder {
 public:
  void latch(const geometry_msgs::msg::Pose& pose);
  // translation_scale multiplies the translation since the latch. rotation_scale multiplies the
  // rotation angle since the latch, about the same axis.
  [[nodiscard]] TransformDelta compute(const geometry_msgs::msg::Pose& current,
                                       double translation_scale, double rotation_scale) const;
  [[nodiscard]] bool latched() const;

 private:
  Eigen::Vector3d latched_position_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond latched_orientation_ = Eigen::Quaterniond::Identity();
  bool latched_ = false;
};

// Rotates a pose from the operator frame into the robot base frame by yaw about +z.
geometry_msgs::msg::Pose operator_pose_to_base(const geometry_msgs::msg::Pose& pose,
                                               double yaw_deg);

geometry_msgs::msg::Transform to_transform_msg(const TransformDelta& delta);

TransformDelta delta_from_transform_msg(const geometry_msgs::msg::Transform& msg);

geometry_msgs::msg::Pose apply_delta(const geometry_msgs::msg::Pose& latched,
                                     const TransformDelta& delta);

}  // namespace leap_teleop
