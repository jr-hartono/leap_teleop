#include "leap_teleop_core/delta_builder.hpp"

namespace leap_teleop {

void DeltaBuilder::latch(const geometry_msgs::msg::Pose& pose) {
  latched_position_ = point_from_msg(pose.position);
  latched_orientation_ = quaternion_from_msg(pose.orientation);
  latched_ = true;
}

TransformDelta DeltaBuilder::compute(const geometry_msgs::msg::Pose& current,
                                     double translation_scale, double rotation_scale) const {
  TransformDelta delta;
  if (!latched_) {
    return delta;
  }
  delta.translation = translation_scale * (point_from_msg(current.position) - latched_position_);
  const Eigen::Quaterniond orientation = quaternion_from_msg(current.orientation);
  delta.rotation = quaternion_power(world_delta(orientation, latched_orientation_), rotation_scale);
  return delta;
}

bool DeltaBuilder::latched() const { return latched_; }

geometry_msgs::msg::Transform to_transform_msg(const TransformDelta& delta) {
  geometry_msgs::msg::Transform msg;
  msg.translation = vector_to_msg(delta.translation);
  msg.rotation = quaternion_to_msg(delta.rotation);
  return msg;
}

TransformDelta delta_from_transform_msg(const geometry_msgs::msg::Transform& msg) {
  TransformDelta delta;
  delta.translation = vector_from_msg(msg.translation);
  delta.rotation = quaternion_from_msg(msg.rotation);
  return delta;
}

geometry_msgs::msg::Pose apply_delta(const geometry_msgs::msg::Pose& latched,
                                     const TransformDelta& delta) {
  geometry_msgs::msg::Pose result;
  result.position = point_to_msg(point_from_msg(latched.position) + delta.translation);
  const Eigen::Quaterniond latched_orientation = quaternion_from_msg(latched.orientation);
  // Rotate about the tool origin in base axes; a full transform would swing the position.
  const Eigen::Quaterniond rotated = delta.rotation * latched_orientation;
  result.orientation = quaternion_to_msg(rotated);
  return result;
}

}  // namespace leap_teleop
