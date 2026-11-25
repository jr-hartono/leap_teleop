#include "leap_teleop_viz/markers.hpp"

#include "leap_teleop_core/frames.hpp"

namespace leap_teleop::viz {

std_msgs::msg::ColorRGBA color(float r, float g, float b, float a) {
  std_msgs::msg::ColorRGBA rgba;
  rgba.r = r;
  rgba.g = g;
  rgba.b = b;
  rgba.a = a;
  return rgba;
}

TransformDelta unscale_delta(const TransformDelta& delta, double translation_scale,
                             double rotation_scale) {
  TransformDelta unscaled = delta;
  if (translation_scale > 0.0) {
    unscaled.translation = delta.translation / translation_scale;
  }
  if (rotation_scale > 0.0) {
    unscaled.rotation = quaternion_power(delta.rotation, 1.0 / rotation_scale);
  }
  return unscaled;
}

visualization_msgs::msg::Marker arrow(const Eigen::Vector3d& start, const Eigen::Vector3d& vector,
                                      double diameter, const std_msgs::msg::ColorRGBA& rgba) {
  visualization_msgs::msg::Marker marker;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.points.push_back(point_to_msg(start));
  marker.points.push_back(point_to_msg(start + vector));
  // With points: shaft diameter, head diameter, head length.
  marker.scale.x = diameter;
  marker.scale.y = 2.0 * diameter;
  marker.scale.z = 3.0 * diameter;
  marker.color = rgba;
  return marker;
}

namespace {

std::array<visualization_msgs::msg::Marker, 3> axes_with_colors(
    const geometry_msgs::msg::Pose& pose, double length, double diameter,
    const std::array<std_msgs::msg::ColorRGBA, 3>& colors) {
  const Eigen::Vector3d origin = point_from_msg(pose.position);
  const Eigen::Matrix3d rotation = quaternion_from_msg(pose.orientation).toRotationMatrix();
  std::array<visualization_msgs::msg::Marker, 3> markers;
  for (int axis = 0; axis < 3; ++axis) {
    markers[axis] = arrow(origin, length * rotation.col(axis), diameter, colors[axis]);
  }
  return markers;
}

}  // namespace

std::array<visualization_msgs::msg::Marker, 3> axes(const geometry_msgs::msg::Pose& pose,
                                                    double length, double diameter, float alpha) {
  return axes_with_colors(pose, length, diameter,
                          {color(1.0F, 0.0F, 0.0F, alpha), color(0.0F, 1.0F, 0.0F, alpha),
                           color(0.0F, 0.0F, 1.0F, alpha)});
}

std::array<visualization_msgs::msg::Marker, 3> axes(const geometry_msgs::msg::Pose& pose,
                                                    double length, double diameter,
                                                    const std_msgs::msg::ColorRGBA& rgba) {
  return axes_with_colors(pose, length, diameter, {rgba, rgba, rgba});
}

geometry_msgs::msg::Transform base_to_operator(double yaw_deg, const Eigen::Vector3d& offset) {
  geometry_msgs::msg::Transform transform;
  transform.translation = vector_to_msg(offset);
  transform.rotation = quaternion_to_msg(Eigen::Quaterniond(yaw_rotation(yaw_deg)));
  return transform;
}

geometry_msgs::msg::Transform operator_to_device(MountMode mode) {
  geometry_msgs::msg::Transform transform;
  transform.rotation = quaternion_to_msg(Eigen::Quaterniond(device_to_operator_rotation(mode)));
  return transform;
}

Eigen::Quaterniond palm_axes_in_operator(const Eigen::Quaterniond& operator_orientation,
                                         MountMode mode) {
  return normalized(operator_orientation * Eigen::Quaterniond(device_to_operator_rotation(mode)));
}

}  // namespace leap_teleop::viz
