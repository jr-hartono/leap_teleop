#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <string>

namespace leap_teleop {

constexpr double kPi = 3.14159265358979323846;

double degrees_to_radians(double degrees);
double radians_to_degrees(double radians);

// How the Leap device is mounted. Selects the device -> operator axis mapping.
enum class MountMode {
  // Flat on the desk, cameras up, green LED edge towards the operator.
  kDesktop,
  // On the forehead, cameras facing forward away from the face, the green LED edge facing down
  // and the end that is on the operator's right on a desk on the operator's left.
  kHeadMounted,
};

// Parses the `mount_mode` parameter ("desktop" | "head_mounted"). Throws std::invalid_argument.
MountMode mount_mode_from_string(const std::string& name);

// Step 1: Leap device axes -> operator axes (+x forward from the operator, +y left, +z up).
Eigen::Matrix3d device_to_operator_rotation(MountMode mode);
Eigen::Vector3d device_position_to_operator(const Eigen::Vector3d& position, MountMode mode);
Eigen::Quaterniond device_orientation_to_operator(const Eigen::Quaterniond& orientation,
                                                  MountMode mode);
// Step 1 as the static transform leap_operator_frame -> leap_device_frame.
geometry_msgs::msg::Transform operator_to_device_transform(MountMode mode);

// Step 2: operator axes -> robot base axes, a rotation by `yaw_deg` about +z.
Eigen::Matrix3d yaw_rotation(double yaw_deg);
Eigen::Vector3d operator_position_to_base(const Eigen::Vector3d& position, double yaw_deg);
Eigen::Quaterniond operator_orientation_to_base(const Eigen::Quaterniond& orientation,
                                                double yaw_deg);

// Both steps: Leap device axes -> robot base axes.
Eigen::Matrix3d device_to_base_rotation(double yaw_deg, MountMode mode);
Eigen::Vector3d leap_position_to_base(const Eigen::Vector3d& position_m, double yaw_deg,
                                      MountMode mode);
Eigen::Quaterniond leap_orientation_to_base(const Eigen::Quaterniond& orientation, double yaw_deg,
                                            MountMode mode);

Eigen::Quaterniond normalized(const Eigen::Quaterniond& q);
Eigen::Quaterniond quaternion_power(const Eigen::Quaterniond& q, double exponent);
Eigen::Quaterniond slerp(const Eigen::Quaterniond& a, const Eigen::Quaterniond& b, double t);
Eigen::Quaterniond axis_angle(double x, double y, double z, double angle_rad);
double rotation_angle(const Eigen::Quaterniond& q);

Eigen::Quaterniond world_delta(const Eigen::Quaterniond& current,
                               const Eigen::Quaterniond& reference);

Eigen::Vector3d vector_from_msg(const geometry_msgs::msg::Vector3& msg);
geometry_msgs::msg::Vector3 vector_to_msg(const Eigen::Vector3d& value);
Eigen::Vector3d point_from_msg(const geometry_msgs::msg::Point& msg);
geometry_msgs::msg::Point point_to_msg(const Eigen::Vector3d& value);
Eigen::Quaterniond quaternion_from_msg(const geometry_msgs::msg::Quaternion& msg);
geometry_msgs::msg::Quaternion quaternion_to_msg(const Eigen::Quaterniond& value);

}  // namespace leap_teleop
