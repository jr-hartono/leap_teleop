#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>

namespace leap_teleop {

constexpr double kPi = 3.14159265358979323846;

double degrees_to_radians(double degrees);
double radians_to_degrees(double radians);

Eigen::Matrix3d leap_to_ros_rotation();
Eigen::Matrix3d yaw_rotation(double yaw_deg);
Eigen::Matrix3d device_to_base_rotation(double yaw_deg);

Eigen::Vector3d leap_position_to_base(const Eigen::Vector3d& position_m, double yaw_deg);
Eigen::Quaterniond leap_orientation_to_base(const Eigen::Quaterniond& orientation, double yaw_deg);

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
