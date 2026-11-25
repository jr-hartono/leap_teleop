#include "leap_teleop_core/frames.hpp"

#include <algorithm>
#include <cmath>

namespace leap_teleop {

double degrees_to_radians(double degrees) { return degrees * kPi / 180.0; }

double radians_to_degrees(double radians) { return radians * 180.0 / kPi; }

Eigen::Matrix3d leap_to_ros_rotation() {
  Eigen::Matrix3d rotation;
  rotation << 0.0, 0.0, -1.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0;
  return rotation;
}

Eigen::Matrix3d yaw_rotation(double yaw_deg) {
  return Eigen::AngleAxisd(degrees_to_radians(yaw_deg), Eigen::Vector3d::UnitZ())
      .toRotationMatrix();
}

Eigen::Matrix3d device_to_base_rotation(double yaw_deg) {
  return yaw_rotation(yaw_deg) * leap_to_ros_rotation();
}

Eigen::Vector3d leap_position_to_base(const Eigen::Vector3d& position_m, double yaw_deg) {
  return device_to_base_rotation(yaw_deg) * position_m;
}

Eigen::Quaterniond leap_orientation_to_base(const Eigen::Quaterniond& orientation, double yaw_deg) {
  const Eigen::Matrix3d rotation = device_to_base_rotation(yaw_deg);
  const Eigen::Matrix3d in_base =
      rotation * normalized(orientation).toRotationMatrix() * rotation.transpose();
  return normalized(Eigen::Quaterniond(in_base));
}

Eigen::Quaterniond normalized(const Eigen::Quaterniond& q) {
  Eigen::Quaterniond result(q);
  const double norm = result.norm();
  if (norm < 1e-12 || !std::isfinite(norm)) {
    return Eigen::Quaterniond::Identity();
  }
  result.coeffs() /= norm;
  return result;
}

Eigen::Quaterniond axis_angle(double x, double y, double z, double angle_rad) {
  const Eigen::Vector3d axis(x, y, z);
  const double norm = axis.norm();
  if (norm < 1e-12 || std::abs(angle_rad) < 1e-12) {
    return Eigen::Quaterniond::Identity();
  }
  return normalized(Eigen::Quaterniond(Eigen::AngleAxisd(angle_rad, axis / norm)));
}

double rotation_angle(const Eigen::Quaterniond& q) {
  const Eigen::Quaterniond unit = normalized(q);
  return 2.0 * std::acos(std::clamp(std::abs(unit.w()), -1.0, 1.0));
}

Eigen::Quaterniond quaternion_power(const Eigen::Quaterniond& q, double exponent) {
  Eigen::Quaterniond unit = normalized(q);
  if (unit.w() < 0.0) {
    unit.coeffs() = -unit.coeffs();
  }
  const double angle = 2.0 * std::acos(std::clamp(unit.w(), -1.0, 1.0));
  if (angle < 1e-10) {
    return Eigen::Quaterniond::Identity();
  }
  const Eigen::Vector3d vector(unit.x(), unit.y(), unit.z());
  const double sin_half = std::sin(angle / 2.0);
  const Eigen::Vector3d axis = (sin_half > 1e-10) ? vector / sin_half : vector.normalized();
  return axis_angle(axis.x(), axis.y(), axis.z(), exponent * angle);
}

Eigen::Quaterniond slerp(const Eigen::Quaterniond& a, const Eigen::Quaterniond& b, double t) {
  Eigen::Quaterniond from = normalized(a);
  Eigen::Quaterniond to = normalized(b);
  if (t <= 0.0) {
    return from;
  }
  if (t >= 1.0) {
    return to;
  }
  if (from.dot(to) < 0.0) {
    to.coeffs() = -to.coeffs();
  }
  const double cosine = std::clamp(from.dot(to), -1.0, 1.0);
  if (cosine > 0.9995) {
    return normalized(Eigen::Quaterniond(from.coeffs() + t * (to.coeffs() - from.coeffs())));
  }
  const double theta = std::acos(cosine);
  const double sine = std::sin(theta);
  Eigen::Quaterniond result;
  result.coeffs() = (std::sin((1.0 - t) * theta) / sine) * from.coeffs() +
                    (std::sin(t * theta) / sine) * to.coeffs();
  return normalized(result);
}

Eigen::Quaterniond world_delta(const Eigen::Quaterniond& current,
                               const Eigen::Quaterniond& reference) {
  return normalized(normalized(current) * normalized(reference).inverse());
}

Eigen::Vector3d vector_from_msg(const geometry_msgs::msg::Vector3& msg) {
  return Eigen::Vector3d(msg.x, msg.y, msg.z);
}

geometry_msgs::msg::Vector3 vector_to_msg(const Eigen::Vector3d& value) {
  geometry_msgs::msg::Vector3 msg;
  msg.x = value.x();
  msg.y = value.y();
  msg.z = value.z();
  return msg;
}

Eigen::Vector3d point_from_msg(const geometry_msgs::msg::Point& msg) {
  return Eigen::Vector3d(msg.x, msg.y, msg.z);
}

geometry_msgs::msg::Point point_to_msg(const Eigen::Vector3d& value) {
  geometry_msgs::msg::Point msg;
  msg.x = value.x();
  msg.y = value.y();
  msg.z = value.z();
  return msg;
}

Eigen::Quaterniond quaternion_from_msg(const geometry_msgs::msg::Quaternion& msg) {
  return normalized(Eigen::Quaterniond(msg.w, msg.x, msg.y, msg.z));
}

geometry_msgs::msg::Quaternion quaternion_to_msg(const Eigen::Quaterniond& value) {
  const Eigen::Quaterniond unit = normalized(value);
  geometry_msgs::msg::Quaternion msg;
  msg.w = unit.w();
  msg.x = unit.x();
  msg.y = unit.y();
  msg.z = unit.z();
  return msg;
}

}  // namespace leap_teleop
