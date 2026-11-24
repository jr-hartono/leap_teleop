#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdint>
#include <geometry_msgs/msg/pose.hpp>
#include <string>

#include "leap_teleop_core/filters.hpp"

namespace leap_teleop {

struct HandMapperConfig {
  double yaw_deg = 0.0;
  double translation_gain = 1.0;
  double rotation_gain = 1.0;
  double deadband_translation_m = 0.002;
  double deadband_rotation_rad = 0.5 * kPi / 180.0;
  double deadband_hysteresis = 1.5;
  double filter_min_cutoff_hz = 1.0;
  double filter_beta = 0.05;
};

struct HandPoseInput {
  Eigen::Vector3d position_m = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
  double time_sec = 0.0;
  std::string side;
};

class HandMapper {
 public:
  explicit HandMapper(const HandMapperConfig& config);

  geometry_msgs::msg::Pose update(const HandPoseInput& input);
  void reset();

 private:
  HandMapperConfig config_;
  OneEuroFilter filter_x_;
  OneEuroFilter filter_y_;
  OneEuroFilter filter_z_;
  OrientationFilter orientation_filter_;
  TranslationDeadband translation_deadband_;
  RotationDeadband rotation_deadband_;
};

}  // namespace leap_teleop
