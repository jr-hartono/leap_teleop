#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>

#include "leap_teleop_core/filters.hpp"

namespace leap_teleop {

struct HandMapperConfig {
  TrackingMode tracking_mode = TrackingMode::kDesktop;
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
};

// Device axes -> operator axes (REP 103), filtered. Yaw to the robot base is the mapper's job.
class HandMapper {
 public:
  explicit HandMapper(const HandMapperConfig& config);

  geometry_msgs::msg::Pose update(const HandPoseInput& input);
  // Updates tuning fields only; filter, deadband and frame state is preserved.
  void set_tuning(const HandMapperConfig& config);
  // Changes the device mounting and restarts filters, as the old and new poses do not connect.
  void set_tracking_mode(TrackingMode mode);
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

// Leap pinch and grab values for the messages. Strengths are limited to [0, 1], distance to >= 0;
// NaN becomes 0.
float clamp_unit_strength(float value);
float non_negative_distance(float value);

}  // namespace leap_teleop
