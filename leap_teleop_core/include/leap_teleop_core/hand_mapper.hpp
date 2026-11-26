#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>
#include <cstdint>
#include <geometry_msgs/msg/pose.hpp>
#include <optional>

#include "leap_teleop_core/filters.hpp"

namespace leap_teleop {

struct HandMapperConfig {
  MountMode mount_mode = MountMode::kDesktop;
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

enum class HandSide { kLeft = 0, kRight = 1 };

// One HandMapper per side, so a hand that disappears resets only its own filters and deadbands.
class PerSideMapper {
 public:
  explicit PerSideMapper(const HandMapperConfig& config);

  // A present hand is mapped; an absent one (nullopt) resets that side only and returns nullopt.
  std::optional<geometry_msgs::msg::Pose> update(HandSide side,
                                                 const std::optional<HandPoseInput>& input);
  void set_tuning(const HandMapperConfig& config);
  void reset_all();

 private:
  std::array<HandMapper, 2> mappers_;
};

}  // namespace leap_teleop
