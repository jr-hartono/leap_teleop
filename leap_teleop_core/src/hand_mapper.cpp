#include "leap_teleop_core/hand_mapper.hpp"

namespace leap_teleop {

HandMapper::HandMapper(const HandMapperConfig& config)
    : config_(config),
      filter_x_(config.filter_min_cutoff_hz, config.filter_beta),
      filter_y_(config.filter_min_cutoff_hz, config.filter_beta),
      filter_z_(config.filter_min_cutoff_hz, config.filter_beta),
      orientation_filter_(config.filter_min_cutoff_hz),
      translation_deadband_(config.deadband_translation_m, config.deadband_hysteresis),
      rotation_deadband_(config.deadband_rotation_rad, config.deadband_hysteresis) {}

geometry_msgs::msg::Pose HandMapper::update(const HandPoseInput& input) {
  const Eigen::Vector3d in_base = leap_position_to_base(input.position_m, config_.yaw_deg);
  Eigen::Vector3d filtered;
  filtered.x() = filter_x_.filter(in_base.x(), input.time_sec);
  filtered.y() = filter_y_.filter(in_base.y(), input.time_sec);
  filtered.z() = filter_z_.filter(in_base.z(), input.time_sec);

  const Eigen::Vector3d with_deadband = translation_deadband_.filter(filtered, input.time_sec);
  const Eigen::Vector3d output_position = config_.translation_scale * with_deadband;

  const Eigen::Quaterniond scaled = quaternion_power(input.orientation, config_.rotation_scale);
  const Eigen::Quaterniond in_base_orientation = leap_orientation_to_base(scaled, config_.yaw_deg);
  const Eigen::Quaterniond filtered_orientation =
      orientation_filter_.filter(in_base_orientation, input.time_sec);
  const Eigen::Quaterniond output_orientation =
      rotation_deadband_.filter(filtered_orientation, input.time_sec);

  geometry_msgs::msg::Pose pose;
  pose.position = point_to_msg(output_position);
  pose.orientation = quaternion_to_msg(output_orientation);
  return pose;
}

void HandMapper::reset() {
  filter_x_.reset();
  filter_y_.reset();
  filter_z_.reset();
  orientation_filter_.reset();
  translation_deadband_.reset();
  rotation_deadband_.reset();
}

}  // namespace leap_teleop
