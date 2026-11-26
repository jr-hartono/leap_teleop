#include "leap_teleop_core/hand_mapper.hpp"

namespace leap_teleop {

float clamp_unit_strength(float value) {
  if (!(value > 0.0F)) {
    return 0.0F;
  }
  return value > 1.0F ? 1.0F : value;
}

float non_negative_distance(float value) { return value > 0.0F ? value : 0.0F; }

HandMapper::HandMapper(const HandMapperConfig& config)
    : config_(config),
      filter_x_(config.filter_min_cutoff_hz, config.filter_beta),
      filter_y_(config.filter_min_cutoff_hz, config.filter_beta),
      filter_z_(config.filter_min_cutoff_hz, config.filter_beta),
      orientation_filter_(config.filter_min_cutoff_hz),
      translation_deadband_(config.deadband_translation_m, config.deadband_hysteresis),
      rotation_deadband_(config.deadband_rotation_rad, config.deadband_hysteresis) {}

geometry_msgs::msg::Pose HandMapper::update(const HandPoseInput& input) {
  const Eigen::Vector3d in_operator =
      device_position_to_operator(input.position_m, config_.mount_mode);
  Eigen::Vector3d filtered;
  filtered.x() = filter_x_.filter(in_operator.x(), input.time_sec);
  filtered.y() = filter_y_.filter(in_operator.y(), input.time_sec);
  filtered.z() = filter_z_.filter(in_operator.z(), input.time_sec);

  const Eigen::Vector3d with_deadband = translation_deadband_.filter(filtered, input.time_sec);

  // No scaling here: the mapper scales the change since engagement (DeltaBuilder::compute).
  const Eigen::Quaterniond operator_orientation =
      device_orientation_to_operator(input.orientation, config_.mount_mode);
  const Eigen::Quaterniond filtered_orientation =
      orientation_filter_.filter(operator_orientation, input.time_sec);
  const Eigen::Quaterniond output_orientation =
      rotation_deadband_.filter(filtered_orientation, input.time_sec);

  geometry_msgs::msg::Pose pose;
  pose.position = point_to_msg(with_deadband);
  pose.orientation = quaternion_to_msg(output_orientation);
  return pose;
}

void HandMapper::set_tuning(const HandMapperConfig& config) {
  config_.deadband_translation_m = config.deadband_translation_m;
  config_.deadband_rotation_rad = config.deadband_rotation_rad;
  config_.deadband_hysteresis = config.deadband_hysteresis;
  config_.filter_min_cutoff_hz = config.filter_min_cutoff_hz;
  config_.filter_beta = config.filter_beta;

  filter_x_.set_parameters(config_.filter_min_cutoff_hz, config_.filter_beta);
  filter_y_.set_parameters(config_.filter_min_cutoff_hz, config_.filter_beta);
  filter_z_.set_parameters(config_.filter_min_cutoff_hz, config_.filter_beta);
  orientation_filter_.set_min_cutoff(config_.filter_min_cutoff_hz);
  translation_deadband_.set_parameters(config_.deadband_translation_m, config_.deadband_hysteresis);
  rotation_deadband_.set_parameters(config_.deadband_rotation_rad, config_.deadband_hysteresis);
}

void HandMapper::reset() {
  filter_x_.reset();
  filter_y_.reset();
  filter_z_.reset();
  orientation_filter_.reset();
  translation_deadband_.reset();
  rotation_deadband_.reset();
}

PerSideMapper::PerSideMapper(const HandMapperConfig& config)
    : mappers_{HandMapper(config), HandMapper(config)} {}

std::optional<geometry_msgs::msg::Pose> PerSideMapper::update(
    HandSide side, const std::optional<HandPoseInput>& input) {
  HandMapper& mapper = mappers_[static_cast<std::size_t>(side)];
  if (!input) {
    mapper.reset();
    return std::nullopt;
  }
  return mapper.update(*input);
}

void PerSideMapper::set_tuning(const HandMapperConfig& config) {
  for (HandMapper& mapper : mappers_) {
    mapper.set_tuning(config);
  }
}

void PerSideMapper::reset_all() {
  for (HandMapper& mapper : mappers_) {
    mapper.reset();
  }
}

}  // namespace leap_teleop
