#include "leap_teleop_core/gripper_mapper.hpp"

#include <algorithm>
#include <cmath>

namespace leap_teleop {

bool valid_pinch_range(double pinch_closed_mm, double pinch_open_mm) {
  return pinch_closed_mm > 0.0 && pinch_open_mm > 0.0 && pinch_closed_mm < pinch_open_mm;
}

double pinch_to_closure(double pinch_mm, double pinch_closed_mm, double pinch_open_mm) {
  const double closure = (pinch_open_mm - pinch_mm) / (pinch_open_mm - pinch_closed_mm);
  if (!(closure > 0.0)) {
    return 0.0;
  }
  return std::min(closure, 1.0);
}

GripperMapper::GripperMapper(const GripperMapperConfig& config)
    : config_(config), filter_(config.filter_min_cutoff_hz, config.filter_beta) {}

void GripperMapper::begin_engagement() {
  filter_.reset();
  last_emitted_.reset();
}

std::optional<float> GripperMapper::update(double pinch_mm, double time_sec) {
  const double raw = pinch_to_closure(pinch_mm, config_.pinch_closed_mm, config_.pinch_open_mm);
  double closure = std::clamp(filter_.filter(raw, time_sec), 0.0, 1.0);
  const bool at_end = raw == 0.0 || raw == 1.0;
  if (at_end && std::abs(closure - raw) <= config_.deadband) {
    closure = raw;
  }
  const auto value = static_cast<float>(closure);
  if (!last_emitted_) {
    last_emitted_ = value;
    return value;
  }
  const bool moved = std::abs(static_cast<double>(value) - *last_emitted_) > config_.deadband;
  const bool end_reached = (value == 0.0F || value == 1.0F) && value != *last_emitted_;
  if (moved || end_reached) {
    last_emitted_ = value;
    return value;
  }
  return std::nullopt;
}

void GripperMapper::set_config(const GripperMapperConfig& config) {
  config_ = config;
  filter_.set_parameters(config.filter_min_cutoff_hz, config.filter_beta);
}

}  // namespace leap_teleop
