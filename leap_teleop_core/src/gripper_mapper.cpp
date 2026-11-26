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

ClosureFilter::ClosureFilter(const GripperMapperConfig& config)
    : config_(config), filter_(config.filter_min_cutoff_hz, config.filter_beta) {}

ClosureSample ClosureFilter::update(double pinch_mm, double time_sec) {
  ClosureSample sample;
  sample.raw = pinch_to_closure(pinch_mm, config_.pinch_closed_mm, config_.pinch_open_mm);
  sample.filtered = std::clamp(filter_.filter(sample.raw, time_sec), 0.0, 1.0);
  return sample;
}

void ClosureFilter::reset() { filter_.reset(); }

void ClosureFilter::set_config(const GripperMapperConfig& config) {
  config_ = config;
  filter_.set_parameters(config.filter_min_cutoff_hz, config.filter_beta);
}

ClosureGate::ClosureGate(const GripperMapperConfig& config) : config_(config) {}

void ClosureGate::begin_engagement() { last_emitted_.reset(); }

std::optional<float> ClosureGate::update(const ClosureSample& sample) {
  double closure = sample.filtered;
  const bool at_end = sample.raw == 0.0 || sample.raw == 1.0;
  if (at_end && std::abs(closure - sample.raw) <= config_.deadband) {
    closure = sample.raw;
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

void ClosureGate::set_config(const GripperMapperConfig& config) { config_ = config; }

}  // namespace leap_teleop
