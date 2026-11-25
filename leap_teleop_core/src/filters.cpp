#include "leap_teleop_core/filters.hpp"

#include <algorithm>
#include <cmath>

namespace leap_teleop {

namespace {

constexpr double kStillTimeSec = 0.1;

}  // namespace

OneEuroFilter::OneEuroFilter(double min_cutoff_hz, double beta, double derivative_cutoff_hz)
    : min_cutoff_hz_(min_cutoff_hz), beta_(beta), derivative_cutoff_hz_(derivative_cutoff_hz) {}

double OneEuroFilter::alpha(double cutoff_hz, double dt) {
  if (cutoff_hz <= 0.0 || dt <= 0.0) {
    return 0.0;
  }
  return 1.0 - std::exp(-2.0 * kPi * cutoff_hz * dt);
}

double OneEuroFilter::filter(double value, double time_sec) {
  if (!std::isfinite(value)) {
    return last_value_;
  }
  if (!initialized_) {
    initialized_ = true;
    last_value_ = value;
    last_time_ = time_sec;
    return value;
  }
  const double dt = time_sec - last_time_;
  last_time_ = time_sec;
  if (!(dt > 0.0) || !std::isfinite(dt)) {
    return last_value_;
  }
  const double derivative = (value - last_value_) / dt;
  last_derivative_ += alpha(derivative_cutoff_hz_, dt) * (derivative - last_derivative_);
  const double cutoff = min_cutoff_hz_ + beta_ * std::abs(last_derivative_);
  last_value_ += alpha(cutoff, dt) * (value - last_value_);
  return last_value_;
}

void OneEuroFilter::set_parameters(double min_cutoff_hz, double beta) {
  min_cutoff_hz_ = min_cutoff_hz;
  beta_ = beta;
}

void OneEuroFilter::reset() {
  initialized_ = false;
  last_value_ = 0.0;
  last_derivative_ = 0.0;
  last_time_ = 0.0;
}

OrientationFilter::OrientationFilter(double min_cutoff_hz) : min_cutoff_hz_(min_cutoff_hz) {}

Eigen::Quaterniond OrientationFilter::filter(const Eigen::Quaterniond& value, double time_sec) {
  const Eigen::Quaterniond input = normalized(value);
  if (!initialized_) {
    initialized_ = true;
    last_value_ = input;
    last_time_ = time_sec;
    return input;
  }
  const double dt = time_sec - last_time_;
  last_time_ = time_sec;
  if (!(dt > 0.0) || !std::isfinite(dt)) {
    return last_value_;
  }
  const double mix = std::clamp(1.0 - std::exp(-2.0 * kPi * min_cutoff_hz_ * dt), 0.0, 1.0);
  last_value_ = slerp(last_value_, input, mix);
  return last_value_;
}

void OrientationFilter::set_min_cutoff(double min_cutoff_hz) { min_cutoff_hz_ = min_cutoff_hz; }

void OrientationFilter::reset() {
  initialized_ = false;
  last_value_ = Eigen::Quaterniond::Identity();
  last_time_ = 0.0;
}

TranslationDeadband::TranslationDeadband(double threshold_m, double hysteresis)
    : threshold_m_(threshold_m), entry_threshold_m_(threshold_m * hysteresis) {}

Eigen::Vector3d TranslationDeadband::filter(const Eigen::Vector3d& value, double time_sec) {
  if (!initialized_) {
    initialized_ = true;
    anchor_ = value;
    last_input_ = value;
    last_time_ = time_sec;
    return value;
  }
  const double dt = time_sec - last_time_;
  last_time_ = time_sec;
  if (dt > 0.0 && std::isfinite(dt)) {
    const double speed = (value - last_input_).norm() / dt;
    still_time_ = (speed < threshold_m_ * 2.0) ? still_time_ + dt : 0.0;
  }
  last_input_ = value;

  if (active_ && still_time_ >= kStillTimeSec) {
    active_ = false;
    still_time_ = 0.0;
    anchor_ = value;
    return value;
  }

  const Eigen::Vector3d delta = value - anchor_;
  const double distance = delta.norm();
  if (!active_ && distance <= entry_threshold_m_) {
    return anchor_;
  }
  active_ = true;
  if (distance <= threshold_m_) {
    active_ = false;
    anchor_ = value;
    return value;
  }
  return anchor_ + delta * (1.0 - threshold_m_ / distance);
}

void TranslationDeadband::set_parameters(double threshold_m, double hysteresis) {
  threshold_m_ = threshold_m;
  entry_threshold_m_ = threshold_m * hysteresis;
}

void TranslationDeadband::reset() {
  initialized_ = false;
  active_ = false;
  still_time_ = 0.0;
  last_time_ = 0.0;
  anchor_.setZero();
  last_input_.setZero();
}

RotationDeadband::RotationDeadband(double threshold_rad, double hysteresis)
    : threshold_rad_(threshold_rad), entry_threshold_rad_(threshold_rad * hysteresis) {}

Eigen::Quaterniond RotationDeadband::filter(const Eigen::Quaterniond& value, double time_sec) {
  const Eigen::Quaterniond input = normalized(value);
  if (!initialized_) {
    initialized_ = true;
    anchor_ = input;
    last_input_ = input;
    last_time_ = time_sec;
    return input;
  }
  const double dt = time_sec - last_time_;
  last_time_ = time_sec;
  if (dt > 0.0 && std::isfinite(dt)) {
    const double speed = rotation_angle(world_delta(input, last_input_)) / dt;
    still_time_ = (speed < threshold_rad_ * 2.0) ? still_time_ + dt : 0.0;
  }
  last_input_ = input;

  Eigen::Quaterniond delta = world_delta(input, anchor_);
  if (delta.w() < 0.0) {
    delta.coeffs() = -delta.coeffs();
  }
  const double angle = rotation_angle(delta);

  if (active_ && still_time_ >= kStillTimeSec) {
    active_ = false;
    still_time_ = 0.0;
    anchor_ = input;
    return input;
  }
  if (!active_ && angle <= entry_threshold_rad_) {
    return anchor_;
  }
  active_ = true;
  if (angle <= threshold_rad_) {
    active_ = false;
    anchor_ = input;
    return input;
  }

  const Eigen::Vector3d vector(delta.x(), delta.y(), delta.z());
  const double sine = std::sin(angle / 2.0);
  const Eigen::Vector3d axis = (sine > 1e-10) ? vector / sine : vector.normalized();
  const Eigen::Quaterniond reduced =
      axis_angle(axis.x(), axis.y(), axis.z(), angle - threshold_rad_);
  return normalized(reduced * anchor_);
}

void RotationDeadband::set_parameters(double threshold_rad, double hysteresis) {
  threshold_rad_ = threshold_rad;
  entry_threshold_rad_ = threshold_rad * hysteresis;
}

void RotationDeadband::reset() {
  initialized_ = false;
  active_ = false;
  still_time_ = 0.0;
  last_time_ = 0.0;
  anchor_ = Eigen::Quaterniond::Identity();
  last_input_ = Eigen::Quaterniond::Identity();
}

}  // namespace leap_teleop
