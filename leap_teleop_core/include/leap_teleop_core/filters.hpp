#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "leap_teleop_core/frames.hpp"

namespace leap_teleop {

class OneEuroFilter {
 public:
  OneEuroFilter(double min_cutoff_hz, double beta, double derivative_cutoff_hz = 1.0);

  double filter(double value, double time_sec);
  void reset();

 private:
  static double alpha(double cutoff_hz, double dt);

  double min_cutoff_hz_;
  double beta_;
  double derivative_cutoff_hz_;
  double last_value_ = 0.0;
  double last_derivative_ = 0.0;
  double last_time_ = 0.0;
  bool initialized_ = false;
};

class OrientationFilter {
 public:
  explicit OrientationFilter(double min_cutoff_hz);

  Eigen::Quaterniond filter(const Eigen::Quaterniond& value, double time_sec);
  void reset();

 private:
  double min_cutoff_hz_;
  Eigen::Quaterniond last_value_ = Eigen::Quaterniond::Identity();
  double last_time_ = 0.0;
  bool initialized_ = false;
};

class TranslationDeadband {
 public:
  TranslationDeadband(double threshold_m, double hysteresis);

  Eigen::Vector3d filter(const Eigen::Vector3d& value, double time_sec);
  void reset();

 private:
  double threshold_m_;
  double entry_threshold_m_;
  Eigen::Vector3d anchor_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d last_input_ = Eigen::Vector3d::Zero();
  double last_time_ = 0.0;
  double still_time_ = 0.0;
  bool initialized_ = false;
  bool active_ = false;
};

class RotationDeadband {
 public:
  RotationDeadband(double threshold_rad, double hysteresis);

  Eigen::Quaterniond filter(const Eigen::Quaterniond& value, double time_sec);
  void reset();

 private:
  double threshold_rad_;
  double entry_threshold_rad_;
  Eigen::Quaterniond anchor_ = Eigen::Quaterniond::Identity();
  Eigen::Quaterniond last_input_ = Eigen::Quaterniond::Identity();
  double last_time_ = 0.0;
  double still_time_ = 0.0;
  bool initialized_ = false;
  bool active_ = false;
};

}  // namespace leap_teleop
