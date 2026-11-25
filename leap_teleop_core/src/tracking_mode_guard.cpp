#include "leap_teleop_core/tracking_mode_guard.hpp"

namespace leap_teleop {

TrackingModeGuard::TrackingModeGuard(int max_attempts, double retry_after_sec)
    : max_attempts_(max_attempts), retry_after_sec_(retry_after_sec) {}

TrackingModeGuard::Action TrackingModeGuard::restart(double time_sec) {
  attempts_ = 0;
  confirmed_ = false;
  gave_up_ = false;
  return request(time_sec);
}

TrackingModeGuard::Action TrackingModeGuard::on_mode(bool matches, double time_sec) {
  if (matches) {
    confirmed_ = true;
    return Action::kNone;
  }
  confirmed_ = false;
  if (gave_up_) {
    return Action::kNone;
  }
  return request(time_sec);
}

TrackingModeGuard::Action TrackingModeGuard::on_tick(double time_sec) {
  if (confirmed_ || gave_up_ || attempts_ == 0 || time_sec - last_request_sec_ < retry_after_sec_) {
    return Action::kNone;
  }
  return request(time_sec);
}

TrackingModeGuard::Action TrackingModeGuard::request(double time_sec) {
  // Attempts are not reset on confirmation, so a client that keeps switching the mode back
  // exhausts them instead of starting an endless exchange.
  if (attempts_ >= max_attempts_) {
    gave_up_ = true;
    return Action::kGiveUp;
  }
  ++attempts_;
  last_request_sec_ = time_sec;
  return Action::kRequest;
}

}  // namespace leap_teleop
