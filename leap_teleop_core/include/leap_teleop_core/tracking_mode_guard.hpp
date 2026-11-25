#pragma once

namespace leap_teleop {

// Decides when to request the Ultraleap tracking mode. The mode is shared by every client of the
// service, so a request can be lost or undone by another application. The guard retries a request
// that is not confirmed in time and gives up after a few attempts instead of fighting forever.
class TrackingModeGuard {
 public:
  enum class Action { kNone, kRequest, kGiveUp };

  explicit TrackingModeGuard(int max_attempts = 5, double retry_after_sec = 1.0);

  // The service connected or a device was attached: start over and request the mode.
  Action restart(double time_sec);
  // The service reported its tracking mode; `matches` is true when it is the wanted one.
  Action on_mode(bool matches, double time_sec);
  // Called periodically. Requests again when the last request was not confirmed in time.
  Action on_tick(double time_sec);

  [[nodiscard]] bool confirmed() const { return confirmed_; }

 private:
  Action request(double time_sec);

  int max_attempts_;
  double retry_after_sec_;
  int attempts_ = 0;
  double last_request_sec_ = 0.0;
  bool confirmed_ = false;
  bool gave_up_ = false;
};

}  // namespace leap_teleop
