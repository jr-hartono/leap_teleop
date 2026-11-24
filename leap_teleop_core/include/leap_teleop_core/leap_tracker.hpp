#pragma once

#include <LeapC.h>

namespace leap_teleop {

// Owns the LeapC connection handle used by the tracker node.
class LeapConnection {
 public:
  LeapConnection();
  ~LeapConnection();

  LeapConnection(const LeapConnection&) = delete;
  LeapConnection& operator=(const LeapConnection&) = delete;

  LEAP_CONNECTION handle() const { return handle_; }

 private:
  LEAP_CONNECTION handle_ = nullptr;
};

}  // namespace leap_teleop
