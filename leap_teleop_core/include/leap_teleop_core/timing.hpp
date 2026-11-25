#pragma once

namespace leap_teleop::timing {

// DeadmanState publish rate, Hz.
constexpr double kDeadmanRate = 30.0;
// Age after which a DeadmanState no longer counts as held, s.
constexpr double kEngageTimeout = 0.1;
// Minimum TeleopDelta publish rate, Hz.
constexpr double kHeartbeatRate = 30.0;
// Age after which the bridge stops publishing a TeleopDelta, s. Must not exceed Servo's
// incoming_command_timeout (0.1 s by default).
constexpr double kCommandTimeout = 0.1;

static_assert(kEngageTimeout >= 3.0 / kDeadmanRate,
              "deadman jitter must not look like a key release");
// The heartbeat skips a tick within half a period of a hand-triggered publish, so the longest
// gap between deltas is 1.5 periods.
static_assert(kCommandTimeout > 1.5 / kHeartbeatRate,
              "the delta stream must stay fresh between hand frames");

}  // namespace leap_teleop::timing
