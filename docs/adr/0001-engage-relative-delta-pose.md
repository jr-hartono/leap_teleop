# Teleoperation is engage-relative delta pose only

Teleoperation commands are a pose delta relative to the tool pose latched when the
operator engages. There is no absolute/calibrated mode and no velocity retargeting mode:
absolute mapping needs a calibration step the operator does not want, and differentiating
palm position amplifies the v1 controller's millimetre-level noise into visible jitter.
The Servo bridge may still *emit* a velocity to MoveIt Servo by running a proportional
controller on the pose error; that is an encoding at the robot edge, not a second
retargeting mode.

## Considered Options

- **Absolute mapping** — needs a one-time calibration pose and drifts with camera
  placement. Rejected by the operator.
- **Velocity retargeting** — natural for velocity-only interfaces, but noise
  amplification and integration drift.
- **Delta pose** — no calibration, no drift, stable when the hand is still.

## Consequences

- Returning the hand to its engage position returns the tool to its latched pose.
- Re-centring is a release and re-press, not a software reset.
- Any interface that only accepts velocity needs a position-to-velocity conversion in its
  bridge, and inherits the tuning burden of that controller.
