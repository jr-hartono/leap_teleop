# Tuning

Tuning changes the feel of teleoperation. Perform all tuning with `dry_run: true` in the
bridge first: commands are logged, not published, so no arm moves. Enable output only after
the logged twists are correct. Parameters and ranges are in
[example_arm.yaml](../leap_teleop_bringup/config/example_arm.yaml).

## What the knobs mean

- `translation_gain` and `rotation_gain` are unitless ratios. 1.0 is 1:1; 2.0 doubles the
  commanded travel for the same hand motion.
- `kp_linear` and `kp_angular` are proportional gains in 1/s, converting pose error into
  commanded twist. They set response speed; they are not limits.
- `max_linear_speed` and `max_angular_speed` are hard caps. They bound the published twist
  whatever the gains request, so they are safety limits, not targets.
- `filter_min_cutoff` (Hz) and `filter_beta` shape the One-Euro filter on position and the
  low-pass filter on orientation.
- `deadband_translation` (m), `deadband_rotation` (degrees) and `deadband_hysteresis` decide
  when a still hand produces no output.

## Symptom → parameter

| Symptom | Parameter | Direction |
| --- | --- | --- |
| Jitter in position or orientation | `filter_min_cutoff` | Decrease (more smoothing) |
| Lag, slow tracking of fast motion | `filter_min_cutoff` | Increase |
| Lag on fast motion only | `filter_beta` | Increase (0 disables speed adaptation) |
| Small drift with a still hand | `deadband_translation`, `deadband_rotation` | Increase |
| Chatter at the edge of the deadband | `deadband_hysteresis` | Increase above 1.0 |
| Sluggish response | `kp_linear`, `kp_angular` | Increase |
| Over-fast, twitchy response | `kp_linear`, `kp_angular` | Decrease |
| Motion too violent | `max_linear_speed`, `max_angular_speed` | Decrease |

Change one parameter at a time; settle gains and caps before filters. Keep
`max_linear_speed` at 0.05 during initial tuning. See [COMMISSIONING.md](COMMISSIONING.md)
for the safe order of steps.

## Mount alignment

`yaw` describes how the robot faces relative to the operator. With `yaw = 0`, hand motion
away from the operator maps to +x of `base_frame`; 180 is used when the arm faces the
operator across the desk. A wrong `yaw` cannot be corrected with gains: it produces wrong
directions. Align the mount configuration first, then tune. With a tilted device, "down"
is off by the tilt; pitch and roll are not part of v0.1. Palm orientation is noisiest when
edge-on to the camera, so extreme wrist angles remain the worst case even after filtering.
