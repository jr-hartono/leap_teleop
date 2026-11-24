# The seam is a pose delta, and the bridge owns the robot pose

The robot-agnostic core publishes a `TeleopTarget`: a pose delta plus engagement state
and a sequence number. It never learns the robot's current pose. Each robot bridge
latches the tool pose from TF on the rising edge of engagement and composes
`target = latched + delta` itself.

## Considered Options

- **Core publishes an absolute `PoseStamped`** — simpler bridges, but the core needs TF, a
  planning frame and knowledge of the robot, and recorded bags no longer replay exactly.
- **Two-way protocol** — the core asks the bridge for its pose. Adds a request/reply
  dependency to a 100 Hz control loop.
- **Bridges latch** — the core stays pure maths; adding a robot touches one package.

## Consequences

- The core can be tested and replayed with no robot, no TF and no MoveIt.
- Every bridge must implement the latch, and must handle a failed TF lookup by not
  publishing.
- Timing is forgiving: the delta is zero at engagement, so a one-frame difference between
  the hand latch and the tool latch cannot produce a jump.
