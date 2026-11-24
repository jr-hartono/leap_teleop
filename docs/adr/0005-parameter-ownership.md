# Each parameter has one owner; shared facts travel in messages

A parameter that describes a fact shared by several components is configured in exactly
one place. Consumers receive the fact in the message instead: the tracker owns which hand
is used and the supervisor copies `side` from `HandTarget`; the supervisor owns
`base_frame` and stamps it into `TeleopTarget.header.frame_id`; the bridge reads the frame
from the message and resolves TF in it. Duplicated knobs were removed rather than
synchronised.

## Considered Options

- **Duplicate the value per node** — every node stays independently configurable, but two
  copies can disagree, and the disagreement is silent: the bridge filters for the wrong
  hand, or latches a pose from a frame whose orientation the mapper never used.
- **One owner plus message transport** — a fact has a single source of truth, and the
  data path proves the two components are talking about the same thing.
- **Central parameter server** — removes duplication, but adds a runtime dependency and
  makes node startup order matter.

## Consequences

- The bridge has no `side` and no `base_frame`, so it cannot contradict the supervisor.
- Frames are named by the producer. The tracker emits the synthetic `leap_teleop/hand`
  frame because its origin is the device, not the robot; the supervisor renames the delta
  into `base_frame`, which is a claim it is allowed to make.
- Cross-node relations that cannot be validated locally (heartbeat versus timeout, lease
  versus publish rate) are documented, not enforced by any one node.
