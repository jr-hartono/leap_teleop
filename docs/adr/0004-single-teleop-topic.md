# One teleop topic, side carried in the message

The supervisor publishes all teleop targets on one topic and identifies the source hand
in `TeleopDelta.side`. Topic names do not encode a side, and the bridge does not filter
by hand: a stack drives one robot from one hand, so the stream it receives is already the
commands for that robot.

## Considered Options

- **Per-side topics** (`/teleop/left/pose_delta`, `/teleop/right/pose_delta`) — convenient for two
  simultaneous arms, but forces every consumer to know which sides exist and makes a
  one-arm setup carry structure it does not use.
- **Aggregate topic with a side field** — one stream per stack, side available for
  diagnostics and for a supervisor that must know which hand it latched.
- **Aggregate topic, bridge filters on side** — same as above plus a bridge parameter that
  can silently disagree with the supervisor, producing no motion and no error.

## Consequences

- A second arm is a second stack in its own namespace, not a second subscriber on a shared
  stream. Nothing needs to coordinate between robots.
- A left-handed operator sets `side: left` in the tracker; topic names are unchanged, so
  launch files and recordings stay valid.
- The `side` field is informational at the bridge. It must not be used to gate publishing,
  or a misconfiguration becomes an invisible failure.
