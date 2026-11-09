# Prepared B/C passages (experimental, disabled by default)

`FollowPath.prepare_passages: true` enables a shared geometric preparation layer
for continuous B/reactive and C/preview. Axis-stop A and default profiles are
unchanged. This option is not enabled in the approved November 9 Jetson packet.
Do not enable it for a physical trial without a separately reviewed deployment,
fresh localization/readiness, route review and the user's separate approvals.

Before a bend exceeds the existing travel cone, preparation begins at a distance
derived from lookahead, footprint extent and response/braking distance. A
conservative clear-room check permits normal continuous cornering. Otherwise,
the whole incoming leg, a stationary turning sweep and the outgoing crossing
are checked before selecting a stop/alignment maneuver. Selection uses route
geometry and current blocked/unknown cells; there are no arena coordinates.
Smaller bends retain B's normal smooth diagonal translation and its live guards.

The selected corner, outgoing heading and exit persist through small route/pose
updates. Translation uses the fixed tangent with bounded lateral correction,
not a moving pursuit-bearing alignment target. Distance-based braking precedes
the turning point. Crossing speed is selected by a bounded search up to the
existing cruise cap using the checked stopping envelope. The exit is derived
from where a full footprint rotation becomes possible, so C's final-heading
preview resumes only after the restricted portion is cleared. A goal inside
an opening never grants permission for an otherwise blocked final rotation.

A temporary hold, stale observation or clearance wait commands zero and resets
motion history while preserving only geometric intent. Revalidation is required
before resuming. An incoming detour, changed crossing or displaced goal invalidates
that intent. New occupied/unknown cells in the maneuver region trigger complete
geometry revalidation. Cancellation, controller reset and lifecycle transitions
forget the maneuver. Existing feedback, hold, scan/TF, observation, collision and
progress guards remain authoritative; the prepared layer cannot release a hold.

Parameters (hypotheses until measured):

| Parameter | Default | Meaning |
|---|---:|---|
| `prepare_passages` | `false` | Explicit opt-in for continuous B/C |
| `passage_position_allowance` | 0.05 m | Bounded translational pose error assumption |
| `passage_yaw_allowance` | 0.08726646 rad | 5-degree yaw error assumption |
| `passage_response_delay` | 0.5 s | Response allowance; never below existing 0.5 s envelope latency |

Uncertainty is a support expansion of the padded footprint and the **whole
stopping envelope**, including recent commands and measured velocity. A costmap
cell half-diagonal is additionally reserved to avoid accepting a zero margin at
a raster boundary. The selected alignment checkpoint uses one quarter of the
existing corner and alignment tolerances; these tighter checks are not new
actuator limits. Linear/yaw/acceleration limits remain the existing profile values.

## Reproducible offline experiment

`test/passage_replay.cpp` runs without ROS initialization, hardware, network or
movement. Arguments are:

```
passage_replay ROUTE INITIAL_YAW OUTPUT_PREFIX NAV2_YAML MAP_METADATA MAP_BYTES GAIN baseline|prepared DELAY_TICKS POSITION_ALLOWANCE YAW_ALLOWANCE
```

The plant hypothesis is first-order body velocity response at 20 Hz after an
explicit command delay. It is not wheel/steering dynamics, live localization,
changing perception or a physical stopping bound. Current-B's one-tick trace
was checked byte-for-byte against the archived guarded replay before comparison.

The November 9 experiment and scientific analysis are under
`/home/pa4/git/Project2_Turtle_Ninja/output/prepared_passage_20261109_v1`.
Both controllers use the same route, map, scans and 0.08 m/s / 0.10 rad/s caps.
Unsuccessful earlier routes and intermediate clearance results are retained.
Read that directory's `REPORT.md` before interpreting completion as readiness.
The narrow modeled residual margin does not qualify physical movement. Native
execution time, measured plant response and localization bounds still matter.

Generate the opt-in physical profile with `configure_trial --prepare-passages true`
plus the existing matching limits. The three allowance arguments use the table's
names with hyphens. They are included in the profile identity, trial log and live
parameter verification. Legacy generation remains byte-compatible when disabled.
Normal tracking outside a selected maneuver is unchanged; review the entire live
route and stopping envelope, not just the passage portion.

## Compute-time protection

Prepared operation has an additional50 ms proposal deadline. A late computation
returns zero and resets motion state while retaining geometry for revalidation;
recent issued commands remain in the stopping history. This does not extend any
scan/feedback timeout. Geometry selection reuses turn/body checks only after they
passed on the same locked map; equal-speed envelopes translate unchanged. The
costmap snapshot checks every lethal/unknown cell in its region and includes grid
origin/resolution/dimensions, while ignoring changes among nonblocking inflation
costs. No obstacle or unknown cell is omitted. Native timing still needs review.

During geometric selection, a per-call cell-count index can prove that the whole
polygon bounding box contains no lethal/unknown cell. Nonempty or out-of-window
boxes use the existing footprint check. The map remains locked and the index is
rebuilt for each geometry call, so this shortcut cannot hide changed obstacles.
