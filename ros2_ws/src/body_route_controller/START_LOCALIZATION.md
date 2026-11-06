# Stationary start localization from rear lidar

This opt-in candidate implements the November 6 operator preference: at the agreed
maze start, use the fixed left and rear walls as the primary position/heading
reference. The front wall may move. Historical A/B/C profiles and default navigation
remain unchanged. This candidate has not been deployed or physically qualified.

## Components

- `fit_start_pose`: offline C++ fitting against occupied map cells in two explicit
  wall regions. It reads two independent held snapshots, uses only rear-lidar points,
  assigns each return to one reviewed wall, trains on alternating points, and reports
  withheld points separately. Each wall has equal fit weight. It writes a new report
  and never publishes a pose, TF, command, or goal. Exit 0 means both fits and pair
  repeatability passed; exit 2 preserves a rejected candidate; exit 1 is invalid input.
- `start_scan_filter`: a stationary ROS component reading `/lidar_rear/scan`, static
  rear mount TF, `/odom`, and `/mission/hold`. It writes only
  `/localization/start_scan` and `/localization/start_status`. It copies the original
  scan metadata/array and sets excluded ranges to NaN; retained ranges are unchanged.
  Signed acquisition timing is preserved. Front and rear safety scans are untouched.
- `start_localization.launch.py`: localization-only composition with map server,
  AMCL, and the filter. No driver, wheel controller, navigation action server, goal,
  or hold publisher is started. `reviewed_start:=true` is required. Existing
  stationary sensor/odometry/TF sources are prerequisites. There must be exactly one
  AMCL/map-to-odom owner: stop the previous localization stack before using this launch.
  It loads the requested navigation file's AMCL parameters and overrides only scan
  topic, reset/initialization policy, and use_sim_time. Apply an independently reviewed
  seed through the established initializer, retaining 0.10 m / 5 degree uncertainty.

## Selection and acceptance

The example configuration is tied to the unchanged November 6 map and a coarse C
start estimate. Its left and rear map rectangles are candidate regions, not surveyed
wall positions. The rays point behind the body: 95–140 degrees toward the left wall
and 160–240 degrees toward the rear wall. Both sectors are expressed about body +X,
using the unchanged rear mount [-0.1145 m, 0 m, 0 rad]. The mount must match runtime TF.
A generous 0.30 m association margin accommodates initialization error; it does not
certify correspondence. Map-cell residuals and independent-snapshot checks determine
whether a fitted candidate is acceptable. No live AMCL pose feeds back into selection.

The offline bounded search stays within +/-0.30 m and +/-6 degrees of the reviewed
reference, followed by 2.5 mm / 0.1 degree refinement. It rejects boundary solutions.
Each wall needs at least 8 training and 8 withheld points, at least 0.25 m span in each
subset, and at most 4 cm 90th-percentile residual. Independent snapshots must be at
least 0.5 seconds apart and agree within 2 cm / 1 degree. These are candidate startup
acceptance criteria, not surveyed accuracy or replacement collision-clearance checks.

The runtime filter rejects missing/stale odometry, malformed or stale/future scan
metadata, mismatched rear TF, or inadequate support from either wall. More than
1 mm/s translation, 0.01 rad/s yaw rate, 1 cm accumulated displacement, 0.01 rad yaw
change, or any observed hold release disables output until process restart. It is a
localization filter, **not a motor interlock**; the established motion hold remains
mandatory. An absent hold message does not prove physical readiness.

This is intentionally a start-only component. It must not be carried into a motion
trial, and it never automatically switches back to front-lidar localization. After
stationary qualification, the driving-localization configuration and transition must
be separately prepared and reviewed. Preserve both lidar safety inputs, gray UNKNOWN,
and all observe-before-enter, timestamp, stopping, and speed guards.

AMCL's likelihood-field model skips NaN/max-range readings; this mask relies on that
specific model and launch rejects other model selections. See the [upstream model
implementation](https://github.com/ros-navigation/navigation2/blob/main/nav2_amcl/src/sensors/laser/likelihood_field_model.cpp).
Uniform AMCL beam subsampling reduces the retained subset; verify usable left/rear
support and resulting pose stability in the held readiness check, not just total raw
point count. Jetson build and actual held AMCL convergence remain unverified.

## Offline use

Build the package normally, then run (no ROS graph or hardware is used):

```text
fit_start_pose CONFIG_YAML MAP_YAML SNAPSHOT_1_JSON SNAPSHOT_2_JSON NEW_RESULT_YAML
```

Snapshots are the existing live-view JSON format: `scans[0]` front, `scans[1]` rear,
map-frame endpoints, recorded base `pose`, held/live/fresh metadata and ROS capture
time. The tool inverts each snapshot's own base pose before fitting, so old AMCL
coordinates are not treated as ground truth. Never overwrite historical evidence.
For a new placement, review the reference, wall rectangles, ray selection and output
plot before any hardware startup. A prior successful report is not a reusable live pose.

## November 6 local validation

The first retained C pair (`held-localization-state.json` and
`held-localization-second-state.json`) passed each individual wall-residual check but
failed independent heading agreement: 1.118 cm / 1.1 degrees against the 2 cm / 1 degree
limits. That rejection is retained; no threshold was relaxed and no pose was applied.
Small map residuals alone do not establish repeatable heading. The disagreement could
include sampling, map structure or sensor effects; its cause is not established.

The local evidence directory is
`Project2_Turtle_Ninja/output/rear_start_localization_20261106_v1`.
No Jetson transfer, hardware startup or Git command is part of this preparation.
