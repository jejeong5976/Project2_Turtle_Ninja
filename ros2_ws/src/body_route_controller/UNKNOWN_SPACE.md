# Unknown-space navigation

Hardware remains off during this workflow. These commands build software and run
mock simulations only. No Jetson connection, serial device or physical driver is
needed. The user retains all Git operations.

## Policy and configuration

`blocked` is the default. It retains the existing controller plugins, costmap
combination method, unknown inflation, footprint, speed limits and stop/turn guards.

`observe-before-enter` separates provisional planning from execution clearance:

- The global orthogonal planner may translate and rotate through raw UNKNOWN cells,
  including a provisional unknown destination. Known occupied cells still block
  the complete swept footprint and rotation checks.
- A positive `unknown_penalty` (default **2.0**) adds a distance/turn cost proportional
  to unobserved raw-unknown coverage of the footprint's bounding box. This is a preference
  approximation; collision checks still use the polygon. Comparable known-clear or freshly observed-free
  routes therefore cost less. The original raw identity is independent of inflation.
- Both comparison controllers run inside `ObservationController`. It gates each
  output command, and `ObservedGoalChecker` also checks the current and goal
  footprints before allowing success. Neither a short path nor an already-reached
  position bypasses the evidence requirement.
- `ObservationLayer` retains the immutable `/map` classification separately from
  free-space timestamps, lidar hits, continuity certificates and Nav2 inflation.
  Its optimistic costmap representation is only for planning/controller candidate
  generation. It is **not execution clearance**. Raw unknown becomes locally
  executable only through the evidence gate. Raw static occupied cells always win.
- The existing `ValidatePath` BT action sees the same optimistic costmap, so unknown
  alone does not invalidate a provisional route. Marked lidar obstacles do invalidate
  it and trigger the existing planner branch. The observation gate stops while a
  conflicting route is replaced; planning can also fail safely if no detour exists.

Generate matching navigation, drivetrain and trial profiles together. The output
must be a **new directory**:

```bash
source /opt/ros/lyrical/setup.bash
source /tmp/body-route-main-20261104-install/setup.bash
ros2 run body_route_controller configure_trial \
  --mode custom --speed 0.08 --yaw-rate 0.30 \
  --unknown-policy observe-before-enter --unknown-penalty 2.0 \
  --observation-ttl 4.0 --output /tmp/body-route-observe-profile
```

Use `--mode original` for the existing omni MPPI delegate, or
`--unknown-policy blocked` for the default comparison. Restart while stopped when
changing profiles. The launch rejects mismatched planner, costmap, controller and
goal-checker policies. Do not enable unknown planning by changing only one Nav2
parameter or by changing a collision threshold.

## Evidence and stopping contract

Defaults in each generated `observation_layer` configuration:

| Parameter | Default | Meaning |
|---|---:|---|
| `ttl` | 4.0 s | Maximum age of external observed-free cells, including the predicted stopping horizon |
| `scan_timeout` | 0.3 s | Both lidars must have recent, accepted stamped scans |
| `latency` | 0.5 s | 0.3 s scan age + 0.05 s control period + 0.15 s actuation allowance |
| `linear_decel` | selected linear acceleration | Assumed lower bound on physical braking capability |
| `angular_decel` | selected angular acceleration | Assumed lower bound on angular braking capability |
| `self_radius` | 0.08 m | Central circular lidar occlusion mask in the robot frame |
| `endpoint_margin` | 0.02 m | Free-ray truncation before a measured endpoint |

Only finite in-range returns establish clearance. Infinity, NaN, out-of-range
returns and missing angular sectors establish none. Finite maximum-range returns
clear to the declared range minus the endpoint margin and do not mark a hit.
Adjacent valid rays can fill their intervening sector only with a sufficiently
small angular gap, nearly identical origins and the nearer of the two ranges.
Known static obstacles terminate clearing rays. Older rays cannot clear newer hits.
Observed hits persist until a newer valid free ray clears them; expiry alone never
erases an occupied observation.
The same marked hit cells feed both costmaps and the execution gate. Hits are
not inflated twice: occupied-obstacle inflation and footprint padding stay unchanged.
A separate observation grid reduces the planning penalty for freshly observed free
cells without overwriting raw UNKNOWN identity. Unobserved cells remain provisional.

The ROS adapter checks sensor frame names, scan metadata, monotonically increasing
positive stamps and age. It looks up the map-to-sensor and map-to-body transforms
at **each ray's timestamp**, including `time_increment`; it permits a bounded 50 ms
wait for TF arrival. It rejects missing/non-planar transforms instead of substituting
a latest transform. The shared wrapper also checks fresh pose and odometry.

Every request is checked together with measured motion and all admitted commands
from the last latency interval, including requests still queued in the drivetrain. The horizon includes
`latency + max(linear_speed/linear_decel, yaw_speed/angular_decel)`, using the
largest component-wise intermediate linear speed during a command change. A
conservative rectangle encloses all footprint orientations and translations
possible between measured/requested/pending/zero velocities during this interval. This
also covers independent translation/yaw braking and steering transitions; a single
constant-curvature arc would not. It can reject some geometrically safe motions.
Evidence must remain valid until the end of the stopping horizon. This assumes
bounded, non-overshooting velocity transitions and braking at least as strong as
the configured bounds. The existing occupied-obstacle, full-turn, collision monitor,
command hold and trial guards remain in place.

The starting footprint must be known free or actually observed free. There is no
blanket unknown initialization. A bounded continuity certificate retains only
cells already cleared under the current footprint, covering lidar self-occlusion.
It advances only with continuous pose feedback and newly certified cells; it does
not clear surrounding unknown. A pose gap, implausible pose jump or map replacement
invalidates it. Scan loss commands zero but continuous valid pose feedback can retain
this footprint certificate while the robot brakes or remains stationary, allowing
recovery when valid scans return. Lidar hits and static occupied cells override it.

An insufficient sweep returns zero velocity and publishes the cause and first blocking cell/stopping horizon on
`/controller_server/observation_status`. Motion resumes when clearance is sufficient.
The unchanged 20-second progress guard can abort a prolonged wait; the trial's
separate sensor-health guard can abort a sensor outage sooner. Recovery is therefore
bounded, not an indefinite autonomous exploration behavior. No full-route visibility
is required.

## Offline commands

Build the main-based integration and simulation tools:

```bash
bash /home/pa4/git/Project2_Turtle_Ninja-body-route/tools/scanned_map_sim/build.sh
```

Run the deterministic evidence/controller suite. Use a new output directory each time:

```bash
source /opt/ros/lyrical/setup.bash
source /tmp/body-route-main-20261104-install/setup.bash
/tmp/scanned-map-controller-sim-final-build/unknown_suite /tmp/unknown-navigation-check 0.08
```

It uses the production planner, custom controller core and shared observation model,
with ideal odometry, 150 ms command delay and bounded acceleration. Its complete
world is available only to synthetic sensors and the independent swept-footprint
evaluator. The navigation object receives a separate partial map. Cases cover clear
unknown travel, a concealed obstacle and detour, partial visibility, stale/missing/
invalid scans, missing transforms, outage recovery while already in unknown space,
and turns. This fast runner is **not** the MPPI or ROS integration test.

Run the actual Nav2/controller/mock-wheel chain, with AMCL and both controller modes:

```bash
bash /home/pa4/git/Project2_Turtle_Ninja-body-route/tools/scanned_map_sim/run.sh \
  --scenario corridor --mode custom --speed 0.08 --yaw-rate 0.30 \
  --unknown-policy observe-before-enter --goal-timeout 180 --rviz

bash /home/pa4/git/Project2_Turtle_Ninja-body-route/tools/scanned_map_sim/run.sh \
  --scenario corridor --mode original --speed 0.08 --yaw-rate 0.30 \
  --unknown-policy observe-before-enter --goal-timeout 180 --rviz
```

Replace `corridor` with `concealed`, `partial`, or `turn`. `partial` suppresses the
forward viewing sector initially, then restores it. Omit `--rviz` for headless runs.
The synthetic world is fully occupied/free; navigation receives a masked partial
map, including masked obstacles. The feeder stamps scans with the odometry sample
used to generate their rays. Truth data is never published as a navigation map.

Scan-derived L2 and maze trials support `--map-mode partial`. This constructs a
**synthetic complete world** by filling the source scan's unknown exterior as solid,
then masks a corridor band and destination regions in a separate navigation map.
It does not claim that the unscanned real arena is solid. The original scan files
are unchanged. `matched` and `stale` retain the older blocked-policy benchmark.

```bash
bash /home/pa4/git/Project2_Turtle_Ninja-body-route/tools/scanned_map_sim/run.sh \
  --route l2 --shift 0.05 --seed 11 --map-mode partial \
  --mode custom --speed 0.08 --yaw-rate 0.30 \
  --unknown-policy observe-before-enter --goal-timeout 300 --rviz

bash /home/pa4/git/Project2_Turtle_Ninja-body-route/tools/scanned_map_sim/run.sh \
  --route maze --shift 0.02 --seed 11 --map-mode partial \
  --mode custom --speed 0.08 --yaw-rate 0.30 \
  --unknown-policy observe-before-enter --goal-timeout 600 --rviz

# Existing blocked-policy regressions:
bash /home/pa4/git/Project2_Turtle_Ninja-body-route/tools/scanned_map_sim/run.sh \
  --route l2 --shift 0.05 --seed 11 --map-mode matched \
  --mode custom --speed 0.08 --yaw-rate 0.30 --unknown-policy blocked
bash /home/pa4/git/Project2_Turtle_Ninja-body-route/tools/scanned_map_sim/run.sh \
  --route maze --shift 0.02 --seed 11 --map-mode matched \
  --mode custom --speed 0.08 --yaw-rate 0.30 --unknown-policy blocked
```

Tests, including the installed Nav2 inflation regression and actual stamped-TF adapter:

```bash
source /opt/ros/lyrical/setup.bash
source /tmp/body-route-main-20261104-install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export FASTDDS_DEFAULT_PROFILES_FILE=/home/pa4/git/Project2_Turtle_Ninja-body-route/tools/scanned_map_sim/loopback.xml
ctest --test-dir /tmp/body-route-main-20261104-build/body_route_controller \
  --output-on-failure
```

## Displays and evidence

Saved trajectories use the **navigation** map as their background: purple unknown,
cyan observed-free, black occupied, translucent costmap inflation, translucent blue
footprints, and separate plan/truth/estimated paths. Hidden ground truth is used for
evaluation, not silently revealed in the navigation background. RViz shows the raw
navigation map, costmaps at 25% opacity, footprints at 40%, and observed-free cells
in cyan at 30%. The truth map is saved separately for deliberate independent review.

Each run retains map/profile inputs, ROS logs, `observation-events.csv`, raw costmaps,
observed-space raster, trajectories, arrival witnesses and evaluation results.
The full observation raster publishes from the global costmap at 1 Hz; the local
controller publishes only nearby observed-free cells. Visualization publication
happens after releasing the observation lock.
The complete-world truth monitor can terminate a failed experiment, but never
supplies truth-derived obstacle cells, avoidance commands or route choices to navigation.
A runtime pass requires action success, all independently checked truth arrivals,
and zero swept-footprint overlaps. Failed attempts are retained; there is no automatic
motion retry. DDS lifecycle response timeouts can still produce startup-only failures.

## Validation recorded on 2026-11-04

The feature is implemented, but **full-maze physical qualification is not complete**.
Do not interpret unit-test or simple-corridor success as a successful full maze.
The preserved evidence directory is
`/home/pa4/git/Project2_Turtle_Ninja/output/unknown_navigation_20261104_v1`.
`runtime-validation.json` includes every retained evaluated ROS attempt, including
startup failures, stopped runs and earlier implementation revisions.

| Check | Result |
|---|---|
| Main-based build, warnings treated as errors | Passed |
| Package CTest | 21/21 targets passed; includes 64 core/planner/profile checks |
| Final deterministic partial-map suite | 9/9 passed, zero unsafe swept-footprint samples |
| Concealed obstacle in fast suite | Detected progressively; three plans; goal reached |
| Partial visibility and scan/TF faults | Waited/stopped; resumed and reached goal after valid observations returned |
| Existing blocked L2, 5 cm shifts, seed 11 | Passed; 74.65 s navigation; zero modeled overlaps |
| ROS custom clear corridor and concealed obstacle | Earlier integration runs passed; final matrix identifies their revisions |
| Final partial-map L2, 5 cm shifts, seed 11 | Passed in 232.21 s; zero modeled overlaps; truth arrival error 17.9 mm and 0.00423 rad |
| Partial-map six-goal maze, 2 cm shifts | Not qualified; latest completed first goal, then no footprint-safe route near goal 2 |
| Final ROS original MPPI corridor | Failed progress check after 1.08 m; observation gate stayed clear; zero modeled overlaps |

The latest maze run before the queued-command extension recorded zero modeled
body overlaps but up to **12.1 cm AMCL position error**. An earlier failure raster
replays successfully on the raw navigation and complete world maps but has no
route on the live costmap: projected lidar hits and the orthogonal planner's
turning constraints can close a passage or trap a constrained start pose.
No collision threshold, footprint or occupied-obstacle inflation was relaxed to
force completion. The existing 0.4 s status and 20 s progress guards remain active.

Timing from these stochastic AMCL/mock-steering runs is not a qualified controller
speed comparison. The deterministic suite uses ideal odometry and is not MPPI.
The original MPPI configuration also logs that the preserved 0.26 m inflation
radius is below the padded footprint's circumscribed radius; it retains full
footprint collision checking but cannot use that particular costmap shortcut.

The final build (including queued commands) was exercised in runs
`20261104-200522-custom-V73pSm` (L2 pass) and
`20261104-200201-original-DyT1Bt` (MPPI progress failure). Earlier attempts are
retained and are not counted as final-build qualification. The full six-goal maze
was last exercised immediately before the queued-command extension and remains
unqualified; its no-route failure is not fixed by the more conservative envelope.
The final runtime acceptance entries are recorded in `runtime-validation.json`.
The supplied ROS commands reproduce the remaining integration tests; they do not
constitute a recommendation to run the physical robot.

## Limits

This is observed-space navigation, not frontier exploration or a new localization
system. It requires sufficient known map structure for AMCL and a correctly registered
initial pose. Static occupied cells are deliberately immutable; use the separate
wall-layout adaptation workflow before navigation when known walls have moved.

Clearance is limited by 2D lidar coverage, beam spacing, calibrated transforms and
occlusions. It cannot certify obstacles outside the lidar plane. The orthogonal planner can also have no feasible escape from a constrained pose
after live lidar hits invalidate a previously planned corridor. Persistent projected
hits and AMCL drift can therefore cause a safe stop even when a route exists in the
complete test world. Braking and latency
bounds, the provisional footprint, the central self-mask and localization errors
still require physical calibration. Large selected speeds can exceed the available
visibility or evidence lifetime and therefore cause waiting. The offline model does
not establish hardware braking, wheel slip, contact or manipulation safety.

No physical trial is authorized or started by any command above.
