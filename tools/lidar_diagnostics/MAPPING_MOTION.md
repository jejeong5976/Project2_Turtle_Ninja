# First bounded movement with the live mapper

Latest completed trial: gcQZct passed the forward-after-turn check in NJWQJ7:
84.389 mm forward, healthy input, confirmed automatic stop and unchanged scene.
Map pixels/settings, odometry pose and map->odom agree across the JHydTT/gcQZct
boundary. Cumulative map correction is 8.201 mm / 1.400°. Preserve this session
and current placement; neither pulse needs repeating. See the
[combined review](../../docs/navigation/jetson_mapping_turn_forward_review.md).

Earlier completed trial: fresh-map rotation NJWQJ7/JHydTT passed, with 7.4063° yaw,
healthy input, small map correction and confirmed automatic stopping. Preserve
that session; see the [rotation review](../../docs/navigation/jetson_mapping_rotation_review.md).
The installed v2 runner supports both forward and rotation flags. This document
retains the forward-mode protocol and v1 history; v2 installation is described in
[MAPPING_ROTATION.md](MAPPING_ROTATION.md).

## Executed continuation — one forward pulse after JHydTT

2026-10-27, about 17:08 KST. User confirms >=0.5 m clear level floor ahead at the
new heading, steering clearance, unchanged placement/scene since JHydTT and
manipulator disconnected. Read-only checks confirm NJWQJ7 and the existing wheel
launch still running, active [3], healthy input (published=5600, rejected=4,
fault=0, fresh=1), zero command publishers and one swerve subscriber. The v2
source manifest and accepted mapper configuration hashes pass. No rebuild or
restart was required. This procedure is now complete in gcQZct/IcK9xK, as reviewed
above. Retain these commands as history; do not repeat them merely to follow the guide.

Keep the wheel and mapper terminals running. In a separate Jetson terminal, save
a new before-map without moving the robot or changing the scene:

```bash
source /opt/ros/lyrical/setup.bash
export ROS_DOMAIN_ID=182
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

turn_forward_dir=$(mktemp -d "$HOME/jetson-map-turn-forward.XXXXXX")
echo "$turn_forward_dir"
rotation_reference_dir="$HOME/jetson-map-rotation-snapshots.sIdcDI"

ros2 run nav2_map_server map_saver_cli -f "$turn_forward_dir/before_forward" \
  --ros-args -p save_map_timeout:=10.0 -p map_subscribe_transient_local:=true &&
cmp "$rotation_reference_dir/after_rotation.pgm" "$turn_forward_dir/before_forward.pgm" &&
diff -u <(sed '/^image:/d' "$rotation_reference_dir/after_rotation.yaml") \
        <(sed '/^image:/d' "$turn_forward_dir/before_forward.yaml") &&
echo "MAP CONTINUITY CHECK PASSED"
```

If the save/comparison fails or that message does not appear, retain the output
and pause before motion for inspection. A mismatch is not proof of a sensor fault.
After the check passes, with confirmed clearance still available, run exactly once:

```bash
~/jetson-mapping-motion-v2/build/run_mapping_motion --execute-forward-pulse
```

This records existing streams and sends forty +0.025 m/s forward commands at
10 Hz, then five zeros. Nominal travel is about 10 cm; steering realignment after
rotation can reduce it. All forward bounds and stop checks below remain unchanged.
Observe physical movement/stopping. On failure, preserve the output and directory;
do not retry or extend. Use the accessible motor cutoff if stopping is unsafe.

Only after `SOFTWARE CHECKS PASSED` and an observed complete stop, in the same
terminal:

```bash
ros2 run nav2_map_server map_saver_cli \
  -f "${turn_forward_dir:?Missing snapshot directory}/after_forward" \
  --ros-args -p save_map_timeout:=10.0 -p map_subscribe_transient_local:=true
```

Report the snapshot and new motion directories, runner output and whether it
stopped completely on its own. Keep NJWQJ7 and continuous wheel odometry for
comparison with JHydTT; do not reset the map or manually reposition the robot.

## Earlier forward trials and implementation contract

The latest clean new-scene forward trial is AEjKu5 in map session nEKS7T, with
102.077 mm travel, healthy input, consistent stationary baselines and a confirmed
automatic stop. Preserve its evidence; see the
[new-scene review](../../docs/navigation/jetson_mapping_newscene_review.md).
Use the [new-scene procedure](MAPPING_NEW_SCENE.md) if the robot/scene is changed
again. Historical session paths below describe the earlier trials.

Prepared 2026-10-27 after the user confirmed the first map was recorded stationary
and at least 0.5 m of clear level floor ahead plus steering clearance is available.
The manipulator remains disconnected. The user executes all hardware and Git
commands; assistant SSH is read-only.

Completed trial: `jetson-mapping-motion.gUGiCK`, 96.919 mm forward, software checks
passed, user-confirmed complete automatic stop. The user then reported repositioning
between the initial stationary map and this pulse. Its large map->odom correction
therefore cannot establish a clean mapping result or a SLAM fault. Preserve the
data and restart the same mapping launch at the current pose for a fresh stationary
baseline before another pulse. See the [review](../../docs/navigation/jetson_mapping_motion_review.md).

For a controlled comparison, do not manually reposition the robot or reset wheel
odometry between starting the map, saving the stationary baseline and executing
the pulse. If that happens, establish a new map baseline. The runner cannot detect
an unrecorded repositioning that preceded its recording, and its success message
does not validate map quality.

Keep the existing wheel-only launch and `front_mapping_trial.launch.xml` running
in domain 182. The current mapping trial is `jetson-map-trial.700Cez`; its first
snapshot is `jetson-map-snapshot.gjlydJ/stationary.{pgm,yaml}`. The current mapper
owns the front port. Do not run the original front driver or `run_motion_check`
alongside it. Preserve sensor ports, mounting geometry, wheel calibration, plate
zero offsets and all existing profiles.

## What this trial measures

`run_mapping_motion` observes existing ROS topics and commands one finite forward
pulse. It never opens a serial port, launches/stops another process, publishes TF
or changes ROS parameters. Nominal travel is about 10 cm: forty +0.025 m/s commands
at 10 Hz, then five zero commands on the same already-matched publisher. Steering
settling and wheel behavior affect actual travel. A short movement is recorded as
failure; there is no automatic retry or pulse extension.

Before any motion, the runner requires fresh odometry and corrected scans, healthy
nominal/zero-delay input status, an active mapper, map/TF reception, the accepted
front mount, expected source publishers and one matched swerve command subscriber.
It must be the only `/cmd_vel` publisher. Ten seconds are allowed for readiness,
followed by a two-second stationary baseline. Existing static peers are not changed;
topic ownership is checked rather than assuming LOCALHOST alone isolates the graph.

During the trial it rejects stale inputs, increased input rejection counts, mapper
inactivity, competing command ownership, time discontinuities and odometry outside
the existing input envelope (40 ms gaps, 0.04 m/s translation, 0.15 rad/s rotation).
Displacement limits relative to the baseline are 150 mm total, 10 mm backwards,
30 mm lateral or 0.10 rad yaw. Command gaps must stay below 150 ms, and the pulse
must reach its first zero before 4.3 seconds. No delayed catch-up messages are sent.

These are software checks, not guaranteed physical stopping distances. On a fault
after motion begins, it attempts five zeros on the existing publisher and exits
without retry. A blocked process, failed transport, slip or power failure can defeat
software stopping evidence; keep the motor cutoff accessible and use it if motion
does not stop. Ctrl+C is caught while ROS remains available for the abort zeros.

Completion requires at least 30 mm forward displacement, at least ten corrected
scan reference stamps within the command interval, and a stationary odometry window
of 25 samples spanning at least 0.4 seconds. The whole window must stay within
0.5 mm and 0.005 rad. After settling, five more stationary seconds are recorded.
The overall deadline is 35 seconds. Physical stop still needs the user's observation.

Each new `~/jetson-mapping-motion.XXXXXX` directory contains:

- A SQLite rosbag of `/odom`, `/lidar_front/scan_mapping`,
  `/lidar_front/mapping_status`, `/map`, `/tf`, `/tf_static`, `/cmd_vel`.
- `odom.csv`, `events.csv` and `result.json`, including completion, publication
  counts, displacement and whether occupancy cell data changed.

The bag preserves unknown scan bins as NaN. Commands are recorded at publication;
they are not controller-delivery acknowledgements. The raw input capture in 700Cez
already sealed, so this new bag records corrected output rather than new raw timing
provenance. Acquisition timing remains uncalibrated (`acquisition_time_valid=0`).
`map_cells_changed` is an occupancy-data hash comparison, not map-quality acceptance.

## Transfer and build — user runs these

On the desktop:

```bash
scp /tmp/jetson-mapping-motion-20261027-v1.tar.gz turtle@10.2.12.123:/home/turtle/
```

On Jetson, create a new directory and build the separate executable. The chained
commands stop on failure. Do not overwrite an existing directory or rebuild the
running mapper workspace.

```bash
mkdir ~/jetson-mapping-motion-v1 &&
tar -xzf ~/jetson-mapping-motion-20261027-v1.tar.gz -C ~/jetson-mapping-motion-v1 &&
cd ~/jetson-mapping-motion-v1 &&
sha256sum --check SOURCE.sha256 &&
source /opt/ros/lyrical/setup.bash &&
cmake -S mapping_motion -B build &&
cmake --build build --target run_mapping_motion -j2
```

## Run once — user operates the robot

Proceed only after the checksum and build succeed, with the robot stationary and
the already-confirmed clear floor and steering space still available. Keep both
current launches running. In a third Jetson terminal:

```bash
source /opt/ros/lyrical/setup.bash
export ROS_DOMAIN_ID=182
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
~/jetson-mapping-motion-v1/build/run_mapping_motion --execute-forward-pulse
```

Observe the movement and stop. A failed run retains partial data: share the printed
failure and directory; do not rerun automatically or loosen a gate to obtain a pass.
On success and after confirming a complete physical stop, save a separate snapshot:

```bash
source /opt/ros/lyrical/setup.bash
export ROS_DOMAIN_ID=182
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
map_after_dir=$(mktemp -d "$HOME/jetson-map-after-forward.XXXXXX")
echo "$map_after_dir"
ros2 run nav2_map_server map_saver_cli -f "$map_after_dir/after_forward" \
  --ros-args -p save_map_timeout:=10.0 -p map_subscribe_transient_local:=true
```

Send the runner output/trial directory, new snapshot directory and physical movement
and stopping observations. The next analysis compares odometry, map->odom changes,
scan alignment and before/after occupancy on their world coordinates. A successful
short trial does not establish route mapping, loop closure, relocalization or Nav2
readiness; determine the next bounded movement from this evidence.

## Hardware-free verification

Desktop C++17 warnings-as-errors build and all eight ROS scenarios passed on
2026-10-27. Normal simulated travel was 101.130 mm, with 40 forward plus five zero
publications, 124 scans (40 during the command interval), 25 maps and a readable
bag. Every injected fault after motion stopped further nonzero publication and
recorded five zeros; a competing publisher at preflight produced no commands.
Test artifacts: `/tmp/mapping-motion-test.9ownbb`; build/output log:
`/tmp/lidar-mapping-motion-build/test-output.txt`.

The C++ fixture uses only `/mapping_trial_test/*` in desktop domain 193 with no
static peers. It supplies simulated odometry, scans, status, map, TF and mapper
lifecycle. It checks normal delivery and reopened bag contents, stale odometry,
stale scans, input faults, inactive mapper, a competing command publisher during
motion, a stalled command schedule and a competing publisher before motion.
Core checks cover body-frame bounds, speed/gap limits, strict status parsing and
interior excursions in the stationary window.

```bash
source /opt/ros/lyrical/setup.bash
cmake -S tools/lidar_diagnostics/mapping_motion -B /tmp/lidar-mapping-motion-build
cmake --build /tmp/lidar-mapping-motion-build -j2
ROS_DOMAIN_ID=193 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_STATIC_PEERS= \
ROS_LOG_DIR=/tmp/lidar-mapping-motion-build/ros-log \
  /tmp/lidar-mapping-motion-build/test_mapping_motion
```

These desktop tests validate the runner and recording behavior; they do not actuate
or validate the physical robot. The source bundle contains no desktop binaries.
