# Front-only continuous mapping input prototype

Status: 2026-10-27. Offline analysis, desktop tests, stationary Jetson live input
and initial mapper integration/map saving passed. Moving-route validation,
physical map quality, loop closure, relocalization and navigation remain untested.
The user executes all hardware and Git commands; assistant SSH is read-only.

Latest result: NJWQJ7 now has a controlled turn-to-forward sequence: JHydTT's
7.4063° turn followed by gcQZct's 84.389 mm forward travel. User confirms automatic
stops and unchanged scene; map/odom/TF boundary checks establish continuity.
Cumulative map correction is 8.201 mm / 1.400°, with healthy input throughout.
Preserve this session and placement; see the
[combined review](../../docs/navigation/jetson_mapping_turn_forward_review.md).
The next validation is a short bounded route revisiting a known view, with geometry
chosen from available physical space. No longer route has been prepared or run.

Earlier completed trial: fresh-map rotation NJWQJ7/JHydTT, 7.4063° yaw with <0.06 mm
translation, matching stationary baselines, healthy input and confirmed smooth
automatic stopping in an unchanged scene. Final map->odom correction is 3.235 mm /
0.600°. Preserve this session and placement; see the
[rotation review](../../docs/navigation/jetson_mapping_rotation_review.md).
The subsequent forward move in that same session is summarized above.

Earlier completed new-scene trial: mapping session nEKS7T and forward capture AEjKu5.
The user confirms no intervening repositioning/scene changes and a complete
automatic stop. Stationary maps A/B are identical; the 102.077 mm forward move
updates occupancy with only 2.858 mm / 0.600° map->odom correction. Input status
stays healthy without new rejections. Preserve this completed trial's evidence.
See the [new-scene review](../../docs/navigation/jetson_mapping_newscene_review.md).
The user subsequently reported another physical scene change and completed the
fresh-map rotation summarized above. The v2 live-map recorder supports that mode; see
[MAPPING_ROTATION.md](MAPPING_ROTATION.md). The old port-owning rotation coordinator
cannot run alongside this input.

For a future physical scene change, use [MAPPING_NEW_SCENE.md](MAPPING_NEW_SCENE.md)
for restart, stationary baseline, bounded movement, save and shutdown instructions.

Earlier result: the user completed the first bounded live-map forward pulse in
`jetson-mapping-motion.gUGiCK` and confirms automatic complete stopping. Motion/input
checks passed (96.919 mm forward, 128 scans, 679 odom messages, no new rejections).
The map->odom correction was about 25 cm / 14.20°. The user subsequently confirmed
the robot was repositioned after the earlier stationary map, so this does not
establish a clean mapping comparison or a SLAM fault. Start a fresh stationary map
at the current pose before further movement, preserving all configurations and
continuous wheel odometry. Detailed [review](../../docs/navigation/jetson_mapping_motion_review.md).

The first saved map is `jetson-map-snapshot.gjlydJ/stationary.{pgm,yaml}` from
trial `jetson-map-trial.700Cez`. Verified 93 x 70 cells at 0.05 m resolution:
66 occupied, 1,235 free and 5,209 unknown. Map/save-file checksums match the
desktop copy. The raw 200-frame capture is complete and passes strict provenance
and clock checks. User lifecycle/status checks now confirm active [3],
published=3670, rejected=3, fault=0, fresh=1, reason=publishing. Read-only SSH
independently confirms active [3] and published=3880 with unchanged rejection
count. The earlier no-output report was caused by pasting all commands onto
one line as arguments to `source`; separate lines fixed it. The user now confirms
stationary capture and >=0.5 m clear level floor ahead plus steering clearance.
The bounded forward pulse is documented in [MAPPING_MOTION.md](MAPPING_MOTION.md);
its completed result and the intervening repositioning supersede the preparation
state described below.

The startup minimum-range warning is a float precision boundary, not a 0.1 m
configuration change. The YAML double 0.05 compares slightly below the scan's
float32 0.05000000074505806, and upstream clamps to that scan minimum; its warning
formats both as 0.1. Keep profiles unchanged. Source:
https://raw.githubusercontent.com/SteveMacenski/slam_toolbox/2.10.0/src/laser_utils.cpp

The combined mapping launch remained running at the 15:37 KST SSH inspection.
It owns the front port: do not start the original driver or `run_motion_check`
concurrently. The separate moving-map runner records corrected scans, odometry,
status, maps, TF and commands; the first raw capture has already sealed and will
not record later motion.

Relocation checkpoint, 2026-10-27 15:24 KST: the user completed mapper package
installation and launch XML transfer, verified through read-only SSH. The combined
mapper trial has not started. Shutdown was requested for relocation but is not
yet confirmed. After reboot, inspect runtime and restart the existing wheel-only
profile before the first-map launch; do not assume it survived shutdown. Use a
new capture directory and scene reference. No reinstall/rebuild is needed.

Resume at 15:34 KST: SSH confirms a fresh boot, no wheel/LiDAR/mapper processes,
unowned drive/front ports and matching launch/config hashes. User confirms Jetson
on, motors off, manipulator disconnected and stationary. User turns motor power
on, restarts the existing wheel-only opencr/drive launch in domain 182/LOCALHOST,
and checks controllers plus /odom before starting the combined mapping launch.
There is no original front process to stop in this inspected fresh-boot state.

The stationary capture is `/home/turtle/jetson-mapping-first.m6Nmzn`: 200 complete
raw input frames, overflow=0 and clock_bad=0; strict timing replay passes. Runtime
status advanced from 220 to 330 accepted outputs with rejected=4 unchanged,
fault=0, fresh=1 and about 10 Hz. Startup logged stale odom. The wait-set error
at the rate observer's timeout did not stop the driver; a subsequent status was
fresh. User Ctrl+C closed the sensor cleanly, and read-only SSH confirmed the
original front driver restored at 15:12 KST. Acquisition timing is still uncalibrated.

## Evidence and approximation

Inputs are the complete translation capture `jetson-motion-auto.9RvB4b` and
rotation capture `jetson-motion-auto.H9GTjD`. The interrupted T60ife capture is
excluded. All figures below use nominal 250 us raw-source timing and **zero
hypothetical delay**, not a calibrated sensor clock. Acquisition_time_valid stays 0.

`mapping_scan.hpp` takes corrected endpoints AND origins in the front sensor
reference frame. It rounds endpoint bearings to the nearest uniform angular bin,
keeps the nearest return on collision, then rejects excessive approximation.
Selecting the foreground first prevents a rejected foreground from exposing a
farther candidate in the same bin. Unknown/body/invalid/rejected bins are NaN;
there is no infinity, interpolation of missing returns or temporal filling.
Exact half-bin ties are deterministic across decimal replay and live arithmetic.

| Capture | Exterior rays | Kept: 400 bins | Kept: 800 bins | Kept: 1600 bins | 1600-bin endpoint RMS / max | Max origin shift |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Translation | 42,353 | 36,595 | 41,690 | 42,353 | 4.399 / 22.519 mm | 2.545 mm |
| Rotation | 39,385 | 34,680 | 38,765 | 39,385 | 4.272 / 21.373 mm | 1.105 mm |

The 1,600-bin grid has no collisions or approximation rejects on these captures.
It carries at most the original 400 selected returns, with the rest unknown.
The policy rejects endpoint displacement above 25 mm or origin shift above 5 mm.
For corresponding fractional positions along the original corrected ray and the
new radial ray, separation is bounded by the larger endpoint/origin difference.
This geometric statement assumes the timing model; it does not prove equivalent
visibility at occlusion boundaries or validate Nav2 free-space clearing.

The frozen-scene motion-group RMS changes from 11.852 to about 11.945 mm for
translation and from 16.151 to about 16.136 mm for rotation when resampling is
added. Associations are the same as the existing per-ray analysis. These small
changes describe the added approximation, not physical accuracy or map quality.
The original correction benefit remains modest and dependent on timing assumptions.

## Separate implementation and limits

The new package is `coin_d4_mapping_driver`; its executable is `front_mapping_input`.
`build_mapping.sh VERIFIED_UPSTREAM NEW_WORKSPACE` verifies `source.sha256`, copies
the driver, applies `timing.patch` then `live_mapping.patch`, and builds the isolated
package. It never opens hardware. It refuses an existing destination. The patched
copy replaces the stationary experiment entry point; the installed original and
its stationary-only guard remain unchanged.

The raw parser still selects the same 400 source bins and bearings. Opening and
closing ring boundaries and selected samples must have valid checksums. The
reference is the latest estimated selected beam time, including body-masked beams.
Body returns are excluded before correction using the accepted front-frame box.
The planar mount remains (+0.110,+0.005,pi); plate X/Y/yaw remain zero.

The live node enforces:

- Explicit `mapping_experiment=true`, system time, nominal/scaled model, and delay
  0..100 ms. No default calibration is invented.
- Exact front socket `platform-3610000.usb-usb-0:2.4.4:1.0-port0`, version 4,
  reverse false, 230400 baud and `lidar_front_link`. A startup fuser check must
  report an unowned device before it is opened.
- Odom frames `odom` and `base_footprint`, finite planar poses, advancing stamps;
  latest odom <=100 ms old, no future stamp >5 ms. Invalid/regressing/future odom
  latches a fault requiring restart.
- Bracketed interpolation without extrapolation, odom gaps <=40 ms, pose-derived
  speed <=0.04 m/s and yaw rate <=0.15 rad/s for the relevant interval. These are
  provisional input acceptance limits, not motor limits or stopping guarantees.
- Raw ring period 50..200 ms; scan reference age <=250 ms and <=5 ms in the future;
  clock offset change <=5 ms. Clock/accepted-scan regression latches a fault.
- Bounded queues (four sensor frames, at most 200 odom poses/three seconds), at
  least 30 accepted returns per output scan, and a final freshness check before
  publication. Missing data withholds scans; nothing commands a hardware stop.

Output `/lidar_front/scan_mapping` uses SensorDataQoS, frame `lidar_front_link`,
1,600 bins from 0 through 1599*(2*pi/1600), range 0.05..12 m. Its stamp is the
estimated reference time; `time_increment=0` means the geometry has already been
corrected to that reference, not that original acquisition was instantaneous.
The node owns no TF and publishes no commands. Status is emitted at 1 Hz on
`/lidar_front/mapping_status`, with assumptions, counters, freshness and faults.

Raw diagnostic recording seals after `timing_scans` input conversions while live
processing continues until stopped. Recorder publish records count converted
input frames, **not accepted mapping outputs**; use mapping_status for the latter.

## Verification

- `check_offline.sh` passes C++ warnings-as-errors/UBSan regression checks for
  timing, clocks, motion, coordinator, fixed-scene geometry and mapping policy/core.
- `test_mapping_replay` reconstructs both complete captures from raw provenance
  and compares all 400 scans with exported 1,600-bin CSV, including every NaN.
  All ranges agree within 1e-8 m; stricter live gap/speed gates also pass.
- A fresh build from verified upstream plus both patches succeeds.
- `test_mapping_parser` passes synthetic packets and corrupt-boundary tests using
  a pseudo-terminal, without physical hardware.
- `test_mapping_node` on desktop domain 193/LOCALHOST disables sensor construction
  and passes publication, NaN preservation, stale scan/odom rejection, recovery,
  and the latched regression fault. The normal executable has no CLI sensor bypass.
- `test_mapping_karto.cpp` against installed desktop slam_toolbox 2.10.0 confirms
  the scan count and that NaNs produce no occupancy rays. This does not validate
  a physical SLAM map or other mapper versions.

Desktop artifacts are under `/tmp/lidar-mapping-prototype/v2`,
`/tmp/lidar-offline-check-Bf2iKr`, and `/tmp/lidar-mapping-release-ws`.
They are temporary; raw captures also remain on Jetson.

## First Jetson run: stationary input only

Source-only bundle: `/tmp/jetson-front-mapping-20261027-v1.tar.gz` on the desktop.
It contains the separate package, this guide, mapper profile and SHA-256 manifest;
no desktop binaries. The user transfers it:

```bash
scp /tmp/jetson-front-mapping-20261027-v1.tar.gz turtle@10.2.12.123:/home/turtle/
```

On Jetson, build in a new directory (stop if mkdir fails; do not overwrite):

```bash
mkdir ~/ros2_front_mapping_v1_ws &&
tar -xzf ~/jetson-front-mapping-20261027-v1.tar.gz -C ~/ros2_front_mapping_v1_ws &&
cd ~/ros2_front_mapping_v1_ws &&
sha256sum --check SOURCE.sha256 &&
source /opt/ros/lyrical/setup.bash &&
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --executor sequential --packages-select coin_d4_mapping_driver
```

Proceed only if the checksum and build succeed. Keep the robot stationary, the
manipulator disconnected, full robot launch stopped and the wheel-only launch
running. Stop **only the original front LiDAR driver** with Ctrl+C in its own
terminal. Start the input in a new terminal:

```bash
source /opt/ros/lyrical/setup.bash
source ~/ros2_front_mapping_v1_ws/install/setup.bash
export ROS_DOMAIN_ID=182
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
mapping_capture_dir=$(mktemp -d "$HOME/jetson-mapping-first.XXXXXX")
echo "$mapping_capture_dir"
ros2 run coin_d4_mapping_driver front_mapping_input --ros-args \
  -p mapping_experiment:=true -p use_sim_time:=false \
  -p timing_model:=nominal -p assumed_delay_ns:=0 \
  -p port:=/dev/serial/by-path/platform-3610000.usb-usb-0:2.4.4:1.0-port0 \
  -p timing_scans:=200 -p timing_jsonl:="$mapping_capture_dir/timing.jsonl"
```

The zero delay is an explicit provisional hypothesis. After about ten seconds,
use a second terminal on Jetson:

```bash
source /opt/ros/lyrical/setup.bash
export ROS_DOMAIN_ID=182
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 topic echo /lidar_front/mapping_status std_msgs/msg/String --once --timeout 10
timeout 8s ros2 topic hz /lidar_front/scan_mapping
ros2 topic echo /lidar_front/mapping_status std_msgs/msg/String --once --timeout 10
```

Expect roughly 10 Hz, increasing `published`, `fresh=1`, `fault=0`, and no
sustained rejections after startup. The hz command intentionally stops after
eight seconds; timeout exit 124 by itself is expected. Save/share both status
messages, rate output and the printed capture directory. Allow about 30 seconds
total runtime so 200 input frames can be recorded after warmup. Stop the new node
with Ctrl+C, then restore the original front driver using the existing unchanged
domain-181 command. If faults or sustained rejects appear, preserve output and
stop this trial; do not weaken limits or add motion to make it pass.

Restore with the unchanged command verified in the running process list at
15:02 KST, after the new node exits:

```bash
source /opt/ros/lyrical/setup.bash
source ~/ros2_lidar_experimental_ws/install/setup.bash
export ROS_DOMAIN_ID=181
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 run coin_d4_experimental_driver experimental_coin_d4_node --ros-args \
  -r __ns:=/lidar_front -r __node:=driver \
  --params-file /home/turtle/jetson-lidar-source/project/config/lidar_drivers.yaml \
  -p port:=/dev/serial/by-path/platform-3610000.usb-usb-0:2.4.4:1.0-port0 \
  -p stationary_test:=true
```

## Next after the input check

The separate `config/slam_front_experiment.yaml` prepares a 5 cm front-only map
with the guarded input and accepted frames. On the last read-only inspection,
Jetson did not have slam_toolbox or nav2_map_server installed. The user handles
installation and execution after the input check.

Before starting a mapper, inspect TF ownership in domain 182: swerve owns
odom->base_footprint, robot_state_publisher owns the existing base/wheel tree,
one owner attaches lidar_front_link using accepted geometry, and slam_toolbox
alone owns map->odom. The estimated sensor height is 0.0545 m above ground,
not 0.0545 m above base_link (which is 0.033 m above base_footprint). Do not
introduce a duplicate child transform. This stationary input test needs no new TF.
Then validate stationary occupancy, a separately bounded slow continuous route,
revisit/loop consistency and map saving. A map is not accepted merely because
the input status is fresh. Rear merging and autonomous navigation remain later work.

## User-run stationary mapper trial

`config/front_mapping_trial.launch.xml` starts only the already-built front input,
one accepted front static transform and async slam_toolbox. It does not start or
modify the wheel stack. The original front driver must be stopped first. The
running wheel URDF/configuration and process list were inspected: no LiDAR TF
owner was present, and swerve owns odom->base_footprint. Recheck if runtime changes.
The XML parses; the YAML configured/activated on desktop slam_toolbox 2.10.0.
No physical map has yet been produced with it.

On Jetson, install the missing mapper and saver (no full navigation bringup):

```bash
sudo apt-get update &&
sudo apt-get install --no-remove --no-install-recommends \
  ros-lyrical-slam-toolbox ros-lyrical-nav2-map-server
```

The pre-install simulation against the current index proposed 195 new packages,
two non-ROS upgrades and no removal; many are packaged build/GUI dependencies.
The actual apt summary may change after updating the index. Do not run autoremove
or a general upgrade as part of this task. Existing LiDAR/wheel profiles stay intact.

On desktop, transfer the new XML; the YAML is already present and its hash matches:

```bash
scp /home/pa4/git/Project2_Turtle_Ninja/config/front_mapping_trial.launch.xml \
  turtle@10.2.12.123:/home/turtle/ros2_front_mapping_v1_ws/config/
```

On Jetson, keep the robot stationary and wheel launch running; stop only the
original front driver in its own terminal, then use a new terminal:

```bash
source /opt/ros/lyrical/setup.bash
source ~/ros2_front_mapping_v1_ws/install/setup.bash
export ROS_DOMAIN_ID=182
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
mapping_trial_dir=$(mktemp -d "$HOME/jetson-map-trial.XXXXXX")
echo "$mapping_trial_dir"
export ROS_LOG_DIR="$mapping_trial_dir/ros-log"
ros2 launch ~/ros2_front_mapping_v1_ws/config/front_mapping_trial.launch.xml \
  capture_dir:="$mapping_trial_dir"
```

After about ten seconds, use a second terminal:

```bash
source /opt/ros/lyrical/setup.bash
export ROS_DOMAIN_ID=182
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 lifecycle get /slam_toolbox
ros2 topic echo /lidar_front/mapping_status std_msgs/msg/String \
  --once --full-length --timeout 10
ros2 topic echo /map nav_msgs/msg/OccupancyGrid --once --field info \
  --qos-durability transient_local --timeout 15
```

Expected: mapper `active`, input `fresh=1 fault=0`, resolution 0.05 m and positive
map width/height. No sustained TF or scan-matching errors. These establish initial
mapper integration only; map contents and behavior during motion still need checks.
If those checks pass, save an initial stationary map for inspection in the same
second terminal (the mapper must still be running):

```bash
map_snapshot_dir=$(mktemp -d "$HOME/jetson-map-snapshot.XXXXXX")
echo "$map_snapshot_dir"
ros2 run nav2_map_server map_saver_cli -f "$map_snapshot_dir/stationary" \
  --ros-args -p save_map_timeout:=10.0 -p map_subscribe_transient_local:=true
```

Send both printed directories, startup errors (if any), lifecycle/status/map-info
output and map-saver result. The snapshot is an initial observation, not an
accepted navigation map; it allows inspection of occupied/free/unknown cells.
Keep this trial stationary. Stop it with Ctrl+C when finished; that stops all three
trial nodes while leaving the wheel launch alone. Restore the original front driver
with the unchanged command above. The saved raw timing and ROS logs remain in the
trial directory. Do not overwrite an earlier trial or rerun a motion pulse now.
