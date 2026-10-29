# Bounded reverse-and-return revisit

Completed 2026-10-27 in NJWQJ7 / caGdIe after JHydTT / gcQZct.
User confirms both automatic stops and unchanged scene. Motion/input/coverage
pass, but revisit scan consistency remains unresolved (16.929 mm RMS versus
2.973 mm holdout). See [route review](../../docs/navigation/jetson_mapping_route_review.md).
The procedure below is completed history; do not repeat it.
The user executes every hardware and Git command. Assistant SSH is read-only.

## Confirmed physical state and route

User confirms no reboot, manual relocation or scene change since gcQZct; motors
powered, manipulator disconnected. Robot front faces the nearby brown box.
Clearance from the outer robot envelope: 250 mm front, 450 mm rear; cables clear;
at least 50 mm each side of the full steering envelope along the proposed route.
These observations apply to this placement, not a future moved/restarted robot.

Keep NJWQJ7, wheel odometry, front-only continuous mapping, LiDAR sockets/settings,
mounts/body mask, assumed wheel calibration and zero plate offsets unchanged.
No mapper reset, AMCL, IMU fusion or wheel calibration is introduced.

One invocation of the separate C++ `run_mapping_route` does the following:

1. Wait up to 10 seconds for stationary odometry, healthy scans/status, active
   mapper, map/TF and sole matched command ownership. Record a 2-second baseline.
2. Reverse at -0.025 m/s, with zero yaw command. Stop at 100 mm backward odometry
   displacement or after 40 commands at 10 Hz, whichever comes first. Steering
   settling may reduce travel; no extension or retry is allowed.
3. Publish five zeros, require a stationary odometry window, and record five
   stationary seconds. Reverse displacement must be at least 30 mm to continue.
4. Return at +0.025 m/s without changing heading. Stop when odometry reaches
   5 mm behind the initial pose or after 40 commands, whichever comes first.
5. Publish five zeros, verify settling, record five stationary seconds, then exit.
   The final position must be within 20 mm and yaw within 0.030 rad of the initial
   odometry pose. An incomplete return stops and fails; it never adds a correction.

Both legs have a 4.3-second first-command-to-stop deadline; command intervals
must stay below 150 ms. Whole trial deadline: 45 seconds. Each stop must settle
within three seconds after the zero sequence. The window is 25 odometry samples
spanning >=0.4 seconds, with <0.5 mm / <0.005 rad variation. Stationary captures
also remain within those bounds of a fixed anchor to reject slow cumulative creep.

Global odometry bounds relative to the start: backward <150 mm, forward <20 mm,
lateral <20 mm, yaw <0.050 rad. Retain the existing input envelope: odometry gaps
<=40 ms, translation <=0.04 m/s, rotation <=0.15 rad/s. Increased input rejection,
stale streams, mapper inactivity, command competition and clock discontinuity
abort the route. After any motion attempt, an exception attempts five abort zeros
on the same publisher. No following leg or automatic retry occurs after failure.

These are software bounds, not physical braking guarantees or obstacle avoidance.
Observe both stops and keep the motor cutoff accessible. On unexpected motion or
failure to stop, use it. Ctrl+C requests abort zeros while ROS remains available.
The return starts automatically after the five-second healthy stationary capture.
The accepted body mask is not being promoted to a validated collision footprint.

## Present runtime evidence

Read-only SSH around 17:40–17:41 KST found wheel launch PID 4948, NJWQJ7 launch
8963, mapping input 8982 and mapper 8983. Mapper active [3]; input published=24975,
rejected=4, fault=0, fresh=1. Six seconds of observation yielded 301 odometry
messages and 60 corrected scans. Latest observed receipt ages were about 10 ms
and 24 ms. Map->odom still matched gcQZct: (+0.551,-8.183) mm, yaw 0.024430 rad.
Command publishers: zero; subscriber: swerve_controller. Front serial owned by
8982, drive by 4953, rear unowned in the inspected process view. Storage: 190 GB
available. Static peer 10.2.12.120 remains configured in the running input.
These are observations, not instructions to act on old PIDs. The runner rechecks
current readiness. A reboot/manual relocation invalidates this continuation.

## User-run transfer and build

Desktop terminal:

```bash
scp /tmp/jetson-mapping-route-20261027-v1.tar.gz turtle@10.2.12.123:/home/turtle/
```

Separate Jetson terminal, leaving existing wheel/mapper launches running:

```bash
mkdir ~/jetson-mapping-route-v1 &&
tar -xzf ~/jetson-mapping-route-20261027-v1.tar.gz -C ~/jetson-mapping-route-v1 &&
cd ~/jetson-mapping-route-v1 &&
sha256sum --check SOURCE.sha256 &&
source /opt/ros/lyrical/setup.bash &&
cmake -S mapping_route -B build &&
cmake --build build --target run_mapping_route -j2
```

Do not overwrite an existing directory. A failure stops this chain; retain the
output for inspection. No running workspace is rebuilt or changed.

## User-run baseline, one route, and final save

In the same Jetson terminal after a successful build:

```bash
source /opt/ros/lyrical/setup.bash
export ROS_DOMAIN_ID=182
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

route_snapshot_dir=$(mktemp -d "$HOME/jetson-map-route-snapshots.XXXXXX")
echo "$route_snapshot_dir"
ros2 run nav2_map_server map_saver_cli -f "$route_snapshot_dir/before_route" \
  --ros-args -p save_map_timeout:=10.0 -p map_subscribe_transient_local:=true &&
cmp "$HOME/jetson-map-turn-forward.IcK9xK/after_forward.pgm" "$route_snapshot_dir/before_route.pgm" &&
diff -u <(sed '/^image:/d' "$HOME/jetson-map-turn-forward.IcK9xK/after_forward.yaml") \
        <(sed '/^image:/d' "$route_snapshot_dir/before_route.yaml") &&
echo "MAP CONTINUITY CHECK PASSED"
```

Require the continuity message before motion. A mismatch needs inspection; it
is not proof of a sensor failure and does not justify resetting the mapper.
With unchanged confirmed clearance, run exactly once:

```bash
~/jetson-mapping-route-v1/build/run_mapping_route --execute-reverse-return
```

Do not run the v2 pulse runner, old port-owning diagnostic, teleop or Nav2 alongside
this route. Observe the backward stop, automatic return and final stop.
On failure, retain the printed directory/output and do not rerun or manually
reposition. After software success AND observed complete physical stopping:

```bash
ros2 run nav2_map_server map_saver_cli \
  -f "${route_snapshot_dir:?Missing snapshot directory}/after_route" \
  --ros-args -p save_map_timeout:=10.0 -p map_subscribe_transient_local:=true
```

Send the route directory, snapshot directory, console output and whether both
legs stopped on their own. Keep the map and wheel launches running for retrieval.

## Evidence and acceptance

The route creates `~/jetson-mapping-route.XXXXXX` containing all seven existing
streams in a SQLite bag, `odom.csv`, `events.csv`, `legs.csv` and `result.json`.
The leg CSV records each interval, message/zero/scan counts and stopped forward
position. No serial access, hardware configuration or TF publishing occurs.
Acquisition timing remains nominal/zero hypothetical delay and
`acquisition_time_valid=0`. Corrected scans do not provide new raw timing evidence.
The raw `physical_stop_confirmed=false` stays unchanged; human observations are
recorded separately. Commands in the bag are publications, not delivery receipts.

Software success is necessary but does not establish mapping quality. Before
prescribing more travel, review:

- Both human-observed stops, no unrecorded movement and traceable NJWQJ7 continuity.
- Odometry brackets covering all command-interval scans and motion transitions;
  exclude only uncovered preflight scans without extrapolating.
- Healthy input and bounded scheduling throughout both legs, including the dwell.
- Initial/revisited scan boundaries under fixed association gates, matched-point
  coverage, map->odom changes and occupancy maps registered by their exact origins.
- No persistent doubled boundaries or unexplained pose jumps. Flag a map->odom
  translation step above one map cell (50 mm), or yaw step above 0.050 rad, for
  investigation before further motion; these are screening thresholds, not
  universal SLAM accuracy limits. Smaller steps alone do not establish a pass.

Compare revisit residuals with this run's stationary holdout; previous millimetre
residuals are not universal pass thresholds. A small odometry return error is not
independent ground truth. Loop closure requires explicit graph/event evidence;
enabling it or revisiting a view does not prove it occurred. The updated
`review_mapping_motion BAG NEW_OUTPUT --reverse-return` handles both route legs
and requires `events.csv` beside the bag directory; its default single-pulse mode
still rejects a two-leg route.

## Hardware-free verification

The separate test executable uses only `/mapping_route_test/*`, desktop domain
193, LOCALHOST and empty static peers. It exercises normal reverse/return and
injected failures before motion, in reverse and in return, checking reopened bag
commands and explicit abort zeros. It does not open a serial port or operate the
robot. Reproduce with:

```bash
source /opt/ros/lyrical/setup.bash
cmake -S tools/lidar_diagnostics/mapping_route -B /tmp/lidar-mapping-route-build
cmake --build /tmp/lidar-mapping-route-build -j2
ROS_DOMAIN_ID=193 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_STATIC_PEERS= \
ROS_LOG_DIR=/tmp/lidar-mapping-route-build/ros-log \
  /tmp/lidar-mapping-route-build/test_mapping_route
```

Validation completed locally: C++17 `-Wall -Wextra -Wpedantic -Werror` build
and 16 ROS scenarios passed (14 baseline/fault scenarios, plus a simulated
0.85-second steering delay and anchored stationary-creep rejection). Normal
return ended 4.674 mm behind the initial pose; delayed response ended 4.810 mm
behind it. Both published 40 reverse / 38 return commands and 10 normal zeros.
Preflight competition published no commands; injected post-motion faults
attempted five abort zeros without further moving commands. All test bags were
reopened and command delivery compared against the local fixture. Sandbox DDS
interface-inspection warnings occurred; the tested local delivery checks passed.
These simulations do not validate physical reverse travel or physical stopping.

Build and logs: `/tmp/lidar-mapping-route-build/{test-output.txt,test-extra-output.txt}`.
