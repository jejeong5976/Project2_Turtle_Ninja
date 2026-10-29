# Fresh-map bounded rotation with the live mapper

Completed on 2026-10-27: NJWQJ7/JHydTT has matching stationary baselines,
7.4063° rotation, <0.06 mm translation, healthy input and a user-confirmed smooth
automatic stop in an unchanged scene. Final map->odom correction is 3.235 mm /
0.600°. Preserve this session and placement; do not repeat the turn or reset the
map merely to follow this guide again. See the
[completed review](../../docs/navigation/jetson_mapping_rotation_review.md).
The subsequent forward pulse also passed within NJWQJ7; see the
[combined sequence review](../../docs/navigation/jetson_mapping_turn_forward_review.md).
The instructions below retain the executed procedure for future reference.

Prepared 2026-10-27 after the user reported another physical scene change and
requested a fresh map followed by a rotation test. The user confirms stationary
robot, motor power on, manipulator disconnected and clear level space around the
whole robot and steering modules. User executes hardware, deployment and all Git
commands. Assistant SSH remains read-only.

The previous clean forward run AEjKu5 remains valid historical evidence. This
rotation must start from a new map; do not compare it directly with the old scene.
Read-only inspection at about 16:49 KST found wheel launch PID 4948 and old mapping
launch PID 8071 (nEKS7T) still running. Launch XML checksum is unchanged. These
observations are not a guarantee about a later runtime.

## What the new runner does

`run_mapping_motion --execute-rotation-pulse` records the seven existing streams
without opening any serial device, changing parameters/TF, or restarting a driver.
It waits for healthy input and a stationary baseline, then sends exactly twenty
TwistStamped commands at 10 Hz with linear.x=0 and angular.z=+0.10 rad/s. All other
components are zero. This requests a positive yaw turn, about 0.20 rad / 11.5° over
two seconds, followed by five explicit zeros. Actual rotation depends on steering
and motor response. There is no retry, extension or subsequent reverse pulse.

The existing forward behavior remains available under its original explicit flag.
No motion mode is selected by default. `result.json` now records the selected mode.

Rotation-specific bounds relative to the stationary baseline:

- Translation under 30 mm; yaw greater than -0.020 and less than +0.250 rad.
- First zero before 2.2 seconds; command gaps below 150 ms, with no catch-up.
- At least +0.100 rad actual wheel-odometry yaw and ten corrected scans during
  the command interval are required for completion; insufficient motion fails.

Existing checks stay in force: expected sole publishers/subscriber, active mapper,
accepted front mount, nominal/zero-delay status, no new rejection count, fresh data,
no clock jump, odometry gaps <=40 ms and pose speed <=0.04 m/s / 0.15 rad/s.
After zeros, odometry must settle within three seconds: 25 samples spanning at
least 0.4 seconds, all within 0.5 mm / 0.005 rad. Five more stationary seconds are
recorded. Overall deadline remains 35 seconds. On a fault after a motion attempt,
the existing publisher attempts five abort zeros and exits. A late fault may
therefore record the normal five zeros plus five abort zeros. No automatic retry.

These checks do not establish physical stopping. Observe the robot and use the
accessible motor cutoff if needed. Ctrl+C requests best-effort abort zeros.

## 1. Transfer and build v2 — user runs

On the desktop:

```bash
scp /tmp/jetson-mapping-motion-20261027-v2.tar.gz turtle@10.2.12.123:/home/turtle/
```

On Jetson in terminal C (a new directory is intentional; preserve v1):

```bash
mkdir ~/jetson-mapping-motion-v2 &&
tar -xzf ~/jetson-mapping-motion-20261027-v2.tar.gz -C ~/jetson-mapping-motion-v2 &&
cd ~/jetson-mapping-motion-v2 &&
sha256sum --check SOURCE.sha256 &&
source /opt/ros/lyrical/setup.bash &&
cmake -S mapping_motion -B build &&
cmake --build build --target run_mapping_motion -j2
```

Continue only after every checksum and build step succeeds. If the directory
already exists, report it; do not overwrite or silently reuse an unverified build.
No mapper workspace rebuild is needed.

## 2. Start a fresh map — terminals A and B

Keep the existing wheel-only launch in terminal A running. If it has stopped or
the Jetson has rebooted, re-establish wheel runtime before this procedure.
In terminal B, Ctrl+C the old `front_mapping_trial.launch.xml` and wait until its
shell prompt returns. Check that the old front input released the port:

```bash
fuser /dev/serial/by-path/platform-3610000.usb-usb-0:2.4.4:1.0-port0
```

Expect no PID output (exit status 1 is normal for an unowned port). If a PID remains,
do not start another driver. Keep the original driver and old `run_motion_check`
coordinator stopped: both would contend for this same front port.

Start the unchanged launch in terminal B, copying the line breaks:

```bash
source /opt/ros/lyrical/setup.bash
source ~/ros2_front_mapping_v1_ws/install/setup.bash
export ROS_DOMAIN_ID=182
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

rotation_map_dir=$(mktemp -d "$HOME/jetson-map-rotation.XXXXXX")
echo "$rotation_map_dir"
export ROS_LOG_DIR="$rotation_map_dir/ros-log"

ros2 launch ~/ros2_front_mapping_v1_ws/config/front_mapping_trial.launch.xml \
  capture_dir:="$rotation_map_dir"
```

Wait about 30 seconds for activation and `TIMING CAPTURE SEALED`. Leave it running;
only the initial raw diagnostic file sealed, while corrected scans and mapping
continue. Keep wheel odometry continuous and the robot/scene still until the pulse.
Do not reposition or restart the wheel controller between baseline and after-map.
Keep accepted mounts, USB sockets, wheel calibration and zero plate X/Y/yaw.

## 3. Check health and save a stable fresh baseline — terminal C

```bash
source /opt/ros/lyrical/setup.bash
export ROS_DOMAIN_ID=182
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

timeout -s INT -k 2s 15s ros2 lifecycle get /slam_toolbox \
  --no-daemon --spin-time 3 --timeout 5
ros2 topic echo /lidar_front/mapping_status std_msgs/msg/String \
  --once --full-length --timeout 10
ros2 topic info /cmd_vel --verbose --no-daemon --spin-time 2
```

Require active [3], fault=0, fresh=1, reason=publishing, model=nominal,
assumed_delay_ns=0, acquisition_time_valid=0 and bins=1600. Zero command publishers
and exactly one swerve_controller subscriber of type TwistStamped must be present.
Existing static peers are unchanged; LOCALHOST alone does not prove isolation.
If a check fails or times out, retain the output and do not proceed to motion.

When healthy, save the two maps without any robot or scene movement:

```bash
rotation_snapshot_dir=$(mktemp -d "$HOME/jetson-map-rotation-snapshots.XXXXXX")
echo "$rotation_snapshot_dir"

ros2 run nav2_map_server map_saver_cli -f "$rotation_snapshot_dir/baseline_a" \
  --ros-args -p save_map_timeout:=10.0 -p map_subscribe_transient_local:=true &&
sleep 10 &&
ros2 run nav2_map_server map_saver_cli -f "$rotation_snapshot_dir/baseline_b" \
  --ros-args -p save_map_timeout:=10.0 -p map_subscribe_transient_local:=true

ros2 topic echo /lidar_front/mapping_status std_msgs/msg/String \
  --once --full-length --timeout 10

cmp "$rotation_snapshot_dir/baseline_a.pgm" "$rotation_snapshot_dir/baseline_b.pgm" &&
diff -u <(sed '/^image:/d' "$rotation_snapshot_dir/baseline_a.yaml") \
        <(sed '/^image:/d' "$rotation_snapshot_dir/baseline_b.yaml") &&
echo "BASELINE MATCHES"
```

Both saves must report success. Require a rising published count, unchanged rejected
count, fault=0 and fresh=1. Startup rejections need not be zero. Require
`BASELINE MATCHES` before the rotation; a difference is a conservative pause for
inspection, not proof of sensor failure. Send paths/output if it does not match.
This pixel/settings check does not replace the user's observation of a still scene.
Known default-format and post-save lifecycle warnings do not invalidate a successful
save. Keep terminal C open to retain its snapshot-directory variable.

## 4. Execute exactly one turn — terminal C

After the build, fresh-baseline and health checks pass, with confirmed physical
clearance still available and terminals A/B running:

```bash
~/jetson-mapping-motion-v2/build/run_mapping_motion --execute-rotation-pulse
```

Observe steering, rotation direction and complete physical stopping. Save the
printed `jetson-mapping-motion.XXXXXX` path. On any failure, retain the partial
files and printed reason; do not rerun or relax limits. Intervene with motor cutoff
if needed rather than relying on a software pass. The runner neither saves the
occupancy snapshot nor stops the existing wheel/mapper launches.

## 5. Save and report the after-map

Only after software success and an observed complete stop, in the same terminal C:

```bash
ros2 run nav2_map_server map_saver_cli -f "${rotation_snapshot_dir:?Missing snapshot directory}/after_rotation" \
  --ros-args -p save_map_timeout:=10.0 -p map_subscribe_transient_local:=true
```

Send the new map-launch, snapshot and motion directories; runner output; whether
rotation was smooth and stopped completely on its own; and any scene change or
intervention. Keep A/B and placement unchanged during review. The assistant can
retrieve the files read-only and compare stationary consistency, yaw/translation,
stopping evidence, map->odom correction and frozen-scene residuals from this run.
The reviewer supports both finite pulse modes. Timing remains uncalibrated; the
sealed initial raw file does not record later rotation acquisition provenance.

To finish instead, confirm stopping, stop B then A with Ctrl+C, wait for both to
exit and switch motor power off. Shut Jetson down normally before removing power.

## Desktop verification

All 20 desktop ROS scenarios passed with C++17 warnings-as-errors on 2026-10-27.
Artifacts: `/tmp/mapping-motion-test.TIBLoz`; log:
`/tmp/lidar-mapping-rotation-build/test-output-network.txt`.
Normal rotation recorded 20 yaw + five zero commands, 0.202260 rad yaw, no
translation, 103 scans (20 within the command interval), and readable seven-topic
bag data. Its first zero followed the first yaw command by 2.02256 s; maximum
command gap was 102.113 ms and maximum odometry gap 22.394 ms. These are simulated
results, not Jetson stopping or mapping evidence.

The initial sandbox run passed 18 scenarios but a real desktop odometry gap
tripped the 40 ms guard during the translation-limit case, before that case's
intended fault. ROS also reported blocked local sockets. A complete rerun with
normal local ROS networking passed all 20, without loosening any runner limit.
The reviewer processes the synthetic rotation bag, and reanalysis of real forward
capture AEjKu5 reproduces all prior metrics (only the new `motion=forward` label
is added). Root topic names for the synthetic reviewer input are relabeled in an
offline bag copy; test ROS publications remain under the test namespace.

The test fixture uses only `/mapping_trial_test/*`, domain 193, LOCALHOST and empty
static peers. It checks forward/rotation delivery, reopened bag contents, five-zero
stopping sequences, stale odom/scans, input faults, inactive mapper, command
competition, scheduling stalls, plus wrong yaw, excess yaw/translation and
insufficient rotation. Core checks include wrapped yaw and explicit mode selection.
Desktop tests do not move or validate the physical robot.

```bash
source /opt/ros/lyrical/setup.bash
cmake -S tools/lidar_diagnostics/mapping_motion -B /tmp/lidar-mapping-rotation-build
cmake --build /tmp/lidar-mapping-rotation-build -j2
ROS_DOMAIN_ID=193 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_STATIC_PEERS= \
ROS_LOG_DIR=/tmp/lidar-mapping-rotation-build/ros-log \
  /tmp/lidar-mapping-rotation-build/test_mapping_motion
```
