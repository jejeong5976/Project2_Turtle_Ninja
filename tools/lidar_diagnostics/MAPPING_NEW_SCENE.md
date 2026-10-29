# Start continuous mapping in a new physical scene

This procedure was completed in nEKS7T/AEjKu5, with identical stationary baselines
and a successful controlled forward trial. The user subsequently reported another
physical scene change and completed a fresh-map rotation in NJWQJ7/JHydTT.
Preserve that current session while the scene stays unchanged; see the
[rotation review](../../docs/navigation/jetson_mapping_rotation_review.md).
Use the [rotation procedure](MAPPING_ROTATION.md) only when another such trial is
needed, not to repeat the completed turn. See the
[completed review](../../docs/navigation/jetson_mapping_newscene_review.md).
The instructions below retain the earlier forward-trial procedure.

2026-10-27. The user confirms the robot or surroundings physically changed again.
Start a new mapping session at the chosen current pose. Preserve the previous maps
and recordings; no reinstall, rebuild, calibration change or Git command is needed.
The user executes hardware and all Git commands; assistant SSH remains read-only.

Read-only inspection found the existing wheel launch/controller active and the old
mapping launch still using `jetson-map-trial.700Cez`. Both wheel controllers report
active. `/cmd_vel` has zero publishers and one swerve subscriber. Mapping status is
healthy (published=26763, rejected=3, fault=0, fresh=1); the old input owns the front
port. Launch XML and YAML hashes match the previously verified copies. These are
observations at preparation time, not guarantees about a later runtime.

## 1. Settle the new scene

Place the robot where this mapping session should begin. Keep the manipulator
disconnected and the robot stationary. Before the later forward pulse, verify at
least 0.5 m of clear level floor ahead of the robot and space for all steering
modules. Keep the motor cutoff accessible. Avoid people crossing the scan plane
and furniture movement during the stationary baseline and short trial.

From the new map's startup until the trial ends, keep wheel odometry continuous.
Do not manually push, lift, reposition or rotate the robot, restart the wheel
controller, power-cycle the motor system, or change sensor mounting. If the robot
is repositioned or substantial scene geometry changes, make a fresh baseline again.
This requirement is specific to this controlled comparison; minor environmental
changes do not generally imply that every SLAM map must be discarded.

Preserve wheel calibration, zero plate X/Y/yaw and the accepted front mount
(.110,.005,.0545), yaw=pi relative to base_footprint. Full robot launch, teleoperation,
Nav2 and the old port-owning motion diagnostic stay stopped. Do not start the
original front driver alongside the mapping input. Rear LiDAR configuration is
unchanged; this experiment uses the front input only.

## 2. Keep three terminal roles

| Terminal | Role | Action now |
| --- | --- | --- |
| A | Existing wheel-only opencr/drive launch | Keep running |
| B | Combined mapper, front input and static mount | Stop old session, start new one |
| C | Health checks, map snapshots, later bounded motion runner | Use for the commands below |

The wheel launch is already verified running. Do not start a second copy. If it
has stopped or the Jetson has rebooted since inspection, re-establish wheel runtime
before proceeding instead of assuming these instructions describe the live state.

## 3. Stop the old mapping session — Terminal B

Press Ctrl+C in the terminal running `front_mapping_trial.launch.xml`. Wait for
the launch to exit and the shell prompt to return. Leave Terminal A running.
Check the front port is no longer owned:

```bash
fuser /dev/serial/by-path/platform-3610000.usb-usb-0:2.4.4:1.0-port0
```

No PID output is expected; fuser normally returns status 1 when no owner exists.
If a PID remains or the old launch does not exit, stop here and report it rather
than starting another port owner. Do not use a broad kill command.

## 4. Start the new mapping session — Terminal B

Copy the commands with their line breaks:

```bash
source /opt/ros/lyrical/setup.bash
source ~/ros2_front_mapping_v1_ws/install/setup.bash
export ROS_DOMAIN_ID=182
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

new_map_dir=$(mktemp -d "$HOME/jetson-map-newscene.XXXXXX")
echo "$new_map_dir"
export ROS_LOG_DIR="$new_map_dir/ros-log"

ros2 launch ~/ros2_front_mapping_v1_ws/config/front_mapping_trial.launch.xml \
  capture_dir:="$new_map_dir"
```

Record the directory path. This unchanged launch starts a new SLAM instance and
does not load the earlier map or pose graph. Wait roughly 30 seconds, until the
mapper activates and `TIMING CAPTURE SEALED` appears. **Leave the launch running**:
the raw diagnostic file has finished, while live mapping continues. The seal alone
does not prove the corrected input is healthy; check the status next.

## 5. Check stationary health and save two baselines — Terminal C

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

Expect active [3], fault=0, fresh=1, reason=publishing, model=nominal,
assumed_delay_ns=0, acquisition_time_valid=0, bins=1600. The last timing flag is
expected because timing is uncalibrated. `/cmd_vel` must have zero publishers and
one subscriber, swerve_controller, with TwistStamped type. LOCALHOST alone is not
proof of isolation because existing static peers may be inherited; inspect the
actual endpoints. If health is missing, a command publisher exists, or a check
times out, retain its output and do not move.

If those checks pass, keep the robot and scene still and save:

```bash
map_check_dir=$(mktemp -d "$HOME/jetson-map-baseline.XXXXXX")
echo "$map_check_dir"

ros2 run nav2_map_server map_saver_cli -f "$map_check_dir/baseline_a" \
  --ros-args -p save_map_timeout:=10.0 -p map_subscribe_transient_local:=true

sleep 10

ros2 topic echo /lidar_front/mapping_status std_msgs/msg/String \
  --once --full-length --timeout 10

ros2 run nav2_map_server map_saver_cli -f "$map_check_dir/baseline_b" \
  --ros-args -p save_map_timeout:=10.0 -p map_subscribe_transient_local:=true
```

Both saves must report success. The second status must have a larger published
count, unchanged rejected count, fault=0 and fresh=1. Some startup rejections are
acceptable; the count need not be zero. A changed grid extent or different cell
count alone is not proof of bad mapping; the saved files need spatial comparison.
Do not continue a block after a failed save. Default threshold/format notices and
the known lifecycle-destructor warning after a successful map save do not alone
invalidate the saved files.

**Current checkpoint:** send the new_map_dir and map_check_dir paths plus health
output, and confirm the robot stayed stationary. The assistant can inspect the
new data and live state through read-only SSH. Keep Terminals A/B and this terminal
open, with no repositioning. The following motion stage is for after that baseline
inspection, not an instruction to repeat motion immediately after map saving.

## 6. One bounded forward movement — after baseline inspection

Recheck the current physical clearance; do not reuse the old scene's clearance
confirmation. Keep A/B running. In Terminal C:

```bash
~/jetson-mapping-motion-v1/build/run_mapping_motion --execute-forward-pulse
```

The existing runner waits for fresh data and proper command ownership, records a
stationary baseline, then sends forty +0.025 m/s commands at 10 Hz and five zeros.
Nominal travel is about 10 cm. It checks odometry settling and records five more
stationary seconds; it also records corrected scans, odometry, status, maps, TF and
commands. Record its new `jetson-mapping-motion.XXXXXX` directory.

Observe the actual movement and complete stop. On a failure, retain the printed
reason and directory; do not rerun, extend the pulse or relax limits. Ctrl+C asks
the runner to attempt zero commands. If motion does not stop or becomes unsafe,
use the motor cutoff. Software publication and odometry checks are not physical
stopping guarantees. Full implementation limits are in [MAPPING_MOTION.md](MAPPING_MOTION.md).

## 7. Save the after-movement map — Terminal C

Only after successful software completion and observed complete physical stopping:

```bash
ros2 run nav2_map_server map_saver_cli -f "$map_check_dir/after_forward" \
  --ros-args -p save_map_timeout:=10.0 -p map_subscribe_transient_local:=true
```

Use the same Terminal C so map_check_dir still refers to this session. If it was
closed, restore that variable to the exact previously printed baseline directory
before saving; do not save under an empty variable or reuse an older scene.

Send the motion directory/output and whether it stopped on its own, plus any scene
changes or intervention. The assistant will compare these maps with this session's
baseline, inspect scan consistency and map->odom changes, and determine the next
bounded mapping step. A single successful pulse does not validate a full route,
loop closure, relocalization or navigation. Do not start a long drive or Nav2 yet.

## 8. Finish or relocate

To continue the current experiment, keep the launches and physical placement
unchanged while reviewing. To end it: save the desired map, confirm the robot is
stopped, Ctrl+C Terminal B and wait for exit, then Ctrl+C Terminal A and wait for
exit before switching motor power off. Shut down Jetson normally before removing
its power. All old/new directories remain preserved.

If moving the robot or changing the scene for another controlled trial, start a
new baseline next time. The saved PGM/YAML is an occupancy snapshot; the launch
above does not restore the previous running mapping session.
