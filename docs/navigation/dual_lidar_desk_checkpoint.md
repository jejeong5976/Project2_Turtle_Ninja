# Reproducible stationary desk checkpoint

Reviewed 2026-10-26. This is the current build/run procedure for the stationary
experiment. Earlier setup and alignment notes retain historical intermediate steps.
Both-driver restart recovery and body exclusion were already confirmed by the user;
fine calibration and timing research remain deferred.

## Scope

The independent `desk_lidar.launch.py` starts two experimental drivers and one
static transform, optionally RViz. It does not load the robot model or include
`lidar.launch.py`. The merger runs separately. The desk RViz preset shows merged
points colored by source_id; enable its two disabled raw scan displays to inspect
alignment before the merger starts. No merged cloud is expected from drivers alone.
No driver respawn is configured. A stopped driver requires a user restart; the
merger resumes when fresh pairs return.

The launch fixes frame IDs, relative scan topics, stationary acknowledgment and
wall time to the merger's contract. The YAML supplies version 4, 230400 baud,
reverse=false, warmup_time=5. Port overrides are per sensor. Default `/dev/lidar_*`
aliases are placeholders, so provide verified ports unless aliases were installed.
Desk TF is front -> rear: (+0.220, 0, 0) m, (0, 0, pi) rad. This is the accepted
provisional desk arrangement, not a calibrated base_link transform.

## Fresh workspace build (user runs)

Prerequisites: the tested Ubuntu 24.04/ROS Lyrical development environment, colcon,
CMake, C++ compiler, patch, Python launch/ament packages, RViz, tf2_ros, and the ROS
message/rclcpp dependencies declared by the driver and merger. This procedure does
not install OS dependencies or certify a different ROS/Jetson environment.
The repository path below must contain this checkpoint and the earlier merger and
experimental-driver commits. Choose new workspace destinations; the driver builder
intentionally refuses to overwrite an existing directory.

Obtain the official source at the inspected revision (Git commands are user-run):

```bash
git clone https://github.com/ROBOTIS-GIT/coin_d4_driver.git "$HOME/coin_d4_driver_desk_source"
git -C "$HOME/coin_d4_driver_desk_source" checkout --detach 8277b288902f28b37948b42e1bb80ba59afbba60
```

The existing local source is a source-only directory without `.git`; its full
manifest matches `tools/lidar_diagnostics/source.sha256`. The builder checks that
manifest before copying/patching source. A mismatch must be investigated, not bypassed.
No diagnostic or timing patches are needed for this checkpoint.

In a fresh terminal:

```bash
repo="$HOME/git/Project2_Turtle_Ninja"
bash "$repo/tools/lidar_diagnostics/build_experimental.sh" \
  "$HOME/coin_d4_driver_desk_source" "$HOME/ros2_lidar_desk_driver_ws"
source /opt/ros/lyrical/setup.bash
source "$HOME/ros2_lidar_desk_driver_ws/install/setup.bash"
mkdir -p "$HOME/ros2_lidar_desk_ws"
cd "$HOME/ros2_lidar_desk_ws"
colcon build --base-paths "$repo/ros2_ws/src/description" \
  "$repo/ros2_ws/src/bringup" "$repo/ros2_ws/src/lidar_cloud_merge" \
  --packages-select description bringup lidar_cloud_merge \
  --executor sequential --cmake-args -DBUILD_TESTING=ON
colcon test --base-paths "$repo/ros2_ws/src/lidar_cloud_merge" \
  --packages-select lidar_cloud_merge --event-handlers console_direct+
colcon test-result --verbose
source install/setup.bash
ros2 launch bringup desk_lidar.launch.py --show-args
```

The merger must report its `cloud_core` test as passing; zero discovered tests is
not a pass. `--show-args` does not start hardware. `--base-paths` is necessary because
these workspaces do not contain a source checkout. Build directly from the repository:
bringup installs root `config/*.yaml` using a repository-relative CMake path; copying
only the bringup package elsewhere loses those files. Installed files are copies;
rebuild after editing source/config/RViz. Explicit package paths deliberately exclude robot-control packages from discovery.
Scanning all of ros2_ws/src with only --packages-select is insufficient: colcon then
expects discovered robot-control dependencies to have been built as well. The original driver dependency in bringup belongs
to the separate original-driver workflow and is not used by this desk launch.

## Run (user runs hardware commands)

Check for existing driver/TF/merger processes and stop duplicates first. In each
terminal source `/opt/ros/lyrical/setup.bash`, then the new driver and desk workspace
`install/setup.bash` files in that order.

Terminal 1, using the last confirmed desktop socket assignments (recheck after
rewiring or changing hosts):

```bash
ros2 launch bringup desk_lidar.launch.py \
  stationary_test:=true use_rviz:=true \
  rear_x:=0.220 rear_y:=0 rear_z:=0 \
  rear_roll:=0 rear_pitch:=0 rear_yaw:=3.141592653589793 \
  front_port:=/dev/serial/by-path/pci-0000:80:14.0-usb-0:6.4:1.0-port0 \
  rear_port:=/dev/serial/by-path/pci-0000:80:14.0-usb-0:6.1:1.0-port0
```

Terminal 2:

```bash
ros2 run lidar_cloud_merge stationary_cloud_merge --ros-args \
  -p stationary_test:=true -p self_filter_enabled:=true
```

Terminal 3:

```bash
ros2 topic echo /lidar/points_merged --once --no-arr --timeout 10
```

Expect front/rear scans and merged points in `lidar_front_link`; both source colors
should be visible. The geometric body mask uses the accepted 256 mm square plus
10 mm per side. It removes returns at all Z inside the mask, including real objects
there. Raw scans remain available. Stamps are publication times, time_increment=0;
this output is not validated for moving-platform use or navigation clearing.

## Focused Git checkpoint (user runs)

Review and stage exactly these files from the repository root:

```bash
git add -- config/lidar_drivers.yaml \
  ros2_ws/src/bringup/CMakeLists.txt \
  ros2_ws/src/bringup/package.xml \
  ros2_ws/src/bringup/launch/desk_lidar.launch.py \
  ros2_ws/src/bringup/rviz/desk_lidar.rviz \
  docs/navigation/dual_lidar_desk_checkpoint.md
git diff --cached --check
git diff --cached --stat
git diff --cached
git commit -m "feat: add reproducible stationary dual lidar desk bringup"
```

Keep robot.launch.py, lidar.launch.py, lidar_mounts.yaml, lidar.xacro,
robot.urdf.xacro, navigation.rviz, historical setup/alignment/handoff notes, udev
and diagnostic/timing changes for separate review. Preserve `.codex-attachments/`.
The desk launch and installed RViz preset do not depend on those untracked changes.

## Offline review evidence

A snapshot of HEAD plus only the five runtime/build files listed above was used
for the review build, excluding all remaining robot-model and diagnostic changes.
The experimental driver was rebuilt from checksum-verified local upstream source
in a new `/tmp/desk-checkpoint-review/experimental` workspace. The driver, description, bringup and merger built successfully. The merger
reported 1 test, 0 failures, 0 skips. Installed launch argument discovery passed.
A hardware-free declaration check verified both drivers, YAML/port overrides, TF
direction, RViz paths/QoS/source coloring, and 32 invalid-input rejections before
Node construction. No launch actions were executed. The checks confirmed the
installed desk launch works without lidar.launch.py or the pending robot changes.
The first build attempt exposed whole-tree package discovery pulling in missing
robot dependencies; the explicit paths above correct that. No sensor or motor process
was started by these checks.

## User-run publication and RViz smoke test (2026-10-26)

The user supplied a live topic echo and screenshot after the rebuild/run procedure.
The echo shows frame_id=lidar_front_link, height=1, width=622, five fields,
point_step=20, row_step=12440, data length=12440, and is_dense=true.
The byte counts are consistent with 622 points at 20 bytes per point.
The screenshot shows the installed bringup/rviz/desk_lidar.rviz preset, Global
Status OK and merged-display Status OK, both red and blue points, source_id
coloring with bounds 0/1, and both raw scan displays disabled. This confirms
publication and live rendering of the new preset with both sensor sources.

This completes the publication/display smoke test for the desk checkpoint.
It does not measure sustained rate, dropped pairs, fine alignment, or range accuracy,
and does not repeat the previously completed body-mask and driver-restart checks.
Keep overlapping returns and provisional geometry unchanged. The six-file checkpoint
is ready for user review and commit; Git operations remain user-controlled.
