# Dual LiDAR robot-model checkpoint

Reviewed 2026-10-26, after desk checkpoint de87c0b. This checkpoint provides the
model/configuration interface; it does not establish final robot extrinsics or
moving-platform readiness. Use dual_lidar_desk_checkpoint.md for the working desk
experiment. Earlier dual_lidar_setup.md and alignment notes describe historical
intermediate defaults and investigations.

## Behavior and geometry

`robot.launch.py` adds with_lidars (default false) and lidar_mounts. It still launches
robot control, so it is not a model-only test command. With the flag false, the model
has no LiDAR frames. The old assumed base_scan/scan_joint is removed, including its
borrowed housing visual, collision, and inertia. This is a deliberate model change;
no replacement housing or mass properties are asserted. Repository source/config
search found no remaining base_scan consumers. External consumers must migrate.

With the flag true, the model adds fixed joints:

```text
base_link -> lidar_plate_link -> lidar_front_link
                             -> lidar_rear_link
```

The plate origin is the assembly center on the sensor mounting surface. XY and yaw
are relative to base_link; sensor XY and yaw are relative to that plate. The model
assumes a level plate and level sensors. Plate Z relative to base_link is
mount_surface_above_ground - wheel_r, because the current model puts base_link at
wheel_r above base_footprint. With the provisional inputs, scan height is 54.5 mm
above base_footprint and 21.5 mm above base_link. These are derived model values,
not measured physical scan heights. Revisit this formula if teammates change the
base origin or ground-height convention.

config/lidar_mounts.yaml preserves the accepted provisional +/-110 mm X, +5 mm Y,
28 mm mounting surface and 26.5 mm scan offset. plate_x, plate_y, plate_yaw,
front_yaw and rear_yaw remain null. The desk's front yaw pi and rear-relative yaw pi
do not determine plate placement relative to base_link. Do not populate nulls from
synthetic test fixtures. Final teammate geometry and scan-origin validation remain
required before using the robot-frame configuration.

Every mounting value must be a finite YAML number, not a boolean, quoted string,
null, NaN or infinity; h_scan must also be nonnegative. The review fixed scalar
Xacro argument coercion that previously accepted quoted numeric strings.

## Driver and TF ownership

`lidar.launch.py` defaults to experimental_stationary and requires
stationary_test:=true. The older original driver remains an explicit comparison
option, not the normal default. Neither choice establishes moving-platform timing.
The default publish_model=true expands the dual model and fails on missing geometry
before creating node actions. publish_model=false skips geometry expansion for raw
sensor testing or when another robot_state_publisher already owns the model.

For a future assembled robot, choose exactly one model owner:

- Standalone lidar.launch.py with publish_model=true: starts robot_state_publisher,
  selected drivers and optional RViz; no control nodes. Movable robot joints have
  no joint-state source supplied by this launch.
- Existing full robot launch with with_lidars=true: start lidar.launch.py with
  publish_model=false using the same scan frame names. The full robot launch owns TF.

Stop desk_lidar.launch.py before either model arrangement. Its front -> rear static
TF conflicts with the robot model's plate -> rear joint. The desk launch remains
independent and unchanged by this checkpoint. navigation.rviz now displays separate
front and rear raw scans; the dedicated desk preset remains separate.

The body mask in the merger still uses provisional geometry in lidar_front_link.
Recompute it if sensor/plate geometry changes. No per-sensor filtered scan topics,
Nav2 observation sources, mapping configuration, or motion compensation are added.

## Hardware-free verification

Build in a new workspace without touching the working desktop installations:

```bash
source /opt/ros/lyrical/setup.bash
source ~/ros2_lidar_experimental_ws/install/setup.bash
repo="$HOME/git/Project2_Turtle_Ninja"
mkdir -p ~/ros2_lidar_model_review_ws
cd ~/ros2_lidar_model_review_ws
colcon build --base-paths "$repo/ros2_ws/src/description" "$repo/ros2_ws/src/bringup" \
  --packages-select description bringup --executor sequential
source install/setup.bash
python3 "$repo/tools/lidar_bringup/check_model_launch.py"
```

The check uses Python only to exercise the Xacro/ROS launch Python APIs. It creates
synthetic YAML in a temporary directory and instantiates declarations without
executing any nodes. Do not run it with Python -O, which disables assertions.

Review results: description and bringup built in /tmp/lidar-model-review. Default
and synthetic dual models expanded; frame names are unique, expected fixed poses
match, both ros2_control systems remain, and non-LiDAR XML is unchanged between
with_lidars=false/true. All 52 invalid-geometry cases reject, including quoted
numeric strings. Raw/model launch declarations and stationary/missing-geometry
fail-before-node guards pass. No hardware or controller process was started.
These tests validate model plumbing, not physical calibration or live robot TF.

## Focused checkpoint (user Git operations)

```bash
git add -- config/lidar_mounts.yaml \
  ros2_ws/src/description/urdf/lidar.xacro \
  ros2_ws/src/description/urdf/robot.urdf.xacro \
  ros2_ws/src/description/rviz/navigation.rviz \
  ros2_ws/src/bringup/launch/robot.launch.py \
  ros2_ws/src/bringup/launch/lidar.launch.py \
  tools/lidar_bringup/check_model_launch.py \
  docs/navigation/dual_lidar_robot_model_review.md
git diff --cached --check
git diff --cached
git commit -m "feat: add guarded dual lidar robot model integration"
```

This preserves the guarded integration interface while physical geometry remains
pending. Keep diagnostic/timing tooling, udev templates, historical notes and
attachments out of this checkpoint. Do not start robot control to validate these
configuration-only changes.
