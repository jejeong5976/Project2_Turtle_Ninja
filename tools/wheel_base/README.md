# Wheel configuration integration — 2026-10-27

This is our existing `hardware` and `swerve_controller` stack with a separate
wheel-only calibration, model and launch. It does not import the colleague's
controller behavior, Python teleop, SLAM launch, speed increase or LiDAR poses.
The ordinary `robot.launch.py`, full-robot/desk configuration and live LiDAR
setup remain separate. Never run both robot launches in the same ROS domain.

## Configuration and coordinates

- `config/hardware_map_drive_only.yaml`: wheel IDs 1/2/41/42, scales +1/-1/-1/+1;
  steering IDs 11/12/13/14, scales -1 and zero ticks 3066/1552/2577/3589.
  Source is the colleague's October 27 archive, supported by October 26 motor
  readbacks and physical tests. These are not today's live readbacks.
- `config/controllers_drive_only.yaml`: radius 0.033 m; calibrated module
  positions and signed offsets; our 0.10 m/s, 0.5 rad/s limits and existing
  steering-selection algorithm. Arm permission and IMU use are disabled.
- `description/urdf/drive_only.urdf.xacro`: uses that same geometry file for TF.
  No arm, IMU mounting transform, body collision footprint or LiDAR frame is
  inferred. Simple wheel visuals are not a navigation footprint.

Calibrated +X is old CAD -Y; +Y is old CAD +X. Retained suffixes `fl/fr/rl/rr`
mean rear-left/front-left/rear-right/front-right in the calibrated frame.
Both `base_footprint` and `base_link` use these axes; their fixed transform is
only +0.033 m Z. Do not use the legacy full-robot model to publish these frames
alongside this launch. The recorded steering zeroes remain unchanged.

The launch derives per-module controller bounds from motor ticks 20..4075.
The URDF exposes the full 0..4095 motor window. These bounds are not an assertion
of cable/mechanical clearance across the entire sweep. ROS command limits stay
enabled. Our controller retains its existing nearest-feasible steering choice;
the colleague's `coherent`/outward-preference/blend behavior is not imported.

## Build and initial feedback (user-executed on Jetson)

Keep the live LiDAR processes in domain 181 running as they are. Use a fresh
terminal and a separate wheel workspace/domain. For the prepared source bundle:

```bash
source /opt/ros/lyrical/setup.bash
mkdir -p "$HOME/ros2_wheel_config_v1_ws"
cd "$HOME/ros2_wheel_config_v1_ws"
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build \
  --base-paths "$HOME/jetson-wheel-config-20261027-v1/ros2_ws/src" \
  --packages-up-to bringup --executor sequential \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
export ROS_DOMAIN_ID=182
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 launch bringup drive_only.launch.py hardware:=mock mode:=feedback
```

After stopping that mock launch with Ctrl+C, the real feedback command is below.
Only the drive OpenCR is connected; its confirmed USB socket is 2.4.3. The user
checks motor power/cabling and executes the command. **This sends CONFIG:** it
pings the eight IDs and writes torque-off, operating modes, profiles and watchdog
settings. It is not the earlier receive-only serial test. No EEPROM current/PWM
limits are changed by this profile, and no firmware upload is needed for this
integration. Hardware stays INACTIVE, the swerve controller is not loaded, and
only the joint-state broadcaster is activated. No motion commands are sent by
the launch.

```bash
ros2 launch bringup drive_only.launch.py hardware:=opencr mode:=feedback \
  port:=/dev/serial/by-path/platform-3610000.usb-usb-0:2.4.3:1.0
```

In another terminal, source the same ROS/workspace and set domain 182/LOCALHOST:

```bash
ros2 control list_hardware_components
ros2 control list_controllers
ros2 topic echo /joint_states sensor_msgs/msg/JointState --once --timeout 10
```

Expected: one `bus_drive`, `hardware/OpenCRHardware`, state **inactive**;
only `joint_state_broadcaster` active; eight wheel/steering names with finite
positions and velocities. No `/odom` publisher is expected in feedback mode.
Unaligned wheels can have nonzero steering positions; do not change zeroes just
to make a stationary reading zero. Ctrl+C stops this launch only.

`mode:=drive` is a separate later motion step: hardware activates/torque enables,
and our swerve controller publishes `/odom` and `odom -> base_footprint`. Review
the feedback and mechanical range first. IMU fusion is still off. Real motion
and odometry accuracy have not been tested with this integration. Our LiDAR
driver remains stationary-only, and plate-to-base pose is unresolved; this
profile does not authorize moving SLAM or connect the current sensor TF tree.

## Offline checks

Six workspace packages built on desktop Lyrical (the `bringup` package also has
a build-order dependency on `manipulation`, but this launch starts no arm nodes).
The existing hardware/controller suite passed 73 tests. Launch/API checks cover
mock/opencr XML, 13 invalid configurations before node construction, and unchanged
full-robot torque defaults. C++ checks verify recorded IDs/signs/zeroes, model
and controller geometry, tick limits and 33 kinematic motion cases. Local mock
runtime checks verify inactive feedback, active drive, effective steering bounds,
+X odometry and timeout stop. None establishes physical accuracy on Jetson.

Reproduce after building these packages into a separate validation root:

```bash
source /opt/ros/lyrical/setup.bash
source /tmp/wheel-base-integration.uUoKAL/install/setup.bash
python3 tools/wheel_base/check_launch.py /tmp/wheel-base-integration.uUoKAL/validation
g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic -Werror \
  -Iros2_ws/src/hardware/include -Ifirmware/nt_opencr \
  -Iros2_ws/src/swerve_controller/include \
  tools/wheel_base/validate_profile.cpp ros2_ws/src/swerve_controller/src/swerve.cpp \
  -lyaml-cpp -ltinyxml2 -o /tmp/wheel-base-integration.uUoKAL/validate_profile
/tmp/wheel-base-integration.uUoKAL/validate_profile config \
  /tmp/wheel-base-integration.uUoKAL/validation/drive_only_opencr.urdf \
  /tmp/wheel-base-integration.uUoKAL/validation/resolved_controller.yaml
bash tools/wheel_base/check_mock.sh /tmp/wheel-base-integration.uUoKAL
```

The Python check is restricted to Python launch/Xacro API testing. Geometry and
motor checks are C++. The mock script uses domain 218, a private ROS home under
the validation root, and no physical ports. ROS sockets require permission if
the execution environment blocks local networking.
