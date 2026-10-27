# Robot SLAM readiness audit

Audited 2026-10-26 (Asia/Seoul). The software has measured planar odometry and the
necessary local ROS packages, but the assembled robot is **not yet validated for
moving SLAM**. The next milestone is a first map using one physical scan origin,
verified odometry and explicit robot mounting transforms. Dual-sensor refinement
and autonomous navigation follow that milestone.

This audit preserves the desk configuration and all mounting nulls. No physical
devices, robot bringup, or Git commands were run. The handoff's branch/commit/index
record is historical and was not reverified; the user owns Git inspection too.

Latest October 27 update: the colleague's supplied
`lidar_scan_jetson_20261027.tar.gz` recovers the physical drive-only ID/sign/zero
configuration and real motor/odometry test records. See the
[current wheel-configuration checkpoint](dual_lidar_handoff.md#latest-checkpoint--our-calibrated-wheel-only-profile-builds-on-jetson).
This supersedes older statements below that only assumed IDs and mock evidence
were available. Independent odometry accuracy is only partially supported, and
the colleague's changed forward convention must be reconciled with our base
frames. The user explicitly limited adoption to wheel configuration: preserve
our LiDAR setup and do not replace it with the bundle's integration stack.
The user then authorized a separate wheel-only profile in our stack. It has
coupled controller/model geometry, recovered calibration, inactive feedback mode,
explicit drive activation and IMU use disabled. Desktop build, configuration,
kinematic and mock runtime checks pass. The user also verified the source-bundle
checksum and built all six required packages on Jetson (Release, tests disabled);
only CMake compatibility/policy warnings were reported. Existing default profiles
and live setup remain unchanged. Jetson mock/real feedback and physical odometry
validation with this profile remain pending.

Later 2026-10-26 update: the user built the stationary driver natively on Jetson
and verified both raw scans near 10 Hz with the expected frames and 400-bin metadata.
See the [Jetson checkpoint](dual_lidar_handoff.md#jetson-stationary-raw-scan-checkpoint)
for source provenance, ports and environment. This advances driver deployment;
robot mounting geometry, return quality, odometry and moving SLAM remain open.
Subsequent [bag inspection](jetson_lidar_capture_review.md) confirmed 274 scans per
sensor with consistent metadata and monotonic stamps. Many close returns remain
unclassified because the robot was near objects/walls. Read-only SSH also confirmed
the four mapping/localization packages named below are absent from Jetson's
`/opt/ros/lyrical` prefix; their installed versions in this audit refer to the desktop.

## Readiness matrix

October 27 follow-up: the user reports prior odometry testing on Jetson and gives
`/home/pa4/git/Project2_Turtle_Ninja/ros2_ws`. SSH found that path absent from Jetson
(it exists on the desktop). An older Jetson deployment at
`/home/turtle/orin_mock_20261018.03inTQ/deployment_source` has a saved successful
`mock_orin_02/result.json`, explicitly scoped to GenericSystem ROS wiring with no
physical motor commands. Its test checks joint/control behavior, not an odometry
trajectory. The user is unsure whether the recalled test was mock or physical;
therefore physical odometry remains unverified. This does not prove that a separate
test never occurred. The next step is drive-controller identification, then verified
real-feedback configuration. The user has since connected the drive OpenCR only
at USB path `2.4.3` (`ttyACM0`); the manipulator is disconnected. The user-run
receive-only capture then observed 502 advancing STATE and 5 DIAG packets in five
seconds, no malformed/checksum failures, zero configured joints and an IMU-init
flag. This confirms project-format telemetry, not real motor feedback, actual
per-motor torque state or calibrated IMU behavior. Actual wheel/steering IDs are
pending; CONFIG changes motor settings, so assumed IDs are not a discovery method.
No live robot controllers have been started. The recorded
stationary body-filter check now passes for all
404 fully captured pairs (405 total clouds, one recording-boundary cloud); see
the [live-output result](jetson_lidar_body_view.md#recorded-live-output-check).
Raw scans remain unfiltered, plate-to-base geometry remains open, and moving SLAM
has not been accepted by this result.

October 27 update: the user accepted the FL0 drawing as assumed assembly geometry.
Front/rear yaws are now pi/zero in `config/lidar_mounts.yaml`; plate X/Y/yaw remain
null. The [body-box review](jetson_lidar_body_view.md) validates a stationary
cloud-filter experiment against the saved bag. The historical audit below still
applies to odometry, TF ownership, motion timing and mapping prerequisites.

| Capability | Repository implementation | Local installation or evidence | Remaining gate |
| --- | --- | --- | --- |
| Wheel odometry | Measured steering positions/rates and wheel velocities; planar x/y/yaw | Core and controller tests; see verification below | Wheel signs, steering zeroes, radius, slip and measured trajectory accuracy |
| Odometry TF | Swerve controller owns `odom → base_footprint` | Configured at 50 Hz | Live timestamp continuity and sole ownership |
| Robot sensor TF | Guarded Xacro; robot_state_publisher owns fixed frames | Existing offline model checks in handoff | Plate-to-base pose and sensor yaws still null |
| IMU | OpenCR → broadcaster → gyro Z substitution in odometry | Configured, not EKF fusion | Board axes, sign, freshness and bias on assembled robot |
| LiDAR | Two separate LaserScan streams; stationary experimental conversion | Desk milestone plus user-verified Jetson build and raw scans near 10 Hz | Moving-platform input contract and assembled-robot occlusion |
| Merged cloud | Stationary endpoints plus source IDs | Visualization/body exclusion implemented | Not a single-origin free-space sensor |
| Mapping | No robot mapping launch or parameters | `slam_toolbox` 2.10.0 installed | Mapper decision, scan path, TF and replay validation |
| Localization | No robot configuration | `nav2_amcl` and `robot_localization` 3.10.0 installed | Saved map, motion model, estimator/TF ownership |
| Navigation | `navigation/.gitkeep`; no package manifest | `nav2_bringup` 1.5.0 and servers installed | Footprint, holonomic control, dual observations, lifecycle and fault response |
| Camera perception | Intrinsic YAML files; perception directory placeholder | Calibration records exist | No visual odometry integration; unnecessary for first 2D map |

Installation refers to `/opt/ros/lyrical/share` on this desktop, not the Jetson.
The seven source packages are bringup, description, hardware, interfaces,
lidar_cloud_merge, manipulation and swerve_controller. Navigation, perception and
mission directories contain placeholders. Package presence does not establish a
running or configured stack.

## Command and odometry path

Evidence: [controller](../../ros2_ws/src/swerve_controller/src/swerve_controller.cpp),
[kinematics](../../ros2_ws/src/swerve_controller/src/swerve.cpp),
[gyro logic](../../ros2_ws/src/swerve_controller/include/swerve_controller/swerve.hpp),
[controller configuration](../../config/controllers.yaml).

1. `/cmd_vel` is `geometry_msgs/msg/TwistStamped`. The controller consumes body
   `linear.x`, `linear.y` in m/s and `angular.z` in rad/s. It ignores the header
   frame and timestamp; send body-frame commands and monitor upstream freshness.
   Arrival time drives its 0.3 s command timeout.
2. At the configured 100 Hz controller-manager loop, inverse kinematics produces
   steering position in rad and wheel angular velocity in rad/s. Limits are
   0.10 m/s, 0.5 rad/s and accelerations 0.2 m/s², 0.8 rad/s². Steering alignment,
   offset rolling compensation and motion-mode permission affect actual motion.
3. Fresh `/control/motion_mode` DRIVE permission is required (0.5 s timeout).
   `control_master` starts locked and requires successful HOME plus measured arm
   folding confirmation before DRIVE. The normal launch starts both hardware
   systems and arm controllers; it is not a wheel-only or model-only launcher.
4. OpenCR reports measured motor states. Firmware converts position ticks using
   2π/4096 and velocity ticks using 0.229×2π/60. Host conversion applies each
   joint's scale and position zero. See [motor bus](../../firmware/nt_opencr/motor_bus.cpp),
   [link conversion](../../ros2_ws/src/hardware/include/hardware/opencr_link.hpp)
   and [hardware map](../../config/hardware_map.yaml).
5. Forward kinematics solves for vx, vy and wz using module geometry and steering
   rate compensation. This is measured-state odometry in OpenCR mode; mock
   hardware mirrors commands and cannot prove physical accuracy. A ready gyro
   replaces wz only; vx/vy remain the wheel solution. Integration uses the control
   period, rotates body velocity by the current yaw, and accepts `0 < dt < 0.5 s`.
6. `/odom` is `nav_msgs/msg/Odometry`, with `header.frame_id=odom` and
   `child_frame_id=base_footprint`; pose is in odom, twist in the body frame.
   A 50 Hz wall timer publishes the latest control-loop-stamped sample and TF.
   Covariance diagonals are fixed provisional values: x/y 0.001, yaw 0.01,
   z/roll/pitch 1e6 for both pose and twist. They are not calibrated or propagated.

`SwerveDrive::reset()` clears commands/output, **not pose**. Reactivation preserves
pose and skips integration on its first update. Reconfiguration recreates the
drive object and starts pose from zero. Restart/reconfigure during mapping needs
a coordinated restart or explicit continuity policy.

### Findings that need attention

- **Calibration blocks physical odometry acceptance.** Module pivots are currently
  ±0.112 m X, ±0.064 m Y, signed wheel offset ±0.0269 m and radius 0.033 m. The
  hardware map still has assumed drive IDs 1–4, steer IDs 5–8 and default scale/zero
  for these joints. Its comment says steering needs scale −1 and calibrated zeroes,
  but the referenced `tools/bench_drive_2026-10-16.yaml` is absent. Recover the
  actual tested mapping from the user/team; do not invent it or copy arm IDs from
  unrelated experiments. Per-module physical steering limits also remain unset.
  An expanded October 27 filesystem search, including ignored local configurations
  and Jetson source/installed/deployment-archive copies, found only this candidate
  table: FL 1/5, FR 2/6, RL 3/7, RR 4/8 (wheel/steer). Git history was not inspected.
  README and hardware_map.yaml also reverse the W210/W350 drive/steer model
  assignments; a matching repeated default is not independent hardware evidence.
- **Freshness is bounded, not acquisition synchronized.**
  [OpenCRLink](../../ros2_ws/src/hardware/src/opencr_link.cpp) timestamps host receipt;
  board milliseconds are used by the optional position-derived steering-rate
  estimator, not mapped to ROS odometry time. Joint failures or STATE age above
  0.05 s become NaN; invalid wheel/steer positions suppress odometry and request
  stop. STATE age above 0.5 s produces hardware ERROR; motor failures have a
  0.2 s threshold. Valid cached measurements can be integrated repeatedly within
  the freshness window. Default `reported` mode substitutes zero for a nonfinite
  steering rate; optional `position` mode rejects it.
- **IMU frame and freshness are assumptions.** `imu_link` has an unrotated,
  provisional origin `(0.066, 0, 0.100)` relative to base_link. The controller
  uses `angular_velocity.z` directly and does not transform from `header.frame_id`
  or use message timestamps/covariance. A two-second stationary bias estimate
  precedes gyro use; loss of valid callbacks for 0.2 s falls back to wheel yaw.
  In [publish_states](../../ros2_ws/src/hardware/src/opencr_hardware.cpp), IMU
  validity uses `ST_IMU_OK`, without the joint age gate. Firmware sets that flag
  from initialization success. Repeated cached or frozen IMU values therefore
  need a dedicated freshness test; callback receipt alone cannot detect them.
  First-map wheel-only odometry is an explicit alternative if it passes trajectory
  tests; do not require an EKF just because it is installed.
- **Timer publication is not proof of advancing odometry.** `publish_odom()` can
  repeat the same timestamp when updates stall; it has no age/sequence guard.
  Invalid-state handling clears readiness only if its try-lock succeeds. Assess
  header progression and TF age, not only topic Hz. Existing `control_master`
  rejects repeated odometry stamps, but every future consumer needs its own
  freshness behavior.
- **Clock jumps are not explicitly handled.** Commands/gyro use ROS callback time,
  hardware age uses steady time, publishing uses wall timers. Gyro readiness does
  not reject negative age. Live robot nodes default to system time; robot launch
  has no common `use_sim_time` argument. Do not mix live control with bag clocks.

Follow-up C++ fixes should add packet/sample-aware IMU validity, odometry
publication freshness and clock-jump tests, preserving the existing interfaces.
These need targeted behavior tests before changes to the control path; this audit
does not claim those faults have been reproduced on the physical robot.

## TF ownership

```text
map                        mapper while mapping; localizer later
 └─ odom                   map → odom has no owner in current robot bringup
     └─ base_footprint      swerve_controller, publish_tf=true
         └─ base_link       robot_state_publisher, fixed +0.033 m Z
             ├─ imu_link   robot_state_publisher, mounting assumption above
             └─ lidar_plate_link
                 ├─ lidar_front_link
                 └─ lidar_rear_link
```

The lower three LiDAR transforms exist only when explicitly enabled with valid
[mounting values](../../config/lidar_mounts.yaml). `plate_x`, `plate_y`,
`plate_yaw`, `front_yaw`, `rear_yaw` remain null. The accepted desk relative TF,
220 mm sensor separation and provisional 54.5 mm scan height do not determine the
plate pose relative to base_link. Teammate geometry may change base height too.

Run one robot_state_publisher. With `robot.launch.py` owning the model, a later
LiDAR launch must use `publish_model:=false`. Stop the desk static TF before using
robot TF: the desk gives rear a different parent (front instead of plate).
Do not run `description display.launch.py` alongside the live robot: it starts
another model publisher and GUI joint-state source. If an EKF later owns
`odom → base_footprint`, disable the swerve TF publisher and define separate raw
and fused odometry topics. Mapper and AMCL must not simultaneously own map→odom.

## Sensor interfaces and mapper proposal

The [driver configuration](../../config/lidar_drivers.yaml) and
[LiDAR launch](../../ros2_ws/src/bringup/launch/lidar.launch.py) preserve separate
`/lidar_front/scan` and `/lidar_rear/scan` (`sensor_msgs/msg/LaserScan`) and their
physical frame IDs. The experimental path requires `stationary_test:=true` and
uses publication stamps with `time_increment=0`. The original driver is an
explicit comparison path, not a verified moving-platform replacement. Do not
disable the stationary guard to obtain a mapping demo.

The cloud merger is outside the first-map data path. Its body filter does not
filter the raw scans. Inspect assembled chassis/arm returns and measured visible
sectors before selecting a sensor. Any future scan filter should keep a separate
output topic and preserve unknown/occluded beams rather than inventing clear rays.

**Proposed baseline, pending mapper choice:** asynchronous SLAM Toolbox with a
single front scan if its assembled coverage is adequate (otherwise select the
rear). Its documented interface takes LaserScan and odom-to-base TF and produces
an occupancy map; it does not directly accept this merged PointCloud2. The local
2.10.0 config has one `scan_topic`. Native dual-stream behavior is not validated
here; do not multiplex the scans or convert the cloud into a virtual LaserScan as
an assumed solution. See the [upstream API](https://github.com/SteveMacenski/slam_toolbox#api).

Once prerequisites pass, add a separate robot mapping YAML/launch without changing
desk defaults. Start from the installed mapper YAML and explicitly set:

| Parameter | Proposed value |
| --- | --- |
| `mode` | `mapping` |
| `odom_frame`, `base_frame`, `map_frame` | `odom`, `base_footprint`, `map` |
| `scan_topic` | `/lidar_front/scan` or its separately validated filtered output |
| `use_sim_time` | `false` live; `true` for isolated replay only |
| `resolution` | 0.05 m initial evaluation |
| `min_laser_range`, `max_laser_range` | 0.05 m, 12.0 m within driver limits |
| `minimum_travel_distance`, `minimum_travel_heading` | 0.10 m, 0.10 rad initial evaluation |

Keep the installed solver settings initially; validate scan queue, transforms,
map update period and motion thresholds in replay before tuning. The installed
online launch defaults `use_sim_time=true`, so live launch must override it.
This table is a proposal, not an installed or tested mapper configuration.

For a first engineering map, a separately designed stop-and-observe scan admission
path could admit scans only after motion has settled. That avoids demanding fine
per-beam timing research now, but requires a verified settling interval and fresh
odometry; simply moving while `stationary_test=true` is not equivalent. Continuous
mapping needs a moving-driver contract and motion-distortion evaluation. At 10 Hz,
0.10 m/s corresponds to 1 cm and 0.5 rad/s to 0.05 rad per revolution, illustrating
why slow motion alone does not establish valid acquisition timing.

## Ordered user validation

All commands in this section are user-run. Source the actual rebuilt robot and
driver overlays first. No supplied command starts motors, homes the arm or changes
calibration; use the team's verified hardware procedure for those actions.

1. Inspect your checkout with `git status --short --branch` and
   `git diff --cached --stat`. Resolve the actual calibration/port map before
   robot bringup. Keep the desk environment as its own unchanged preset.
2. Get teammate values for plate pose, sensor scan-zero yaws, current base height,
   steering zeroes/signs/limits, rolling radius and IMU orientation. Record which
   measurements remain provisional. Do not reconfirm the accepted desk dimensions.
3. After user-controlled robot startup, inspect ownership and interfaces:

   ```bash
   ros2 control list_controllers
   ros2 control list_hardware_interfaces
   ros2 topic info /cmd_vel --verbose
   ros2 topic info /odom --verbose
   ros2 topic echo /odom --once
   ros2 topic echo /control/motion_mode --once
   ros2 topic info /imu_sensor_broadcaster/imu --verbose
   ros2 param get /swerve_controller use_sim_time
   ros2 param get /robot_state_publisher use_sim_time
   ros2 run tf2_ros tf2_echo odom base_footprint
   ```

   Stop the TF observer with Ctrl-C. Expect one odometry source, advancing stamps,
   correct frame IDs, finite values and system time. Topic rate near the configured
   50 Hz is useful but insufficient. HOME/DRIVE gating must remain in place.
4. Perform controlled forward, lateral and yaw segments using the verified motion
   procedure. Compare measured distance/angle against odometry in both directions;
   record errors and repeatability, stationary drift, steering-only drift and yaw
   sign. Establish quantitative tolerances before declaring odometry accepted.
   Test stop, stale-command and feedback-loss behavior under the team's procedure.
5. With real robot mounting values and a validated scan mode, inspect both frames:

   ```bash
   ros2 topic info /lidar_front/scan --verbose
   ros2 topic echo /lidar_front/scan --once
   ros2 run tf2_ros tf2_echo base_footprint lidar_front_link
   ros2 run tf2_ros tf2_echo base_footprint lidar_rear_link
   ```

   Run observers separately. Verify compatible QoS, expected sensor origin and
   direction, finite angular metadata and sensible invalid-return values. Confirm
   transforms are available at scan times, not only at the latest time.
6. Capture the acceptance sequence, then stop recording with Ctrl-C:

   ```bash
   ros2 bag record /odom /tf /tf_static /joint_states \
     /imu_sensor_broadcaster/imu /lidar_front/scan /lidar_rear/scan \
     /cmd_vel /control/motion_mode /control/motion_mode_ack
   ```

   Verify topic counts and retained `/tf_static` in `ros2 bag info BAG_DIRECTORY`.
   Check timestamp progression/gaps and scan-time TF lookup in an offline reader
   or replay test; this audit supplies no automatic acceptance analyzer.
7. After selecting the mapper and creating the proposed YAML, test in an isolated
   domain with no live hardware/control nodes. In both replay terminals:

   ```bash
   export ROS_DOMAIN_ID=181
   export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
   ```

   Mapper terminal, using the future validated YAML:

   ```bash
   ros2 launch slam_toolbox online_async_launch.py \
     use_sim_time:=true slam_params_file:=/ABSOLUTE/PATH/robot_mapping.yaml
   ```

   Replay terminal:

   ```bash
   ros2 bag play /ABSOLUTE/PATH/acceptance_bag --clock
   ```

   Recorded TF supplies the model and odometry; do not add a second model/odom
   publisher. Restore the live domain afterward. First-map acceptance requires
   an expanding `/map`, sole `map → odom` ownership, stable walls on revisits,
   reasonable loop closure, and no persistent TF/message-filter errors. A static
   desk image is not this milestone. Save the accepted occupancy map with
   `ros2 run nav2_map_server map_saver_cli -f /ABSOLUTE/PATH/first_map`; also serialize
   the pose graph if continued SLAM is required.

## From first map to navigation

Use separate front/rear observation sources so clearing retains each origin.
Nav2's [obstacle layer](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/costmap_2d/costmap_plugins/obstacle/)
documents independent source topics, sensor frames, marking and clearing. Verify
the exact installed parameter types before writing YAML: rolling documentation
and installed examples can differ. Source IDs in a merged cloud do not configure
multiple ray origins automatically.

Installed Nav2 defaults specify `nav2_amcl::DifferentialMotionModel`, an MPPI
`diff_drive` model and a single scan source. For lateral swerve navigation, choose
and test a compatible omnidirectional localization/controller configuration,
including steering-transition delay, vx/vy/wz limits and acceleration limits.
Confirm `TwistStamped` end to end, including smoother and collision monitor.
Retain control_master's DRIVE arbitration and verify Nav2 handles a denied permit.

Before autonomous operation, configure the measured collision footprint including
sensor housings and the travel arm pose; the desk body mask is not that footprint.
Validate saved-map localization, planning, goal tracking and independent LiDAR-loss,
odom-loss and TF-loss behavior. LiDAR loss currently does not withdraw motion
permission. Existing motor/command watchdogs are not a sensor-coverage stopping
policy. Jetson OS/ROS installation, performance, clocks and USB identities remain
separate deployment checks.

## Verification and next implementation boundary

An isolated build of hardware, interfaces and swerve_controller completed under
`/tmp/slam-readiness-audit`, without rebuilding desk installations. Existing
hardware and swerve tests passed: 69 GoogleTest cases across four executables;
`colcon test-result` reports 73 tests including CTest wrapper accounting, with zero
errors, failures or skips. The tests exercise
synthetic joint inputs and pseudo-terminal OpenCR packets, not physical odometry
or firmware on a board. Controller tests emitted sandbox `getifaddrs: Operation
not permitted` warnings but passed through in-process communication; live DDS
discovery and network transport remain unverified.

Reproduction from the repository root, with all outputs outside the desk setup:

```bash
source /opt/ros/lyrical/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon --log-base /tmp/slam-readiness-audit/log build \
  --base-paths ros2_ws/src/interfaces ros2_ws/src/hardware ros2_ws/src/swerve_controller \
  --executor sequential --build-base /tmp/slam-readiness-audit/build \
  --install-base /tmp/slam-readiness-audit/install
source /tmp/slam-readiness-audit/install/setup.bash
ROS_LOG_DIR=/tmp/slam-readiness-audit/ros-log ROS_DOMAIN_ID=181 \
ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
colcon --log-base /tmp/slam-readiness-audit/test-log test \
  --base-paths ros2_ws/src/interfaces ros2_ws/src/hardware ros2_ws/src/swerve_controller \
  --executor sequential --build-base /tmp/slam-readiness-audit/build \
  --install-base /tmp/slam-readiness-audit/install \
  --packages-select hardware swerve_controller
colcon test-result --test-result-base /tmp/slam-readiness-audit/build --verbose
```

No runtime configuration was changed. The smallest next implementation is a
separate mapping entry point after the mapper/input-mode decision, backed by an
acceptance bag. Calibration and mount values must come from the robot team.
Duplicate suppression, fine timing research, dual-scan fusion and a new EKF are
not prerequisites for completing this software audit.
