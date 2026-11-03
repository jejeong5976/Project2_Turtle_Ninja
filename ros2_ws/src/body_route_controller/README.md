# Body-route controller (ROS 2 Lyrical)

Experimental Nav2 controller: **body +X faces the upcoming path tangent**, independently of camera mounting. It stops before large turns, permits only one translation axis at a time, allows bounded backward/side corrections, and permits small yaw corrections while translating. Final goal yaw provides the shelf-facing heading.

This package is a standalone overlay because the evidence checkout does not contain the newer `navigation` package. Build it over the main-based/team navigation workspace. It does not replace that package or change the original MPPI/physical-run configurations.

## Command and feedback chain

`mission_manager -> bt_navigator -> planner_server / controller_server -> /cmd_vel_nav -> collision_monitor -> /cmd_vel_safe -> cmd_vel_hold -> /cmd_vel -> swerve_controller`

The plugin is loaded inside `controller_server`. Its status is published at `/controller_server/body_route_status`. There is no extra publisher competing for `/cmd_vel`. The launch starts no hardware driver and neither releases hold nor sends a goal. The supplied behavior tree aborts on planner/controller failure instead of executing automatic spin/backup recovery commands outside the motion contract.

AMCL uses the scanned map and front scan; both filtered scans feed obstacle costmaps and collision monitoring. Navigation commands use the full padded footprint. This package does **not** implement changed-wall map clearing, simulator/controller parity, or a calibrated physical response model. Stale static occupancy may still prevent a feasible route. Those remain separate qualification tasks.

## Profile and behavior

The Nav2 profile is derived from the retained November 3 executed profile, with a new controller, stopped goal checker, rotation-aware progress checker, explicit scan freshness checks, corrected 0.26 m inflation and a recovery-free behavior tree. It keeps the 0.36 m padded square and 0.38 m stop zone; those are provisional envelopes, not surveyed dimensions.

Initial engineering settings: 0.025 m/s forward, 0.015 m/s corrections, 0.1 rad/s in-place rotation, alignment entry/exit 0.12/0.035 rad, final XY/yaw tolerance 0.03 m/0.05 rad. Small heading corrections are capped by both 0.02 rad/s and 0.4 times translation speed. These are test settings, not physically validated accuracy limits.

The existing swerve controller still owns steering alignment, branch choices, wheel limits and command timeout. Use `config/drive_limits.yaml` with the main-based wheel-only bringup: it disables that controller's separate heading-hold gain for this candidate, because route heading feedback is computed here. Do not disable steering alignment. Preserve the original heading-hold profile for physical-reference replay. The supplied bringup override is supported only with `arm:=false` by the inspected main launch.

The plugin requires fresh `/odom`, a fresh pose, current costmaps, and a continuing `/mission/hold` heartbeat set to false. Its default is held. A goal alone cannot release it. Heading changes and translation-axis changes wait for measured translation to stop. Steering alignment remains enforced by the lower controller rather than by an arbitrary delay.

Footprint checking includes occupied/unknown cells inside the polygon, intermediate rotation poses, command lookahead and a conservative measured-motion stopping envelope. Leaving the costmap is rejected. This is sampled geometric checking with explicit speed/deceleration assumptions, not a physical safety certification. If blocked it reports a controller failure; it does not search arbitrary evasive maneuvers.

## Build and offline test

The repository owner handles all Git commands. No branch switch is required to build the new package as an overlay.

```bash
source /opt/ros/lyrical/setup.bash
source /home/pa4/git/Project2_Turtle_Ninja-team-sim/ros2_ws/install/setup.bash
cd /home/pa4/git/Project2_Turtle_Ninja
colcon --log-base /tmp/body-route-colcon-log build \
  --base-paths ros2_ws/src/body_route_controller \
  --build-base /tmp/body-route-colcon-build \
  --install-base /tmp/body-route-colcon-install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON
source /tmp/body-route-colcon-install/setup.bash
colcon --log-base /tmp/body-route-test-log test \
  --base-paths ros2_ws/src/body_route_controller \
  --build-base /tmp/body-route-colcon-build \
  --install-base /tmp/body-route-colcon-install
colcon test-result --test-result-base /tmp/body-route-colcon-build --verbose
```

Verified locally on November 4: the overlay builds against installed ROS 2 Lyrical/Nav2 and the team workspace; **23 test cases pass** (21 controller/collision/plugin cases, one ROS adapter case, one launch-integration case). CTest reports three passing test targets; `colcon test-result` counts those wrappers too and reports 26. The ROS adapter test uses localhost domain 231, synthetic odometry and an empty costmap, with no hardware drivers or velocity publisher. Launch integration constructs actions and resolves parameters without executing nodes. These checks do not establish a complete Nav2 mission, Isaac fidelity, physical tracking accuracy or 5 cm wall-offset performance. Retained results: [validation record](/home/pa4/git/Project2_Turtle_Ninja/output/body_route_controller_20261104_v1/VALIDATION.md).

## User-run map test

First test in an isolated simulation ROS domain. Confirm simulation provides measured-state odometry/TF and matching filtered scans; the earlier bridge's truth odometry is only an ideal diagnostic mode. Physical-map testing remains unqualified until these comparisons are reviewed.

For a later physical run, use the existing wheel/LiDAR bringup used for the recorded crossing, select the candidate drive override above, and ensure no other Nav2/teleop publisher is running. In that wheel-only `bringup/robot.launch.py` invocation, set `arm:=false` and `drive_test_params:=<absolute path to this package's config/drive_limits.yaml>`; preserve the recorded slow hardware-map profile. Do not run the historical combined held launch alongside this navigation launch, because it already starts Nav2. Bringup commands depend on the currently connected robot and are not started by this package. Verify the full footprint can rotate at the chosen starting pose.

With the underlying sensors/controller already running, the navigation-only launch is:

```bash
# Source ROS, the main-based navigation workspace, and this overlay first.
ros2 launch body_route_controller body_route.launch.py \
  map:=/home/pa4/git/Project2_Turtle_Ninja/output/A1_H2_occupancy_1cm_2p5cm_5cm_20261102_v2/maps_1cm/map.yaml \
  use_sim_time:=false
```

Set the measured initial pose in RViz. Retain the existing mission/operator hold mechanism; a continuous false heartbeat is required only when the operator intends motion. Do not bypass collision monitoring or publish directly to `/cmd_vel` to make the test move.

Check resolved parameters before releasing hold: `FollowPath.plugin` must be `body_route_controller::BodyRouteController`, swerve `heading_hold_kp` must be zero for this candidate, speed/acceleration limits must match, and steering alignment must remain enabled. Verify front/rear scans, `/odom`, TF and the footprint in RViz.

Use one short, clearly free segment first; then a clear in-place turn, a side correction, a backward correction and an oriented final goal. Record status, commands at every stage of the chain, scans, odometry, joints/IMU, AMCL, TF, plans and costmaps. Stop the trial on unexpected heading, branch flips, stale feedback or repeated blocked messages. Test changed walls only after nominal behavior is understood. A nominal success does not establish 5 cm robustness or whole-mission completion.
