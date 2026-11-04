# Body-route controller

Nav2 controller for **body +X facing the route**, L2 crossing, multi-turn maze travel,
and oriented arrival at a shelf. Development workspace:
`/home/pa4/git/Project2_Turtle_Ninja-body-route`.

## Motion contract

- Stop translation using measured feedback before a large turn; rotate, settle,
  then move. Final goal yaw overrides the route tangent.
- Only one translation axis at a time. Forward is preferred; side/backward
  corrections have distance and time bounds. Small yaw corrections may accompany
  translation.
- Forward/correction/turn caps remain 0.025 m/s, 0.015 m/s, 0.1 rad/s.
- Occupied and unknown cells, the entire rotation, and the measured stopping
  envelope are checked against the padded footprint. Blocked motion aborts.
- Fresh odometry, pose, costmaps, and a continuing released mission-hold heartbeat
  are required. Startup is held. The BT does not invoke spin/backup recovery.

The recorded L2 planner path contains closely spaced points with noisy tangents.
The controller now simplifies those points within a **0.01 m deviation bound**
before selecting a tangent. Endpoints and larger corners are retained. This
prevents small grid variations from becoming repeated stop/turn commands.
The live swept-footprint check still rejects an unsafe shortcut. This tolerance
is a geometric path-processing bound, not a physical localization accuracy claim.
Goal alignment also requires reaching the end of the remaining route, so a goal
near the beginning of a loop does not immediately stop the controller.

## Main and manipulator integration

`mission_manager -> NavigateToPose -> planner/controller -> /cmd_vel_nav ->
collision_monitor -> /cmd_vel_safe -> cmd_vel_hold -> /cmd_vel -> swerve_controller`

The plugin runs inside `controller_server`; it adds no competing velocity
publisher. Existing mission `Executor` goals and `/mission/hold` remain the
integration interface. During a mission, **mission_manager is the sole hold
owner**; manipulator readiness continues to gate navigation through that existing
interface. The optional `route_trial` below is a wheel-only test client and must
not run alongside mission_manager, teleop, or another trial client.

AMCL owns map->odom; measured swerve odometry owns odom->base_footprint. Front
filtered scans localize; both filtered scans feed obstacle costmaps and collision
monitoring. The original MPPI profile, firmware, mounting transforms, and arm
implementation are unchanged.

Use `config/drive_limits.yaml` with the existing wheel-only bringup. It disables
the low-level controller's separate heading-hold gain because this plugin owns
route heading feedback. Steering alignment stays enabled. The inspected main
bringup accepts this override only with `arm:=false`; combining it with the arm
bringup needs team integration review, not an unverified launch flag change.

## Build and test offline

The user performs every Git operation. Build the main-based workspace directly,
using only the ROS underlay (no team-sim overlay):

```bash
source /opt/ros/lyrical/setup.bash
cd /home/pa4/git/Project2_Turtle_Ninja-body-route
colcon --log-base /tmp/body-route-maze-build-log build \
  --base-paths ros2_ws/src --packages-up-to body_route_controller \
  --build-base /tmp/body-route-maze-build \
  --install-base /tmp/body-route-maze-install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON
source /tmp/body-route-maze-install/setup.bash
colcon --log-base /tmp/body-route-maze-test-log test \
  --base-paths ros2_ws/src --packages-select body_route_controller \
  --build-base /tmp/body-route-maze-build \
  --install-base /tmp/body-route-maze-install --ctest-args --output-on-failure
colcon test-result --test-result-base /tmp/body-route-maze-build --verbose
```

Tests include the retained 553-point L2 path, delayed measured feedback,
multi-turn/return routes, near-start loop goals, full rotation collisions,
mission hold and stale odometry, and route-relative trial checks.
`test/data/l2_recorded.path` is the November 4 planning output from the retained
physical trial; that trial did not physically follow the whole path.
`replay_map` is an additional offline executable in the test build. It accepts
`P5_MAP RESOLUTION ORIGIN_X ORIGIN_Y PATH INITIAL_YAW OUTPUT_CSV`; the P5 reader
expects the recorded no-comment, 8-bit trinary format and thresholds .65/.25.
No ROS graph or hardware is started by that replay.

## Navigation-only launch

With the matching underlying wheel and sensor stack already available:

```bash
ros2 launch body_route_controller body_route.launch.py \
  map:=/absolute/path/to/scanned/map.yaml use_sim_time:=false rviz:=true
```

This launches no hardware, sends no goal, and does not release hold. Supply the
current independently measured initial pose in RViz. Historical coordinates
must not be reused after teammates move the robot. Preserve the recorded slow
hardware map, steering limits, filtered scans, and current USB topology.

## Optional route trial client

Write an explicit goal file: one **map-frame x y yaw-radians** row per destination;
blank lines and `#` comments are accepted. Goals are visited in order. There is
no default L2 goal or implicit arena-to-map conversion.

```text
# Example file format only: replace with surveyed map goals.
0.50 0.70 1.57079632679
0.90 0.70 0.0
```

Preview while the stack is held:

```bash
ros2 run body_route_controller route_trial --ros-args \
  -p goals_file:=/absolute/path/to/goals.path
```

Preview is the default. It plans all legs, sends **no NavigateToPose goal**, and
publishes **no hold heartbeat**. A plan is not a swept-footprint or physical
clearance approval. Do not preview concurrently with another active navigator:
both use the planner's `/plan` topic.

For an operator-supervised physical trial, after checking pose, clear space,
stowed arm and immediate stop access, explicitly select execution:

```bash
ros2 run body_route_controller route_trial --ros-args \
  -p goals_file:=/absolute/path/to/goals.path -p execute:=true \
  -p max_goal_duration:=900.0
```

The client refuses a competing `/mission/hold` publisher, owns hold only for
that trial, requires two seconds of stationary fresh feedback before every leg,
plans while held, then invokes the normal NavigateToPose BT with replanning.
It restores hold and cancels its action on failure/interruption, and requires
measured stopped arrival before the next destination. SIGINT/SIGTERM request
that cleanup. A hard process kill is handled by the existing hold heartbeat
and drive-command timeouts.

The former 25 cm length and fixed initial-heading limits are replaced by:

- 0.10 m distance to the current planned route; 0.25 rad target-heading error
  while translating above 0.008 m/s.
- During turn phases, 0.025 m odometry drift and 0.008 m/s translation bounds.
  The controller's tighter measured-stop gate remains 0.003 m/s.
- Measured speed caps 0.04 m/s and 0.15 rad/s; odometry/scans/map pose freshness
  0.35 s; controller-status freshness 0.4 s after startup.
- Both scans require 50 valid returns and fewer than two combined returns inside
  the existing 0.38 m square stop region. Sensor transforms come from TF.
- A finite per-goal deadline (default 900 s) and total travel budget twice the
  initial planned length plus 0.25 m. Failure requires explicit investigation
  and a new invocation; it never silently retries or releases itself.

The diagnostic viewer must publish fresh `/live_view/fresh` and retain its
latched `/live_view/stop`; this is required by default. Only an isolated simulation
fixture without that UI should use `require_live_view:=false`. That setting does
not disable any sensor, motion, collision, ownership, or deadline checks.
`/trial/status` shows the actual goal index, route length and deadline.
Record it with `/live_view/diagnostics`, controller status, all command stages,
`/swerve_controller/commands`, odometry, scans, AMCL, TF, plans and costmaps.

## Qualification boundary

Offline passes establish software behavior under the stated fixtures, not a
physical L2 crossing or complete competition mission. The 0.36 m padded square
and 0.38 m stop region remain provisional; full stowed-body/arm/cable geometry,
localization, turning drift, stopping response and battery condition need the
next physical check. Speed and collision limits remain unchanged; the new
route-relative supervisory bounds still require physical qualification.

Changed-wall static-map clearing and 5 cm wall-displacement robustness are not
implemented here. NavFn can also propose a path whose footprint/rotation is
infeasible; the controller stops rather than inventing an evasive maneuver.
Training remains on hold. See the accompanying November 4 offline validation
record for exact test results and remaining evening checks.
