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
- Default forward/correction/turn caps are 0.025 m/s, 0.015 m/s, 0.1 rad/s.
  Generate a matching speed profile with the CLI below to change them.
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
monitoring. The original MPPI source profile, firmware, mounting transforms, and arm
implementation are unchanged. Comparison profiles select either controller.

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

## CLI speed and controller comparison

`configure_trial` is an offline C++ utility. It generates **three matching files**
for navigation, the wheel controller, and the trial supervisor; it starts no nodes.
Choose `custom` for body-forward motion or `original` for the existing omni MPPI
plugin. Use a new output directory for each profile; existing files are not overwritten.

```bash
ros2 run body_route_controller configure_trial \
  --mode custom --speed 0.025 --yaw-rate 0.1 \
  --output /tmp/custom-025
ros2 run body_route_controller configure_trial \
  --mode original --speed 0.025 --yaw-rate 0.1 \
  --output /tmp/original-025
```

`--speed` is m/s; `--yaw-rate` is rad/s. Additional flags:
`--linear-accel` (default 0.05 m/s²), `--angular-accel` (0.2 rad/s²),
`--correction-speed` (custom only, normally 60% of speed).
The accepted software range is 0.005–0.26 m/s and 0.02–1.6 rad/s; these upper
bounds come from the original software profile and **are not physically qualified
speeds**. Acceleration must be positive, at most 1 m/s² and 6 rad/s² respectively.
Defaults remain slow. Increasing a maximum alone does not guarantee faster travel:
acceleration, corner/goal slowing, steering settling and collision limits still apply.

Select a profile in each terminal (example uses custom):

```bash
trial_profile=/tmp/custom-025
```

1. Start the existing wheel-only bringup with its verified hardware map and port
   arguments, replacing its `drive_test_params` argument with:
   `drive_test_params:=$trial_profile/drive_limits.yaml`.
2. Start navigation with that profile and the current measured start pose:

```bash
ros2 launch body_route_controller body_route.launch.py \
  params:=$trial_profile/nav2.yaml map:=/absolute/path/to/scanned/map.yaml \
  use_sim_time:=false initial_pose:=X,Y,YAW_DEGREES rviz:=false
```

3. Preview or execute with matching trial settings:

```bash
ros2 run body_route_controller route_trial --ros-args \
  --params-file "$trial_profile/trial.yaml" \
  -p goals_file:=/absolute/path/to/goals.path

# After the normal physical preflight; a fresh results filename is required.
ros2 run body_route_controller route_trial --ros-args \
  --params-file "$trial_profile/trial.yaml" \
  -p goals_file:=/absolute/path/to/goals.path -p execute:=true \
  -p results_file:=/absolute/path/to/custom-run-01.yaml
```

The client queries the running controller plugin and its speed/acceleration
parameters, and the wheel controller's corresponding limits and heading-hold
setting, **before every release**. A mismatch remains held. Parameter values are
readback checks, not evidence that a controller has reconfigured its cached state:
**stop the trial, shut down navigation and wheel bringup, then restart both with
the selected profile**. Do not switch plugins or use `ros2 param set` while moving.
For the other mode repeat with `/tmp/original-025`. Keep sensors/localization under
one owner, and re-establish the same measured start pose before each run.

The comparison substitutes only `FollowPath` and its speed/acceleration/noise
settings into the shared navigation profile. MPPI retains its original motion
model and critics; sampling noise scales with the requested speed to preserve
its original noise-to-limit ratio. Map resolution, footprint, planner, BT,
collision monitor, progress checker and stopped goal tolerances are shared.
This compares **controller modes under common trial conditions**, not the entire
untouched legacy launch. Both modes disable separate wheel heading hold. MPPI
has per-axis XY limits; the shared wheel controller enforces the requested XY
vector magnitude. Its route supervisor permits simultaneous XY and yaw, while
retaining speed, corridor, sensing, ownership and deadline checks.

### Replay exactly the same path

For controller-only timing, save one planned path while held. The goal file must
have exactly one destination. This does not release hold or send a motion goal:

```bash
ros2 run body_route_controller route_trial --ros-args \
  --params-file "$trial_profile/trial.yaml" \
  -p goals_file:=/absolute/path/to/one-goal.path \
  -p save_path_file:=/absolute/path/to/comparison.path
```

Use **that same saved file** for both modes:

```bash
ros2 run body_route_controller route_trial --ros-args \
  --params-file "$trial_profile/trial.yaml" \
  -p path_file:=/absolute/path/to/comparison.path -p execute:=true \
  -p results_file:=/absolute/path/to/custom-fixed-01.yaml
```

`path_file` directly invokes Nav2 `FollowPath` with the same map-frame
`x y yaw-radians` samples and goal orientation; it performs no replanning or
recovery. It refuses a start more than 3.5 cm from the first sample. Reset the
**heading as well as position** to the same measured start before each trial;
planner sample headings need not represent the robot's original starting yaw.
The normal collision monitor, controller collision checks, hold, scans and
freshness checks stay active. Use `goals_file` instead for whole-maze navigation
with ordinary NavigateToPose replanning. These two measurements answer different
questions and must be labelled separately.

### Timing results

Each invocation creates a new YAML results stream and prints its path. If
`results_file` is omitted it uses a unique `/tmp/route-trial-*.yaml` file; copy
results off a robot before shutdown. Each leg records:

- `status` and failure reason; failed/aborted runs are never marked successful.
- `navigation_s`: steady-clock time from the first released hold heartbeat to
  the action result (or fault). Startup, preflight and initial planning are excluded.
- `completion_s`: time from that same release through the measured stopped,
  oriented arrival check. This includes the one-second arrival confirmation.
- `distance_m`, peak measured speed/yaw rate, start/end/goal poses, planned length,
  and a stable `path_id`. The run header records mode, profile/input identifiers,
  speed, yaw rate and acceleration. Identifiers detect differences; they are not
  cryptographic signatures.

The final `summary` records `route_wall_s`, total navigation time and travel.
Route wall time includes waits and planning **between** goals. A result without a
successful final summary (including a process killed before cleanup) is incomplete
or failed. For the same saved path, compare successful `completion_s` values with
matching `path_id`, map, start pose, limits and robot configuration. Repeat at
least three runs per mode, alternate their order and compare medians plus failure
counts; a mode that aborts is not a faster completion. Do not infer physical gains
from a mock test. Raise one speed/acceleration setting at a time after qualification.

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
- Default measured speed caps 0.04 m/s and 0.15 rad/s. Generated profiles use
  requested speed + max(0.015 m/s, 20%) and yaw rate + max(0.05 rad/s, 20%)
  as fault thresholds, not extra commanded speed. Odometry/scans/map pose freshness
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
physical L2 crossing or complete competition mission. The 0.41 m navigation
envelope, 0.36 m mock truth square and 0.38 m stop region remain provisional;
full stowed-body/arm/cable geometry, localization, turning drift, stopping response
and battery condition need the next physical check. The CLI supports new speed
limits and route-relative supervisory bounds, which require physical qualification.

Changed-wall maps can now be prepared with the offline workflow below; autonomous
map adaptation during motion and physical 5 cm robustness remain unqualified.
The previous NavFn profile could propose routes with infeasible footprint or
rotation clearance. The maze planner below checks that geometry explicitly;
the controller still stops if live safety checks reject a planned motion.
Training remains on hold. See the accompanying November 4 offline validation
record for exact test results and remaining evening checks.

## Offline wall-layout adaptation

`capture_layout` passively records stationary dual-lidar views, and `adapt_layout`
fits bounded wall/assembly changes and robot poses, exports a separate map, and
checks opening paths with the existing polygon collision code. Both controller
modes can use the same accepted snapshot. See [WALL_LAYOUT.md](WALL_LAYOUT.md)
for the input schema, commands, uncertainty gates and qualification limits.
This experimental workflow does not enable live remapping or robot motion.

## Maze routing and simulation diagnostics (November 4)

The comparison profile now uses `body_route_controller::MazePlanner` for both
controller modes. It searches a 5 cm orthogonal lattice aligned with the map,
including the exact start/goal coordinates, and charges 0.5 m of path cost per
90-degree turn. Each straight leg checks the swept polygon. Intermediate turns
require the full circumscribed footprint circle to be clear; narrow openings
can therefore be traversed straight while turns remain outside them. Start and
final heading changes are also checked. Unknown space is blocked. Cancellation,
map bounds, memory and planning-time limits fail explicitly; this planner does
not silently fall back to an unchecked path. It is intended for this arena's
map-aligned corridors, including translated walls, rather than arbitrary curved
routes. `grid_step`, `turn_cost`, `cost_weight`, and `max_seconds` are parameters
under `planner_server.ros__parameters.GridBased`.

The behavior tree retains a valid route and checks it at 1 Hz, replanning when
it becomes invalid or the goal changes. This avoids moving the target tangent
every second. After entering a corner within 2 cm, the core retains that outgoing
leg through small pose shifts, releasing it beyond 6 cm or when the current path
no longer matches. This prevents localization jitter from reversing a turn in
progress; hold/reset clears the commitment. Live local footprint/stopping checks
remain active. The local
costmap now includes the static map. Lidar layers use combination method 2 to
preserve static unknown cells, and inflation treats unknown cells as obstacle
sources (`inflate_around_unknown: true`). This prevents unknown cells near walls
from becoming merely inscribed cost 253, which polygon checks do not treat as
physical occupancy. The installed Nav2 behavior has a regression test. Both navigation
costmaps use the same 0.34 m square plus 0.035 m padding (0.41 m overall), adding
2.5 cm per side to the previous navigation envelope for localization/tracking
error. This is a conservative planning margin, not a measured manipulator
collision envelope or a guarantee against arbitrary localization error.

The trial guard treats `stop_before_turn` as braking, bounded by the measured
entry speed, configured deceleration, feedback delay and existing drift margin.
The 2.5 cm turn-drift bound begins when actual turning starts; the measured-speed
check still forbids turning before translation has stopped. Increasing the CLI
speed also increases straight-leg lookahead so a fixed target distance does not
silently cap the command at 0.06 m/s. Lookahead still ends at sharp corners.
The maze profile starts lateral correction at 1 cm and releases it below 4 mm
(previously 2.5 cm / 1 cm). This preserves clearance through the shifted L2 gap
after local-map rasterization; correction still brakes before changing axis.
Footprint and collision checks are unchanged. Controller and Nav2 arrival
targets are 2 cm / 0.03 rad, leaving margin inside the unchanged trial/truth
acceptance bounds of 3.5 cm / 0.055 rad. This margin is not a guarantee for
unbounded localization error.

For scanned-maze simulation, use `tools/scanned_map_sim` at the repository root.
RViz shows global/local inflation at 25% opacity and the controller/global/mock
footprints and robot model at 40% opacity. Reports retain turn counts, mode durations, the latest
raw costmaps and translucent costmap trajectory plots. Tests and mock trials do
not establish physical qualification; hardware remains operator-controlled.

## Unknown-space policy

See [UNKNOWN_SPACE.md](UNKNOWN_SPACE.md) for `--unknown-policy blocked` (default) and
`--unknown-policy observe-before-enter`, positive planning penalties, shared lidar
execution safeguards for custom/MPPI, complete-truth partial-map simulations, and
exact offline build/run commands. Unknown goals are provisional until observed.
