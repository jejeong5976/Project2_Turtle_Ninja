# Scanned-map controller simulation

This runs the actual main-based Nav2 stack and either controller on the retained
A1–H2 **scanned occupancy map**. Synthetic variants translate selected scanned
wall pixels before each run. It does not connect to the Jetson or start physical
drivers. The launch fixes `hardware:=mock`, `mock_dynamics:=true`, `arm:=false`;
DDS traffic uses only `127.0.0.1`. The user performs every Git operation.

## Commands

On the laptop:

```bash
sim_tools=/home/pa4/git/Project2_Turtle_Ninja-body-route/tools/scanned_map_sim
bash "$sim_tools/build.sh"

# Original scanned geometry, recorded L2 destination, conservative speed.
bash "$sim_tools/run.sh" --route l2 --shift 0 --rviz

# Same fixed start and destination, movable inner and outer wall assemblies.
bash "$sim_tools/run.sh" --route l2 --shift 0.05 --seed 11 --walls all --speed 0.08 --yaw-rate 0.30 --rviz

# Localization/planning stress: lidar sees shifted walls, navigation retains old walls.
bash "$sim_tools/run.sh" --route l2 --shift 0.05 --seed 11 --map-mode stale --rviz

# Six editable maze-navigation goals, including return to the same start.
bash "$sim_tools/run.sh" --route maze --shift 0.02 --seed 11 --speed 0.08 --yaw-rate 0.30 --rviz

# Compare controller modes at identical limits. Run sequentially and retain failures.
bash "$sim_tools/run.sh" --mode original --route l2 --shift 0 --speed 0.08 --yaw-rate 0.30 --rviz
bash "$sim_tools/run.sh" --mode custom --route l2 --shift 0 --speed 0.08 --yaw-rate 0.30 --rviz

# Generate/review a layout without starting a ROS graph.
bash "$sim_tools/run.sh" --route maze --shift 0.05 --seed 29 --prepare-only
```

Omit `--rviz` for headless operation. Each invocation starts a fresh robot state
and saves a new output directory. The trial begins automatically after startup;
Ctrl+C stops the simulation. Do not send an additional RViz navigation goal
during the scripted trial. A lock prevents two instances of this launcher.

Defaults are custom, 0.025 m/s, 0.10 rad/s, unchanged geometry, seed 11,
all wall groups, matching map, L2 route, no RViz. `--speed` and `--yaw-rate`
configure matching Nav2, wheel and trial profiles using `configure_trial`.
The per-goal deadline is 600 seconds; `--goal-timeout` accepts 30–3600 seconds.
A separate overall deadline is 90 seconds plus the per-goal deadline times the
number of goals. Speed caps are simulation inputs, not physically qualified speeds.

## What moves

`maze.yaml` identifies nine groups by map-frame rectangular selection strips.
Only occupied source pixels inside those masks move. The connected southwest
zigzag, west L, east zigzag and north L each receive a single rigid translation;
the perimeter is one coherent outer-wall group. Other wall strips are separate
groups. Shelf/prop returns outside the masks remain fixed. These masks are
simulation annotations drawn from the scanned raster, not physical wall surveys.

`--shift` is a maximum **translation norm**, not a per-axis allowance. It accepts
0–0.05 m, quantized to the source map's 1 cm cells. Each enabled group receives a
deterministic nonzero offset inside that disk when the bound is positive. `--seed`
selects another layout. `--walls inner` keeps the perimeter fixed; `all` moves it
too. The robot start and goals stay fixed. Rotations and motion during a run are
not synthesized by this version.

All selected source occupied cells are removed before translated hits are added,
so new occupied cells win over cleared cells. No unknown cell is cleared. Pixels
outside the source/destination wall masks are checked unchanged. The source map
is never modified. Unknown scan shadows remain conservative even when walls move;
the generator does not invent newly observed free space behind them.

## Navigation map versus simulation truth

- `matched` (default): AMCL and the planner receive the synthesized current map.
  This is an **oracle-map controller test**. It does not run the wall-layout
  estimator and is not evidence that localization/map adaptation solved a shift.
- `stale`: AMCL and the planner receive the unshifted map while synthetic lidar
  sees the displaced truth map. This directly exposes stale-map localization and
  planning effects, with existing live obstacle layers still active.

Both modes use the real AMCL estimate during navigation. The fixed initial pose
is supplied explicitly. Debug truth trajectories never broadcast map TF or feed
perfect current pose to the controller. All truth motion is derived from the mock
wheel odometry transformed by the fixed initial pose; there is no independent
slip/contact physics model. Ray casts use separate front/rear scanner transforms
matching main bringup, with 0.5-cell range stepping and no added range noise.
Unknown cells terminate a ray with an invalid return, not clearing evidence.

**Unknown encoding:** the scanned PGM uses 0 occupied, 254 free and 205 unknown.
Its source YAML has `free_thresh: 0.25`, which Nav2's actual map loader classifies
205 as free. Simulation copies use `free_thresh: 0.196`, and every generated truth
cell is verified through Nav2's loader. This intentional simulation metadata
correction preserves the documented unknown cells; it does not patch the source
map or the wall-layout export utility. Earlier results using the old metadata are
not identical map-semantics comparisons.

## Routes and output

The start `[-1.59000020138919, 0.459999826550483, 1.573]` and L2 crossing destination
`[-0.646512819, 2.60013038, 0]` come from the retained scanned-map trial. Coordinates
are map metres and yaw radians, not surveyed arena coordinates or the robot's
current physical pose. The six-goal `maze` route consists of simulation waypoints,
not registered shelf/manipulator targets. Edit `maze.yaml` or pass
`--route /absolute/path/to/goals.path` with one `x y yaw_radians` row per goal.
Use `--config /absolute/path/to/another-maze.yaml` for an explicit alternate setup.

The planner receives goals through the existing `route_trial` and `NavigateToPose`
interfaces; it replans on the full scanned map. This does not replay a hand-drawn
path. Every start/goal footprint is checked against truth before launch. The
reported full-turn-circle connectivity is a conservative geometry diagnostic;
a false value is not by itself proof an oriented robot cannot traverse a passage.
The runtime controller still checks its motion and complete turn sweeps.

Outputs are under:

`/home/pa4/git/Project2_Turtle_Ninja/output/scanned_map_controller_sim_20261104_v1/runs/`

- `maps/nominal.yaml`, `truth.yaml`, `navigation.yaml` and their PGMs.
- `maps/scenario.yaml`: wall offsets, source, map mode, registration, changed-cell
  counts, unknown-cell verification, goals and geometry diagnostics.
- `maps/layouts.png`: original and displaced scanned maps.
- `result.yaml`: per-leg Nav2/trial outcomes and timing.
- `runtime.csv`: simulated truth, estimated pose, velocity stages and hold state.
- `plans.csv`: actual Nav2 planned paths, including replans.
- `arrivals.csv`: simulated truth pose at each reported stopped arrival.
- `trajectory.png`: Nav2 plans, mock truth trajectory and AMCL trajectory.
- `evaluation.yaml`: status, localization error, swept collision and arrival checks.

The feed requests a trial stop on truth overlap with occupied/unknown space.
Evaluation also interpolates the trace at half-cell spacing with rotational motion
included. A pass requires a successful route, no modeled footprint collisions,
and a truth-position/yaw witness within 3.5 cm / 0.055 rad at every stopped arrival.
Missing evidence is incomplete, not success. The footprint is the current
provisional padded 0.36 m square, not a qualified loaded arm/body envelope.

Compare successful completion times with matching route, map mode, seed, limits
and startup conditions; failed durations are not completion scores. Wall layout
is seeded, while AMCL retains its random sampling, so repeat trials before
claiming a timing improvement. Planning can
produce different paths in different layouts. The prior original MPPI progress
failure remains unresolved. Neither this mock model nor map synthesis establishes
physical L2 or whole-mission readiness.

## Historical baseline observations (before navigation fixes)

The simulation implementation compiles with warnings treated as errors. The
unchanged-map custom-controller L2 run at 0.025 m/s and 0.10 rad/s did **not**
complete: the simulation truth-footprint monitor stopped it near the crossing
after about 206 seconds. The trial records this simulated stop input as
`operator_stop`; the launch log identifies the occupied/unknown overlap that
triggered it. Inspection of the first stop found an unknown cell near the wall tip,
not a confirmed occupied-wall contact. At 0.08 m/s and 0.30 rad/s, a separate run stopped earlier with
`translation_during_turn`. Neither failure is a successful completion time.
The harness preserves both traces and does not relax controller guards.

With a 5 cm bound, seed 11, matching map and the same conservative speed limits,
the L2 trial stopped after about 57 seconds when Nav2 could no longer create a
path to the goal. The retained trace had no modeled footprint overlap. Neither
scanned-maze test completed L2. The six-goal route has been checked for valid
start/goal footprints across nominal, 2 cm and 5 cm layouts, but has not been
completed in a runtime trial. RViz configuration is provided; these validations
ran headless.

## Updated navigation and displays

Inflation is visible in RViz as `Global inflation` and `Local inflation`, both at
25% opacity. `Controller footprint`, `Global footprint`, and `Simulated truth
footprint` and `RobotModel` use 40% opacity. Toggle them in Displays or adjust Alpha there. The
controller footprint is the padded 0.41 m navigation envelope; mock truth checks
continue to use the provisional 0.36 m model. Saved trajectory images also blend
the latest global costmap at 25% and mock start/end footprints at 40%.

The common comparison planner now creates footprint-checked orthogonal routes
with a turn penalty. It keeps valid routes instead of replacing them every
second. Corner-selection hysteresis prevents small localization shifts from
switching back to the incoming leg halfway through a turn. Static unknown space
is included in the local safety map, and the trial
guard distinguishes bounded braking travel from drift during an actual turn.
The higher-speed profile now scales lookahead to reach its requested speed on
straight legs while retaining corner stopping. See the controller README for
planner assumptions and bounds.

Additional files: `global-costmap-raw.pgm`, `local-costmap-raw.pgm` and corresponding
YAML metadata retain actual Nav2 costs (0 free, 1–252 inflated, 253 inscribed,
254 occupied, 255 unknown); these PGMs are diagnostics, not map-server map images.
`evaluation.yaml` includes turn episodes, cumulative absolute rotation and time
spent in each controller mode. A turn interrupted by braking can count as two
episodes even at one geometric corner. Arrival messages use a queue of ten so a
back-to-back final route status does not overwrite the stopped-arrival witness.

An intermediate 5 cm, seed 11 L2 trial passed in approximately 67 seconds.
Subsequent unknown-space handling exposed a local-map clearance stop in a later
run: 1.8 cm lateral drift plus grid rasterization exhausted the navigation margin.
The final maze profile corrects lateral drift above 1 cm, releasing below 4 mm.
All intermediate successes and failures remain retained; final results below
identify the exact configuration checked.

The longer-route regression also exposed unknown cells being converted to cost
253 during inflation. Both costmaps now preserve unknown cells during lidar
merging and inflate around them as obstacle sources. The installed Nav2 inflation
library is tested directly for this case. See the upstream
[inflation implementation](https://github.com/ros-navigation/navigation2/blob/main/nav2_costmap_2d/plugins/inflation_layer.cpp)
for the distinction between unknown and inscribed costs.

Discrete turn/mode metrics are available for the custom controller; MPPI has no
such mode status, so its turn-episode count is reported as unavailable. Measured
rotation and route timing remain available for both controllers.

A small number of runs stopped before motion when a ROS lifecycle service reply
was lost during startup (`failed to send response ... client will not receive
response`). Their traces are retained as startup failures, not navigation scores.
This resembles the upstream [Fast DDS service discovery race](https://github.com/ros2/rmw_fastrtps/issues/842).
If this occurs, let the launch shut down fully before starting a new run; the
harness does not automatically retry a trial or bypass readiness checks.

## Final checked runs (November 4)

These runs use the final corner-selection hysteresis, 1 cm / 4 mm lateral
tracking thresholds, 2 cm / 0.03 rad arrival targets, unchanged truth acceptance
limits, and the custom controller.
Both use seed 11, `matched` maps, 0.08 m/s and 0.30 rad/s limits.

| Route | Wall translation bound | Route time | Stopped arrivals | Unsafe swept samples |
| --- | --- | ---: | ---: | ---: |
| L2 | 5 cm | 126.11 s | 1/1 | 0 |
| Six maze goals and return | 2 cm | 799.57 s | 6/6 | 0 |

Run directories: `20261104-174457-custom-mP7YcG` and `20261104-173009-custom-XsMVTE` under the output location above.
Route time starts at the first motion release and includes stopped-arrival waits
and inter-leg work; it excludes initial ROS startup. The six-goal route travelled
19.92 m. Its largest truth arrival errors were
2.36 cm and
0.0373 rad.

Before the arrival-target refinement, the corner-hysteresis comparison reduced
observed turn-phase time from 25.40 s to 9.75 s and absolute rotation from 2.96 rad
to 1.66 rad. Those paired runs are `20261104-164501-custom-WcZQbH` and
`20261104-170843-custom-nQe77L`; they used the same map and limits. The final L2
run recorded four turn episodes at one geometric corner, including braking
interruptions, and 9.15 s in turn mode. Lateral correction and steering waits
remain substantial: less turning does not imply a proportional total-time gain.
These are individual observations, not averaged performance estimates. The
earlier six-goal pass before corner hysteresis is `20261104-164726-custom-Hc8Cl7`
(845.64 s); its arrival targets were less strict than the final profile.

The final build passed 74 tests across the five selected CTest targets for the
controller, adapter, launch, wall layout and layout CLI. Geometry-only route
checks also passed for all six legs at 2 cm and 5 cm shifts. This does not qualify
all layouts, the original MPPI controller on this maze, stale-map localization,
the wall-adaptation estimator, or the physical robot.

## Partial navigation maps and observed execution

`run.sh` accepts `--unknown-policy blocked|observe-before-enter`, `--unknown-penalty`,
`--observation-ttl`, and complete-world scenarios `--scenario corridor|concealed|partial|turn`.
For scan-derived L2/maze tests, `--map-mode partial` completes source unknown as
synthetic solid truth and masks a separate navigation map. The original scan is unchanged.

The fast `unknown_suite NEW_OUTPUT_DIRECTORY [SPEED_MPS]` helper tests the production
custom core/planner/evidence model with ideal odometry and delayed commands; full
`run.sh` trials exercise actual custom or MPPI plugins with AMCL and mock wheels.
See [UNKNOWN_SPACE.md](../../ros2_ws/src/body_route_controller/UNKNOWN_SPACE.md) for
the safety contract, exact commands, display legend, evidence and limitations.

## Portable shared scenarios

`build.sh` now resolves the repository relative to this script. Build products go
under `<repo>/.scanned-map-sim`; results go under
`<repo>/output/scanned_map_controller_sim/runs`. This replaces the older hardcoded
laptop build/output paths above. ROS defaults to `/opt/ros/lyrical/setup.bash`.
Override `SCANNED_SIM_ROS_SETUP`, `SCANNED_SIM_BUILD_ROOT`, `SCANNED_SIM_INSTALL`,
`SCANNED_SIM_HELPERS`, or `SCANNED_SIM_RESULTS` if required. The supplied API and
source snapshot target ROS 2 Lyrical; other distributions are not validated.

```bash
bash tools/scanned_map_sim/build.sh
bash tools/scanned_map_sim/run.sh --maps /path/to/shared/case --route l2 \
  --mode custom --speed 0.08 --yaw-rate 0.30 --rviz
bash tools/scanned_map_sim/run.sh --maps /path/to/shared/partial-case --route maze \
  --unknown-policy observe-before-enter --speed 0.08 --yaw-rate 0.30 --rviz
```

`--maps` imports `nominal.yaml`, `navigation.yaml`, `truth.yaml`, their images and
`scenario.yaml` into a new run without changing the shared files. Import checks
map registration, complete-world claims and start/goal collision footprints.
Use `--route scenario` to retain the stored goals, `l2`/`maze` for included routes,
or a goals file. It rejects simultaneous map-generation options (`--shift`,
`--seed`, `--walls`, `--map-mode`, `--config`, `--scenario`) to avoid silently
ignoring them. `--prepare-only` validates/imports maps and creates profiles without
starting the ROS graph. The runner still fixes hardware to mock and DDS to loopback.

An optional `complete_world: true` in the generation YAML fills remaining
unscanned world cells as occupied **after** displacing walls, for all map modes.
Partial mode already defaults to complete-world generation. Existing matched and
stale generation defaults remain unchanged. This completion is a synthetic
conservative assumption, not recovered geometry. Matched navigation receives the
completed current world; stale navigation retains the nominal scan including
unknown cells; partial navigation hides a corridor band and goal neighbourhoods
in the completed current world. Hidden truth is used only by sensors and independent
evaluation; matched mode explicitly grants the planner the world as an oracle.

The share bundle contains checksums, both routes, per-wall translations and a
source snapshot, so teammates do not depend on uncommitted local code or the
original laptop's output directory. Its README distinguishes map/import validation
from completed controller trials. A generated layout is not a promise of success.


## Startup readiness and failures

The simulation starts lifecycle managers with autostart disabled, lets their
service request/reply discovery settle, activates localization, waits for the
`map -> base_footprint` transform, then activates navigation. A C++ startup barrier
checks every managed node is active before it launches `route_trial`. The normal
navigation launch retains `autostart=true` by default. Existing runtime preflight,
hold, speed and collision guards remain in effect.

`startup.yaml` records success or the failed phase. Missing services, lost lifecycle
responses and missing localization TF terminate startup within a bounded deadline
(60 seconds for the barrier). No trial goal is sent on startup failure. A failed
run is retained; the launcher does not silently retry or discard it. Lifecycle
bonds remain managed by Nav2. This addresses the previously observed startup stall
at a lost `/map_server/change_state` response, followed by missing map TF. It does
not change map pixels, occupancy policy or planner feasibility.

Rebuild after updating these scripts: the new `startup` helper and installed
launch files must come from the same source revision. RViz can remain open during
startup; the robot stays held until readiness and trial preflight succeed.
