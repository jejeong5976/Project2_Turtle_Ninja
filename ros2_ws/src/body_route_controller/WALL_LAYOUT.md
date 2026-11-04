# Offline wall-layout adaptation

This experimental workflow builds a **current-run map snapshot** from stationary
dual-lidar keyframes. It estimates robot poses and wall placement together from
a fixed start, exports a new occupancy map, and checks requested opening paths
with the controller's polygon collision code. Both controller modes can consume
the same snapshot and path. It does not replace AMCL during motion, broadcast
TF, issue commands, change the authoritative map, or start drivers.

The walls must remain stationary throughout the survey and subsequent trial.
This is the first offline implementation, not qualified autonomous remapping or
a demonstrated physical L2 crossing. A new physical survey and registered wall
annotations are required before evaluating it on the robot.

## Inputs

Copy `config/layout_template.example.yaml` to a new survey template and supply:

- `map`: the existing trinary P5 occupancy-map YAML (relative to the template or
  absolute). Grid resolution must be 0.005–0.05 m; rotated origins are rejected.
- `fixed_start`: the first keyframe's known **map-frame** `[x, y, yaw_radians]`.
  A repeatable floor start is not by itself a surveyed registration to the old
  map. Preserve the uncertainty of that registration; do not copy an unverified
  AMCL estimate into this field.
- `start_uncertainty`: explicit positive `position_sigma_m` and `yaw_sigma_rad`,
  including placement and map-registration error. The CLI has no hidden default
  for these values. Excessive uncertainty causes a rejection instead of a map
  rewrite. The synthetic test values are not physical calibration results.
- `walls`: nominal straight wall-face segments, each with a unique `id`, endpoints
  `a` and `b`, and an optional `assembly` ID. Segments with the same assembly ID
  share one rigid transform about their common centroid. Assign IDs only to
  physically rigid assemblies, not neighboring but independently movable walls.
- `portals`: optional pairs of wall endpoints defining openings to check, plus
  crossing heading and entry/exit distances. Endpoint names are `a` or `b`.
  A map alone can be exported when this list is empty.

Wall annotations must use the same frame as the map and start. There is no
automatic raster-to-wall labeling in this version. Retained posts/props that are
not included in supported wall-face bands remain obstacles. The synthetic test
fixture is not a registration or annotation of the physical L2 opening.

## Capture stationary observations

`capture_layout` is a passive subscriber. It accepts an already-running sensor
and odometry graph, including a recorded bag with TF. It has no publishers,
action clients, motion commands or driver launch. The operator or a separately
supervised system controls any movement between views.

```bash
ros2 run body_route_controller capture_layout --ros-args \
  -p layout_file:=/absolute/path/layout_template.yaml \
  -p output_directory:=/absolute/path/NEW_survey_directory \
  -p fixed_start_verified:=true \
  -p max_keyframes:=8
```

The confirmation parameter asserts that the **first** recorded view is at the
template's fixed start. It is false by default. For bag replay, use
`-p use_sim_time:=true` and replay a suitable capture from that start with clock.

The recorder requires `/odom` in `odom -> base_footprint`, both
`/lidar_front/scan` and `/lidar_rear/scan`, and the corresponding lidar TFs. It
requires at least one second with measured speed ≤0.003 m/s and yaw rate ≤0.01
rad/s, stable odometry, fresh scans, scan pairing within 0.05 s, and odometry near
the scan time. Subsequent keyframes require 5 cm translation or 0.08 rad rotation
in odometry. Different views, preferably exposing corners or wall ends, matter
more than many scans of one unchanged view.

Each accepted capture writes a new cumulative `survey_N.yaml`. Earlier files
are preserved. Ctrl+C leaves the last snapshot available. The output directory
must be new. Capture exits after the requested 2–32 keyframes. Passive capture
does not verify physical placement or diagnose a common sensor calibration error.

## Adapt and check

Use the same generated comparison profile that will be used for the trial:

```bash
ros2 run body_route_controller adapt_layout \
  --input /absolute/path/survey_directory/survey_8.yaml \
  --params /absolute/path/comparison_profile/nav2.yaml \
  --output /absolute/path/NEW_layout_directory
```

This command starts no ROS node and needs no robot connection. Outputs:

- `report.yaml`: acceptance reason, pose/wall uncertainty and observability,
  association statistics, endpoint verification, source fingerprints, wall
  displacements, changed-cell counts, survey duration and processing time.
- `map.yaml` / `map.pgm`: a separate current-layout snapshot, only when the
  estimator accepts the survey. The original map is never overwritten.
- `portal_N.path`: dense map-frame `x y yaw_radians` samples only for a crossing
  whose opening, approach, translations and complete turns pass checks.

Exit code 0 means the estimator and all requested paths passed; 3 means an
estimator or crossing rejection; 2 means invalid input or an I/O error. A rejected
crossing does not invalidate an otherwise accepted map, so inspect each status.
No output path is written for a rejected crossing. An existing output directory
is rejected, and the tool never overwrites previous results.

## What the estimator checks

The C++ core extracts contiguous line segments from each laser separately and
preserves each scanner's actual ray origin. Missing/maximum-range returns are
unknown, not evidence of free space or of a wall end. A wall end requires an
adjacent valid farther return, matching nominal geometry, and observations from
independent views before it can authorize an opening.

The batch solver estimates SE(2) robot poses and wall/assembly transforms from
line/endpoint observations, the fixed-start prior and odometry. Wall transforms
are constrained to at most 5 cm displacement at every annotated endpoint and
0.10 rad rotation. A robust residual check can reject a constrained fit; a bound
is not permission to discard conflicting obstacles. Small out-of-envelope
physical changes cannot be distinguished perfectly from measurement noise.

Observability is computed from geometry plus the start anchor, **excluding
odometry and nominal-wall priors**. Priors therefore cannot make a disconnected
new wall group or a parallel-only corridor appear geometrically verified.
Covariance is a model-based diagnostic, not surveyed accuracy. It does not
capture every calibration, correspondence or systematic sensor error.

Map clearing requires an accepted pose solution, a verified wall face and free
rays from separated estimated views. Only occupied cells near the annotated old
wall face can be cleared. If wall ends are not constrained, their possible
movement envelope remains occupied; clearing is limited to the wall interior.
New hits override free evidence. Unrelated occupied cells, occlusions and
unsupported no-return rays are never used to clear old geometry. Normal Nav2
inflation settings stay unchanged and are applied when the new map is loaded.

Crossing checks use the profile's padded polygon and an additional model-based
pose uncertainty allowance. They require repeated verified opening edges and
check the complete approach, translation and turn sweeps against occupied and
unknown cells. A blocked straight approach returns a rejection; this tool does
not silently generate an unchecked shortcut. Use the global planner to select a
different route. All requested portal previews begin at the last survey pose;
they are independent alternatives, not a sequential multi-opening mission.

## Integration and comparison

After offline review, use the accepted `map.yaml` with the existing
`body_route.launch.py map:=...` interface. `report.yaml` includes
`initial_pose_launch` in that launch's `x,y,yaw_degrees` format and also the
radian pose. This pose applies only if the robot is still at the final survey
position. It must be re-established if the robot moves afterwards.

`portal_N.path` uses the existing `route_trial path_file:=...` format; its default
is preview. Normal `NavigateToPose` planning can use the adapted map for routes
through the wider maze. The existing trial supervisor and collision checks
remain required. No new launch changes TF ownership or releases hold.

Compare original/custom controllers with the **same map snapshot**, path or
goals, limits and starting conditions. Keep total survey/verification time
separate from the existing driving/completion timers, then report both. This
version preserves the CLI speed cap; it does not add automatic speed scheduling
or continuous map changes during a trial.

## Verification

`test_wall_layout` exercises synthetic raw dual-lidar scans with independent
odometry error, range noise, coherent inner/outer wall translation, 5 cm endpoint
displacements, small rotations, rigid assemblies, disconnected geometry,
parallel-corridor ambiguity, conflicting sensors, missing returns, map-clearing
restrictions, unknown space and narrow openings. `test_layout_cli` checks the
actual executable and files. `test_layout_capture` checks the passive recorder
against a local synthetic ROS graph. These are not calibrated physical tests.

The test-only `layout_fixture NEW_DIRECTORY` executable generates the repeatable
synthetic survey, nominal and truth maps used for CLI/actual-stack checks. It is
built with `BUILD_TESTING=ON` and is not installed as a robot operation command.
