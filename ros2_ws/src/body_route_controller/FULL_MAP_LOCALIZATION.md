# Fresh full-map startup and front-lidar AMCL

The physical A/B/C trial path uses fresh stationary **both-lidar scans against the full existing occupancy map**, followed by reviewed initialization of AMCL. AMCL uses `/lidar_front/scan` throughout stationary verification and driving. There is no localization sensor transition.

| Consumer | Effective input |
| --- | --- |
| Startup registration | `/lidar_front/scan` and `/lidar_rear/scan`, transformed using the existing sensor TFs |
| AMCL, stationary | `/lidar_front/scan` |
| AMCL, driving | `/lidar_front/scan` |
| Obstacles, collision monitor, observation guards | Both filtered lidar topics, unchanged |

The trial launcher does not run `fit_start_pose`, `start_scan_filter`, or `start_localization.launch.py`. Their source, configuration, tests, and historical evidence remain available for comparison. `/localization/start_scan` is not a trial input. See [the retained fixed-wall workflow](START_LOCALIZATION.md).

## Scope and limits

The implementation retains the earlier `register_stationary.cpp` algorithm from `output/three_mode_observe_timing_20261106_v2/held_diagnostics/`: nearest occupied-cell centers over the **full map**, XY ±30 cm/yaw ±6° coarse search, 2.5 mm/0.1° refinement, 10 cm capped RMS, alternating training/withheld endpoints, and independent front/rear diagnostic fits. The default range is explicitly 7.5 m, as used in the historical comparison. This is a bounded fit around an operator-reviewed coarse placement, not a global kidnapped-robot search. Gray pixels are not occupied fitting targets and remain unknown for navigation.

The comparison `output/l2_startup_policy_comparison_20261106_v1/REPORT.md` found improvement with front AMCL but persistent angular/distance disagreement. This restoration does **not** prove localization is fixed or qualify another crossing. Fitting against a movable front wall can bias the result. Review both lidars' overlays, residuals and independent fits; repeatability alone does not establish a correct pose or clear path.

The map, calibration, sensor transforms, timing/metadata corrections, controller/drive settings, speed limits, motion hold and observe-before-enter guards are unchanged. No tool here releases hold or sends movement goals.

## Local profile preparation

After building this package, produce new copies of the A/B/C navigation profiles:

```sh
ros2 run body_route_controller front_localization_params INPUT_NAV2 NEW_NAV2
```

This changes only `amcl.ros__parameters.scan_topic`, `set_initial_pose` (false), and `always_reset_initial_pose` (true). Preserve the existing `trial.yaml`, `drive_limits.yaml`, map and runtime assets. Outputs must be new. The physical `body_route.launch.py` rejects an automatic `initial_pose` or a non-front AMCL input. Simulation may still use an explicit initial pose.

The held L2 launcher is `tools/jetson/l2_trial/held_trial.launch.py`. It requires the newly reviewed `packet_root` and `body_route_prefix`, retains the corrected lidar-driver overlay and all prior hardware/mode/guard checks, and starts no fit, initial-pose publisher or trial client automatically. Deployment and hardware startup require fresh authorization; these instructions do not grant it.

## Authorized held session procedure

1. Establish physical readiness, start only the reviewed held stack, and confirm sole `/cmd_vel` ownership by `cmd_vel_hold`, zero commands/motion, no hold owner/trial, and the effective AMCL topic. Do not replay a historical fitted pose.
2. Specify the robot's **current coarse placement** from the agreed physical start and map axes, within the fit's search bounds. This is not an AMCL estimate and must not be copied from a saved fitted pose. Capture two fresh scan pairs:

   ```sh
   ros2 run body_route_controller full_map_startup capture NEW_CAPTURE_DIR COARSE_X COARSE_Y YAW_DEGREES
   ```

   The parent directory must exist. Capture uses existing `base_footprint`→lidar TFs and does not require AMCL or `map`→`odom`. It verifies fresh stationary odometry/zero commands, hold configuration and ownership, corrected scan timing/angles and both lidar frames. It observes for 3.2 seconds, then takes two pairs at least 0.7 seconds apart. Motion or hold release invalidates the operation.
3. Fit both snapshots, using the **same unchanged map and metadata as map_server**:

   ```sh
   ros2 run body_route_controller register_stationary MAP_PGM MAP_METADATA_JSON NEW_CAPTURE_DIR/first.json NEW_FIT_1.json 7.5
   ros2 run body_route_controller register_stationary MAP_PGM MAP_METADATA_JSON NEW_CAPTURE_DIR/second.json NEW_FIT_2.json 7.5
   ```

   Metadata is a JSON representation of the existing map YAML: resolution, origin, negate, occupied_thresh, free_thresh. Do not alter map thresholds. Historical snapshots remain valid offline regression inputs to this file-only program; they are not valid live initialization inputs.
4. Review the fresh fit overlays and metrics against the whole map, including disagreement between the independent front/rear fits and solutions near the search boundary. If ambiguous or inconsistent, stop and investigate. No automatic fit-quality approval is provided. After review, initialize:

   ```sh
   ros2 run body_route_controller full_map_startup initialize NEW_FIT_1.json NEW_FIT_2.json --reviewed front
   ```

   This sends one `/initialpose` with x/y sigma **0.10 m** (variance 0.01) and yaw sigma **5°** (variance 0.0076154355). It requires front AMCL, continued held startup, fits within 2 cm/1°, matching metadata, captures ≥0.5 s apart and ≤180 seconds old, and unchanged odometry within 1 cm/0.01 rad. Old viewer snapshots lack the required capture provenance and are rejected. If review expires, capture and fit again; never edit timestamps to bypass this check.
5. Verify stationary alignment and covariance on **front AMCL**, while reviewing both scan overlays and current obstacle/unknown-space guards. Keep hold engaged. Any stationary AMCL updates must continue using front input. There is no rear transition and no automatic movement approval. Review any proposed route and obtain the required separate movement authorization.

Freshness checks reduce accidental reuse; they do not replace continuous supervision or detect every possible manual displacement/odometry reset. If the robot is moved, power-cycled, or the capture provenance is uncertain, discard the fit and begin again.
