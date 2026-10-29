# Continuous-motion diagnostic prototype

## Current follow-up

The turn-to-forward sequence in NJWQJ7 is now complete: JHydTT rotated 7.4063°,
then gcQZct moved forward 84.389 mm with confirmed automatic stopping and an
unchanged scene. Map/odom/TF boundary checks support continuity. Input stayed
healthy; cumulative map correction is 8.201 mm / 1.400°. Preserve current session
and placement; do not repeat either pulse. See the
[combined review](../../docs/navigation/jetson_mapping_turn_forward_review.md).
Next validation is a short bounded route revisiting a known view, with geometry
chosen from available physical space. No longer route is prepared or executed.

Earlier rotation:

Fresh-map rotation NJWQJ7/JHydTT is now complete: matching baselines, 7.4063° yaw,
<0.06 mm translation, healthy input, final map->odom correction 3.235 mm / 0.600°.
The user confirms smooth automatic stopping and an unchanged scene with no other
movement. Preserve that current map and wheel runtime; do not repeat the turn.
See the [rotation review](../../docs/navigation/jetson_mapping_rotation_review.md).
The subsequent forward check is complete as summarized above. Timing remains
uncalibrated.

Earlier progression:

The new clean mapping session nEKS7T and forward capture AEjKu5 now pass the short
stationary-to-moving comparison: identical stationary maps, 102.077 mm forward,
only 2.858 mm / 0.600° map->odom correction, healthy input, no intervening scene
change, and user-confirmed automatic stop. The user has since reported another
scene change and requested a fresh-map rotation. Preserve the wheel runtime and
use [MAPPING_ROTATION.md](MAPPING_ROTATION.md) to restart only the mapper and run
the separate v2 live-map recorder after fresh baseline checks.
See [new-scene review](../../docs/navigation/jetson_mapping_newscene_review.md).
The next live-map heading-change diagnostic uses the mapping-compatible v2 recorder;
the older port-owning rotation coordinator below cannot coexist with this mapper.

Earlier progression:

The successful translation and v4 rotation captures have now passed the offline
mapper-input comparison. The separate guarded live input has passed desktop
build/replay/parser/ROS tests and the stationary Jetson trial m6Nmzn. Mapper/TF
integration now runs in 700Cez and the initial map was saved in gjlydJ. Lifecycle
and input health are confirmed; the user confirms stationary capture and forward
clearance. The first live-map forward pulse has since passed motion/input checks
in gUGiCK, with a user-confirmed automatic stop. The user also reports repositioning
after the earlier stationary map, so a fresh map baseline is required before
further movement. See the [review](../../docs/navigation/jetson_mapping_motion_review.md)
and the separate live-map movement procedure in
[MAPPING_MOTION.md](MAPPING_MOTION.md), which records existing streams without
opening the front port. Do not run the old port-owning coordinator alongside it.
See [MAPPING.md](MAPPING.md) for measured approximation effects and runtime rules.
The original driver/configuration remains unchanged; physical map quality remains
unvalidated.
Historical preparation statements below are superseded by this current status.

## Successful rotation and next measurement contract

v4 completed on Jetson in `/home/turtle/jetson-motion-auto.H9GTjD`: 200 scans,
1,389 odom samples, max gap 20.300 ms; +11.29256 degrees net yaw and 0.07063 mm
maximum translation. User confirms smooth rotation, physical stop and restored
front driver. Provenance and all ten nominal/scaled/delay replays pass. Exterior
rays: 39,385; body exclusions: 12,411. Estimated exterior beam intervals overlap
motion in 23 scans at nominal/zero delay. No repeat pulse is needed now.

The default frozen-reference comparison uses 39 segments from 15 training scans:

| Phase | Accepted/exterior returns | Raw median | Corrected median | Raw/corrected RMS |
| --- | --- | --- | --- | --- |
| Pre holdout (45 scans) | 2637/9570 | 1.346 mm | 1.346 mm | 12.818/12.818 mm |
| Motion + margin (27 scans) | 1029/5351 | 3.844 mm | 3.729 mm | 16.795/16.151 mm |
| Post (113 scans) | 2480/21284 | 8.272 mm | 8.272 mm | 20.260/20.260 mm |

These results use nominal timing and zero hypothetical delay. At 100 ms assumed
delay motion RMS is 17.746/17.916 mm, so improvement is not robust across delay
hypotheses. `analyze_scene` optionally accepts `REFERENCE_SPREAD_M SEGMENT_LENGTH_M`
after the two CSV inputs. Fixed sensitivity settings (0.020,0.080), (0.030,0.080),
(0.020,0.160) increase motion matches to 1901, 2369, 1979 respectively (of 5351),
but RMS improvements are only 0.297, 0.124, 0.303 mm at nominal/zero delay.
Do not choose reference settings or delay solely to maximize reported improvement.
The larger stationary post residual cannot be attributed to within-scan motion.
Visibility/finite-segment association and sensor/geometry effects are unresolved;
wheel odometry remains assumed calibrated as instructed.

Maximum nominal zero-delay endpoint correction is 71.605 mm and ray-origin shift
1.10491 mm. Maximum nominal-vs-scaled endpoint difference is 0.36388 mm. Model
agreement does not establish sensor-to-host delay or a physical error bound.

The mapper-input prototype implements the following provisional contract; the
completed offline comparison and stricter live gates are documented in MAPPING.md:

- One front sensor with the confirmed (+0.110,+0.005,pi) planar mount. Preserve
  original stationary drivers/profiles; no TF changes for recorded-data processing.
- Explicit nominal/scaled selected-source timing and nonnegative delay hypothesis;
  never substitute uniform scan_time/400 across the reversed source order/seam.
  Use latest selected estimated beam time as reference, including body-masked beams.
  Preserve acquisition_time_valid=0 and assumption metadata beside exported scans.
- Interpolate bracketed odom with no extrapolation. For a future live path, define
  enforceable clock/freshness/gap rejection separately; existing gross 100 ms gap
  tolerance in the diagnostic is not a proven live safety envelope.
- Mask body returns before correction. Empty/invalid/body/rejected output bins
  remain NaN, never clear-space infinity. No temporal filling of missing sectors.
- Retain corrected origins/endpoints as truth under each hypothesis. A conventional
  single-origin scan at reference time is an explicit approximation. Measure its
  origin and angular-resampling effects separately; collisions keep the nearest
  accepted endpoint and uncovered bins remain unknown. Do not use this approximation
  to claim validated Nav2 clearing. The 1.10 mm origin shift is this run's model
  output, not a general operating bound.
- Compare the resampled result against the corrected per-ray data and fixed scene
  on BOTH 9RvB4b translation and H9GTjD rotation before live implementation.
  Adopt a stated provisional speed/range envelope only with measured support;
  0.025 m/s translation and 0.10 rad/s yaw are exercised conditions, not guaranteed
  simultaneous-motion or navigation limits.
- The later live integration must join domains deliberately: swerve owns
  odom->base_footprint, robot_state_publisher owns base/wheel TF, one owner attaches
  the front sensor using accepted geometry, and mapper alone owns map->odom.
  A complete map/revisit/save test is still required to demonstrate physical SLAM.

## Coordinator timeout correction (v4, local preparation)

The first v3 rotation attempt is `jetson-motion-auto.T60ife`. All 20 yaw messages
were printed after ROS discovery waits, but the combined six-second process
deadline expired. Cleanup then tried a new zero publisher with only three seconds
including discovery; its log contains no publications. User confirms automatic
physical stopping. Odom shows +6.48765 degrees, <1 mm translation, then 3.54 s of
constant pose. Timing was interrupted at 116/200 scans, complete=0; strict replay
must reject it. Keep it as failure/stopping evidence, not accepted mapping data.

v4 keeps the motion amplitude/count and hardware settings. It uses one ROS CLI
publisher with a finite 23-document YAML sequence: 20 yaw messages followed by
3 zero messages. Each gets the current ROS timestamp. Logs are unbuffered; progress
is checked separately from process exit. Discovery has a 5 s CLI limit and an
8 s outer startup allowance, the sequence has 3.2 s from first observed progress,
and process exit has 2 s from final progress. Completion requires all 23 progress
records and successful child exit; progress precedes publish in the CLI and is
not an acknowledgement of controller delivery. Odometry/physical stop checks
remain necessary. There is no second discovery step for normal zeros.

Abort still stops the pulse process group first and attempts a bounded separate
zero publisher, with discovery allowance, three zero messages and checked exit.
Failure is printed explicitly instead of silently swallowed. Command timeout and
physical power cutoff remain fallbacks; neither delivery nor stopping is guaranteed
by a CLI log. No hardware was executed by the assistant. The user subsequently
validated v4 in H9GTjD as recorded above. Original installed v2 recorders and
v3 source/binary are preserved.

Regression tests cover delayed discovery extending beyond the old total deadline,
missing/late publications, stalled/extra progress, child exit and exact ordered
20-yaw/3-zero YAML. `test_rotation_publisher.cpp` additionally exercises real ROS
CLI delivery to a delayed desktop receiver, always on a dedicated test topic in
domain 193/LOCALHOST, never /cmd_vel. It passed with 23 received messages over
2.214 s. Reproduce locally with a new build directory:

```bash
source /opt/ros/lyrical/setup.bash
cmake -S tools/lidar_diagnostics/ros_test -B /tmp/lidar-rotation-ros-test
cmake --build /tmp/lidar-rotation-ros-test -j2
ROS_DOMAIN_ID=193 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
ROS_LOG_DIR=/tmp/lidar-rotation-ros-test/ros-log \
  /tmp/lidar-rotation-ros-test/test_rotation_publisher
```

## Bounded rotation coordinator (v3 baseline, v4 amendments above)

The installed Jetson v2 coordinator remains unchanged. The current standalone
`run_motion_check.cpp` adds `--execute-rotation-pulse` and hardware-free
`--verify-rotation DIRECTORY`. Compile this single file with C++17 separately;
source the existing v2 recorder workspace when running it. No recorder rebuild,
driver replacement, wheel calibration, TF change, or Git command is required.

The user executes one +0.10 rad/s yaw pulse: twenty messages at 10 Hz, then three
explicit zero messages. The usual readiness checks and 200-scan file capture
remain. Nominal commanded yaw is about 0.20 rad before steering/acceleration.
Steer-then-drive may consume part/all of the pulse; offset compensation can roll
wheels during steering. A short/absent rotation is a failed diagnostic, never an
instruction to extend or retry automatically. Observe a fixed wall/corner without
moving people, leave clearance around the full steering envelope, and keep motor
power cutoff accessible. The manipulator stays disconnected; full robot launch
stays stopped.

While the pulse, explicit-zero sequence, and stopping/capture checks run, fresh
odom is required. Translation >=30 mm, yaw <=-0.03 or >=+0.30 rad triggers abort.
These software checks are not hard physical bounds: detection and stopping have
latency and odometry may not capture slip. Success requires >=0.05 rad positive
yaw within scan recording, bounds satisfied, and a stationary final window.
Stop windows now inspect all 25 samples and require >=0.4 s span, rather than
comparing just the first/last pose. Abort retains bounded best-effort zeros and
child-only cleanup. No process is restored or motion retried automatically.

After the break the user confirmed power on/manipulator disconnected; read-only
inspection found no sensor or wheel runtime. Start only the wheel profile needed
for the diagnostic; the front port must remain free until the capture claims it.
If stationary sensor processes have since been restarted, stop only the front
driver in its own terminal. Preserve all original profiles and socket identities.

## Fixed-scene analysis of 9RvB4b

Build `analyze_scene.cpp` with the same C++17 warning flags as replay_motion.
Run it on the CSV produced by successful replay_timing and the original odom CSV.
It reports nominal/scaled results for explicitly hypothetical 0/10/25/50/100 ms
delays, with no delay fitting or scan registration.

The reference uses median XY per bin from the first 15 stationary scans. Adjacent
bins need >=12 observations each, median spread <10 mm, and segment length between
2 and 80 mm. There is no seam connection. Held-out rays are associated with the
nearest segment using raw geometry at the reference pose (50 mm gate); corrected
geometry uses the same association. Reference time includes body-masked selected
beams, matching replay_motion. Both geometries are evaluated in odom using the
confirmed front mount. Association membership can vary across delay hypotheses;
the raw/corrected pair within each hypothesis shares membership. No uncertainty
bound follows from the delay sweep.

On this capture: 28 reference segments; 20 pre-motion holdout scans, 19 motion
plus transition-margin scans, 146 post-motion scans. At nominal/zero delay:

| Phase | Accepted/exterior returns | Raw median | Corrected median | Raw/corrected RMS |
| --- | --- | --- | --- | --- |
| Pre holdout | 841/4308 | 1.327 mm | 1.327 mm | 11.295/11.295 mm |
| Motion + margin | 792/4060 | 2.670 mm | 3.363 mm | 11.702/11.852 mm |
| Post | 5762/30794 | 4.376 mm | 4.376 mm | 12.765/12.765 mm |

The motion group includes scans within 200 ms before odom movement and 300 ms
after its end, conservatively shared across hypotheses. It is not the earlier
15-scan exact-overlap count. Only ~20% of exterior rays support this comparison;
short-segment edges, scene changes/occlusion and sensor noise affect residuals.
No correction benefit is demonstrated. Both models behave similarly; at 100 ms
nominal assumed delay motion median is 3.584 mm raw versus 4.421 mm corrected.
Do not select a physical delay from this result or reopen wheel calibration.
Rotation should produce a larger range-dependent signal and its own stationary
reference. Mapping acceptance remains pending that evidence and ray semantics.

## Coordinated capture revision (v2)

Use `ros2 run coin_d4_motion_capture run_motion_check --execute-forward-pulse`
after building v2 and stopping only the original front driver. The user executes
this command; the assistant must not execute it over SSH. Source Lyrical and the
v2 workspace, set domain 182/LOCALHOST, retain the active wheel controller and
keep motor power cutoff accessible. It commands physical motion once.

The C++ coordinator creates a new `~/jetson-motion-auto.XXXXXX` directory. It
checks the front port with fuser, active hardware/controller, and zero command
publishers / one subscriber. It starts a 60-second odom recorder, waits for
fresh stationary data, starts a 200-scan file-only LiDAR recorder, and waits
for the new `MOTION CAPTURE READY` marker emitted after the first converted scan.
It then waits two seconds, sends ten +X 0.025 m/s commands at 10 Hz, and checks
for a stop. Normal completion sends no explicit zero, retaining the already
tested command-timeout behavior. The coordinator waits for sealing, stops only
its own process groups, and checks odometry coverage and >=5 mm displacement
inside the scan publication interval. This is a coarse overlap check, not
calibrated acquisition-time validation; full provenance replay still follows.

On failure/interrupt, its publisher is stopped; if the pulse was attempted it
sends a bounded best-effort zero command. Motor power cutoff remains the fallback.
It never retries motion, kills existing processes, or restores the front driver
automatically. Restore the original domain-181 front driver after it exits; logs
and partial data remain for diagnosis. Other LiDAR/TF/wheel processes are untouched.
The coordinator's file inspection also supports `--verify-only DIRECTORY`, which
never launches ROS commands and rejects both earlier stationary captures.

Offline tests exercise overlap success, missing footer, absent movement, timestamp
regression, partial CSV writes, child status, timeout and interrupt cleanup. The
previous tbO0uX recording confirms the cause: scans ended 12:59:31 KST while
movement began about 13:00:31 KST, with 29.77 mm net forward odometry. The original
front driver was restored. The new readiness coordination removes manual terminal
timing, but requires user-run Jetson runtime validation.

Prepared 2026-10-27. This is a file-only experiment, not a mapping driver.
The user assumes wheel odometry calibrated and confirmed plate X/Y/yaw=0.
The front sensor's planar pose is therefore (+0.110,+0.005,pi) in the calibrated
base frame. Height remains estimated and is not used by this planar model.

## What is implemented

- `motion_model.hpp`: integer-nanosecond timing hypotheses; piecewise linear
  position and shortest-arc yaw interpolation; front mounting lever arm; beam
  endpoint **and origin** correction into the sensor frame at a scan reference
  time. No extrapolation, repeated/backward timestamps, nonfinite poses, interpolation
  gaps over 100 ms, or adjacent motion above 0.5 m/s / 2 rad/s are accepted.
  These last limits reject gross discontinuities; they do not detect every reset.
- `replay_timing.cpp` retains the original twelve CSV columns and appends range_m
  and raw_angle_rad from the selected raw sample. This uses the actual decoded
  bearing, not the rounded bin centre. No seam unwrapping or bin-index timing.
- `replay_motion.cpp` consumes that CSV and odometry CSV. It requires an explicit
  `nominal|scaled` timing model and assumed nonnegative sensor-to-parser delay in
  nanoseconds (0..1e9); there is no selected/default physical delay. It subtracts
  that delay from the midpoint of each conditional host-clock envelope. This is
  an estimate, not a measured uncertainty interval. Every output row retains
  acquisition_time_valid=0, model, delay, source and packet IDs.
- `coin_d4_motion_capture` is built separately from checksum-verified upstream
  plus the existing timing patch and `motion_capture.patch`. Its executable
  `motion_capture_node` requires `motion_diagnostic:=true`. It deliberately
  does not activate or call the scan publisher. The bounded timing recorder
  retains its caps, clock checks and exclusive output creation. For compatibility,
  `publish`/`published`/`published_scans` in this diagnostic's JSONL denote completed
  scan conversions, **not ROS publications**. After sealing, it still owns the
  sensor until the user stops it with Ctrl+C. Original stationary drivers and
  their guards are unchanged.
- `capture_motion_odom` subscribes to `/odom`, verifies odom/base_footprint frames,
  planar finite poses and advancing stamps, and saves CSV for 60 seconds by default
  (maximum 120 s, 10,000 samples). No motor serial port or command publisher.
  Discard partial captures if it reports an error; a complete raw timing footer
  and odometry bracketing every selected beam/reference are required for replay.

Body exclusion occurs in the original beam frame before motion correction, using
the accepted front-frame box. Invalid/empty bins contribute no rays. Each scan's
reference is its latest selected estimated beam time, before body exclusion.
The CSV preserves ray origins because translated/rotating acquisition does not
give all corrected rays a common origin. **It must not be fed to a mapper by
blindly projecting endpoints into a new LaserScan.** The map-input approximation
and free-space semantics remain a separate design/validation step.

## Build (hardware-free)

```bash
bash tools/lidar_diagnostics/build_motion_capture.sh \
  /PATH/TO/VERIFIED/coin_d4_driver /NEW/WORKSPACE
```

The destination must not exist. The script builds only; no device is opened.
The generated source package can instead be transferred and built with colcon
on Jetson; no desktop binaries should be copied. The package requires the existing
ROS Lyrical underlay including nav_msgs.

Offline tools:

```bash
g++ -std=c++17 -Wall -Wextra -Wpedantic -Werror \
  tools/lidar_diagnostics/replay_timing.cpp -lboost_json -o /tmp/replay_timing
g++ -std=c++17 -Wall -Wextra -Wpedantic -Werror \
  tools/lidar_diagnostics/replay_motion.cpp -o /tmp/replay_motion
/tmp/replay_timing /CAPTURE/timing.jsonl /NEW/timing.csv
# ODOM header: stamp_ns,x_m,y_m,yaw_rad; ROS time, odom -> base_footprint.
# Replace DELAY_NS with an explicitly chosen hypothesis, not a claimed calibration.
/tmp/replay_motion /NEW/timing.csv /CAPTURE/odom.csv /NEW/rays.csv nominal DELAY_NS
```

Outputs are created exclusively only after input validation and correction pass.
Disk-write failures may leave partial files that must be discarded. Inputs remain
unchanged. Compare both time models and several explicit delay hypotheses; their
spread is sensitivity to assumptions, not an accuracy bound.

## Historical manual physical stage (superseded by coordinated capture)

1. Leave the wheel stack in domain 182 and preserve rear LiDAR, relative TF and
   body-view configuration. Temporarily stop **only the front stationary driver**
   in its original terminal before the motion recorder claims USB socket 2.4.4.
   The stationary body cloud will lack fresh front data during this interval.
2. Start the odometry recorder in domain 182 before the timing recorder so it
   brackets the entire scan interval, including pre-publication acquisition estimates.
3. Run the file-only motion recorder on the same Jetson, same system clock/domain,
   front port `/dev/serial/by-path/platform-3610000.usb-usb-0:2.4.4:1.0-port0`,
   frame lidar_front_link, baud 230400, version 4, reverse false, warmup 5 seconds,
   100 scans and a fresh timing_jsonl path. No mapper or LiDAR TF change is needed
   for raw/odom capture. Do not run a second driver on that port.
4. After warmup, the user performs a single previously tested bounded +X pulse
   facing a fixed wall/corner, with no moving people/objects in the scene. Capture
   stationary samples before/after and physical observations. Translation alone
   does not validate rotation; prepare a separate yaw check after inspecting this.
5. Wait for the complete timing footer; Ctrl+C the motion recorder, then restore
   the original front driver with its unchanged workspace, domain 181, parameters
   and USB port. Keep odometry recording long enough to bracket the last scan.
6. Retrieve saved files read-only. Validate order, provenance, timing, gaps and
   corrected-wall consistency. No automatic sensor-delay fitting or timing
   acceptance follows from a visually plausible cloud.

Exact terminal commands are to be supplied in chat after reviewing the Jetson
build, so the user does not accidentally replace the current front process early.
This stage does not install SLAM Toolbox; mapping remains pending timing evidence
and a defined mapper input contract.

## Verification

Desktop ROS package build; C++ tests with warnings as errors and UBSan cover
stationary identity, translation, static-world-wall invariance, rotation lever arm,
yaw wrap, source-index seam, epoch precision, invalid times/poses, gaps and missing
coverage. Existing recorder/parser/clock/replay checks still pass. Saved real
front capture replay validates 100 scans / 32,239 selected rays. With explicitly
synthetic stationary odometry, 2,228 body returns are removed and 30,011 exterior
rays independently match original endpoints/origins. This combination verifies
the pipeline only, **not real moving acquisition or delay calibration**.
