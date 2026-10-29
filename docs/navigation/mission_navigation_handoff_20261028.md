# Handoff: Desktop dual-LiDAR, carpet drift and staged maze mapping

Prepared: 2026-10-28. Updated through trial 14 and the mapping discussion on
2026-10-29, Asia/Seoul. The original filename is retained as the single entry point.
Workspace: `/home/pa4/git/Project2_Turtle_Ninja`.

## Start here

Read the current October 29 checkpoint below first. The assignment, October 28
timing evidence and historical Jetson paths remain later in this file. Consult
`dual_lidar_handoff.md` only for needed older history; its motion commands are
not current instructions.

The user's immediate priority is a working integrated LiDAR system and an accurate
maze map, with incremental adaptation to the dice-memory mission. The latest user
question was whether carpet rotation followed by lateral movement explains the
drift, and whether whole-maze mapping can proceed despite odometry disagreement.
The assistant recommended **supervised, provisional mapping on the actual carpet,
expanding section by section while checking consistency**. This has not yet been
executed. Do not turn the earlier wheel-inspection proposal into a mandatory
calibration gate before any mapping progress. Actual wheel alignment/contact
inspection remains useful; perfect odometry is not a prerequisite for a mapping
trial. A validated map and autonomous navigation are still future outcomes.

The longer-term mission remains **navigate → locally align → manipulate →
independently verify** at one workstation, then expand. Keep assignment rules,
observations, calculated estimates, hypotheses and proposed work distinct.

### User boundaries and retained decisions

- User executes **all hardware and all Git commands**, including read-only Git.
  Assistant may read local files, perform read-only SSH/retrieval, edit local
  documentation and run offline/mock checks. Do not access serial devices, start
  or stop hardware processes, deploy, move/energize hardware or flash firmware.
- User explicitly wants future experiments controlled through execution CLI
  parameters, **without revising firmware or the codebase for each experiment**.
  Use the existing workbench/build. Offline numerical reviews are analysis,
  not runtime changes. Do not implement a proposed correction without direction.
- Preserve continuous mapping, assumed wheel calibration, zero plate x/y/yaw
  offsets, LiDAR mounts/filter/socket configuration and existing motion limits.
  Keep nominal timing, assumed delay zero and acquisition time unverified. Do
  not adopt the numerical timing candidate or repeat completed timing-only trials.
- **Carpet is the actual operating surface.** Firm-floor tests are comparisons,
  not a replacement deployment requirement. Higher speed is a future qualification.
- Use **USB connection-path names** in all future hardware commands. Never fall
  back to transient `/dev/ttyUSB0`/`1` assignments or ambiguous identical serials.
- Explain decisions and user-run steps **in English, here in chat**. Minimize
  terminals: one launcher/interactive console plus the optional RViz window.
- Keep `slam_toolbox` as the working baseline. IMU fusion, Cartographer, Nav2,
  additional filtering and mission integration remain options, not prerequisites.
  Do not silently change mapping input to the merged dual-sensor cloud.
- Jetson will use the user's **external 19 V supply**. The earlier stock-battery
  through OpenCR proposal was superseded. Do not assume Jetson is powered or
  connected. Desktop resource constraints are not a calibrated Jetson emulator.
- Apply project C++ conventions and no-Git-history instructions. No subagents,
  new chats, background goals or external publication without applicable authority.
- Current branch, HEAD, index and overall worktree state were not inspected.
  No Git command was run. Preserve pre-existing work. This handoff changes only
  this documentation file; it introduces no runtime, firmware or calibration change.

## Current October 29 checkpoint

### Working desktop baseline and device identity

Runtime files rechecked locally:

- Launcher and console documentation: `tools/lidar_workbench/run.sh`,
  `tools/lidar_workbench/README.md`, `tools/lidar_workbench/options.hpp`.
- Existing build: `build/lidar_workbench_20261029_pairing_review`.
- Source the environment before launching:
  `build/lidar_transport_review_20261029/cyclone_env.sh`. It selects locally
  extracted `rmw_cyclonedds_cpp` dependencies; it installs no system packages.
- ROS 2 Lyrical; local domain 215; speed 0.025 m/s; turn rate 0.10 rad/s;
  scan output bins 1600; occupancy resolution 0.05 m; extra filter neighbors 0.
- Defaults: maximum segment 0.30 m or 30 degrees; settling 0.5 s plus stopped
  observation 2 s; session limit 1800 s. These are settings, not guarantees of
  actual travel or stopping distance. Plan the full-maze duration before launch;
  do not discover the 30-minute cap mid-survey. A different session duration is
  CLI-configurable; no duration increase has been selected for the next trial.
- Preserved guards: supervisor odometry freshness 100 ms, sensor odometry gap
  40 ms, pair skew 75 ms, pair age 250 ms. Do not loosen guards to hide failures.

| Role | Last successful connection path |
|---|---|
| OpenCR, directly connected to PC | `/dev/serial/by-path/pci-0000:80:14.0-usb-0:10:1.0` |
| Front LiDAR, hub socket 1 | `/dev/serial/by-path/pci-0000:80:14.0-usb-0:6.1:1.0-port0` |
| Rear LiDAR, hub socket 4 | `/dev/serial/by-path/pci-0000:80:14.0-usb-0:6.4:1.0-port0` |

Front/rear roles were checked through the user's disconnection and unchanged-socket
confirmation. `/dev/ttyUSB*` numbers changed with enumeration order; both adapters
have identical serial identifiers. A by-path name identifies topology, not an
intrinsic sensor identity: recheck roles if a hub/PC socket or cable layout changes.
At the latest local inspection, **`/dev/serial/by-path` was absent**. This is a
visibility observation, not a diagnosis of why or proof of hardware power state.
Do not assume devices remain connected after trial completion.

Earlier failures matter only as context: an initial no-rotation/power/setup issue;
trial 02 rejected an unpaired sensor time skew; trial 05 stopped after the user
carried the robot; trial 06 completed a short forward command then lost fresh
odometry in the console although the bag had fresh messages. Offline middleware
comparisons favored the retained Cyclone environment. Trial 07 and later recordings
completed successfully with that user-run setup. This does not establish a unique
USB/power or middleware root cause. Never carry the robot during a live mapping
session: start a fresh session after repositioning.

### Frames, interfaces and mapping assumptions

- Robot frame: +X forward, +Y left, +yaw counterclockwise. Commands are relative
  to the robot frame **at each segment's start**, not a fixed floor frame.
- TF owners: wheel feedback owns `odom → base_footprint`; workbench static
  publishers own the two LiDAR mounts; SLAM owns `map → odom`.
- Mounts in `base_footprint`: front `(0.110, 0.005, 0.0545)` m, yaw pi;
  rear `(-0.110, 0.005, 0.0545)` m, yaw 0. Plate x/y/yaw offsets remain zero.
- Wheel radius/geometry/steering zeroes remain the **assumed** baseline;
  reproducing encoder-based odometry does not establish physical calibration.
- `/lidar_front/scan_mapping` alone feeds continuous SLAM. Rear scans, raw rays,
  corrected rays and merged clouds are recorded for observation/cross-checks.
- Wheel odometry also supplies within-scan motion correction and inter-sensor
  alignment. Correcting `map → odom` does not automatically repair distorted
  individual scans or change the wheel-based command stopping reference.
- Extra filtering is disabled. Neighbor filtering is spatial selection, not a
  proven sensor-noise classifier, and currently does not change the SLAM input.
- `config/slam_front_experiment.yaml` enables scan matching and loop closure.
  Global odometry drift can be corrected by observations, but correction is not
  guaranteed in a repetitive/weak scene. An offline scan fit is not the online
  mapper result. Successful map saving does not establish map accuracy.

### Physical trials and corrected observations

All trial directories are local evidence under `output/lidar_workbench/` and use
the prefix `physical_20261029_`. All listed translations were nominal 200 mm at
0.025 m/s except trial 07's short forward movement. Keep raw records unchanged.

| Trial | Actual command order / condition | Evidence and limits |
|---|---|---|
| 07 | Short forward, automatic stop | User confirmed motion, both heads rotating, no carrying; integration passed. Not a full accuracy/rotation test. |
| 08 | Forward, right, backward, left | Completed; later scan fits suggest about 37 mm backward and 8.2° return yaw. No independent position/heading measurement. |
| 09 | Right, forward, left, backward; carpet, cables clear | User remeasured **55 mm backward**, superseding the original 60 mm estimate. Front/rear stopped-scan fits: 55.9/54.7 mm backward; wheels: 2.7 mm backward; SLAM: about 44.1 mm backward. |
| 10 | Forward, backward; different firm, level surface | User reported no displacement, cables clear. Scan fits: about 0.7 mm return magnitude, near-zero yaw; not calibrated submillimetre accuracy. Saved `lateral_return` is only a filename: this was not lateral travel. |
| 11 | Forward, backward; original carpet | User reported slight heading change, cables clear. Scan fits: about 7–8 mm position offset and 4.2° clockwise return yaw. No numerical independent pose measurement. |
| 12 | Right, left; carpet | User confirmed final **25 mm backward, 15 mm right** against starting marks, clear cables, no visible heading change. Front/rear fits closely corroborate both components. |
| 13 | Right, forward, left, backward; non-carpet comparison | All targets/saves completed and normal quit recorded, but user suspected cable interference. Retain as **potentially affected**; do not treat as a clean controlled comparison. |
| 14 | Same square on the non-carpet comparison surface | User: **“No displacement. All clear.”** Exact material, measurement precision and numerical heading were not supplied. Integration passed; detailed results below. |

Trial 12: front/rear final estimates were `(-24.5,-15.0)` and `(-23.3,-16.1)` mm;
wheel odometry approximately `(-1.2,-0.1)` mm; online SLAM `(-11.6,-0.3)` mm.
Scan-derived yaw was about 0.5–0.7° counterclockwise. The user's “no visible
heading change” is not a measured zero-angle bound.

Trial 14 was reviewed directly from the SQLite bag in read-only mode:

- Four targets, both occupancy/pose-graph saves and normal operator quit completed.
  SQLite integrity check passed; both saved PGM images decoded.
- 1,991 merged frames, all with both sources; all recorded post-READY status
  messages were ready. Odometry stamps had no repeats/regressions; maximum gap
  20.33 ms and maximum recorded age 4.25 ms. These are recorded pipeline checks.
- Final command was zero; wheel zero/torque-off, both LiDAR closes and recorder
  stop were logged. SLAM exit -2 was during requested SIGINT shutdown after saves.
- Parsed `input.yaml`, `slam.yaml` and `manifest.yaml` matched trial 09 exactly.
  The environment source is part of the user command; the manifest alone does
  not independently prove which RMW was active.

| Trial 14 final estimate, initial robot frame | X mm | Y mm | Yaw degrees |
|---|---:|---:|---:|
| Front stopped-scan alignment | +6.0 | -6.0 | +2.03 |
| Rear stopped-scan alignment | +1.6 | -3.3 | +1.54 |
| Combined stopped-scan alignment | +2.6 | -2.8 | +1.79 |
| Wheel odometry | -2.1 | +0.4 | +0.58 |
| Online SLAM | -19.4 | -5.8 | +2.08 |

Thus separate scans estimate only about **4–9 mm** position offset, consistent
with the user's qualitative return report, while suggesting residual heading
change. These calculated values are not calibrated ground truth. Front/rear
fits share mounting/timing assumptions and differ by several mm and about 0.5°.

The surface comparison has an important confound: just before the first move,
trial 09 steering encoder angles were approximately `[92.81,86.57,-90.18,90.62]°`
for FL/FR/RL/RR, whereas trial 14 had `[177.54,-185.98,179.56,174.81]°`.
These are encoder coordinates, not independently measured rolling directions.
The starting wheel orientations and surrounding scan geometry differed. The
result strengthens a surface/contact-related hypothesis; it does not prove
carpet alone caused the previous displacement or identify a calibration offset.

### Rotation/lateral-motion hypothesis: evidence and inference

The user proposes that rotation on carpet introduces error and later lateral
motion exaggerates displacement. This is plausible but incomplete:

- Square-trial commanded angular velocities were zero. Wheel steering changes
  and unintended chassis yaw are different from a commanded chassis-rotation
  experiment. Commanded `cw`/`ccw` performance on carpet remains unqualified.
- A 5° heading error redirects a nominal 200 mm lateral movement by about
  `200*sin(5°) = 17 mm` fore–aft. This illustrates propagation, not a fitted
  explanation of all 55 mm.
- Trial 09 phase analysis attributed about 8 mm of X disagreement to steering
  reorientation and about 45 mm after steering settled. This is an approximate
  moving-scan decomposition that inherits the existing deskew assumptions.
- Trial 12's return leg added roughly 15–18 mm backward in scan estimates while
  steering angles largely stayed lateral and drive direction reversed.
- Recorded wheel feedback reproduced the odometry calculation. A calculation
  can be correct for an assumed model while actual rolling/contact differs.
  Slip, wheel alignment, contact loading and deformation are still hypotheses.

Do not claim that filtering would remove this measured displacement, that a
timing offset fixes stopped-view disagreement, or that changing the surface
establishes physical wheel calibration. Do not promise SLAM will absorb all drift.

### Proposed next physical work: progressively map the actual maze

This is the latest recommendation, not a completed experiment or an exact
preapproved hardware route. The user executes each command.

1. Recheck current instructions, saved build/environment and connection topology.
   Establish actual maze placement, accessible corridor and full robot/cable
   swept clearance before issuing a route. The simulated route is not a physical
   clearance certificate. Keep robot load/posture comparable and mark start pose.
2. Start fresh on the actual carpet, with continuous mapping and recording.
   Retain 0.025 m/s, the 0.10 rad/s turn setting, zero extra filtering, calibration
   and timing assumptions. Plan session duration and checkpoint names first.
3. Map one corridor and its first corner with short, separate bounded commands.
   Check a small commanded chassis turn before committing to larger turns.
   Wait for READY and stopped observations after each turn/direction change;
   a roughly 5 s pause is an observation interval, not a correction for slip.
4. Revisit a previously observed view and save a checkpoint. Check stopped scans
   against existing walls, corner shape and independently measured robot
   position/heading. Prefer straightforward corridor travel, using lateral
   motion where needed; this simplifies diagnosis, not a proven accuracy ranking.
5. Extend section by section **in the same continuous mapping session** if
   consistency holds. Save map and pose graph at each section. Stop expansion
   on persistent doubled walls, distorted corridors, implausible pose jumps or
   any health failure. Do not carry the robot, reset odometry, bypass a guard or
   automatically retry a failed mapping run.
6. Before accepting the full map, compare measured corridor widths, wall
   separations and corner positions with the map and check revisited geometry.
   Report errors against measured references. Neither a clean-looking map,
   correct return, 5 cm cells, nor successful save proves navigation accuracy.

No quantitative global-map acceptance tolerance has been agreed. Choose it from
measured passage/robot clearances and intended navigation use; do not borrow the
assignment's 10 mm puck-placement tolerance as a chassis or map requirement.
The workbench does not provide autonomous obstacle avoidance. Actual command
distances, heading and clearances must be observed independently of wheel targets.

Physical steering alignment/contact inspection remains worthwhile in parallel.
A stricter surface A/B trial would match initial wheel orientations, load, route
and cables and measure heading. It is optional supporting work, not a requirement
to repeat all small trials before the first supervised maze section.

### Last successful launcher pattern (user-run only)

This is a reference, not an instruction to execute automatically. Trial 14 is
occupied. On 2026-10-29 trial 15 is a candidate name only; recheck the directory
and date before use. Recheck all three paths after reconnection. Use `--dry-run`
instead of `--execute --rviz` for syntax inspection without opening devices.

```bash
cd /home/pa4/git/Project2_Turtle_Ninja
(
  source build/lidar_transport_review_20261029/cyclone_env.sh || exit 1
  for trial_device in \
    /dev/serial/by-path/pci-0000:80:14.0-usb-0:10:1.0 \
    /dev/serial/by-path/pci-0000:80:14.0-usb-0:6.1:1.0-port0 \
    /dev/serial/by-path/pci-0000:80:14.0-usb-0:6.4:1.0-port0
  do
    if [[ ! -e "$trial_device" ]]; then
      printf 'Missing device: %s\n' "$trial_device"
      exit 1
    fi
  done
  tools/lidar_workbench/run.sh \
    build/lidar_workbench_20261029_pairing_review \
    --hardware opencr \
    --drive-port /dev/serial/by-path/pci-0000:80:14.0-usb-0:10:1.0 \
    --front-port /dev/serial/by-path/pci-0000:80:14.0-usb-0:6.1:1.0-port0 \
    --rear-port /dev/serial/by-path/pci-0000:80:14.0-usb-0:6.4:1.0-port0 \
    --speed 0.025 --turn-rate 0.10 --filter-neighbors 0 \
    --output output/lidar_workbench/physical_20261029_15 \
    --execute --rviz
)
```

Interactive commands: `status`, `forward METRES`, `backward METRES`,
`left METRES`, `right METRES`, `ccw DEGREES`, `cw DEGREES`, `stop`,
`save NAME`, `quit`. Enter one at a time, waiting for READY; do not paste a whole
hardware route. Hardware scripts are refused. Trial 09/14's comparison order
was right 0.2 → forward 0.2 → left 0.2 → backward 0.2. Do not repeat it by default
or mistake that local square for the next maze route.

### Retained simulation work and its limits

The confirmed model is the October 26 release in **C1_P0**, at
`/home/pa4/Downloads/Learning/turtle_ninja_model_lab/releases/nt_urdf_release_20261026/urdf/robot.urdf`.
The user authorized clearly labeled provisional maze dimensions. See
`tools/maze_mapping_lab/README.md`, `arena.json`, and
`output/maze_mapping_lab/{controlled,resources,datasets_v2,scene}/`.

The README records completed C++/MuJoCo geometry and actual `slam_toolbox`
comparisons. Provisional walls: 6 mm thickness, 300 mm height; posts: 60 mm
diameter. Physics is kinematic: carpet traction, real steering/contact, material
returns and USB contention are not validated. Earlier accelerated asynchronous
`trials/` rankings are superseded by `controlled/` for geometry comparisons.
The failed `controlled/holonomic025` attempt is retained; its successful retry
is `holonomic025_retry`. Do not count the failure as a successful result.

Synthetic fixed-heading interior paths were promising, but observed carpet
lateral drift now limits direct transfer of that ranking. No simulation result
authorizes a physical 0.04/0.08 m/s speed increase or guarantees actual clearance.
The scene/route visual exists at
`/home/pa4/.codex/visualizations/2026/10/28/01a181a7-3aac-7fb2-b483-7bb16273053d/physical-scan-path.html`;
it is a plan, not execution evidence.

Desktop cgroup CPU/RAM limits and synthetic communication latency are stress
envelopes, not calibrated Jetson performance modes. The CPU SLAM workload does
not benchmark future GPU/camera processing, unified-memory bandwidth, ARM timing,
power/thermal throttling or physical USB transport. The user selected external
19 V Jetson power; no live Jetson check was made for this handoff.

### Evidence locations, review method and Notion status

Read actual files before quoting new values. Primary local evidence:

- `output/lidar_workbench/physical_20261029_{09,10,11,12,13,14}/`: event/segment
  CSVs, odometry, bag, exact configs, process logs, saved maps/pose graphs.
- `physical_20261029_{09,10,11,12,14}_review.json` in the same output root.
- Corresponding `_stationary_review/results.json`; trial 09 additionally has
  `corners.json`; `_09_motion_review/conclusions.json` and
  `phase_scan_analysis.json`; `_12_motion_review/wheel_feedback_analysis.json`.
- Latest direct review: `physical_20261029_14_review.json` and
  `physical_20261029_14_stationary_review/results.json`. The latter has five
  stopped windows, each ending before the next motion; odometry spans are
  negligible. Pose estimates are relative to the initial robot frame.
- `output/lidar_workbench/console_review_20261029/review_status.json` is useful
  transport history but **stops at trial 12**; its proposed next step and hardware
  visibility timestamps are superseded by this checkpoint.

The scan method uses stopped, per-input-bin median ray endpoints, recorded mount
transforms and robust planar point-to-line alignment; front/rear are fitted
separately and combined, with five starting poses and 4/8 m range cutoffs. Seed
agreement is convergence evidence, not an uncertainty bound. For trial 14's final
return comparison, all initial/final median points were within 4 m, so the 4/8 m
cutoffs selected the same data and are not independent range-sensitivity checks.
Do not confuse these offline fits with online SLAM or independent metrology.

Ad hoc analysis scripts remain temporary: `/tmp/trial09_stationary_review.py`,
`/tmp/trial09_corner_review.py`, `/tmp/trial09_phase_scans.py`,
`/tmp/trial09_wheel_review.py`, `/tmp/trial10_review.py`, `/tmp/trial11_review.py`,
`/tmp/trial12_review.py`, `/tmp/trial12_wheel_review.py`, `/tmp/trial14_review.py`.
Trial 14 imports numerical functions from the trial 09 script. These were present
at handoff creation but may disappear. Preserve/review dependencies before relying
on them for reproduction. Analysis uses read-only SQLite and ROS deserialization,
without ROS node initialization, hardware replay or serial access. Results under
`output/` are durable local files, not guaranteed to be included in a Git checkout.

Notion was updated earlier at the user's explicit request; API readback was
rechecked for this handoff:

- Parent [Project 2](https://app.notion.com/p/4033980c8a4481bf9016c232ad01c84c)
  contains all eight identified project learning notes, covering October 19–29
  through consolidated/dated pages. Four newer notes were moved under it.
- [October 29 learning note](https://app.notion.com/p/4083980c8a448147b9e6e04731d477f3)
  currently covers learning **through trial 12**, with three plots, one evidence
  ledger and two original user photos inserted beside trial 09's 55 mm result.
  Trials 13/14 and this latest mapping recommendation have **not** been added.
- Photos: `/home/pa4/Downloads/20261029_121309.jpg` and
  `/home/pa4/Downloads/20261029_121549.jpg`. They belong to trial 09, not trial 12.
  Captions distinguish visual context from the user's measured 55 mm value;
  filename order was not asserted as independently verified before/after timing.
- Local publication package: `output/learning_notes/20261029_lidar_carpet/`.
  Its `page_body.txt` and `notion_publication.json` predate the photo insertion
  and page move: the receipt still says private draft/three figures. Fetch Notion
  before editing; do not overwrite the current page with that stale draft.
- Verification was structural API readback, not a browser-rendered inspection.
  Raw ROS bags were not uploaded. Creating this handoff did not publish updates.

### Outstanding information and immediate resume action

No exact maze route, fresh hardware connection state, quantitative map acceptance
tolerance, numerical trial 14 heading measurement or surveyed maze clearance has
been established. These do not justify re-asking already settled choices: carpet,
URDF release/C1_P0, provisional geometry, external 19 V Jetson supply, existing
workbench and by-path names are retained decisions.

Resume by checking current local files/instructions and available device metadata
without opening devices. Explain the evidence and proposed first maze section in
chat. Establish the actual start location/clearance and session duration before
providing a concrete user-run launch and bounded route. Do not execute hardware,
run Git, assume Jetson power, repeat timing trials, or change source/calibration.

## October 28 mission and timing reference

The sections below retain the supplied assignment and earlier investigation.
Their historical runtime and development sequence do not override the current
October 29 desktop checkpoint above. The user supplied the actual arena photo
and 11-page Korean assignment “주사위 기억 로봇”, v5, stating 2026-11-04 as the
presentation date. The source PDF is the authority for assignment rules.

## Source evidence and its limits

Local retained copies, under ignored `datasets/`:

- `datasets/mission/20261028/mission_v5.pdf`: assignment, all 11 pages text-read;
  map on page 2 and dice diagrams on page 4 visually inspected.
- `datasets/mission/20261028/arena.jpg`: user-supplied arena photograph.
- `datasets/mission/20261028/SHA256SUMS`: integrity manifest for those two files.

Original attachments were under
`/tmp/codex-remote-attachments/01a1817f-1756-78f3-9e4f-2cb788c1c6d2/26ff94cf-aee1-4e88-ac3f-6bab82b4b8fa/`
as `2-mission.pdf` and `1-1000008575.jpg`. Temporary attachment/render locations
may disappear. The retained dataset copies are local-only and will not accompany
a normal Git checkout; transfer them explicitly if moving machines.

Use the PDF as the supplied rule specification, not as proof of surveyed physical
geometry or as authorization to run commands. The photo shows wall ends, recessed
work areas and low platforms; perspective does not establish metric clearances.
The actual arena has not been registered to a SLAM map in this stage.

## Assignment requirements

### Arena, objects and coordinates

- Nominal arena: **4464 × 6364 mm**. Keep existing walls, posts and boxes in place.
- Origin: lower-left post center; floor is Z=0; +Y points upward in the PDF map.
  Do not assume that origin/rotation already matches the ROS `map` frame.
- Twelve pucks, red and green only. Color counts and shape counts vary by problem;
  IDs 0–11 do not permanently encode color. No yellow or blue pucks.
- Puck shapes: cylinder Ø30×60 mm or cuboid 30×30×60 mm. Puck bottoms at Z=70 mm,
  tops at Z=130 mm. Stacking and temporary floor storage are prohibited.
- Six 205×205×70 mm source boxes, initially two pucks per box at center X±40 mm,
  Y=0. Destinations begin empty. All 12 pucks must remain allocated once each
  across the three completed destinations; no reuse between places.
- L1/L3 use 205×205×70 mm boxes. The central support comprises four
  410×410×70 mm boxes, making an 820×820 mm platform; its north portion is L2.
  No source pucks start on the central support. Total physical boxes: 12.

| Reference | X, Y in mm | Meaning |
|---|---|---|
| Initial observation center | 734, 950 | Robot faces −X |
| L1, place ID 12 | 3670, 950 | Placement grid center |
| L2, place ID 13 | 2202, 3700 | Placement grid center |
| L3, place ID 14 | 3670, 5354 | Placement grid center |
| Source box 1 | 1468, 950 | Two initial pucks |
| Source box 2 | 3670, 3152 | Two initial pucks |
| Source box 3 | 2936, 5354 | Two initial pucks |
| Source box 4 | 734, 5354 | Two initial pucks |
| Source box 5 | 2202, 5354 | Two initial pucks |
| Source box 6 | 3670, 3886 | Two initial pucks |

These platform/grid centers are **not robot driving goals**. Define separate
approach poses with heading, working distance, arm reach and departure clearance.
Convert millimetres to metres explicitly in ROS interfaces. Determine the arena
to SLAM-map transform before using these coordinates for navigation.

### Signal, barcode and allocation

- Traffic signal stands vertically on the floor with its bottom at (80,950,0) mm,
  facing +X. Its actual emitting-face height is not specified numerically.
- Barcode center: (80,650,450) mm, facing +X. Station-tag centers are 35 mm high
  on the approach-facing vertical surfaces. Tag encoding/family is not supplied.
- Operator starts the sequence after robot readiness. Each flash is one color
  on for 1 s, followed by 1 s off; repeated colors are separated by darkness.
- Record exactly 12 colors, in order, and R/Y/G counts. Each count is 1–6 and the
  sum is 12. Confirm 3 s continuous darkness after flash 12; there is no separate
  green departure signal. Move only after recording, termination confirmation
  and barcode decoding. Never guess a missed/unknown color or count darkness.
- Code 128: P2-01/P2-03 mean ascending counts; P2-02/P2-04 mean descending.
  Ties preserve R → Y → G. Assign sorted groups to L1 → L2 → L3.
- R requires red pucks, with mixed shapes allowed. G requires green pucks, with
  mixed shapes allowed. Y requires one common shape, with mixed colors allowed.
- Observe/register the actual 12-object inventory and solve the complete
  allocation before picking pucks that could make Y infeasible. Official problems
  are intended to be solvable, but software must detect invalid inputs.
- Q1 example only: `RGRRGRGGYGYG` gives R=4, Y=2, G=6. P2-01 yields
  L1=Y2, L2=R4, L3=G6. Red/green 6/6 in the public example is not a fixed rule.

### Placement, verification and return

- Each destination uses a 3×3 grid at X/Y offsets −60, 0, +60 mm from its center.
  The specified placement region is 180×180 mm.
- Each required point has one puck, **center error ≤10 mm**, stable for 2 s.
  Extra pucks on unused grid positions fail that destination's dice pattern.
- Pattern orientation is fixed in the arena frame, not relative to robot heading.
  The 2/3 diagonal is upper-left to lower-right. Six uses the left and right
  columns. Rotated alternatives are not accepted. See page 4 for all six patterns.
- After placement, obtain fresh camera observations and record actual IDs and
  positions at each destination; check global duplicates/missing IDs. Copying
  target state into a log does not count as verification.
- Return requires the entire chassis inside the start area, stationary for 3 s.
  Exact start-area boundary coordinates are not explicitly tabulated in the PDF;
  resolve them before implementing a quantitative containment check.
- Under/over-counted signals, unknown colors, unregistered barcode and duplicate/
  missing IDs require an error record and safe stop. Team errors do not permit
  signal replay or manual correction followed by resuming the same official run.
  Operator-equipment errors allow an invalidated run to be retried under the same
  conditions. Preserve valid partial results on emergency stop, withdrawal or fault.

### Scoring and schedule

No time limit or speed bonus. Official scoring uses the better **whole run** of
two attempts, not successes assembled from different attempts.

| Criterion | Points |
|---|---:|
| Signal sequence | 12 |
| Three color counts | 9 |
| Barcode interpretation | 4 |
| Destination group assignment | 9 |
| Correct color/shape selection | 18 |
| Dice placement | 30 |
| Fresh post-placement verification | 8 |
| Return | 10 |
| Total | 100 |

The project integrated-match component is raw score×0.20 plus 5 for complete
autonomous success. Optional perception/allocation/route challenges add no points
and have prerequisite basic functions. Route optimization is not the first goal.
Presentation date is stated as 2026-11-04; intermediate/final deadlines are separately
announced. Do not infer them. Refer to PDF pages 7–11 for full scoring/submission
and optional challenge procedures.

The referenced `placement.csv`, `mission.json`, registered puck inventory and
example-combinations.csv were not attached. Do not invent their schemas or actual
run assignments. The PDF uses both signal.json and signals.json in submission
contexts; settle the required interface when preparing deliverables.

## Completed timing experiment: retain these conclusions

October 28 column trial: three forward/backward pairs, six nominal 200 mm legs
at 25 mm/s, with stopped observations. Robot initially faced away from the column.
User confirmed all automatic stops, no intervention, no manual repositioning and
no scene changes. Nominal route distance is not an independent travel measurement.

Column dimensions, corrected order: **right 660 → bevel 30 → front 755 → bevel 30
→ left 675 mm**, measured at beam height. Bevels are measured along their sloping
faces; angles are unknown. Do not assume 45° or average the unequal sides.
The visible nearby face spanned approximately 583 mm; this was partial coverage,
not a verified measurement of the entire 660 mm face or all five surfaces.

- Independent bag: 263.952 s, 2,533 scans, 13,094 odometry messages, 509 commands.
  Seven trace files have clean footers and zero logger losses; no internal scan
  gaps or packet checksum failures. Startup stale/backlog events are separate.
- Reconstructed raw-ray reference stamps match all 2,551 published scans.
  All bag odometry poses/stamps match the controller trace. Extra scans occurred
  after the bag stopped. Read-only bag integrity and saved-map checks passed.
- Actual wheel-estimated leg lengths: approximately 203–204 mm. Final wheel pose
  differs from initial by x=−1.261 mm, y=−0.550 mm, yaw=0.010668 rad (~0.61°).
  These values do not prove physical return accuracy.

| Moving software stage | Median ms | p95 ms | Maximum ms |
|---|---:|---:|---:|
| Odometry made → published | 0.801 | 0.973 | 13.944 |
| Odometry published → observer | 0.432 | 0.813 | 6.172 |
| Mapper receives frame → scan publication | 17.963 | 27.937 | 29.833 |
| Scan published → observer | 0.363 | 0.568 | 0.942 |

Successful final processing-attempt start→publication was median 3.348 ms across
the run. The broader callback→publication interval includes scheduling/odom waits;
it is not optical acquisition latency. OpenCR state timestamps are not per-motor
acquisition stamps. Absolute optical/encoder timing and SLAM internal processing
duration were not established.

Offline raw-ray alignment searched signed shifts −300..+300 ms by 5 ms under
nominal and period-scaled timing, with fixed mount/wheel assumptions. First two
pairs selected a candidate; the third evaluated it. The numerical choice
scaled/−15 ms improved robust residual by <0.002 mm on fitting and <0.001 mm on
validation. Positive shift means using a later wheel pose than the estimated ray
time. Per-leg preferences conflict; **the candidate was not adopted**. Keep
nominal timing, assumed delay=0 and acquisition_time_valid=0.

Stopped-view apparent wall displacement in the wheel frame accumulated to about
8, 17 and 23 mm after successive pairs. A timestamp shift cannot change a constant
wheel pose during a stationary interval. This exposes a spatial disagreement,
not its cause and not proof of 23 mm actual robot displacement. Actual return
drift, range/angle bias and other spatial effects need independent evidence.
At 25 mm/s, 10 ms corresponds to just 0.25 mm of travel.

An initial steering-limit event and a logged battery minimum of 10.6 V occurred.
Neither proves causation. A charged run alone would not isolate the discrepancy.
Do not recalibrate wheels or install a delay from these observations.

## What this means for the mission

The 10 mm rule concerns **puck centers**, not global chassis navigation accuracy
at every point in the arena. Allocate separate requirements to:

1. Global navigation: collision-free travel and arrival in a usable workstation
   approach region, with a suitable heading and stopping behavior.
2. Local alignment: observe the actual platform/tag/pucks and establish a local
   working pose with sufficient accuracy for the arm.
3. Manipulation: reach, grip, transport and place within the measured error budget.
4. Verification: independently reobserve actual results, including ID uniqueness.

Do not equate a 5 cm occupancy cell with final manipulation accuracy, or claim
that a finer grid fixes sensor/frame errors. Select navigation tolerances and map
resolution after checking measured passage widths, footprint and approach needs.
The latest trial does not validate a particular Nav2 tolerance or autonomous run.

Use the full moving/carrying envelope, including gripper, arm and carried puck.
Confirm a stowed travel configuration. Front-only mapping does not validate rear
or lateral collision coverage. Preserve dual-LiDAR hardware setup; qualify actual
obstacle coverage before backward or lateral autonomous commands.

The previously assumed LiDAR plane is approximately 54.5 mm above the floor;
this remains provisional. **If still true**, rays may intersect 70 mm box sides
while passing below pucks whose bottoms are at 70 mm. LiDAR therefore cannot be
assumed to provide puck identity/placement observations. Verify actual scan height.
Camera coverage must accommodate low 35 mm station tags, 70–130 mm pucks, the
450 mm barcode and the actual traffic-light emitting height. Do not assume one
camera pose sees all of them. No camera/arm calibration is established by this file.

## Longer-term mission sequence retained from October 28

This is the mission development sequence. The October 29 staged mapping plan
above is the immediate next physical work; these later milestones remain open.

1. **Offline mission contract.** Re-read the PDF and current source instructions.
   Inspect existing nodes/interfaces before creating overlapping code. Model
   signal capture, stable sorting, complete allocation, dice geometry and error
   transitions. Keep expected and observed state separate. Acceptance: Q1 is
   correct; ties, descending sort, unknown signal/barcode, duplicate IDs and
   infeasible allocation have explicit, tested outcomes. User supplies missing
   actual inventory/interfaces when needed; no hardware required for these checks.
2. **Arena/frame and approach specification.** Establish mission-to-map transform,
   measured critical clearances, full footprint and source/destination approach
   poses. Platform centers must not become chassis goals. Resolve start-zone
   boundaries, arm reach and tag visibility. Acceptance: each required approach
   and exit is feasible under current motion/obstacle constraints; mark unverified
   physical assumptions rather than treating the photograph as a survey.
3. **One complete workstation operation.** Prepare bounded user-run validation of
   navigate, stop, observe, align, pick/place and fresh verification. Establish an
   independent way to measure puck-center error ≤10 mm and stability ≥2 s.
   Separate navigation, camera alignment and arm errors. Acceptance is measured
   physical success, not a successful action response or copied target log.
4. **Full mission integration.** Scale to six source boxes and three destinations;
   retain all 12-object identities and plan complete allocation before collection.
   Confirm carrying envelope and exception recovery. Verify all destinations from
   fresh observations and full-chassis return containment for 3 s. Prioritize
   reliable completion over shortest route or higher speed.
5. **Supporting comparisons.** Qualify IMU frames/stamps/bias before fusion;
   compare Cartographer offline on the same evidence while retaining slam_toolbox.
   Give each TF edge exactly one owner. These tasks must serve observed mission
   limitations, not delay all progress pending perfect timing calibration.

Combine the next useful recording with a fixed-scene, independently measured
start/return **position and heading** check and appropriate battery state. This
can distinguish physical return displacement from apparent laser geometry drift.
Set route, clearance and acceptance criteria before motion. Do not prescribe a
new experiment or increased speed solely from this handoff.

## Historical Jetson checkpoint and reproducibility — October 28

At the October 28 checkpoint, Jetson was checked stopped after normal
recording/mapping/wheel shutdown;
wheel log reported zero command and torque off. Full evidence was copied before
the user was told shutdown was safe. **Actual power-off is unconfirmed.** Do not
assume a live robot, unchanged IP, current disk space or pose continuity.

- Last target: `turtle@10.2.12.123`, ROS Lyrical, domain 182, LOCALHOST discovery.
- Wheel installation: `~/ros2_wheel_config_v1_ws`.
- Diagnostic overlay: `~/ros2_timing_diagnostic_v1_ws`.
- Original front mapping installation: `~/ros2_front_mapping_v1_ws`.
- Console: `~/jetson-mapping-console-v2/build/run_mapping_console`.
- Completed capture: `~/nt-timing-column-20261028-v1`;
  console session: `~/jetson-mapping-session.zSGMLh`;
  checkpoint: `~/jetson-mapping-console-v2/runs/map_20261028T105722.t1T7Wg`.
- Durable local evidence: `datasets/timing/20261028-column-v1/` and
  `datasets/timing/20261028-stationary-v1/`. The column directory contains the
  compressed capture, C++ review/fitter, results, figures, SHA256SUMS and
  `analysis-method.txt` with offline reproduction commands. Its 14 manifest
  entries passed the preceding handoff check. No hardware replay is required.
- Temporary `/tmp/nt-timing-column-review` may disappear. Dataset archives are
  ignored/local-only; they are not implicitly available on another machine.
- Relevant source: `tools/timing_diagnostics/`, `tools/lidar_diagnostics/`,
  `config/front_mapping_trial.launch.xml`, `config/slam_front_experiment.yaml`,
  `config/controllers_drive_only.yaml`, `firmware/nt_opencr/nt_opencr.ino`, and
  `ros2_ws/src/{hardware,swerve_controller,description,bringup}/`.

As of that October 28 handoff, no mission state machine, arena registration,
approach-pose validation, new IMU fusion, Cartographer deployment or Nav2 mission
execution was implemented by
this assignment review. No external service or Notion page was updated for this
October 28 handoff. The October 29 simulation, physical trials and Notion work
are described in the current checkpoint above.

## Prompt for the next session

> Read `/home/pa4/git/Project2_Turtle_Ninja/docs/navigation/mission_navigation_handoff_20261028.md`
> first, especially its updated October 29 checkpoint through trial 14. Consult
> `dual_lidar_handoff.md` only for needed older history. Resume preparation for
> supervised, staged maze mapping on the actual carpet with the existing desktop
> workbench. Explain the proposed first corridor/corner checkpoint in English
> here; establish actual clearance and start location before giving a motion route.
> Preserve continuous slam_toolbox, both LiDARs, assumed wheel calibration, zero
> plate offsets, nominal timing and USB connection-path names. Do not change
> firmware/runtime code, adopt a timing correction or repeat timing-only trials.
> Distinguish measured observations, offline scan estimates and hypotheses.
> I execute all hardware and all Git commands; read-only SSH is allowed.
> Recheck current files, instructions and device visibility. Do not assume the
> Jetson is powered or that the old Notion draft reflects the current page.
