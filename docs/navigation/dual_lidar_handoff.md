# Handoff: Physical SLAM and navigation

Updated: 2026-10-28 (Asia/Seoul). The user completed one more experiment, using a
measured column instead of a corridor, to investigate LiDAR/odometry acquisition,
publication and processing timing. This superseded the previous "last experiment"
decision; no further run is currently requested. **No new teammate package is requested.** Explain all
steps and findings directly in chat; the user does not read reports.

## Start here in the next session

Handoff review: 2026-10-28 (Asia/Seoul), in response to the user's “Create handoff”
request. The retained column archive, analysis sources, results, figures and
reproduction notes were rechecked against all 14 entries in their SHA256 manifest;
all passed. Existing later integration plans and Notion provenance below were
preserved from this document, not independently reverified in this review. No
Jetson connection, hardware command, Git command or external publication was made
for this handoff. Confirm scope against the user's next request before starting
implementation; the ordered work below is a continuation plan, not evidence that
integration is already complete.

This is the authoritative handoff requested after the October 28 Notion page was
published. Workspace: `/home/pa4/git/Project2_Turtle_Ninja`, local desktop host;
user timezone Asia/Seoul. Read this section and the current-state section before
older checkpoints. Earlier "next", "latest" and launch/motion instructions are
historical unless explicitly adopted below. Do not restart old pulse experiments.

### Next intended outcome and operating boundaries

Prepare the next integration stage: **qualify the OpenCR IMU and prepare an
offline Cartographer comparison while retaining slam_toolbox as the baseline**,
then evaluate odometry fusion and supervised Nav2. The user wants an efficient
workflow, not a new timing calibration at each speed. Current evidence supports
software latency measurements, not a validated optical/encoder delay correction.
Unresolved spatial error does not prohibit offline work or supervised SLAM
development; it does prohibit claiming physical accuracy or navigation readiness.

- The user executes **all hardware and all Git commands**, including read-only
  Git commands. Assistant may inspect through read-only SSH, retrieve evidence,
  edit local files and run offline/mock checks. No remote install/deploy, robot
  process start/stop, serial access, motion/torque, firmware flash or Git action.
- Preserve continuous mapping as the intended mode, assumed wheel calibration,
  plate x/y/yaw=0, LiDAR mounting/filter/socket settings and existing safety limits.
  No automatic AMCL requirement, wheel recalibration or teammate package.
- IMU work is now requested, superseding earlier IMU scope exclusions. This does
  not establish its mounting pose or validate enabling it in the controller.
- Explain decisions/results and concrete user-run steps in chat. User explicitly
  says they do not read reports, but requested a Notion record and this handoff.
- No subagents unless the user or applicable instructions request them. Apply
  project C++ conventions, no-Git-history and clarification skills as relevant.
  Do not infer a new background goal, scheduled automation or separate chat.

### Runtime and evidence snapshot

At the last checked closeout, all wheel/mapping/recording/observer processes had
exited normally; wheel log reported zero command and torque off. The user was
told they could shut the Jetson down after the full data copy was verified.
**Actual power-off is unconfirmed. Do not assume a live robot or pose continuity.**
Do offline preparation first; when hardware access is needed, establish current
boot/process/device state with read-only checks and ask only for physical facts
that may have changed. Do not ask again about completed-trial stop confirmation.

- Target: `turtle@10.2.12.123`; ROS Lyrical, domain 182, LOCALHOST discovery and
  empty static peers in the diagnostic session. IP/boot/free space need checking
  when next accessed; the earlier 189 GB free reading is historical.
- Original wheel install: `~/ros2_wheel_config_v1_ws`; source bundle directory
  `~/jetson-wheel-config-20261027-v1`.
- Diagnostic overlay: `~/ros2_timing_diagnostic_v1_ws`; console v2:
  `~/jetson-mapping-console-v2/build/run_mapping_console`.
- Original front mapping install: `~/ros2_front_mapping_v1_ws`.
- Source entry points: `tools/timing_diagnostics/`,
  `tools/lidar_diagnostics/mapping_frame.hpp`, `motion_model.hpp`,
  `run_mapping_console.cpp`, `config/front_mapping_trial.launch.xml`,
  `config/slam_front_experiment.yaml`, `config/controllers_drive_only.yaml`,
  `firmware/nt_opencr/nt_opencr.ino`, `ros2_ws/src/hardware/`,
  `ros2_ws/src/swerve_controller/`, `ros2_ws/src/description/` and
  `ros2_ws/src/bringup/`.
- Current Git branch/HEAD/index and overall worktree changes were not inspected.
  Source files and docs predate this handoff; preserve unrelated/user changes.
  This request updates the existing handoff only; it does not stage or commit it.

Retained evidence is in ignored, local-only directories:

1. `datasets/timing/20261028-stationary-v1/`: complete stationary archive and C++
   review/results/manifests. Stationary qualification passed; no repeat needed.
2. `datasets/timing/20261028-column-v1/`: 72 MiB compressed capture including the
   independent bag/traces, console zSGMLh and checkpoint t1T7Wg; C++ review/fitter,
   results, fit_curve.csv, holds.csv, analysis-method.txt, figures and SHA256SUMS.
   All retained manifest entries were checked. No raw data were uploaded to Notion.
3. `docs/navigation/jetson_mapping_final_roundtrip_review.md`: October 27 vCrfzl
   final-round-trip findings and provenance; its figure is under `figures/`.

Temporary extraction/build locations `/tmp/nt-timing-column-review` and
`/tmp/nt-timing-stationary-review` may disappear. Prefer durable archives above.
Offline reproduction instructions are in the column `analysis-method.txt`;
`review` reads rosbag in READ_ONLY mode and `fit` does not launch ROS nodes.

### Findings to retain without repeating the investigation

- October 27 final vCrfzl: 29 automatic-stop segments, 8,002 independent scans,
  335-scan/414-edge saved graph; 5 cm output reproduced and 2.5 cm rendering
  generated offline. Approximate 800–810 mm passage width lacked an identified
  cross-section; a loose object moved. No absolute dimension/loop-closure claim.
- October 28 column: six automatic-stop legs, no intervention/reposition/scene
  change, 2,533 bag scans, 13,094 bag odometry messages. All seven full traces
  complete; zero logger losses, internal scan gaps or packet checksum failures.
- Correct measured order is **right 660 / bevel 30 / front 755 / bevel 30 /
  left 675 mm**, at beam height; bevel lengths are along sloping faces. Angles
  and complete polygon coordinates are unknown. Robot faced away from column.
- Moving median/p95: odometry-made→publish 0.801/0.973 ms;
  odom-publish→observer 0.432/0.813 ms; mapper-callback→scan-publish
  17.963/27.937 ms. These are software stages, not absolute acquisition delays.
- Raw-ray fit used first two pairs for selection, third for validation. Numerical
  scaled/-15 ms candidate improved fitting by <0.002 mm and validation by
  <0.001 mm; per-leg shifts conflicted. **Not adopted.** Runtime stays nominal,
  assumed delay=0 and acquisition_time_valid=0. Neither turn timing nor absolute
  sensor acquisition nor internal SLAM processing was calibrated.
- Stopped-view apparent wall displacement in wheel odometry accumulated to
  ~8/17/23 mm over the three pairs. This is LiDAR/odom disagreement, not an
  independent true-position error. A constant-pose timestamp shift cannot remove
  it. Do not blame or recalibrate wheels from this evidence alone.
- Battery minimum 10.6 V and an initial steering-limit event were observed.
  Low battery is a possible contributor, not established causation. A charged
  run alone cannot establish that it caused the discrepancy.

### Ordered next work and acceptance criteria

1. **Audit IMU and frame/data ownership offline.** Drive-only currently sets
   `use_imu=false`, wheel `publish_tf=true`; firmware performs startup gyro-zero.
   Inspect export/publication paths, quaternion order, units, stamps/freshness,
   covariance and board-to-base transform. Separate implemented behaviour from
   measured accuracy. Obtain actual board mounting/axis information only where
   it is genuinely missing; do not assume the full-robot URDF pose is confirmed.
   Outcome: a reviewable sensor/frame contract and bounded user-run qualification
   procedure, without activating IMU or changing wheel calibration automatically.
2. **Prepare a separate offline Cartographer comparison.** Check Lyrical package
   availability/build compatibility rather than assuming a ROS 1 example deploys.
   Use the same saved input for both mappers, isolated from hardware command paths
   and existing TF publishers. Audit corrected-scan timestamp/time_increment
   semantics before any subdivision or additional motion compensation. The saved
   column trial has little route diversity; reuse broader vCrfzl evidence where
   appropriate and label its changed-object condition. Initial 2D comparison can
   omit IMU; do not claim all-sensor equivalence until suitable data are recorded.
   Acceptance: input/TF validity, stable tracking, repeat-view geometry and compute
   measurements on identical evidence—not simply a prettier occupancy image.
3. **Combine the next useful recording with one independent spatial check.**
   Charged battery, fixed scene and measured start/return position AND heading
   against fixed references. Record LiDAR, raw wheel data and qualified IMU if
   available. Set actual route/clearance and acceptance thresholds before motion.
   Distinguish true physical return displacement from apparent range/geometry
   drift. Do not repeat the six-leg timing-only trial by default or prescribe
   higher speeds without reviewing the frontend/controller envelopes.
4. **Evaluate fusion after IMU qualification.** Compare wheel-only and wheel+IMU
   results, retain raw measurements, avoid correlated-input double counting and
   give `odom → base_footprint` exactly one owner. An EKF does not itself prove
   correct physical calibration or remove all translation error.
5. **Proceed to supervised Nav2.** One chosen SLAM/localization publisher owns
   `map → odom`; one odometry estimator owns `odom → base_footprint`. Validate
   footprint, directional obstacle coverage, swerve-compatible commands, velocity
   limits and stopping through the actual final command pipeline before goals.
   Front-only mapping does not validate rear obstacle coverage. Keep continuous
   mapping unless the user changes that requirement. Validate a range of speeds
   and turns; do not recalibrate an arbitrary delay for each speed.

No new IMU/fusion/Cartographer/Nav2 implementation or deployment has yet been
completed in this stage. No further physical experiment is currently scheduled.

### Published Notion record and continuation prompt

Created and fetched back successfully as a **private draft**, with both uploaded
figures resolved to persistent Notion media blocks:
[2026-10-27–28 SLAM experiments — timing evidence, spatial discrepancy and the path to Nav2](https://app.notion.com/p/4073980c8a44811481aeff835eda1f94?pvs=204).
Title, corrected measurement order, numerical findings, unapplied correction,
planned integration work and ending appendix were verified structurally by API
readback. No browser-rendered page verification was claimed. Local source figures
were visually inspected. Earlier October 27 Notion page was left unchanged.
Do not publish another duplicate or move/share this draft without user direction.

Paste into the next chat:

> Read `/home/pa4/git/Project2_Turtle_Ninja/docs/navigation/dual_lidar_handoff.md`,
> starting with “Start here in the next session” and the current-state section.
> Resume toward OpenCR IMU qualification, an offline Cartographer comparison and
> Nav2, preserving the working slam_toolbox baseline. The October 28 timing fit
> did not justify a correction; do not repeat completed trials or recalibrate at
> each speed. Preserve continuous mapping, LiDAR setup, assumed wheel calibration
> and zero plate offsets. I execute all hardware and all Git commands; read-only
> SSH is allowed. Begin with offline source/data/frame audits and explain steps
> here. Recheck current files and applicable instructions; do not assume the
> Jetson is powered or that this chat's full history is available.

## Current state — Column timing experiment complete; no correction adopted

Latest user direction after reviewing the results: prioritize an efficient path
to SLAM, Cartographer, IMU calibration and Nav2; avoid recalibrating at every
speed. IMU investigation is now requested, superseding the earlier scope exclusion
of IMU work, but no IMU activation or mounting assumption is authorized by evidence.
Keep slam_toolbox as the working baseline and compare Cartographer separately.
Do not make exact acquisition-delay identification or zero odometry drift a
prerequisite for offline integration and supervised SLAM development. The 23 mm
disagreement needs a bounded independent return-position/range check, not repeated
identical low-speed timing fits. Low battery is a possible contributor, not a
demonstrated explanation. A charged-battery run alone cannot isolate causation.
Use shared recordings for mapper/fusion comparisons, and validate an operating
speed/turn envelope rather than assigning a separate delay to each commanded speed.
Local drive-only config has use_imu=false and wheel publish_tf=true; firmware
performs a startup gyro-zero routine. Neither proves calibrated IMU mounting,
fresh acquisition stamps or valid residual bias. Before fusion, audit these and
assign exactly one publisher to each TF edge. No new runtime change was made.

Both the stationary capture and the six-leg moving column capture are complete.
All data needed for offline analysis were copied before the user was told the
Jetson could be shut down. Actual power-off is not yet confirmed. The moving
capture has no internal trace/scan gaps, but its geometric fit does not justify
a timing correction: the held-out improvement is below 0.001 mm, while the
stationary raw-ray/odometry wall discrepancy changes by approximately 23 mm.
Keep nominal timing, delay 0 and acquisition_time_valid=0; no hardware settings
were changed. See the completed moving capture below. The prior vCrfzl data and
packages remain intact.
At 2026-10-28 08:29:56 KST the user reports a fresh Jetson boot (uptime 4 minutes),
IP `10.2.12.123`, 189 GB free, NTP enabled/synchronized, and no matching wheel,
front mapping, SLAM, console or bag processes. Read-only SSH at 08:30:08 confirmed
the runtime absence, unchanged USB by-path mapping, and no owners of the wheel
`ttyACM0` or front `ttyUSB1` ports. Installed wheel/mapping source paths and audited
source hashes match the local baseline. The user subsequently says all robot
status is confirmed, following the request to confirm motor power, manipulator
connection and placement; accept that confirmation without asking again. NTP
does not establish sensor acquisition synchronization. The user still executes
all hardware and all Git commands. Continuous mapping mode, LiDAR setup, assumed
wheel calibration and zero plate offsets remain preserved.

Original corridor questions, superseded in part by the completed column trial:

1. The user substituted a column for measured corridor inside width/length and
   fixed landmarks. Parallel side walls alone poorly constrain motion along
   the corridor; a tape-measured length does not itself supply a time reference.
2. Robot status is now confirmed by the user. Jetson reboot is established;
   no old pose/runtime continuity is assumed.
3. Existing hardware only (software delay measurements plus relative sensor timing),
   or an independently timed motion reference for absolute acquisition-delay work.

### Findings from source audit

- `config/front_mapping_trial.launch.xml` limits raw timing to 200 input frames.
  `timing_recorder.hpp` also caps 250,000 records / 64 MiB and accepts at most 1,000
  requested frames. Simply increasing the number is not a full-run capture solution.
  Previous corrected-scan bags lack per-ray raw timing for most motion.
- `mapping_node.cpp` estimates per-beam times from host ring timing/source indices,
  then corrects geometry using odometry. It processes on a 20 ms wall timer and may
  wait for bracketing odometry. Its scan stamp is the estimated latest selected
  beam time, not callback or publication time. Record queue/process/publication
  events with the corresponding scan ID; existing raw `publish/published` records
  in the mapping variant describe conversion/callback, not ROS scan publication.
- `SwerveController::update()` stamps odometry with its supplied update time;
  `publish_odom()` publishes it on a separate 50 Hz timer. Header-to-recorder age
  therefore cannot isolate encoder acquisition, USB, controller or publication delay.
- Firmware assigns `State.ms = millis()` after bus read and health work, near state
  transmission. It is not a per-motor acquisition timestamp. Motors are not known
  to sample simultaneously. Existing DIAG summarizes bus/loop/USB timing but does
  not provide an exact acquisition event for each sample.
- `OpenCRLink::poll()` sees board time and host decode timing, but these are not all
  exposed in the recorded odometry. One-way board/host time observations confound
  clock offset and transport delay; do not label minimum arrival age synchronization.
- A position-derived rate option exists with a trailing 60 ms regression window.
  The archived prior drive profile selected `steer_rate_source: reported`; do not
  attribute that filter to the actual run without verifying current selection.
- The inspected LDS-03 decoder provides no optical timestamp. Known geometry and
  informative changing motion can estimate relative LiDAR–odom alignment, not
  uniquely identify both absolute acquisition delays without another timed reference.

### Required preparation before issuing motion commands

Use an opt-in diagnostic build with bounded low-overhead recording for the complete
run. It must retain pre-correction selected/raw rays and source indices, checksum
and scan IDs, packet/scan handoff clocks; mapping queue/process/publish events;
OpenCR state board time plus host read/decode/use events; odometry update/publish
and subscriber receipt events. Use monotonic durations and bracketed ROS/monotonic
clock reads. Correlate events by IDs/stamps, not callback order. Mark overflow,
clock jumps, missing trace events and logger overhead; never silently drop evidence
and call the capture complete. Preserve normal behavior and hardware safeguards.
If SLAM processing duration is claimed, add its callback/processing events too;
map output frequency alone is insufficient. Firmware-level acquisition boundaries
would require a separately reviewed user-flashed change or external timing evidence;
no firmware change has been prepared or authorized as a concrete operation yet.

An opt-in diagnostic overlay is now prepared locally in
`tools/timing_diagnostics/`. `stage.sh` verifies exact baseline hashes and patches
copies into a new workspace; original production source/installations and firmware
are unchanged. It records full-run parser/source/handoff, mapping, wheel and
passive observer events in bounded binary traces, with loss counters and clean
footers. A 20-million-attempt cap applies per file. It measures software stages;
it does not establish optical/encoder acquisition instants or SLAM internal
processing duration. An offline C++ relative-timing fitter now exists in the
retained column analysis directory described below; it has not established a
validated correction.

Local validation: all four overlay packages build; 73 existing hardware/controller
tests pass; parser trace captures 306 synthetic packets including 300 after legacy
capture seals; mapping fixture preserves stale/regressed-input safeguards; passive
observer matches both fixture scans to publication header stamps. Logger queue,
concurrency, cap/loss, no-overwrite and disabled behavior pass; UBSan passed before
the final event-enum addition; inspector rejects missing/truncated footers.
Final staged patched files and logger headers match compiled sources. No robot
nodes, serial access, firmware operation or Git commands were executed.

**Jetson build passed**: the user supplied all four package builds and the trace
CTest result (1/1 passed). Unused NT_FIRMWARE_DIR warnings apply to packages that
do not consume the header; CMake deprecation/policy messages were nonfatal.
Read-only SSH at 08:52 KST confirms no robot/recorder/observer processes or wheel/
front serial owners and 189 GB free. With original wheel local_setup followed by
diagnostic local_setup, hardware/swerve/coin_d4_mapping_driver resolve to the new
diagnostic installation; bringup remains in the original wheel installation.
`ldd` resolves libmapping_sensor.so to the diagnostic workspace. All four copies
of nt_timing_trace.hpp have matching SHA256
`72bfef43c87c16d6993838ed8916f5307bd70716037b00d55a609202b81c6328`.
Installed controller profile still selects `steer_rate_source: reported` and
SLAM remains continuous mapping at 5 cm.

The user scanned a **column protruding from a wall** and supplied its exposed
face lengths. Its visible corners can constrain motion
along the wall. Record actual cross-section at the laser plane (including any
base/skirting), projection perpendicular to wall, along-wall extent and position
from a fixed reference, with uncertainty; do not assume a rectangular cross-section
or unchanged clearance around it. The completed route used the confirmed clearance.

Confirmed geometry/photo: `660, 30 (bevel), 755, 30 (bevel), 675` and
`/tmp/codex-remote-attachments/01a17c84-05ab-71a3-b568-0894083ce378/d0f7cda9-3e22-4472-ac00-7d64ddfad32d/1-1000008571.jpg`.
User confirms mm, measurements at scan height, and **corrected the order after
the trial to right / front / left**: right straight side 660 / right bevel 30 /
front straight face 755 / left bevel 30 / left straight side 675, with straight
lengths excluding bevels. Each 30 mm is measured **across the sloping face**, not
an adjoining-face setback. Keep these as five lengths; bevel angles and full
polygon coordinates are not independently measured. Do not assume 45 degrees or
average 660/675. User says the robot faces **away from the column** and has enough
clearance in response to the combined forward/backward/side/cable question; exact
clearance distances were not supplied. Accept that qualitative confirmation,
while specifying the concrete swept-space requirement for the proposed short
route. The photo shows a power cable/adapter and loops beside the robot near the
right return wall; the operator must keep these outside the whole route. Current
power/manipulator status was already confirmed; do not repeat those questions.

### Completed stationary capture — 2026-10-28 08:58–09:03 KST

The user ran the wait/save step twice within the same session, then completed
normal shutdown and inspection. This extended the recording, not two separate
trials. Both checkpoints succeeded:
`map_20261028T090107.bu1MIb` and `map_20261028T090258.tEfcZy` under the console-v2
`runs` directory. Their occupancy images and both serialized graph files are
byte-identical. The second `tee` replaced only the top-level map-save.log;
checkpoint-local logs and files remain intact. No repeat is required for this.

Read-only SSH at 09:05:10 confirms wheel/front/SLAM/bag/observer processes exited.
Logs show clean shutdown; hardware reports zero wheel command and torque off.
No assistant hardware or Git command was executed. Complete capture/checkpoints
were retrieved read-only and preserved in ignored local directory
`datasets/timing/20261028-stationary-v1/`, with capture.tar.gz, offline C++ analysis
source/CMake, results.txt, build.log and SHA256SUMS. This is evidence retention,
not a teammate delivery. Temporary extraction is `/tmp/nt-timing-stationary-review`.

Analysis results:

- All seven binary files have valid clean footers, zero logger losses/I/O errors,
  and clock brackets consistent with a common system-minus-monotonic offset per
  file. No terminal logger-invalid message is present. Worst observed individual
  logger call is 0.870 ms; this includes scheduling delay and is not an overhead
  A/B measurement or hard realtime guarantee.
- 282.002 s bag: 2,403 scans / 14,025 odometry records, zero command messages.
  Every bag scan matches its producer stamp. All 48 additional producer/observer
  scans occurred after bag stop, with none missing inside the bag interval.
  All 2,451 published scans matched the observer; all 14,468 odometry publications
  matched the observer. All bag odometry poses/twists were zero, and map data did
  not change. These are recorded observations, not independent physical tracking.
- 42,530 LiDAR packets, zero bad checksums; all 982,808 raw-sample provenance
  references validate. 2,453 converted frames have matching raw counts and valid
  source flags. 34 unconverted handoffs are entirely at startup (warmup); no
  internal handoff gaps. Two final frames were not published around shutdown.
  No mapping queue drops. Full tracing continued beyond legacy 200-frame seal.
- 29,025 decoded OpenCR states, no communication-error flags; all hardware-use
  records match unique board-time/sequence keys. Firmware logs show no bus read/
  write failures and battery approximately 11.9–12.0 V. Controller manager lacked
  FIFO scheduling permission, but observed update intervals were 7.585–12.340 ms
  (median 9.999 ms); no timing-policy changes are applied.
- Five stale-odometry rejects occurred together during startup, before first scan
  publication, and did not latch a fault. Receipt backlog reached 305.570 ms in the
  observer / 175.203 ms in the mapper during initialization. For the explicit
  steady window 08:59:54–09:03:32, odom publish-to-observer median/p95/max are
  0.406/0.667/1.685 ms; mapper 0.389/1.013/7.773 ms. Retain the latter outlier;
  do not describe all receipt latency as sub-millisecond. All fault status samples
  are zero; five fresh=0 samples are startup waiting.
- Across the capture, odometry-made to publish median/p95 6.197/6.277 ms;
  mapping callback to scan publish 35.433/44.412 ms (includes scheduling and
  bracketing-odom waits); successful processing-attempt begin to publish
  3.356/3.738 ms; scan publish to observer 0.396/0.793 ms. These are software
  stages, not calibrated optical/encoder acquisition delays. SLAM internal
  processing duration is still uninstrumented.

The stationary evidence supports proceeding to route preparation; it does not
calibrate relative timing at zero motion or validate moving-load behavior.
No need to repeat the stationary capture. Keep wheel calibration, plate offsets,
mounts and nominal timing/delay unchanged. Read-only SSH at 10:38 KST confirms the
same boot, 189 GB free, no relevant runtime processes (the only pgrep match was the
inspection shell itself) and unowned front/wheel ports. Console v2 binary exists.

**Executed user-run trial:** new directory
`~/nt-timing-column-20261028-v1`, same diagnostic overlay and one continuous SLAM
session. Use existing console v2 at fixed 0.025 m/s, three `f 0.20` / `b 0.20`
pairs with no body turns, one command at each READY and 10 s stationary holds.
Forward initially moves away from the column. Reserve first two pairs for fitting
and third for held-out validation; a return is approximate, not a guaranteed
global-pose reset. Operator checks starting-view/heading after each pair and stops
if return error is obvious; never manually reposition during the recording.
Specify 25 cm forward clearance plus 5 cm rear/side margin around the full steering
envelope and keep cables outside the path; numeric clearance is an execution
requirement, not a user-supplied measurement. Wait 35 s after mapper startup and
check fresh healthy stopped streams before starting the console. The existing
console enforces bounded commands, automatic zeros/settling and input rejection;
do not add automatic retries or paste a batch of six commands. No rebuild needed.
After q at READY, save graph/map while independent recorder runs, then close
recorder, mapping, wheel and observer normally. Assess moving trace coverage and
timing identifiability before recommending any correction. Translation-only data
does not validate timing during turns or absolute acquisition delays.

### Completed moving column capture — 2026-10-28 10:53–10:57 KST

The user confirms all six legs stopped automatically, with no intervention,
manual repositioning or scene changes. Console `jetson-mapping-session.zSGMLh`
reports complete=true, six segments, 479 motion / 30 zero publications and no
ignored input batches. Its machine-written physical_stop_confirmed=false is not
a physical finding; the user's subsequent confirmation supplies that observation.
Legs are f/b/f/b/f/b at nominal 0.20 m and fixed 0.025 m/s. Recorded travel is
203.12, -203.83, 203.70, -204.01, 203.79, -204.04 mm in each segment's start frame.
The first forward leg contains 0.01078 rad of heading change; later legs are
approximately straight. Whole-run final odometry differs from the initial pose
by x=-1.261 mm, y=-0.550 mm, yaw=0.010668 rad. These are wheel estimates, not
external physical return measurements. The proposed extra 10 s at every READY
was not maintained; usable post-stop windows of approximately 1.7–5 s remain,
with zero recorded odometry displacement. No repeat was requested.

Remote capture: `/home/turtle/nt-timing-column-20261028-v1`.
Map/graph checkpoint: `/home/turtle/jetson-mapping-console-v2/runs/map_20261028T105722.t1T7Wg`.
Map is 201×216 cells at 0.05 m, serialize result=0. At 10:59:22 read-only SSH
found all relevant processes exited. Logs show normal bag flush, clean mapping
shutdown and wheel zero/torque off. The archive was retrieved read-only; tar
decompression and all checkpoint SHA256 checks pass. SQLite integrity_check=ok.
No assistant hardware or Git command was executed. Durable local evidence is
`datasets/timing/20261028-column-v1/` (ignored): capture.tar.gz, C++ analysis,
results/curves/plot, build log and SHA256SUMS. Extraction/build is in
`/tmp/nt-timing-column-review`.

Coverage and runtime findings:

- All seven trace files have clean footers and zero loss/I/O-error counters.
  Clock-offset bracket intersections are consistent. Worst measured logger call
  is 0.768 ms; this is not an A/B overhead measurement. No queue drops.
- 44,223 packets, zero bad checksums; all 1,022,149 raw source references validate.
  2,552 converted frames are valid. 36 unconverted handoffs are startup warmup;
  no internal handoff gaps. The last converted frame is unpublished at shutdown.
  All 2,551 published scans match observer receipts. All 2,533 bag scans match
  producer stamps; 18 additional scans follow bag stop, none missing inside.
- Independent bag: 263.952 s, 73,218 messages, 13,094 odometry records, 509 commands.
  All bag odometry poses/stamps exactly match the controller trace. The observer
  misses 43 odometry publications before its first receipt, none internally;
  mapper misses 211 at startup and 90 after its last receipt, none internally.
  Three stale-odometry rejects precede first scan, with no later reject increase
  or fault. Five fresh=0 status messages are startup. No communication errors.
- At first motion, controller manager limits an initial steer_rl position step
  (actual 0.181010, target 0, limited 0.131010 rad). This accompanies the small
  first-leg steering/yaw transient, not a runtime abort. Battery warnings occur
  during motion, with logged minimum 10.6 V; no configured 10.2 V stop is reported.
  Do not silently attribute geometric discrepancy to either of these observations.
- Moving windows (first command through first zero + 1 s), median/p95/max ms:
  odometry-made→publish 0.801/0.973/13.944;
  odometry publish→observer 0.432/0.813/6.172;
  odometry publish→mapper 0.420/0.823/7.006;
  mapping callback→scan publish 17.963/27.937/29.833;
  scan publish→observer 0.363/0.568/0.942.
  Across the full run, successful processing-attempt begin→publish is
  3.348/3.547/6.086 ms, and decoded OpenCR state→hardware use is
  0.028/0.036/9.149 ms. Controller update intervals span 5.534–14.486 ms.
  Startup receipt backlog again reaches hundreds of milliseconds; exclude it
  from motion latency. Publication phase differs from the stationary session,
  so do not install the previous 6.2 ms median as a universal odometry delay.

Relative-timing analysis and its limits:

- Reconstruct nominal/scaled per-ray times from original selected rays, source
  indices, host ring anchors and clock brackets; do not fit deskewed scan/map
  geometry back to itself. All 2,551 reconstructed nominal reference stamps
  exactly match producer stamps. Use 100 Hz recorded controller poses, fixed
  LiDAR mount (.110,.005,pi) and unchanged wheel/plate assumptions. A synthetic
  +50 ms shift is recovered exactly, checking the fitter's sign/interpolation.
- Fit nearby strong straight surfaces from the pre-motion stationary baseline;
  freeze surfaces and point associations before searching offsets. The useful
  face normal is approximately (0.99945,0.03309), with 0.583 m visible span.
  That partial span is **not** a measurement of the entire 660 mm side. The
  trajectory does not provide a defensible comparison of all five column faces;
  do not invent the missing bevel angles or infer a range scale from visible span.
- Search signed relative offsets -300..+300 ms in 5 ms increments under nominal
  and period-scaled ray timing; positive means using a later odometry pose than
  the estimated beam timestamp. Equal weight per scan; Huber distance loss with
  5 mm transition. Legs 1–4 select the candidate; legs 5–6 are held out.
  The numerical choice is scaled/-15 ms. Robust distance measure changes
  7.990135→7.988762 mm on training, 12.673862→12.673561 mm on held-out data.
  Nominal/scaled difference is negligible. This is **not a calibrated delay**.
  Individual forward/backward legs favor conflicting shifts (first +165 ms,
  second -215 ms; later optima hit opposite search boundaries), exposing spatial
  bias rather than a repeatable common timing correction. No confidence interval
  or sub-millisecond accuracy is justified by this descriptive fit.
- Frame-mean signed wall residual at rest has median +2.646 mm initially,
  -5.602 mm after pair 1, -13.994 mm after pair 2, -20.453 mm after pair 3:
  approximately 8.25,16.64,23.10 mm apparent wall displacement relative to the
  initial view in the wheel odometry frame. This survives stationary windows,
  where timestamp changes cannot change a constant odometry pose. It demonstrates
  LiDAR/odometry spatial inconsistency, not which sensor or physical effect is
  responsible. Independent position/range evidence is needed to separate actual
  return drift, range/angle bias and other spatial causes. Preserve assumed wheel
  calibration; do not automatically recalibrate it or claim actual wheel slip.
- Absolute optical and encoder acquisition times remain unobservable here;
  OpenCR State.ms is not a per-motor sample timestamp. SLAM internal processing
  is uninstrumented. Keep nominal delay=0 and acquisition_time_valid=0. No new
  physical run, firmware change, teammate archive or resolution change is needed
  to close this experiment. Explain these findings in chat.

### Original experiment rationale

Establish the unchanged corridor's inside-wall geometry, identifiable endpoints,
width at multiple labeled sections and robot reference positions. Keep people and
objects out during recording. Verify current clearances and command ownership.
Record a stationary baseline, bounded forward/stop/reverse segments with the same
heading, and repetitions reserved for validation. Speed changes/start-stop events
provide timing excitation; any optional small opposite turns require full swept
clearance. Exact distances/speeds/repetitions await geometry and the diagnostic
runner. Do not exceed existing input/motion envelopes merely to improve sensitivity.

At the previous 0.025 m/s, a 10 ms timing offset moves the robot only 0.25 mm; a
5 cm grid cannot resolve it. Use raw ray-to-known-wall residuals and motion
transients, not occupancy-map appearance or command-to-motion lag alone. Fit one
explicit relative timing convention, test nominal versus period-scaled per-ray
hypotheses, retain fixed mounting/wheel calibration, and distinguish spatial bias,
range error, smoothing and motion delay from timing. Evaluate candidate offsets
on held-out passes, show objective flatness and uncertainty, and report an
unidentifiable result if the data do not distinguish offsets. Do not force a
nonzero correction or silently apply a signed relative offset as the current
nonnegative LiDAR-only `assumed_delay_ns` parameter. Keep acquisition_time_valid=0
unless evidence supports changing that claim.

The next response should explain this distinction and the actual steps in chat.
Do not create another teammate package. Do not perform hardware or Git commands.

## Previous completed experiment — vCrfzl (2026-10-27)

### Objective and immediate outcome

Reliable continuous 2D mapping remains the project direction, preserving the
existing wheel/LiDAR integration, assumed wheel calibration and zero plate offsets.
The user changed wall panels/passage geometry after G0I8Np, built console v2 and
restarted only the proven combined front-input/SLAM launch for a fresh map
`w9hxqM`, leaving wheel odometry continuous. In `vCrfzl` they completed one round
trip: 29 bounded segments, 5.169 m summed straight travel and confirmed automatic
stops without intervention. Two ordinary-input batches were ignored successfully.

The independent recorder is closed and retrieved: 8,002 scans, all bracketed by
odometry, plus robot description/TF and every console command. The final checkpoint
`map_20261027T200721.aXvQas` saved successfully; its 335-scan / 414-edge graph loads
offline. Exact 5 cm rendering matches the saved map and both bags; 2.5 cm rendering
is available without changing live settings. See
[final round-trip review](jetson_mapping_final_roundtrip_review.md).

The user supplies an approximate passage width of **800–810 mm** and reports
**a loose object moved on return**. The precise measured cross-section and long-wall
length are unspecified. An illustrative central grid opening is 775–800 mm at
2.5 cm, but it cannot be assigned an absolute error against an unidentified
physical section. Do not claim unchanged-scene revisit, full coverage, verified
loop closure, dimensional accuracy or navigation readiness. No further driving
is requested; finish with the available evidence and preserve these limitations.

| Capability | Evidence / remaining limitation |
| --- | --- |
| Bounded motion and input handling | 29 v2 segments and user-confirmed automatic stops; accidental input did not halt the run |
| Moving front input | All 8,002 independent scans odometry-bracketed; acquisition timing remains provisional |
| Online SLAM | Continuous mapping within fresh w9hxqM; initial restart was for changed structural geometry |
| Map persistence / finer grid | 335-scan archive loads; 5 cm pixels reproduced; offline 2.5 cm ready; live restore remains untested |
| Physical map dimensions | Approximate width only, unmatched section; long-wall length absent; loose object moved |
| Orientation integration | Configured yaw pi retained; base-frame RViz front-box check never performed |
| Navigation / dual moving LiDAR | Still unvalidated; this session uses front only |

SLAM already estimates pose in its map. Live restore/relocalization after a restart
is a separate untested capability. Do not impose AMCL, fixed-map operation, IMU
fusion or wheel recalibration as a new prerequisite. Preserve continuous mapping
as the intended mode. The user may shut down at this final experiment closeout;
shutdown does not establish that a saved graph can resume live without validation.

### User boundaries and preserved decisions

- **User executes all hardware and all Git commands**, including read-only Git.
  Assistant may inspect/retrieve through read-only SSH, edit local project files
  and run offline/mock tests. No remote deployment, installation, process start/stop,
  serial access, CONFIG, torque, motion, flashing, Git staging or history changes.
- Explain findings and user-run steps in chat. Do not rely on the user reading a
  report. Preserve existing unrelated work. No subagents unless requested; use
  project C++ conventions for implementation, with necessary ROS launch glue.
- Wheel odometry is assumed calibrated by explicit user instruction. Plate X/Y/yaw
  are confirmed zero. These decisions do not need reconfirmation.
- Keep existing driver settings, USB sockets, mount geometry, body filter, wheel
  limits and package layout. Preserve original stationary profiles as recovery
  paths. Do not import a colleague's full controller/SLAM stack.
- Historical setup confirmation: motors powered, manipulator disconnected; both
  caGdIe route legs stopped completely on their own, without intervention, manual
  repositioning or scene change. The preceding JHydTT/gcQZct motions also passed.
  After a break/reboot/relocation, ask only about potentially changed physical
  state and clearance. Do not assume this historical confirmation describes now.
- Full robot launch, teleoperation, Nav2 and old motion coordinators were not part
  of this trial. Keep command ownership explicit. The assistant never initiates
  hardware even when the user says "proceed".
- Current branch, HEAD, index and complete change inventory are **not verified**:
  the user reserves all Git commands. This handoff updates the existing document;
  it does not establish that previous changes are committed or staged. Old Git
  snapshots later in this file are historical, not current checkout state.

### Final run, artifacts and closeout

- Fresh map session: `/home/turtle/jetson-map-dimensions.w9hxqM` (about 19:55 KST).
- Console: `/home/turtle/jetson-mapping-session.vCrfzl`, complete, 29 segments,
  474.020 s, 2,533 motion / 145 zero commands, 4,754 scans. Five zeros per segment
  verified. User confirms automatic stops without intervention; raw
  `physical_stop_confirmed: false` is retained separately from human confirmation.
- Independent recorder: `~/jetson-mapping-console-v2/runs/bag_20261027T195454_502949284`,
  starts before fresh mapper, includes its first node, closes at about 20:08:27.
  SQLite integrity, metadata counts and remote/local hashes pass. One console
  scan lacking preceding console odometry is covered by this longer recorder.
- Final checkpoint: `~/jetson-mapping-console-v2/runs/map_20261027T200721.aXvQas`;
  map 92 × 35 at 5 cm, graph save result zero. Offline 2.5 cm map is 184 × 69.
- Earlier pre-restart `map_20261027T195401.yA4OhZ` can contain mixed old/new
  observations. It is not a clean baseline for the revised walls.
- Local review root: `/tmp/jetson-final-roundtrip-vCrfzl`.
- Durable handoff directory:
  `/home/pa4/.codex/visualizations/2026/10/27/01a17c84-05ab-71a3-b568-0894083ce378/wheelbase-handoff/`.
  New `wheelbase-final-roundtrip-20261027-v1.tar.gz` contains both final bags,
  checkpoint, both rasters, analysis and explicit user-observation/orientation limits.
  Earlier source/data archives are unchanged; nothing was sent to the teammate.

Console and recorder have exited. The last read-only check still showed wheel
launch PID 4948 and fresh mapping launch PID 16782. Shutdown instructions were
given: user Ctrl+C mapping, wait for prompt, Ctrl+C wheel launch, wait for prompt,
then motor power off; normal Jetson OS shutdown before removing Jetson power.
Actual subsequent shutdown is unconfirmed. Do not restart anything automatically.
No further experiment or live resolution change is planned.

### Previous request — console input and wheel-base teammate delivery

The user requested accidental-input tolerance and files satisfying the teammate's
independent recording / map-save / LiDAR-yaw report workflow. The user has now
transferred and built console v2 on Jetson; no assistant deployment or hardware/Git
command occurred. Select v2 for new runs; v1 retains its old any-key-stop behavior.

V2 discards ordinary input while busy and flushes queued terminal bytes on entering
READY; invalid/overlong READY commands recover without ending the session. Ctrl+C,
actual EOF/hangup, signals and faults still stop. It permits one explicitly named
passive `/team_mapping_recorder` subscriber alongside the mandatory wheel subscriber;
extra command publishers, missing wheels and unexpected consumers still block motion.
Speeds, segment bounds, settling and sensor guards remain. Local build/mock tests
and validation logs are included in the v2 source bundle.

New `tools/lidar_diagnostics/team_capture/` supplies separate recorder, bounded
teleop wrapper, checked occupancy/graph saver, Korean instructions and frame report.
Use existing wheel/input/SLAM processes; do not start the teammate's original
`run_scan.sh`, a second LiDAR driver or their continuous key teleop. For the completed final run, the user saved while stopped and closed the console
and recorder. NJWQJ7 was replaced by fresh w9hxqM for the changed layout.

Prepared archives and SHA256SUMS are under
`/home/pa4/.codex/visualizations/2026/10/27/01a17c84-05ab-71a3-b568-0894083ce378/wheelbase-handoff/`:
`jetson-mapping-console-20261027-v2.tar.gz` and
`wheelbase-mapping-data-20261027-v1.tar.gz`. A convenience console copy is in `/tmp`.
Data includes G0I8Np, seven explicitly incomplete earlier attempts, original
maps/graphs, 5 cm / 2.5 cm rasters, analysis and runtime launch parameters/URDF.
SQLite integrity and metadata counts were checked for all eight supplied bags.
Original records are unchanged. Nothing has been sent to the teammate.

**User says the base-frame RViz front-box check has not been performed.** Active
yaw is pi; only with identical axes/scan conventions is the equivalent absolute
`NT_SCAN_YAW=3.141592653589793`. It is not yet a physically verified result. Keep
the running TF unchanged; report front-object +X and left-object +Y observations
before teammate URDF integration. The bag is ready now; no new traversal is needed
merely to supply offline data. See [delivery preparation](wheelbase_mapping_delivery.md)
and [orientation report](../../tools/lidar_diagnostics/team_capture/ORIENTATION.md).

### Earlier area session — G0I8Np (prior layout)

User transcript: `/home/pa4/.codex/attachments/976b09a5-23b3-47eb-836f-33a0ffba2e19/Pasted text.txt`.
Recording `/home/turtle/jetson-mapping-session.G0I8Np`, about 18:44–18:57 KST;
checkpoints `/home/turtle/jetson-area-checkpoints.xFPjny`. User confirms all 48
segments stopped on their own, no intervention/repositioning/scene change during
this recording, and passage revisit. They also confirm earlier commanded runs.
Seven earlier console captures account for eight started segments and connect
recovery odometry to G0I8Np's initial pose; their results are incomplete and exit
reasons are not supplied. Do not describe the whole interval as one completed run.

G0I8Np: 759.320 s, 20 straight / 28 turns, 5.757 m summed straight travel, 3,725
motion / 240 zero commands, 7,610 scans all bracketed, maximum odom gap 34.191 ms
and command gap 112.882 ms. No new rejected input, fault/stale status or nonfinite
joint feedback. `rejected=12749` is historical and stable. Map save succeeded and
graph serializer returned zero. The last bag map equals after.pgm, 137 x 94 at
5 cm (548 occupied / 2,634 free / 9,696 unknown). Its bounds include sparse rays
and unknown space; they do not establish that the entire arena is covered.

Zero loop-closure events and zero pose-graph topic messages were captured; 597
new-node events were recorded. The final archive has 702 scans / 1,092 edges,
versus 13 / 18 in the before archive. Neither edge count nor passage revisit is
proof of a loop closure. The map->odom transform changed by up to 0.545 m /
21.696 degrees from the recording's initial value, ending at a 0.116 m / 6.540
degree difference. These are diagnostic transform changes, not ground-truth error.
Keep wheel calibration assumed; compare physical dimensions before navigation.

Desktop data and checks: `/tmp/jetson-mapping-area-G0I8Np`; earlier attempts:
`/tmp/jetson-mapping-area-earlier-attempts`. SHA-256 hashes and all metrics are in
[area review](jetson_mapping_area_review.md). The new local C++ tool
`tools/lidar_diagnostics/render_mapping_graph.cpp` loads both archives and renders
without ROS nodes or new optimization. Both before/after 5 cm renders match the
saved pixels exactly. 2.5 cm output: 274 x 187, retained in
`/tmp/jetson-mapping-area-G0I8Np/graph-resolution/graph_2p5cm.{pgm,yaml}`.
Build and input/output refusal checks passed. This is archive/raster validation,
not live SLAM continuation or relocalization validation.

The former proposal to continue coverage and measure dimensions is superseded
by the final vCrfzl round trip above. G0I8Np is retained as prior-layout evidence;
do not resume it as the current scene. Read-only at 18:58–18:59 found input healthy,
published=59459/rejected=12749, no command publisher and battery about 11.8 V.

### Earlier recovery — motor power and drive reactivation

**Power interruption recovered; do not repeat the recovery commands now.** The
user's `bus_drive active` and controller activation both succeeded at about 18:37.
User output confirms hardware and both controllers active, finite stationary
odometry x=0.282311615 m, y=0.011141051 m, yaw=0.129138300 rad, and healthy input.
Position differs from caGdIe's last recorded odom by only 0.0285 mm and yaw by
-0.10464 degrees. This supports retained odometry rather than a reset; caGdIe was
not an immediate pre-power-off sample, so it is not a precise recovery-error test.
Read-only SSH at 18:38–18:39 confirms the same wheel/input/mapper processes,
active SLAM, published=47400 then 47420, rejected=12749 unchanged, fault=0/fresh=1,
zero command publishers, battery about 12.1 V (recent minimum 12.0 V), successful
motor reads/writes in recent diagnostic windows and 190 GB free. The large
rejection total accumulated during the outage; it is not a current fault.
Resume the existing console with user-selected cleared segments, meaningful
revisits and before/after checkpoints. Recheck clearances after the object move.
No repeat of caGdIe and no additional stationary-residual gate are required.

Incident evidence: read-only SSH at 18:30–18:36 found
the same wheel/controller and NJWQJ7 mapper processes. At 18:16:32, all eight drive
and steering motors stopped responding; the hardware error invalidated feedback,
stopped odometry and left `bus_drive` unconfigured, with `swerve_controller` and
`joint_state_broadcaster` inactive. The user confirms deliberately removing motor
power. Battery had also fallen to about 10.4 V, below the configured 10.8 V warning;
the logged stop was missing motor responses, not the 10.2 V battery-stop condition.
Use a charged motor supply for further coverage. The robot stayed put, but a
physical object was moved after caGdIe, so old clearances/scene comparisons no
longer describe the whole present scene.

Mapping status remained `fault=0 fresh=0`, `published=46649`, with increasing
rejections and `reason=missing/fresh odometry unavailable`. This is a nonlatched
input freshness failure, not a change to the nominal timing hypothesis.
1URnqw's result records zero segments, zero motion publications, zero scans and
one map. `/cmd_vel` had zero publishers. Do not weaken the console's input guards.

The user saved `/home/turtle/jetson-area-checkpoints.xFPjny/before.{pgm,yaml}` and
`before_graph.{posegraph,data}` at 18:29. The PGM is byte-identical to caGdIe's
after map. Desktop backup, including the failed session and controller log:
`/tmp/jetson-mapping-fault-1URnqw` and `.tar.gz`. Graph SHA-256:
`d53d7e1816f44d86429ff1c5f16cc4d9e34612966ab1d01fdbb49b50ca249764`;
data SHA-256: `0c3695bcf05e85f80097e215c97c1dbb05977820ccc8c27875d03ff2d6e172b2`.
Serialization is now evidenced; restore remains untested. Exact-graph resolution
analysis can use this checkpoint later, without restarting the live map.

Recovery procedure is in [MAPPING_AREA.md](../../tools/lidar_diagnostics/MAPPING_AREA.md#recovering-the-confirmed-motor-power-interruption).
The user restored motor power, requested `bus_drive active`
and activated the two existing controllers. The inspected ros2_control 6.9.0 source
configures unconfigured hardware before activating it in the same request. The
deployed swerve `on_activate` clears commands but preserves `pose_`, and sets the
first integration interval to zero. Do not unload/reconfigure the swerve controller
or restart its launch: those recreate its odometry object. Keep input and mapper
running. This recovery now has user execution and renewed odometry/input evidence;
it does not validate restart/relocalization or recovery after moving the base.

After discussing the purpose of the small tests, the user said "Proceed" with
whole-area mapping preparation and offline 5 cm / 2.5 cm evaluation. Do not impose
an additional stationary-residual gate. The next milestone is useful broad map
coverage and meaningful revisits under user-selected, visually cleared movement.

`tools/lidar_diagnostics/run_mapping_console.cpp` is a separate supervised console:
starting it only records/waits. At READY the user chooses f/b 0.05–0.30 m or l/r
10–30 degrees. Speeds stay 0.025 m/s / 0.10 rad/s; each finite segment ends with
five zeros, settling verification and two stationary seconds. No automatic next
segment or retries. Local v2 discards ordinary busy input; Ctrl+C, SIGTERM/SIGHUP,
actual EOF/disconnect or a health/bounds fault requests abort zeros and exits.
Previously deployed v1 aborts on any busy input. User-visible real
clearance must be checked for each straight or swept turn envelope. No blind
whole-maze route is inferred from the photo, and no autonomous navigation is added.

Protocol: [MAPPING_AREA.md](../../tools/lidar_diagnostics/MAPPING_AREA.md), including
user-run source transfer/build, occupancy and serialized pose-graph/data checkpoints,
command meanings, stopping, limits and recording. Console preflight refusal is
verified; G0I8Np now demonstrates console-driven physical motion. Live graph restoration remains pending. The original sensor, wheel and mapper profiles are
unchanged; NJWQJ7 stays live at 5 cm. No assistant hardware or Git command occurred.

C++17 warnings-as-errors build and 14 motion/fault scenarios passed for **v1**. That
version also passed a normal scenario after adding graph/joint-state recording,
and pseudoterminal input/hangup checks. Local logs:
`/tmp/lidar-mapping-console-build/test-output.txt` and `test-final-output.txt`.
Source bundle: `/tmp/jetson-mapping-console-20261027-v1.tar.gz`.
SHA-256: `4c402eea3ff6c501b6ced9581622adc400d637158956828e99c0443819831772`.

The [offline resolution comparison](jetson_mapping_resolution_review.md) uses all
226 caGdIe corrected scans at identical frozen odometry-derived poses, with one
final map transform. 5 cm: 1,023 cells; 2.5 cm: 4,158 cells. It demonstrates grid
representation, not improved accuracy or an exact recreation of NJWQJ7's optimized
graph. The exact-graph comparison from xFPjny is now complete; see the current area review. Live
resolution is configuration-cached in the inspected 2.10.0 source; do not assume
`ros2 param set` alone rerenders the map. Console accepts either 5 or 2.5 cm but
rejects changes within a recording. Do not restart NJWQJ7 to try finer cells now.

At about 18:16 KST read-only SSH found the same NJWQJ7/wheel launches, healthy
input published=46357/rejected=4/fault=0/fresh=1, zero command publishers, one
swerve subscriber and 190 GB free. serialize_map service, pose_graph/new_node_event/
loop_closure_event and joint_states topics were present. Later execution must
still pass console preflight; these observations are not perpetual guarantees.

### Completed route — caGdIe

User confirms unchanged scene/placement and no reboot since gcQZct; motors powered,
manipulator disconnected. Front faces the nearby brown box. Clearances from the
robot envelope: front 250 mm, rear 450 mm, at least 50 mm on each side of the full
steering envelope along the route; cables clear. These answers supersede the need
to ask again unless the state changes.

C++17 warnings-as-errors build and 16 local ROS scenarios passed. Source bundle:
`/tmp/jetson-mapping-route-20261027-v1.tar.gz` (disposable local path).
SHA-256: `c5ee49e668ca93cc7db3eca7033d5ed190ca5a0ae6db15cc2a96016978ab3e54`.

The separate `run_mapping_route` reverses at 0.025 m/s up to about 100 mm, stops,
records five stationary seconds, then returns toward the initial view at the same
speed and records another five stationary seconds. Each leg is capped at 40
commands and 4.3 seconds to first zero; five zeros follow each leg. The return
stops near 5 mm behind the initial odometry pose, with no extension or correction.
Global odometry bounds: -150 to +20 mm forward, 20 mm lateral, 0.050 rad yaw;
final acceptance: <=20 mm position and <=0.030 rad yaw. Software bounds are not
physical stopping guarantees. The user executed it successfully in caGdIe and
confirmed both automatic stops, no intervention and unchanged scene.

The completed procedure is retained in
[MAPPING_ROUTE.md](../../tools/lidar_diagnostics/MAPPING_ROUTE.md); do not repeat it.
Route data: `/home/turtle/jetson-mapping-route.caGdIe`; snapshots:
`/home/turtle/jetson-map-route-snapshots.UWhkln`. Reverse travel 103.680 mm; final
odometry offset 0.331 mm backward / 0.0285 mm lateral / 0.05750 degrees yaw. All
226 scans bracketed; rejected=4 unchanged. Route map correction increment is
15.592 mm / 1.800 degrees; cumulative NJWQJ7 correction 22.363 mm / 3.199 degrees.
Major coarse-map boundaries remain consistent, but revisit RMS is elevated.
See the route review for limits and raw-evidence hashes. Preserve the
v2 single-pulse runner and all running sensor/wheel/mapper configurations. The
offline reviewer now supports `--reverse-return` with sibling events.csv and five
phase groups; unchanged single-pulse mode still rejects this route. All Git operations remain user-only.

Final read-only check around 17:52 KST: the same wheel/NJWQJ7 processes remain;
input published=32290, rejected=4, fault=0, fresh=1; `/cmd_vel` has zero publishers
and one swerve subscriber; 190 GB remains available. No hardware action occurred.

### Last verified runtime and machine identity

Desktop repository: `/home/pa4/git/Project2_Turtle_Ninja`.
Jetson: `turtle@10.2.12.123`, aarch64 Ubuntu 24.04.5, ROS Lyrical at
`/opt/ros/lyrical`. Package versions previously verified on Jetson:
slam_toolbox `2.10.0-2noble.20260819.073924`, nav2_map_server
`1.5.0-1noble.20260819.052057`. Full Nav2 package availability has not been freshly
inventoried; installed packages alone do not establish integration.

| Role | Jetson path / setting |
| --- | --- |
| Wheel-only workspace | `/home/turtle/ros2_wheel_config_v1_ws` |
| Wheel source | `/home/turtle/jetson-wheel-config-20261027-v1` |
| Front mapper workspace and launch | `/home/turtle/ros2_front_mapping_v1_ws/config/front_mapping_trial.launch.xml` |
| Live-map motion runner | `/home/turtle/jetson-mapping-motion-v2/build/run_mapping_motion` |
| Current map session | `/home/turtle/jetson-map-rotation.NJWQJ7` |
| Original stationary driver | `/home/turtle/ros2_lidar_experimental_ws`, domain 181 |
| Original body view | `/home/turtle/ros2_lidar_body_view_v2_ws`, `/home/turtle/lidar_body_view_v2.yaml` |
| Wheel + mapper domain | `ROS_DOMAIN_ID=182`, `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` |
| Inherited peer observed | `ROS_STATIC_PEERS=10.2.12.120`; LOCALHOST alone is not proof of graph isolation |
| Drive USB | `/dev/serial/by-path/platform-3610000.usb-usb-0:2.4.3:1.0` |
| Front USB | `/dev/serial/by-path/platform-3610000.usb-usb-0:2.4.4:1.0-port0` |
| Rear USB | `/dev/serial/by-path/platform-3610000.usb-usb-0:2.4.2:1.0-port0` |

Last post-gcQZct read-only inspection found wheel launch PID 4948, map launch 8963,
front input 8982 and mapper 8983 still running; mapper lifecycle active [3]. Input
status was published=7825, rejected=4, fault=0, fresh=1. `/cmd_vel` had zero
publishers and one swerve subscriber; the finite motion runner had exited. A fresh read-only check around 17:40–17:41 KST found the same processes and active
mapper, input published=24975/rejected=4/fault=0/fresh=1, no command publisher and
one swerve subscriber. Six seconds yielded 301 odometry samples and 60 scans;
map->odom remained at the gcQZct value. Storage had 190 GB available. Reinspect
after a break; do not kill or restart by old PID.

The mapping input owns the front serial port. Never run the original front driver
or old `run_motion_check` alongside it. The v2 **mapping** runner observes existing
streams and publishes a bounded command sequence; it does not open a sensor port.
The rear serial port had no observed owner at the fresh inspection; preserve its
configuration rather than starting it as part of this route. Shared LiDAR USB serial IDs are
nonunique; keep the by-path socket mapping.

### Active pipeline, geometry and timing contract

`wheel feedback -> /odom + odom->base_footprint`

`front sensor + bracketed odometry -> /lidar_front/scan_mapping -> slam_toolbox`

`slam_toolbox -> /map + map->odom`

The launch supplies the sole `base_footprint -> lidar_front_link` transform:
translation **(+0.110,+0.005,+0.0545) m**, yaw **pi**. The scan height is estimated
from 0.028 m mounting-surface height plus 0.0265 m scan-plane height. Plate offsets
remain zero. Wheel `base_link` is 0.033 m above ground-level `base_footprint`; do
not substitute it without accounting for that Z difference. Rear plate coordinates
remain (-0.110,+0.005,+0.0265) m, yaw zero; existing front->rear is (+0.220,0,0),
yaw pi. Assign one owner to each TF child and never add a second mapper TF owner.

Accepted body filtering uses a 0.2552 x 0.3050 m steering envelope plus 0.010 m per
side: front-frame X [-0.0276,0.2476], Y [-0.1575,0.1675] m. It is not a validated
Nav2 collision footprint for every steering/manipulator configuration.

- Mapping uses the front source only, `timing_model=nominal`,
  `assumed_delay_ns=0`, **`acquisition_time_valid=0`**. Zero delay is a hypothesis,
  not a calibrated optical acquisition/transport delay.
- Accepted source provenance and body filtering precede motion correction and
  guarded resampling. Output is 1,600 bins, with at most the original 400 selected
  returns. Unknown/body/invalid bins are NaN; no invented infinity or gap filling.
  Source ordering/seams prevent treating the raw data as uniform scan_time/400.
- Input acceptance limits: odom gaps <=40 ms, translation <=0.04 m/s, yaw rate
  <=0.15 rad/s; freshness and clock checks also apply. Input rejection withholds
  scans; it does **not** itself stop motors. Actual pulse speeds tested here are
  0.025 m/s and 0.10 rad/s. Software caps are not a validated speed envelope.
- Wheel limits remain 0.10 m/s and 0.5 rad/s; IMU fusion remains disabled.
  `/cmd_vel` uses `geometry_msgs/msg/TwistStamped` end to end in this experiment.
- Mapper uses system time, 5 cm resolution and scan matching. Loop closure is
  enabled in configuration but has not been experimentally demonstrated.
- The 200-frame raw timing file sealed before the two live-map movements. Later
  bags contain corrected output, not raw acquisition provenance during motion.

### Latest physical evidence and what it means

Same map session NJWQJ7; user confirms no intervening manual movement or scene change.

| Measurement | JHydTT turn, about 17:01 KST | gcQZct forward, about 17:10 KST |
| --- | ---: | ---: |
| Commands | 20 x +0.10 rad/s, then five zeros | 40 x +0.025 m/s, then five zeros |
| Odometry displacement | 7.4063 degrees yaw; max 0.0583 mm translation | 84.389 mm forward; 0.044 mm lateral; 0.01149 degrees net yaw |
| First nonzero to first zero | 2.081685 s | 4.166129 s |
| Corrected scans / odom samples | 106 / 527 | 140 / 616 |
| Maximum odom gap | 30.009 ms | 20.198 ms |
| Incremental map->odom correction | 3.235 mm / 0.600 degrees | 4.970 mm / 0.800 degrees |
| Input status | rejected=4 unchanged, fault=0, fresh=1 | rejected=4 unchanged, fault=0, fresh=1 |
| Physical stop | User confirms smooth turn and complete automatic stop | User confirms forward movement and complete automatic stop |

The prior after-rotation PGM equals the before-forward PGM byte for byte, with
matching YAML geometry/settings. Last JHydTT odometry/map->odom equals first
gcQZct values. Before/after saves match their corresponding recorded bag maps.
These support continuity across the gap; the finite bags do not record the entire
approximately nine-minute interval continuously.

Cumulative map->odom correction through both motions is **8.201 mm / 1.400 degrees**,
stable after stopping. This is SLAM's correction to odometry placement, not measured
ground-truth error. Occupied/free/unknown counts progress 68/405/519 -> 103/547/373
-> 121/562/340. Final grid is 31x33 at 5 cm, approximately 1.55x1.65 m, with unknown
space remaining. More known cells do not establish collision clearance.

For gcQZct, 123/140 scans have odometry brackets; 17 initial preflight scans precede
the first odometry stamp and are excluded from geometry without extrapolation.
All command-interval scans remain covered. Holdout/motion/post fixed-scene RMS is
4.627/9.560/11.508 mm under fixed association gates. These are exploratory local
repeatability measures, not absolute map accuracy or timing calibration. The final
five-second post phase has exactly constant recorded odometry; physical stopping
is separately supported by the user. Keep raw `physical_stop_confirmed=false`
fields intact; document the later human observation without rewriting captures.

Both pulses show about 0.85 s before substantial body motion. Steering alignment
is plausible, not proven without joint-state evidence, and this is not a measured
LiDAR delay. Do not extend/repeat pulses to force nominal travel or reopen wheel
calibration. The clean forward-only AEjKu5 trial is earlier corroborating evidence.
The much larger correction in older gUGiCK was confounded by user-confirmed manual
repositioning; it does not establish a clean SLAM failure.

### Evidence, source and reproducibility

Read in this order:

1. This current-state section, then [combined sequence review](jetson_mapping_turn_forward_review.md).
2. [MOTION.md](../../tools/lidar_diagnostics/MOTION.md) and
   [MAPPING.md](../../tools/lidar_diagnostics/MAPPING.md), starting with their latest status.
3. [MAPPING_MOTION.md](../../tools/lidar_diagnostics/MAPPING_MOTION.md) and
   [MAPPING_ROTATION.md](../../tools/lidar_diagnostics/MAPPING_ROTATION.md) for runner
   behavior and completed procedures, not instructions to repeat them.
4. [Rotation review](jetson_mapping_rotation_review.md),
   [clean earlier forward review](jetson_mapping_newscene_review.md) and
   [confounded first moving-map review](jetson_mapping_motion_review.md) as needed.

| Evidence | Jetson location |
| --- | --- |
| Current initial timing and launch logs | `/home/turtle/jetson-map-rotation.NJWQJ7` |
| Rotation bag/CSV/events/result | `/home/turtle/jetson-mapping-motion.JHydTT` |
| Rotation baseline/after snapshots | `/home/turtle/jetson-map-rotation-snapshots.sIdcDI` |
| Forward bag/CSV/events/result | `/home/turtle/jetson-mapping-motion.gcQZct` |
| Forward before/after snapshots | `/home/turtle/jetson-map-turn-forward.IcK9xK` |
| Earlier clean forward | `/home/turtle/jetson-mapping-motion.AEjKu5`, map session `jetson-map-newscene.nEKS7T` |
| Earlier raw moving timing captures | `/home/turtle/jetson-motion-auto.9RvB4b` and `/home/turtle/jetson-motion-auto.H9GTjD` |

Desktop retrievals are `/tmp/jetson-turn-forward-gcQZct/review`,
`/tmp/jetson-rotation-map-JHydTT/review`, and `/tmp/jetson-moving-map-AEjKu5`.
These directories and `/tmp/jetson-mapping-motion-20261027-v2.tar.gz` existed when
preparing this handoff, but /tmp is disposable and not portable to a new host.
The sequence figure is [saved in the repository](figures/jetson_mapping_turn_forward_gcQZct_20261027.png).
Detailed reviews preserve capture hashes; raw captures remain on Jetson and are
not embedded in this handoff. Do not assume a new chat has access to either machine.

Key repository files:

- `config/{front_mapping_trial.launch.xml,slam_front_experiment.yaml,lidar_mounts.yaml,lidar_body_view.yaml}`.
- `config/{hardware_map_drive_only,controllers_drive_only}.yaml` and
  `ros2_ws/src/bringup/launch/drive_only.launch.py`.
- `tools/lidar_diagnostics/{mapping_frame.hpp,mapping_scan.hpp,live_mapping.patch}`:
  separate guarded mapping input; preserve original driver implementation.
- `tools/lidar_diagnostics/{run_mapping_motion.cpp,mapping_trial_core.hpp,test_mapping_motion.cpp,review_mapping_motion.cpp}`
  and `mapping_motion/CMakeLists.txt`: bounded runner, desktop fixture and offline reviewer.
- `tools/lidar_diagnostics/{check_offline.sh,motion_model.hpp,replay_motion.cpp,replay_timing.cpp}`:
  earlier timing/provenance/motion analysis.

Previous validation includes C++17 warnings-as-errors builds, offline/UBSan checks,
raw replay/parser checks and 20 desktop ROS runner scenarios covering both pulse
modes and faults. Synthetic checks do not establish physical performance. The
physical evidence above is separate. The separate route implementation and its local validation are documented in
MAPPING_ROUTE.md. For later code changes, use the focused commands in the tool
guides; desktop mock ROS uses domain 193/LOCALHOST with `ROS_STATIC_PEERS=` and
test-only topic names. Never accidentally run the hardware runner as a test.

The v2 source bundle's recorded SHA-256 is
`4aa39c824e8aff1041a9eec79a31012c07657e950981a323a1baa987eaf2e0ce`.
Its Jetson source manifest and accepted launch/config hashes previously passed.
A session change alone requires no reinstallation, rebuild or transfer.

### Historical ordered next actions and acceptance — superseded by Start here

1. **Re-establish present state without changing it.** Read applicable instructions
   and current files. If time/power/scene changed, obtain the user's physical state
   and inspect processes, ports, ROS domain/peers, active mapper, fresh odom/scan/TF
   and unique command/TF ownership using read-only SSH. Last PID/status values are
   evidence, not commands. Preserve NJWQJ7 and placement if still valid. If rebooted
   or manually repositioned, select the documented restart/baseline procedure for
   the actual state; do not pretend continuity survived an odometry reset.
2. **Assess any remaining coverage.** G0I8Np completed supervised area mapping.
   Identify remaining reachable unobserved passages; continue only as needed under
   MAPPING_AREA.md with current physical clearance and new checkpoint filenames.
3. **Check physical consistency.** G0I8Np health/stops and saved data are reviewed.
   Compare a long wall and passage width to physical measurements and investigate
   sparse outside observations. A revisit occurred; loop closure is not established.
   The earlier residual is not a requirement to repeat tiny movements.
4. **Use the completed resolution comparison.** Exact before/after graph renders
   at 5/2.5 cm are complete; 5 cm pixels equal the saved maps. Preserve live NJWQJ7
   and validate live continuation/restore before changing its rendering profile.
5. **Add persistence before claiming restart continuity.** Save occupancy snapshots
   plus serialized pose graph/data, then validate restoration and pose recovery.
   PGM/YAML alone cannot resume the same mapping graph. Keep continuous mapping as
   the selected path; saved-map AMCL is an optional future mode, not a forced switch.
6. **Integrate navigation after the mapping evidence is adequate.** Configure the
   physical footprint, obstacle marking/clearing from correct sensor origins,
   swerve motion/steering transitions, TwistStamped interface and command arbitration.
   Validate one low-speed goal and cancellation/stopping through the actual Nav2
   pipeline, then obstacle/replanning/multiple-goal behavior. Do not equate the
   diagnostic runner's guards with validated Nav2 fault handling. Separate front/rear
   obstacle observations can preserve their physical origins; a merged cloud is
   not automatically a valid single-origin clearing scan.

Authoritative architecture references consulted during the assessment:
[Nav2 mapping/localization](https://docs.nav2.org/rolling/configuration_and_development/first_time_robot_setup_guide/sensors/mapping_localization/)
and [SLAM Toolbox persistence](https://github.com/SteveMacenski/slam_toolbox#lifelong-mapping).
Consult installed Lyrical interfaces before turning upstream examples into commands.

### Historical resume prompt — use the prompt in Start here instead

> Read current state, jetson_mapping_area_review.md and MAPPING_AREA.md. NJWQJ7
> includes successful JHydTT/gcQZct/caGdIe, in-place recovery after motor power-off,
> and G0I8Np: 48 completed segments, 5.757 m straight travel, confirmed automatic
> stops and a passage revisit, no scene changes during that run. An object had
> moved earlier, and separate commanded attempts preceded G0I8Np. xFPjny saves
> and exact offline 5/2.5 cm graph rendering are verified; live restore is not.
> No loop-closure event was captured; SLAM/odom correction varied substantially.
> Next assess remaining coverage and physical dimensions, then validate mapping
> continuation/restore before a live resolution change. No repeated tiny-test gate.
> Preserve NJWQJ7, LiDAR settings, assumed wheel calibration and zero plate offsets.
> User executes all hardware/Git; assistant SSH remains read-only.

## Historical checkpoint log — superseded by the current-state section above

The following entries preserve the investigation in its original sequence.
Their "next", "latest", "pending" and "resume" wording applies to the time of
each entry, not to the current task. Use the current-state section to choose work.

### Earlier checkpoint — Turn-to-forward sequence passed in NJWQJ7

### Latest completed trial — gcQZct (2026-10-27, about 17:10 KST)

- User saved before/after snapshots in `/home/turtle/jetson-map-turn-forward.IcK9xK`
  and completed one v2 forward pulse in `/home/turtle/jetson-mapping-motion.gcQZct`.
  User confirms forward movement, complete automatic stop without intervention,
  no other movement since JHydTT and unchanged surroundings.
- Before-forward PGM bytes/settings match sIdcDI/after_rotation; before/after pixels
  match first/final motion-bag maps. The last JHydTT odometry pose and map->odom
  transform exactly match the first gcQZct values. Same NJWQJ7 mapping session
  and wheel runtime continue across both pulses.
- gcQZct: 84.389 mm forward, 0.044 mm lateral, 0.01149° net yaw; 40 forward + five
  zero commands. First zero at 4.166129 s; maximum odom gap 20.198 ms. 140 scans,
  616 odom samples, eight maps. Recorded rejected=4 unchanged, fault=0, fresh=1.
- Significant forward speed begins about 0.861 s after the first command;
  steering realignment is plausible but not directly established by joint data.
  No extension/retry or wheel recalibration. Last measurable pose change is
  0.274 s after the first zero; final constant-pose span 5.360 s. The five-second
  post phase is exactly constant, with independent user physical-stop confirmation.
- Incremental map->odom correction in this forward pulse: 4.970 mm / 0.800°.
  Cumulative correction through turn and forward: 8.201 mm / 1.400°; stable after
  stopping. Occupied/free/unknown cells progress from 68/405/519 initially, to
  103/547/373 after turn, to 121/562/340 after forward. Resolution remains 5 cm.
- Geometric analysis brackets 123/140 scans; IDs 0..16 are initial pre-odom scans
  retained in the bag but excluded without extrapolation. No command-interval
  scan is removed for missing coverage. Fixed-scene RMS is 4.627/9.560/11.508 mm
  for holdout/motion/post. Acquisition timing remains nominal/zero delay,
  acquisition_time_valid=0; no new raw timing capture or fitting occurred.
- Post-run read-only checks: same wheel PID 4948/map launch 8963/input 8982/mapper
  8983, active [3], status published=7825, rejected=4, fault=0, fresh=1, zero
  command publishers and one swerve subscriber. No assistant hardware/Git command.
- [Combined sequence review](jetson_mapping_turn_forward_review.md), figure and
  hashes saved; local data `/tmp/jetson-turn-forward-gcQZct/review`.
  Preserve current placement and NJWQJ7; no repeat pulse or reset is needed.
  Next validation is a short bounded route revisiting a known view to examine
  accumulated drift. Choose geometry/bounds from available physical space before
  preparing commands. No longer route has been prepared or executed; Nav2 remains
  unvalidated. Keep all LiDAR/wheel configuration and zero plate offsets intact.

### Earlier preparation — forward after JHydTT

- User says proceed and explicitly confirms >=0.5 m clear level floor ahead at
  the new heading, steering clearance, no robot/scene change since JHydTT, and
  manipulator still disconnected.
- Read-only inspection at about 17:08 KST confirms the same wheel/map processes,
  active [3], published=5600, rejected=4 unchanged, fault=0, fresh=1, zero command
  publishers and one swerve subscriber. Installed v2 source manifest and accepted
  mapping XML/YAML checksums match. No restart, reinstall or rebuild is needed.
- Save `before_forward` in a new `jetson-map-turn-forward.XXXXXX` directory and
  compare its PGM/settings to `jetson-map-rotation-snapshots.sIdcDI/after_rotation`.
  If they match, user runs exactly one v2 `--execute-forward-pulse`, then saves
  `after_forward` after software success and an observed complete stop. Keep
  NJWQJ7, wheel odometry, placement and scene continuous throughout.
- Commands are in the current continuation section of
  [MAPPING_MOTION.md](../../tools/lidar_diagnostics/MAPPING_MOTION.md).
  The pulse requests forty +0.025 m/s commands at 10 Hz and five zeros, nominally
  about 10 cm. Steering transition can reduce travel; no extension or retry.
  No new hardware motion or after-map has been reported yet. Assistant SSH is
  read-only; the user executes hardware and all Git commands.

### Earlier completed trial — JHydTT (2026-10-27, about 17:01 KST)

- User started fresh map `/home/turtle/jetson-map-rotation.NJWQJ7`, saved matching
  stationary maps in `/home/turtle/jetson-map-rotation-snapshots.sIdcDI`, ran one
  v2 rotation in `/home/turtle/jetson-mapping-motion.JHydTT`, then saved
  `after_rotation` in that snapshot directory. User confirms smooth rotation and
  complete automatic stopping, with no intervention, other movement or scene change.
- 20 +0.10 rad/s yaw commands and five zeros; first zero at 2.081685 s. Recorded
  rotation 0.129264 rad / 7.4063°, maximum translation 0.0583 mm, 106 corrected
  scans, 527 odom messages and six maps. Maximum odom gap 30.009 ms. All input
  statuses have rejected=4 unchanged, fault=0 and fresh=1.
- Significant yaw begins about 0.85 s after the first command. Steering alignment
  is a plausible cause of the smaller-than-nominal turn, not directly proven by
  this bag. Completion threshold +0.100 rad is met; do not extend/repeat to force
  the nominal 11.5°. Timing remains uncalibrated and wheel calibration is unchanged.
- Baseline PGM bytes/settings match, and before/after pixels match first/final bag
  maps. Occupied/free/unknown counts change 68/405/519 -> 103/547/373; dimensions
  31x32 -> 31x33 at 5 cm. Final map->odom correction is only 3.235 mm / 0.600°,
  after briefly reaching about 4.31 mm / 0.800°. It then stays constant.
- The five-second post phase has only 0.0000381 rad / 0.00594 mm total drift;
  exact final numerical constancy spans 4.200 s. User's independent physical
  confirmation establishes smooth automatic stopping. Do not rewrite the raw
  result.json physical_stop_confirmed=false field.
- Raw stationary file: 200 complete frames, no overflow/clock fault, strict replay
  passes. It sealed before rotation. Frozen-scene analysis brackets 105/106 scans;
  stationary/motion/post RMS is 2.868/9.147/11.036 mm under fixed exploratory gates.
- Read-only post-run inspection: wheel PID 4948, map launch 8963, input 8982,
  mapper 8983 remain running; lifecycle active [3], status published=2463,
  rejected=4, fault=0, fresh=1; zero command publishers, one swerve subscriber.
  v2 installed source manifest passes. No assistant hardware or Git commands.
- [Detailed rotation review](jetson_mapping_rotation_review.md), figure and hashes
  are saved; local artifacts `/tmp/jetson-rotation-map-JHydTT/review`.
  Preserve current placement, scene, wheel odometry and mapper. No repeat turn or
  fresh reset is needed now. Next useful controlled test is a short forward pulse
  from this new heading within the same map session, after checking forward
  clearance. That hardware step has not been executed or scheduled.

### Earlier preparation — v2 fresh-map rotation procedure

- User reports another slight physical scene change and requests rotation with a
  freshly captured map. AEjKu5 remains a completed clean forward trial, but its
  scene is no longer the next trial's baseline. Restart only the mapper, retaining
  the existing wheel runtime and preserving all prior data.
- User reconfirms stationary robot, motor power on, manipulator disconnected and
  clear level space around the whole robot and steering modules. Read-only SSH at
  about 16:49 KST finds wheel PID 4948 and old mapping PID 8071 (nEKS7T) still
  running; launch XML checksum is unchanged. No assistant hardware/Git action.
- The separate live-map runner now supports explicit `--execute-rotation-pulse`:
  twenty +0.10 rad/s yaw commands at 10 Hz, zero linear velocity, then five zeros.
  Expected turn is about 0.20 rad / 11.5°. Rotation bounds: <30 mm translation,
  -0.020 < yaw < +0.250 rad, first zero before 2.2 s, completion yaw >=+0.100 rad.
  Existing source ownership, lifecycle, input, timing, stop and recording checks
  remain. No retry or pulse extension. Existing forward mode is preserved.
- User-run transfer/build, fresh map, two stable baseline snapshots, one turn and
  after-map procedure: [MAPPING_ROTATION.md](../../tools/lidar_diagnostics/MAPPING_ROTATION.md).
  Do not run the old port-owning rotation coordinator alongside this live mapper.
  Acquisition timing stays uncalibrated. Fresh-map rotation has not yet run.
- C++17 warnings-as-errors build and all 20 desktop ROS scenarios pass, including
  both pulse modes and rotation-specific direction/displacement/completion faults.
  Normal synthetic rotation: 20 yaw + five zeros, 0.202260 rad, no translation,
  103 scans, first zero at 2.02256 s. Test artifacts `/tmp/mapping-motion-test.TIBLoz`;
  log `/tmp/lidar-mapping-rotation-build/test-output-network.txt`. An initial
  sandbox run hit a genuine >40 ms simulated-odom gap; the guard was preserved,
  and the complete rerun with normal local networking passed. Offline reviewer
  accepts rotation and reproduces AEjKu5 metrics without changing its method.
- Source-only transfer archive: `/tmp/jetson-mapping-motion-20261027-v2.tar.gz`,
  SHA-256 `4aa39c824e8aff1041a9eec79a31012c07657e950981a323a1baa987eaf2e0ce`.
  Internal source manifest passes. User installs into a new
  `/home/turtle/jetson-mapping-motion-v2`; v1 and the mapper workspace are preserved.
  No fresh rotation map or physical rotation capture is reported yet.

### Earlier completed trial — AEjKu5 (2026-10-27, about 16:32 KST)

- User started `/home/turtle/jetson-map-newscene.nEKS7T` at about 16:27 KST, saved
  two stationary baselines and completed the next forward pulse. Snapshots are in
  `/home/turtle/jetson-map-baseline.f39hWE`; motion bag/result are in
  `/home/turtle/jetson-mapping-motion.AEjKu5`.
- User explicitly confirms that the commanded pulse was the only robot movement
  since this map's startup, surroundings stayed unchanged, and movement ended in
  a complete automatic stop without Ctrl+C or motor-power intervention.
- Baseline A/B PGM bytes and map geometry are identical, and their pixels match
  the first motion-bag map. After-map pixels match the final bag map. All are 31x32
  cells at 5 cm; occupied/free/unknown counts change 65/383/544 -> 91/543/358.
- Forward displacement is 102.077 mm, lateral 0.220 mm, net odom yaw about 0.076°;
  40 forward + five zero publications; 128 scans, 636 odom messages, seven maps.
  Maximum odom gap 30.008 ms, final constant pose 5.220 s. All recorded statuses
  have rejected=1 unchanged, fault=0 and fresh=1. map->odom correction totals only
  about 2.858 mm and 0.600°, in three small updates; it stays fixed after stopping.
- Initial raw timing capture: 200 complete frames, no overflow/clock fault, strict
  replay passed. Motion geometry analysis brackets 127/128 scans; the initial
  pre-odom scan is retained in the bag but skipped without extrapolation.
  Timing remains nominal/zero hypothetical delay with acquisition_time_valid=0.
- This supports a successful short controlled forward-mapping check. Preserve the
  current map/wheel runtime and placement; no fresh reset is needed while the scene
  stays unchanged. Next useful validation is a bounded heading change with this
  same live mapper, requiring a separately prepared mapping-compatible rotation
  mode. Installed live-map runner is forward-only. Do not run the old port-owning
  rotation coordinator alongside the live input. Full route/Nav2 remain unvalidated.
- [Detailed new-scene review](jetson_mapping_newscene_review.md), figure and hashes
  are saved. Desktop artifacts: `/tmp/jetson-moving-map-AEjKu5/review`.
  No assistant hardware, deployment or Git operation occurred.

### Earlier preparation — Start from another changed physical environment

- User says the map/environment changed again and requests comprehensive starting
  instructions. Clarification confirms the robot or surroundings physically
  changed, rather than only a displayed-map change in an unchanged scene.
- Read-only inspection on 2026-10-27 around 16:19–16:21 KST finds the same wheel
  launch and old 700Cez mapping launch running. Both wheel controllers are active;
  `/cmd_vel` has zero publishers and one swerve subscriber. Input status is
  published=26763, rejected=3, fault=0, fresh=1. Existing XML/YAML hashes still
  match and the v1 bounded motion runner is installed. No fresh/baseline directories
  from the preceding proposed restart were found by the bounded path check.
- Comprehensive user-run procedure:
  [MAPPING_NEW_SCENE.md](../../tools/lidar_diagnostics/MAPPING_NEW_SCENE.md).
  Stop only old mapping, keep wheel runtime, start unchanged XML in a new directory,
  verify health and save two stationary snapshots ten seconds apart. Inspect this
  fresh baseline before the next one-pulse trial. Recheck physical clearance in
  this new environment; do not carry forward the old clearance confirmation.
- No hardware, deployment or Git operation was performed by the assistant.

### Latest hardware result: gUGiCK (2026-10-27, about 16:05 KST)

- User ran the bounded forward trial in `/home/turtle/jetson-mapping-motion.gUGiCK`:
  96.919 mm forward, 0.196 mm maximum lateral displacement, 0.101° yaw change;
  40 forward and five zero publications. User confirms movement and a complete
  automatic physical stop, without Ctrl+C or motor-power intervention.
- Bag contains 128 corrected scans, 679 odometry samples and nine maps; all seven
  streams were read back. Maximum odom gap 20.148 ms, final constant-pose span
  5.200 s; status rejected=3 unchanged, fault=0, fresh=1 throughout. Local frozen
  scene residual RMS is 4.47/8.14/10.25 mm for stationary holdout/motion/post,
  under the existing timing and odometry assumptions; no recalibration claim.
- Updated map: `/home/turtle/jetson-map-after-forward.hg0j10/after_forward.{pgm,yaml}`,
  93x78 cells at 5 cm. Saved PGM pixels exactly match the last recorded map.
  map->odom changed by about 25 cm and 14.20°, much larger than measured wheel yaw.
- **User then confirmed the robot was repositioned between the initial stationary
  map and this pulse.** This invalidates a clean before/after map-continuity claim;
  the correction alone does not establish a SLAM fault or successful relocalization.
  Preserve the run as motion/input evidence. Do not change wheel calibration,
  LiDAR setup or timing assumptions from this comparison.
- Next user-run hardware step: stop only the combined mapper/input/static-mount
  launch, keep the wheel launch running, restart the same mapping XML at the
  current stationary pose in a fresh directory, and save a new stationary baseline.
  Keep the robot at that placement and wheel odometry continuous through the next
  controlled movement. No additional motion until the new baseline is inspected.
- Analysis, hashes and figure:
  [jetson_mapping_motion_review.md](jetson_mapping_motion_review.md).
  Desktop data `/tmp/jetson-moving-map-gUGiCK`, completed C++ output `review-v2`.
  Assistant retrieval was read-only SSH; no hardware or Git commands were run.

### Earlier preparation — User confirmed stationary capture and forward clearance

- The latest "Confirmed" answers both pending physical questions: the first map
  capture stayed stationary, and >=0.5 m clear level floor ahead plus steering
  clearance is available. Manipulator remains disconnected. No new movement has
  been executed by the assistant or reported by the user at this checkpoint.
- The separate `run_mapping_motion` records existing ROS streams while the wheel
  and mapper launches continue. One +0.025 m/s forward pulse has forty publications
  at 10 Hz, followed by five zeros, odometry settling and five post-stop seconds.
  Nominal travel is about 10 cm; software motion bounds are not physical guarantees.
  No automatic retries, port ownership changes, configuration changes or Git actions.
- Procedure, limits, source-bundle transfer/build commands and after-map saving:
  [MAPPING_MOTION.md](../../tools/lidar_diagnostics/MAPPING_MOTION.md).
  The capture records corrected scans/odom/status/map/TF/commands; 700Cez's raw
  recorder is already sealed. Timing remains nominal/zero hypothetical delay,
  acquisition_time_valid=0. Review movement and map consistency before a route test.
- C++17 warnings-as-errors build and all eight isolated desktop ROS scenarios pass:
  normal 40-forward/5-zero delivery, stale odom/scans, input fault, inactive mapper,
  competing publisher during motion, stalled command schedule and preflight
  competition (no commands). Saved bags are reopened and checked, including NaNs.
  Simulated normal travel was 101.130 mm; this is not physical robot evidence.
  Test data: `/tmp/mapping-motion-test.9ownbb`; output log:
  `/tmp/lidar-mapping-motion-build/test-output.txt`.
  Source-only bundle: `/tmp/jetson-mapping-motion-20261027-v1.tar.gz`.

### Live status confirmed (2026-10-27, about 15:42 KST)

- User ran the checks on separate lines: slam_toolbox reports active [3];
  mapping_status reports published=3670, rejected=3, fault=0, fresh=1,
  reason=publishing. Model remains nominal, assumed_delay_ns=0,
  acquisition_time_valid=0, bins=1600. Bounded read-only SSH lifecycle/status
  queries independently confirm active [3] and published=3880 with the same
  rejected/fault/fresh values. The input continues after sealing its raw file.
- The earlier "No response" was a shell-paste issue: all commands were pasted
  onto one line after `source /opt/ros/lyrical/setup.bash`, making the remaining
  words arguments to source rather than separate export/ROS commands. It was
  not evidence of a mapper hang. Preserve working runtime/configuration.
- Read-only process inspection confirms domain 182/LOCALHOST for the input,
  mapper and existing daemon. They also inherit ROS_STATIC_PEERS=10.2.12.120;
  no environment was changed. Do not claim LOCALHOST proves complete isolation
  while an explicit static peer is set; inspect the command graph before motion.
- Initial mapper integration and saving are established. The next phase is a
  separately bounded moving-map check with new simultaneous recording, followed
  by a new map snapshot/consistency comparison. The raw recorder from 700Cez has
  sealed; do not reuse the old port-owning motion-capture coordinator alongside
  the live mapper. The user subsequently confirmed stationary capture and
  >=0.5 m of clear level floor ahead plus steering clearance, as recorded above.

### First saved map (2026-10-27, about 15:36–15:37 KST)

- User restarted wheel-only opencr/drive and ran the combined first-map launch.
  Trial: `/home/turtle/jetson-map-trial.700Cez`. Static front mount is the accepted
  xyz=(.110,.005,.0545), yaw=pi. Mapper configured/activated and registered
  lidar_front_link; no repeated TF or scan-matching errors appear in retrieved logs.
- Snapshot: `/home/turtle/jetson-map-snapshot.gjlydJ/stationary.{pgm,yaml}`.
  Map saver reports success. Desktop retrieval:
  `/tmp/jetson-first-map-700Cez/jetson-map-snapshot.gjlydJ`.
  93 x 70 cells, 0.05 m resolution, origin [-1.414,-2.168,0], extent 4.65 x 3.50 m.
  PGM contains 66 occupied (0), 1,235 free (254), 5,209 unknown (205) cells.
  Snapshot is sparse/ray-shaped with partial wall-like boundaries, consistent
  with an initial stationary-view trial, not an accepted navigation map or
  evidence of continuous mapping/loop closure. Physical scene agreement remains
  to be checked; do not infer clearance from gray/unknown cells.
- Map PGM SHA-256:
  `cf9f2f032375f77d14dc495f20526eadefa105bc17327327fd97771ffbfc0b00`;
  YAML SHA-256:
  `af9514c6afe50277d6d08b4f18e1ed582965cf7eb4bbb8f41689abd79aaf25bb`.
  Both match Jetson. Rendered preview: `/tmp/jetson-first-map-700Cez/stationary-map-preview.png`.
- Raw timing capture: 200 frames, complete=1, overflow=0, clock_bad=0.
  Strict timing replay passes with 70,128 selected returns; host ring period
  median 99.573 ms, range 99.049–103.073 ms. Clock brackets overlap. Timing remains
  nominal/zero hypothetical delay with acquisition_time_valid=0; no odom CSV was
  recorded in this trial, so no offline moving correction claim follows.
- The minimum-range warning is explained by upstream slam_toolbox 2.10.0
  `laser_utils.cpp`: YAML double 0.05 is slightly less than LaserScan float32
  0.05000000074505806. The mapper clamps to the message minimum and prints both
  to one decimal (0.1). Difference is <1 nm; preserve sensor and mapper profiles.
  Map-saver threshold/format warnings report defaults; PGM/YAML are valid. Its
  destructor lifecycle warning occurred after successful saving.
- Read-only SSH at 15:37 KST confirms wheel stack, mapping input, static mount
  and mapper are still running; original front driver is absent. Do not start
  another driver or the old capture coordinator while this launch owns the port.
- User supplied wheel startup/periodic logs: both controllers activated, torque
  enabled; after startup, reported board cycles and motor/USB communication show
  no failures, voltage around 11.4–11.5 V. FIFO scheduling warning is recorded;
  it does not establish a timing failure or justify unrelated system changes.
- The subsequent separate-line lifecycle/status output and physical confirmation
  resolved the earlier missing checks; see the current status above. No movement
  command, remote mutation or Git action was executed by the assistant.

### Resume inspection (2026-10-27, 15:34 KST)

- User reports ready to resume. Read-only SSH reaches Jetson at 10.2.12.123;
  uptime is about one minute, confirming a new boot. No matching wheel-control,
  original front-driver, mapping-input, mapper or static-TF process is running.
- Drive socket 2.4.3 resolves to ttyACM0; front socket 2.4.4 resolves to ttyUSB1.
  fuser reports no owner for either. Mapping launch XML and YAML checksums still
  match, and slam_toolbox/map-server packages remain installed.
- User confirms Jetson on, motors OFF, manipulator disconnected and robot
  stationary in the relocated scene. Turn motor power on before the user-run
  wheel launch: real startup requires motor communication and drive mode enables
  torque. No motion command is part of this stationary first-map procedure.
- Resume with the unchanged wheel-only opencr/drive launch in domain
  182/LOCALHOST, after physical readiness is confirmed. Check active controllers
  and fresh /odom before starting the combined stationary mapper trial. No
  original front driver needs stopping in this inspected fresh-boot state.
  All hardware execution remains the user's responsibility; no Git commands.

### Relocation checkpoint (2026-10-27, 15:24 KST)

- User completed steps 1–2 of the first-map procedure: dependency installation
  and transfer of `front_mapping_trial.launch.xml`. Read-only SSH confirms
  slam_toolbox 2.10.0-2noble.20260819.073924 and nav2_map_server
  1.5.0-1noble.20260819.052057 installed. Launch XML SHA-256 matches desktop:
  `6628d8dc2d6884490a788ed2ac4a1b669af42c4d1663795f0d4f9554b0cd648a`.
- User needs to remove Jetson power to relocate the robot. No first-map trial
  has been started. At the last inspection, wheel-only drive launch and original
  front driver were still running; no mapping input, mapper or static-transform
  process was running. Shutdown completion is NOT yet confirmed.
- User-run shutdown order: Ctrl+C wheel-only launch; disconnect motor power;
  Ctrl+C original front driver; `sudo shutdown -h now`; wait for completed OS
  shutdown before removing Jetson power and relocating. Manipulator remains
  disconnected; preserve LiDAR mounts and labelled USB socket assignments.
- On return, confirm physical power/runtime state and inspect read-only. Restart
  the existing wheel-only profile before resuming step 3; that step's earlier
  assumption that the wheel launch is already running will no longer hold after
  reboot. No reinstall, retransfer or rebuild is needed. Start a fresh stationary
  mapping trial/capture in the new scene; do not treat it as the old fixed scene.
  Continuous mapping remains chosen, wheel odometry assumed calibrated, and
  confirmed plate X/Y/yaw stay zero. Do not repeat earlier motion diagnostics.

This checkpoint supersedes the earlier pending-installation statements below.

### Successful stationary live input (2026-10-27, about 15:10–15:11 KST)

- User built and ran the isolated mapping driver. Capture:
  `/home/turtle/jetson-mapping-first.m6Nmzn`; desktop copy:
  `/tmp/jetson-mapping-first-m6Nmzn`. Published count advances 220 -> 330,
  rejected remains 4, fault=0 and fresh=1 at both observations. Rate is
  10.001–10.107 Hz. The startup log reports stale odometry; there is no observed
  sustained rejection. Timing remains nominal with zero hypothetical delay.
- The rate observer printed a wait-set/context error when its external timeout
  ended. Continued mapping publication afterward and the later user Ctrl+C
  shutdown distinguish this from failure of the mapping input node.
- Raw footer: 200 converted input scans, complete=1, overflow=0, clock_bad=0.
  Strict replay_timing passes: 49,577 selected returns; host ring period
  98.889–103.385 ms, median 99.596 ms. All handoff/publication clock-offset
  brackets overlap. This does not calibrate acquisition timing or establish a map.
  No simultaneous odom CSV or corrected output recording was requested in this
  stationary trial; do not claim it was replayed through the odom correction.
- Matching Jetson/desktop raw SHA-256:
  `80414096287d6a84a924edc1b2dd54dbf57e1c99ad712a39f7f90f0c57277edd`.
- User Ctrl+C cleanly stopped publish/grab threads and closed the front port.
  Read-only SSH at 15:12 KST confirms the wheel launch remains running and
  original domain-181 front driver has been restored. No mapping driver or
  mapper remains running. Manipulator-disconnected and full-launch-stopped
  boundaries remain in force.
- Inspected running robot_state_publisher description contains only the wheel
  tree, no LiDAR child. Wheel config publishes odom->base_footprint at 50 Hz.
  No static-transform process was found. The next trial adds a single
  base_footprint->lidar_front_link owner with xyz=(.110,.005,.0545), yaw=pi;
  height remains estimated, plate X/Y/yaw zero.
- Jetson apt index offers slam_toolbox 2.10.0 and nav2_map_server 1.5.0; neither
  is installed yet. Read-only install simulation with no recommends/removals
  proposes 195 new packages and two non-ROS upgrades; no removal. Disk has
  191 GiB free. User performs installation; assistant made no remote changes.
- New `config/front_mapping_trial.launch.xml` groups only the front mapping
  input, its static mount, and async slam_toolbox. No wheel launch, motor commands
  or original profile changes. XML parses successfully; the same mapper YAML
  configured and activated with installed desktop 2.10.0 in isolated domain 194.
  This is configuration/lifecycle evidence, not a map-producing integration test.
- Next: user installs dependencies, transfers this one XML file, then runs a
  stationary first-map trial in domain 182/LOCALHOST using the existing built
  input package and YAML. Check active lifecycle, fresh input and nonempty map
  metadata; save a stationary snapshot into a fresh directory for inspection
  before any new movement. See MAPPING.md for complete commands.

This supersedes the pending stationary-input statements below. Continuous mapping
on a moving route, map quality/revisit consistency and navigation remain unvalidated.

### Offline comparison and isolated live prototype (2026-10-27)

- Both successful captures have now been exported to a conventional front-only
  LaserScan grid. At nominal timing/zero hypothetical delay, 1,600 angular bins
  retain all 42,353 translation and 39,385 rotation exterior returns, with no
  collisions or approximation rejections. Empty/body/invalid bins remain NaN.
  More output bins improve bearing representation; they add no measurements.
- Angular resampling endpoint RMS is 4.399 mm (translation) and 4.272 mm
  (rotation); maxima 22.519 and 21.373 mm. Maximum corrected-origin displacement
  is 2.545 and 1.105 mm. The enforced approximation limits are 25 mm endpoint
  error and 5 mm origin shift, under the stated timing model. They are not
  physical error bounds or proof of equivalent free-space visibility.
- `mapping_frame.hpp` reconstructs the live selected-source input. It reproduces
  every finite/unknown output bin of all 400 saved scans. It uses bracketed odom,
  <=40 ms odom gaps, and provisional pose-derived limits of 0.04 m/s and
  0.15 rad/s. These only withhold scans; wheel-controller limits stay unchanged.
- Separate `coin_d4_mapping_driver/front_mapping_input` preserves the original
  stationary driver and socket configuration. It publishes only
  `/lidar_front/scan_mapping` plus status, owns no TF, and sends no motion commands.
  It requires explicit timing assumptions, fresh scans/odom, monotonic clocks,
  valid source boundaries and the unowned original front socket. Time regression
  latches a fault. Acquisition timing remains uncalibrated (`acquisition_time_valid=0`).
- Clean patched-source build, offline regression checks, both saved-capture core
  comparisons, synthetic serial parser test, and hardware-free ROS freshness/fault
  integration test pass. Installed desktop Karto tests confirm NaNs produce no
  occupancy rays and the 1,600-bin metadata has the intended count. No robot map
  or live sensor output from this new executable has been validated yet.
- Read-only SSH at 14:49 KST and 15:02 KST found the wheel-only launch and restored original
  front driver still running. Jetson has no installed slam_toolbox or nav2_map_server
  in the inspected Lyrical prefix. No software was installed or deployed remotely.
- Next user-run step: build the source-only v1 bundle in a new workspace, stop
  only the original front driver, and run the new input while stationary in
  domain 182/LOCALHOST. Inspect status and scan rate, then stop it and restore
  the original front driver. See `tools/lidar_diagnostics/MAPPING.md` for commands.
  Do not repeat motion pulses now. Mapper/TF integration follows only after
  this input check; the separate `config/slam_front_experiment.yaml` is prepared.
- Desktop bundle: `/tmp/jetson-front-mapping-20261027-v1.tar.gz`, 45,479 bytes.
  SHA-256: `ff08d200898c3f7ae02a0ff35e181ebaf5445d1fbc8a32d03aad163fe96c589f`.
  Its `SOURCE.sha256` checks the isolated package, profile and guide; no compiled
  binaries are included. Build using `&&` so a failed mkdir/checksum stops the
  sequence. The archive is local only; transfer/build/runtime are user actions.
- No Git commands were executed. Existing LiDAR/wheel profiles, manipulator
  disconnection, accepted mount and assumed wheel calibration are preserved.

This section supersedes earlier statements that the offline/live input prototype
is pending. The successful captures below remain the authoritative raw evidence.

### Successful v4 rotation (2026-10-27, about 14:29–14:31 KST)

- Authoritative rotation capture: `/home/turtle/jetson-motion-auto.H9GTjD`.
  Desktop copy: `/tmp/jetson-rotation-H9GTjD`. User confirms smooth rotation,
  complete physical stopping and original front-driver restoration. Read-only
  SSH confirms wheel-only launch and restored front driver; no recorder remains.
- v4 completed 20 yaw + 3 zero publications and 200 scans. Raw provenance/clock
  checks pass; complete=1, overflow=0, clock_bad=0. 1,389 odom samples have maximum
  gap 20.300 ms. Net yaw +11.29256 degrees, max translation 0.07063 mm, net X/Y
  +0.03217/+0.03833 mm. Final pose is constant for 13.120 s. These are odom values,
  not independent metrology. Physical behavior is supported by the user's report.
- Nominal/scaled motion replays at hypothetical delays 0/10/25/50/100 ms all pass:
  39,385 exterior rays and 12,411 body returns removed. Nominal zero-delay exterior
  beam intervals overlap motion in 23 scans. The scene analysis groups 27 scans
  including conservative transition margins; these counts have different meanings.
- Nominal zero-delay maximum endpoint correction is 71.605 mm; ray-origin shift
  1.10491 mm. Scaled maxima: 71.3597 mm and 1.10064 mm. Maximum difference between
  corrected endpoints of the two models at zero delay is 0.36388 mm. These are
  model outputs, not measured distortion or improvement; acquisition_time_valid=0.
- Frozen-reference scene analysis: 15 training scans/39 short segments; 45 pre
  holdout, 27 motion/margin, 113 post scans. Nominal zero-delay motion median
  3.844 -> 3.729 mm, RMS 16.795 -> 16.151 mm; matched 1,029/5,351 exterior rays.
  Pre median 1.346 mm; post median 8.272 mm. Post scans are stationary, so their
  larger residual is not corrected by within-scan deskew. Finite segment matching,
  changed visibility and other geometry/sensor effects remain possible; do not
  infer an odometry recalibration requirement.
- Reference sensitivity (20/30 mm spread vs original 10 mm, and 160 vs 80 mm
  segment length) increases motion coverage to 36–44%, with zero-delay RMS benefit
  only 0.12–0.30 mm. At 100 ms nominal assumed delay, default-reference motion
  RMS becomes 17.746 -> 17.916 mm. There is modest assumption-dependent evidence
  for correction, not calibrated delay or accepted continuous mapping.
- Next: prepare an OFFLINE, front-only mapper-input prototype and compare the
  additional angular resampling/single-origin approximation on both complete
  captures before implementing/deploying the live path. Keep timing hypotheses
  explicit, unknown/body/invalid bins NaN, confirmed mount unchanged, and every
  TF child under one owner. See MOTION.md for the provisional contract. Do not
  repeat the successful translation/rotation pulses just to resume.
- No Git commands or remote mutations were performed by the assistant. Local
  analyzer now accepts bounded optional reference thresholds for sensitivity.

Raw SHA-256: timing.jsonl
`29969d3e2d8effc5c3aa2f547df35734c2859a77e706a829d955d83853a77c75`;
odom.csv `d0d6e60bf57f04258af8146a946fce90a99b83248a909a1bb1f528b61e194b20`.
Desktop and Jetson hashes match. This successful run supersedes the pending-v4
execution statements below; the failed T60ife capture remains historical evidence.

### Rotation attempt and fix (2026-10-27, about 14:19–14:25 KST)

- User ran the v3 rotation coordinator. Capture: `/home/turtle/jetson-motion-auto.T60ife`.
  It failed with `Pulse/recorder interrupted or timed out`. The pulse log prints
  all 20 yaw publications after several discovery waits. The v3 six-second
  deadline incorrectly included process startup/discovery and exit time. Recorder
  logs show no preceding failure; they were interrupted during coordinator cleanup.
- Abort zeros were NOT established: abort_zero.log contains only subscriber waits,
  no publications, before its separate three-second timeout. User confirms steering,
  robot rotation and automatic stopping. Odom: 912 samples, max gap 20.441 ms,
  net X +0.9533 mm/Y +0.2380 mm/yaw +6.48765 degrees, max translation 0.9843 mm,
  final constant pose 3.540 s. These support rotation and automatic stop, not
  explicit-zero delivery or calibrated timing.
- Timing footer: 116 scans, complete=0, overflow=0, clock_bad=0. Preserve raw data;
  do not relabel complete or use it for accepted motion/scene replay. Both strict
  replay and coordinator verification reject it. Desktop copy is
  `/tmp/jetson-rotation-T60ife` (temporary).
- Read-only runtime inspection found wheel-only launch still running and the
  original domain-181 front driver restored. No motion recorder/publisher remained.
- Local coordinator v4 separates startup/discovery (8 s outer bound, 5 s CLI
  matching wait), publication sequence (3.2 s rotation), and exit (2 s) deadlines.
  A single publisher reads a finite YAML stream: 20 yaw commands then 3 zeros,
  with current stamps. Normal stopping no longer creates a second publisher.
  Abort zeros remain best effort, have a discovery allowance, and report failure
  explicitly. Travel/freshness bounds, hardware configuration and capture count
  are unchanged. No automatic retries or pulse extension.
- Tests: C++ warnings-as-errors/UBSan regression gates pass. Desktop ROS integration
  on domain 193/LOCALHOST and a test-only topic received exactly 20 yaw + 3 zero
  messages after a deliberately delayed subscription; sequence span 2.214 s.
  v4 is NOT deployed or physically validated. User transfers/compiles the standalone
  source into a new directory and runs it using unchanged installed v2 recorders.
  Stop only the restored front driver before any next recording; keep wheel launch.

This update supersedes the v3 preparation/runtime snapshot below.

### Resume update after break (2026-10-27, about 14:07 KST)

- User confirms all power on and manipulator disconnected. Robot position relative
  to the scene changed slightly after the previous capture; use a new stationary
  reference within the next recording, not registration against the old scene.
- Read-only SSH found Jetson up about eight minutes, no matching ROS/controller/
  LiDAR processes, and no reported fuser owners of ttyUSB0/ttyUSB1/ttyACM0.
  Socket mapping remains rear 2.4.2 -> ttyUSB0, drive 2.4.3 -> ttyACM0,
  front 2.4.4 -> ttyUSB1. Runtime is a snapshot, not a guarantee before execution.
- Retrieved 9RvB4b into `/tmp/jetson-motion-9RvB4b`; raw timing and odom SHA-256
  match Jetson. Provenance replay and ten motion replays pass (nominal/scaled,
  assumed delays 0, 10, 25, 50, 100 ms). All retain acquisition_time_valid=0.
- `analyze_scene.cpp` freezes 28 stable short segments from the first 15 scans,
  without registration or fitted delay. Twenty held-out pre-motion scans have
  median unsigned segment distance 1.327 mm. Nineteen motion/transition scans
  have 2.670 mm raw vs 3.363 mm corrected at nominal/zero delay; 146 post-motion
  scans have 4.376 mm. Motion RMS is 11.702 vs 11.852 mm. Only 792/4060 motion
  exterior returns match within 50 mm; coverage is limited (~20%). The two models
  and delay sweep do not demonstrate a correction benefit. These are exploratory
  repeatability residuals, not wall accuracy, a timing calibration, or rejection
  of the assumed wheel calibration. Tiny translation is insufficient here to
  establish a defensible correction benefit.
- Local coordinator now supports `--execute-rotation-pulse`: twenty +0.10 rad/s
  messages at 10 Hz, then three explicit zeros; same 200-scan recording. During
  motion/stop it checks fresh odom and aborts at 30 mm translation or yaw outside
  (-0.03,+0.30) rad relative to pre-pulse pose. These are software detection limits,
  not guaranteed travel bounds. Verify requires >=0.05 rad positive yaw during
  capture and a final stationary window. No extension/retry if steering consumes
  the pulse. Existing steer-then-drive and roll-while-steering are preserved.
- The updated coordinator can be compiled separately on Jetson with g++; it uses
  the unchanged installed v2 recorders. It has NOT been deployed or run on hardware.
  C++ warnings-as-errors/UBSan tests pass for model/clock/source timing, coordinator
  gates and cleanup, and fixed-scene distance geometry. Hardware execution remains
  the user's next action; no Git commands or remote mutations were performed.

The detailed baseline below remains useful; this resume update supersedes its
pre-break runtime state and pending-analysis/forward-only statements.

The objective is physical continuous SLAM on the assembled robot, then autonomous
navigation. The user chose continuous mapping first, not stop-and-observe. We have
verified basic wheel feedback/motion/stopping and synchronized moving LiDAR/odom
capture. We have NOT demonstrated a map, loop closure, localization on a saved map,
or autonomous navigation with our integration. The next work is fixed-scene
analysis and a bounded rotation diagnostic, then a defensible live mapping input.
Do not repeat the successful straight-pulse sequence merely to resume this work.

### Working boundaries and shutdown state

- The user executes all hardware and Git commands. Read-only SSH inspection and
  retrieval of saved files are authorized. Local edits and offline tests are allowed.
  Do not independently deploy, install on Jetson, start/stop processes, open serial
  ports, send CONFIG/motion, flash firmware, or run Git commands.
- Explain results and full terminal commands in chat; the user does not rely on
  reading reports. Keep implementation in C++, with necessary ROS launch glue only.
  No subagents unless requested. Preserve the manipulator-disconnected setup and
  leave full robot.launch.py stopped.
- Adopt only recovered wheel configuration from the colleague archive. Preserve
  our controller, limits, LiDAR configuration, and package layout.
- The user left for a break and received shutdown instructions: stop wheel launch,
  disconnect motor power, stop sensor/TF/view processes, then sudo shutdown -h now.
  Completion of shutdown was NOT reported. At next contact, ask about physical power
  state and use authorized read-only SSH to inspect runtime; do not assume it is live
  or shut down, and do not wake/start hardware automatically.
- Git branch/HEAD/index were not inspected this session. Older Git snapshots below
  are historical. No Git commands were executed by the assistant. Changes are not
  claimed committed or staged; the user controls all Git inspection/publication.

### Accepted decisions and geometry

- Wheel odometry is **assumed calibrated at the user's explicit instruction**.
  Skip further distance/yaw calibration. This is not a measured accuracy claim.
  Reverse/lateral/rotation behavior has not been established by the forward tests.
- User confirmed plate centre coincides horizontally with the wheel-base origin,
  and drawing +X matches tested robot-forward motion: plate_x=0, plate_y=0,
  plate_yaw=0 in config/lidar_mounts.yaml. Do not ask to reconfirm these.
- Retain front/rear plate origins (+0.110,+0.005,+0.0265) and
  (-0.110,+0.005,+0.0265) m, yaws pi and zero, from accepted drawing assumptions.
  The 0.028 m mounting-surface ground height remains estimated. Wheel base_link
  is 0.033 m above base_footprint; do not confuse ground height with base_link Z.
- Keep the accepted full steering envelope (0.2552 x 0.3050 m plus 0.010 m/side).
  Front-frame body box: X [-0.0276,0.2476], Y [-0.1575,0.1675] m. This is a
  filtering envelope, not a fully validated navigation footprint.
- IMU fusion remains disabled. Board IMU-init status is not mounted-axis or
  freshness validation. Current drive limits remain 0.10 m/s and 0.5 rad/s.

### Evidence completed

| Milestone | Evidence | Limit |
| --- | --- | --- |
| Stationary dual LiDAR | Two roughly 10 Hz streams; 400 bins; saved body-view comparisons | Existing driver is stationary-only, publication-stamped, time_increment=0 |
| Wheel configuration | Recovered IDs/signs/zeroes integrated into our wheel-only profile | Colleague's controller/SLAM stack was not imported |
| Jetson mock and real feedback | Eight finite joints; real bus inactive; broadcaster active; roughly 100 Hz | First real CONFIG failed with motors unpowered; succeeded after user connected motor power |
| Drive activation | Active hardware/controller, claimed interfaces, zero command publishers, robot physically stationary | Does not establish all motion directions or mechanical fault handling |
| Forward and stopping | Short 0.025 m/s pulses, about 27–30 mm travel; explicit-zero and command-timeout stops observed | Exact timeout latency and independent metrology not established |
| Moving diagnostic | Coordinated 200-scan capture with movement overlapping 15 scans | Timing calibration and physical correction accuracy not established |
| Offline correction | Per-beam model plus odom interpolation and ray-origin preservation execute successfully | No corrected live LaserScan or mapper integration |

### Latest authoritative dataset

Jetson: `/home/turtle/jetson-motion-auto.9RvB4b`.
Desktop retrieval: `/tmp/jetson-motion-9RvB4b` (temporary).
Files include timing.jsonl, odom.csv, timing.log, odom.log, pulse.log and preflight logs.

- 200 scans pass provenance and clock-bracket checks; 1,399 advancing odom samples
  bracket the recording; maximum odom interval 20.250 ms. Movement overlaps 15 scans.
- Net odom: +29.65565 mm X, -0.05140 mm Y, +0.01554 degrees yaw. Final half-second
  pose drift is zero. User explicitly confirmed physical movement; final stop in
  this run is supported by odom, while earlier runs have explicit physical-stop reports.
- No error in recorder/pulse logs. Read-only SSH confirmed original front driver
  restored after this experiment, before shutdown instructions were issued.
- Nominal and period-scaled replays used zero assumed transport delay solely as
  an analysis hypothesis. Both retained 42,353 exterior rays and excluded 11,414
  body returns. Maximum endpoint correction 3.405 mm; ray-origin displacement
  2.545 mm; maximum difference between models 0.035 mm.
- Those magnitudes are model outputs, NOT measured errors or improvements. Both
  models share unknown sensor-to-host delay; agreement does not validate timing.
  acquisition_time_valid remains 0. See the inconclusive fixed-scene analysis above.

Earlier gosVkf and tbO0uX captures were valid recordings without overlapping motion.
In tbO0uX, scans ended 12:59:31 KST and movement started about 13:00:31. This caused
us to replace fragile manual terminal timing with the coordinated v2 runner.
Do not repeat that investigation or the old three-terminal procedure.

### Machines, source, and runtime ownership

Desktop repo: `/home/pa4/git/Project2_Turtle_Ninja`.
Jetson: `turtle@10.2.12.123`, aarch64 Ubuntu 24.04.5, ROS Lyrical `/opt/ros/lyrical`.

| Purpose | Jetson path / setting |
| --- | --- |
| Wheel source | ~/jetson-wheel-config-20261027-v1 |
| Wheel workspace | ~/ros2_wheel_config_v1_ws |
| Latest diagnostic source | ~/jetson-motion-capture-20261027-v2 |
| Latest diagnostic workspace | ~/ros2_motion_capture_v2_ws |
| Wheel/diagnostic isolation | ROS_DOMAIN_ID=182, ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST |
| Original stationary LiDAR | ~/ros2_lidar_experimental_ws; domain 181/LOCALHOST |
| Body view | ~/ros2_lidar_body_view_v2_ws; ~/lidar_body_view_v2.yaml |
| Front driver parameters | ~/jetson-lidar-source/project/config/lidar_drivers.yaml |
| Drive USB | /dev/serial/by-path/platform-3610000.usb-usb-0:2.4.3:1.0 |
| Front USB | /dev/serial/by-path/platform-3610000.usb-usb-0:2.4.4:1.0-port0 |
| Rear USB | /dev/serial/by-path/platform-3610000.usb-usb-0:2.4.2:1.0-port0 |

Reinspect enumeration after reboot. The two LiDARs share a nonunique USB serial;
retain the physical socket mapping. Never start two drivers on one port.
Swerve owns odom -> base_footprint; robot_state_publisher owns base/wheel transforms.
The existing LiDAR tree is separate: front -> rear, translation (+0.220,0,0), yaw pi.
Body-view output is /lidar/points_body_filtered. It is not a mapping LaserScan.
Live integration must deliberately join domains and assign each TF child one owner;
do not publish guessed/redundant transforms or use the legacy full-robot model.
A mapper would own map -> odom; none is configured/running in our integration.
Jetson slam_toolbox and nav2_map_server were absent at the last read-only inspection.
Desktop package availability does not establish Jetson installation.

### Implementation and verification to preserve

- `config/lidar_mounts.yaml`: confirmed horizontal transform; remaining estimates explicit.
- Wheel files: `config/{hardware_map_drive_only,controllers_drive_only}.yaml`,
  `ros2_ws/src/bringup/launch/drive_only.launch.py`,
  `ros2_ws/src/description/urdf/drive_only.urdf.xacro`.
- `ros2_ws/src/lidar_cloud_merge/`: separate stationary_front_scan_filter, new
  scan_filter.hpp/test, CMake additions. Keeps physical origin/header/grid; body
  and invalid beams become NaN, never clear-space infinity. Desktop build and
  both core/filter tests pass. No live moving-SLAM acceptance follows.
- `tools/lidar_diagnostics/{motion_model.hpp,replay_motion.cpp,replay_timing.cpp}`:
  selected-source timing, explicit delay hypothesis, interpolation without extrapolation,
  source seam preservation, and both ray origins/endpoints at a reference time.
  replay_timing CSV now appends range_m and raw_angle_rad to the original columns.
- `capture_motion_odom.cpp`, `run_motion_check.cpp`, `motion_capture.patch`,
  `build_motion_capture.sh`, `test_motion_model.cpp`, `test_motion_check.cpp`,
  updated `check_offline.sh`, and `MOTION.md`: reproducible diagnostic implementation.
  v2 file-only recorder adds first-scan-ready marker; no scan publisher is advertised.
  JSON publish/published fields count converted diagnostic scans, not ROS publication.
- User-run `ros2 run coin_d4_motion_capture run_motion_check --execute-forward-pulse`
  launches two recorders, waits for readiness and two stationary seconds, publishes
  exactly ten +X 0.025 m/s messages at 10 Hz, checks automatic stop and capture overlap.
  It opens the front LiDAR and commands motion: assistant must not execute it.
  It stops only child process groups, attempts bounded zero on abort after pulse
  attempt, never automatically retries, and leaves front-driver restoration to user.
  `--verify-only DIRECTORY` is hardware-free. The installed v2 runner supports ONLY
  the forward pulse. The locally prepared v3 rotation extension is described above;
  it must be transferred/compiled by the user before use.
- Desktop builds with -Werror, UBSan motion/clock/recorder tests, synthetic parser,
  invalid-input/no-overwrite checks and patch reproduction pass. v2 executed on
  Jetson successfully, as the authoritative dataset demonstrates.

Source archive `/tmp/jetson-motion-capture-20261027-v2.tar.gz` (89,714 bytes), SHA-256
`d9eb3c5793b4583682b37a5c20c2d767b1e11d38ec2751442f6fc4a4047b1fe9`.
Desktop /tmp artifacts may disappear. Repo sources and Jetson saved source/captures
are recovery inputs. Do not rebuild/retransfer solely because the session changed.

### Ordered next work and completion criteria

1. Power/runtime inspection and saved-file retrieval completed as recorded above.
   Recheck unique port ownership at user-run startup; retain wheel/LiDAR isolation.
2. Fixed-scene comparison completed, inconclusive for correction benefit. Preserve
   that limitation; do not claim success because correction code ran.
3. User transfers/compiles the prepared bounded rotation coordinator, starts the
   wheel-only profile and runs one rotation capture. Explain steering transitions
   and stopping before execution. Analyse the new capture against its own fixed
   scene reference. This evaluates timing/distortion, NOT renewed wheel calibration.
4. Define a practical continuous-mapping measurement contract: timing reference,
   latency assumption/uncertainty, motion correction, freshness rejection, and
   mapper-compatible rays. Require fixed-scene consistency during translation and
   rotation. A uniform scan_time/400 is invalid for the reversed/seamed source order.
   Corrected endpoints have different ray origins; blindly binning them into a
   single-origin scan can invent free-space rays. Address that approximation explicitly.
5. Implement a separate live mapping path and user-run Jetson dependency installation.
   Connect the confirmed sensor geometry to the calibrated wheel tree, check sole TF
   ownership and scan-time coverage, and validate body-filter/unknown-beam handling.
   Start with one physical LiDAR. Preserve original stationary profiles as recovery.
6. Demonstrate first physical SLAM: build a map while moving and rotating, revisit
   known places, check wall consistency/loop closure, and save map plus pose graph.
   Only then call physical SLAM demonstrated. Perfect optical timing calibration is
   not itself the goal; a stated, experimentally defensible operating envelope is.
7. Navigation follows: saved-map localization, physical footprint, holonomic/swerve
   control, command interface, obstacle marking/clearing from correct origins,
   stale-data/TF/odom fault response, then bounded user-run goal and obstacle tests.
   Second-LiDAR mapping/refinement is optional after the single-sensor baseline;
   navigation can use separate front/rear observations with their correct origins.

### Resume prompt

Read the current-state section of docs/navigation/dual_lidar_handoff.md and
 tools/lidar_diagnostics/MOTION.md. Resume after the successful 9RvB4b moving capture.
The user executes hardware and all Git commands; read-only SSH is allowed. Confirm
power/runtime state after the break. Continuous mapping is the chosen path; wheel
odometry is assumed calibrated; plate X/Y/yaw are confirmed zero. Start with analysis
of fixed-scene consistency, then prepare a bounded rotation diagnostic. Explain
steps in chat, preserve existing LiDAR/wheel configuration, and do not repeat the
completed straight-motion checks or treat estimated timing as calibrated.

## Historical checkpoints — superseded starting point


The intermediate goal is SLAM on the assembled robot; the ultimate goal is
navigation. The stack audit and stationary dual-LiDAR body-filter check are
complete. A separate calibrated wheel-only profile now builds on Jetson. **The
next task is user-run mock feedback on Jetson, then real torque-off wheel feedback
after reviewing the mock result.** Physical drive, odometry accuracy and moving
SLAM with this integration have not been validated. Do not restart the audit or
repeat the completed LiDAR capture merely because this is a new session.

Read this current-state section and the implementation checkpoint immediately
below it first. The later historical checkpoints preserve evidence and superseded
instructions; their older "next step" and Git snapshots are not current orders.
On resumption, reread applicable repository/skill instructions and the current
wheel launch/configuration files. Check any changed environment through authorized
read-only inspection; retain the user-run boundary for hardware and Git actions.

### Scope and working boundaries

- Prioritize our stack. The user explicitly limited the colleague's archive to
  recovering wheel-system configuration. Keep our controller behavior, LiDAR
  integration and conservative limits; do not import its SLAM/teleop/high-speed
  configuration or replace our packages with its `nt_*` layout.
- The user executes hardware and **all Git commands**. Assistant read-only SSH
  inspection and retrieval of already saved data are authorized. Assistant local
  source/document edits and offline/mock checks are allowed. Do not independently
  open physical serial ports, deploy files, start/stop Jetson processes, send
  CONFIG or motion commands, flash firmware, or stage/commit/push/switch branches.
- Preserve the desktop desk setup and the last known live Jetson LiDAR setup.
  Keep the manipulator disconnected; it is not required by the wheel-only launch.
  Leave the full `robot.launch.py` stopped. Building `manipulation` as a dependency
  did not start any arm nodes.
- Explain findings and next actions in chat: the user said, "I don't read reports;
  explain it here." The handoff is for continuity, not a substitute for explanation.
- Use C++ for project implementation, retaining only necessary ROS launch/Xacro
  Python glue and direct API tests. Do not spawn subagents unless requested.
  Search other session records only within this same project folder and host.
- Geometry accepted as an assumption must remain labeled as such. No new sensor
  mounting values may be inferred merely to connect TF or pass launch guards.

### Verified progress and remaining evidence

| Item | Evidence available | What remains |
| --- | --- | --- |
| Stationary raw LiDAR | Both Jetson streams near 10 Hz, 400 bins, correct sensor frames | Motion-compatible acquisition timing; raw scans are not body-filtered |
| Stationary body filter | All 404 complete recorded scan pairs reproduce live cloud XYZ/intensity/source/order; all 405 clouds obey the revised box | This is a cloud visualization check, not a single-origin SLAM scan |
| OpenCR protocol | User-run five-second receive-only capture: 502 STATE, 5 DIAG, advancing board time, no malformed/checksum failures | Zero configured joints at that time; no fresh motor feedback/readback for this profile |
| Recovered calibration | Colleague archive has drive-only IDs/signs/zeroes, readbacks and physical test records | Current assembly/firmware feedback and independent odometry accuracy |
| Our wheel profile | Desktop six-package build, 73 existing tests, 13 invalid-launch cases, 33 kinematic cases, mock feedback/drive/timeout checks pass | Jetson mock and real feedback have not been reported |
| Jetson wheel build | User verified archive checksum and built six packages in 2 min 30 s | Release used `BUILD_TESTING=OFF`; build success is not a Jetson runtime test |

The colleague's real motor tests supersede the earlier "only mock evidence found"
picture. Ten basic motions passed in its recorded setup, but much of the numeric
distance evidence is derived from `/odom`; independent yaw accuracy and continuous
direction transitions remain unresolved. Do not equate those tests with validation
of our newly integrated profile. Details and the motor table are preserved in the
wheel-configuration recovery checkpoint below.

### Machines, workspaces and the setup to preserve

Desktop repository: `/home/pa4/git/Project2_Turtle_Ninja`.
Jetson: `turtle@10.2.12.123`, aarch64 Ubuntu 24.04.5, ROS Lyrical at
`/opt/ros/lyrical`; the user previously verified NTP synchronization and dialout
access. These are last reported observations, not a fresh live inspection during
handoff creation. USB enumeration and process lifetime can change after reboot.

| Purpose | Jetson path or setting |
| --- | --- |
| New wheel source | `/home/turtle/jetson-wheel-config-20261027-v1` |
| Successfully built wheel workspace | `/home/turtle/ros2_wheel_config_v1_ws` |
| Wheel verification isolation | `ROS_DOMAIN_ID=182`, `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` |
| Last known live stationary LiDAR domain | `181`, `LOCALHOST`, `use_sim_time=false` |
| Experimental raw drivers | `/home/turtle/ros2_lidar_experimental_ws` |
| Stationary body-view workspace/config | `/home/turtle/ros2_lidar_body_view_v2_ws`, `/home/turtle/lidar_body_view_v2.yaml` |
| Drive OpenCR USB socket | `/dev/serial/by-path/platform-3610000.usb-usb-0:2.4.3:1.0` (previously `ttyACM0`) |
| Front LiDAR USB socket | `/dev/serial/by-path/platform-3610000.usb-usb-0:2.4.4:1.0-port0` |
| Rear LiDAR USB socket | `/dev/serial/by-path/platform-3610000.usb-usb-0:2.4.2:1.0-port0` |

The desktop repository path does not exist on Jetson. Its older
`/home/turtle/orin_mock_20261018.03inTQ/deployment_source` is a different deployment;
do not source it for the new wheel checks. The shared CP2102 serial `0001` cannot
uniquely identify the two LiDARs; use the established physical USB socket mapping.

Last known running LiDAR nodes: `/lidar_front/driver`, `/lidar_rear/driver`,
`/jetson_lidar_relative_tf`, `/jetson_lidar_body_view` (plus its TF listener).
The relative TF is `lidar_front_link -> lidar_rear_link`, (+0.220,0,0) m, yaw pi.
The body-view output is `/lidar/points_body_filtered`. Preserve these processes
if still running. Do not replay a bag into live domain 181 or start duplicate
publishers. The wheel feedback launch in domain 182 does not connect these trees.

### Geometry, interfaces and TF ownership

Our new controller/model share `config/controllers_drive_only.yaml`: module
X `[-0.064,0.064,-0.064,0.064]`, Y `[0.112,0.112,-0.112,-0.112]`, offsets
`[0.0269,0.0269,-0.0269,-0.0269]` metres and radius 0.033 m, in retained
`fl/fr/rl/rr` order. Calibrated base +X is old CAD -Y, +Y is old CAD +X.
The suffixes now correspond to rear-left/front-left/rear-right/front-right.
Keep recovered steering zeroes coupled to this convention. The new model's
`base_footprint -> base_link` is +0.033 m Z with no yaw offset.

Plate-relative front/rear scan origins are assumed (+0.110,+0.005,+0.0265) and
(-0.110,+0.005,+0.0265) m, with yaws pi/zero. The accepted FL0 drawing/specification
is assumed to reflect the assembly, not measured optical calibration. Plate +X
is drawing-forward, +Y left; its relationship to the new wheel base axes remains
unresolved. `config/lidar_mounts.yaml` keeps `plate_x`, `plate_y`, `plate_yaw` null;
the 0.028 m mounting-surface ground-height value remains an estimate.

The revised tight envelope is 0.2552 x 0.3050 m with a 0.010 m margin per side.
In `lidar_front_link`, the inclusive box is X [-0.0276,0.2476],
Y [-0.1575,0.1675] m, all Z. It represents the accepted full steering-sweep
envelope. The user said the wheels were at 90 degrees in the clearance capture;
do not revert to the earlier 0-degree interpretation. The approximately 800 mm
scene clearance was measured from the scan origin, not the body edge.

In eventual drive mode, our swerve controller owns `odom -> base_footprint` and
publishes `/odom` at configured 50 Hz; its command input is `/cmd_vel`
(`geometry_msgs/msg/TwistStamped`). Robot-state publisher owns the base/wheel
tree. Feedback mode deliberately has no odometry publisher. The mapper would
own `map -> odom`; it has not been configured. Never add an odometry TF substitute
just to make a disconnected tree appear complete. Later sensor integration must
replace the temporary relative TF consistently, avoiding duplicate child owners.

### Next user-run check, starting from SSH

No transfer or rebuild is required for the reported successful wheel build.
Start with mock feedback only. In a fresh desktop terminal (Terminal 1):

```bash
ssh turtle@10.2.12.123
source /opt/ros/lyrical/setup.bash
source "$HOME/ros2_wheel_config_v1_ws/install/setup.bash"
export ROS_DOMAIN_ID=182
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 pkg prefix bringup
ros2 launch bringup drive_only.launch.py hardware:=mock mode:=feedback
```

The package prefix should resolve under `/home/turtle/ros2_wheel_config_v1_ws/install`.
If it resolves to an older deployment, correct the shell's sourced workspace before
launching. In another fresh desktop terminal (Terminal 2):

```bash
ssh turtle@10.2.12.123
source /opt/ros/lyrical/setup.bash
source "$HOME/ros2_wheel_config_v1_ws/install/setup.bash"
export ROS_DOMAIN_ID=182
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 control list_hardware_components
ros2 control list_controllers
ros2 topic echo /joint_states sensor_msgs/msg/JointState --once --timeout 10
```

Accept the mock step when `bus_drive` uses `mock_components/GenericSystem` and is
**inactive**, only `joint_state_broadcaster` is active, and all eight wheel/steering
joint positions and velocities are finite. No `/odom` is expected. Review these
outputs before proceeding; Ctrl+C in Terminal 1 stops this wheel launch only.

After the mock result passes, the next physical check is real feedback using the
same environment. The user checks the drive OpenCR path, serial access, motor
power/cabling and absence of another port owner. The assistant must explain that
this step **sends CONFIG**, pings the eight IDs, requests torque off and writes
operating modes, profiles and watchdog settings. It is not passive listening.
It neither flashes firmware nor repeats the colleague's EEPROM current/PWM edits.

```bash
ros2 launch bringup drive_only.launch.py hardware:=opencr mode:=feedback \
  port:=/dev/serial/by-path/platform-3610000.usb-usb-0:2.4.3:1.0
```

Repeat the Terminal 2 observations. Expected: `hardware/OpenCRHardware`, one
inactive `bus_drive`, only the joint-state broadcaster active and eight finite
joint measurements. Capture startup errors and reported communication/age faults
if this fails; do not substitute guessed IDs or reset zeroes. Nonzero steering
positions can be valid when wheels are not aligned with calibrated forward.
Successful feedback still does not prove physical torque-register state or
odometry accuracy. No real feedback output has yet been supplied for this profile.

`mode:=drive` is a separate later step that enables hardware/torque and the swerve
controller. Prepare a bounded user-run physical validation after feedback and
mechanical range are reviewed; do not start it as part of this handoff. Current
limits are 0.10 m/s and 0.5 rad/s. IMU use is intentionally off. Steering tick
bounds are not proof of cable or mechanical clearance.

### Gates between feedback, SLAM and navigation

1. Validate real wheel/steering feedback, signs, zeroes and timestamp continuity;
   then test bounded motion, stopping, independent distance/yaw and live sole TF
   ownership. Keep the current stationary-only LiDAR workflow separate from motion.
2. Resolve plate-to-base pose in the calibrated wheel frame. Do not fill nulls
   from the colleague's unrelated `base_scan` pose or from wheel geometry alone.
3. Define a motion-compatible scan/timing contract or an explicitly validated
   stop-and-observe approach. The experimental driver uses publication-time stamps,
   `time_increment=0` and a stationary guard; do not simply disable that guard.
4. Establish a body-filtered LaserScan from one physical sensor origin and valid
   odometry/TF for the first map. The merged filtered cloud is not that input;
   projecting it into a scan does not preserve either sensor's free-space rays.
5. Add mapping/localization/navigation packages and configuration after checking
   the target installation. Earlier inspection found Jetson mapping packages absent;
   desktop installation is not evidence of Jetson availability. No such packages
   were installed during the wheel integration. Navigation then needs a physical
   footprint, holonomic control, obstacle observations and fault/stop behavior.

### Latest user-reported Git state and checkpoint instructions

The last supplied `git status --short --branch` reports branch `feat/dual-lidar`
with no staged entries. The ten wheel-profile files no longer appear among working
changes, which is consistent with the user committing them after the instructions.
No commit output/hash was supplied; do not assert a verified new HEAD. The much
older `71fe42f` snapshot below is historical. Remote/push status is unknown.

Last reported remaining changes were:

- Modified: `config/lidar_mounts.yaml` and
  `ros2_ws/src/lidar_cloud_merge/test/test_cloud_core.cpp`.
- Untracked: `config/lidar_body_view.yaml`, `tools/lidar_body_review/`,
  `tools/opencr_diagnostics/`, seven navigation documents and four Jetson figures
  listed below, plus `.codex-attachments/` (local photos to preserve and exclude).

The user was given these additional focused commit instructions, but no execution
results were provided before requesting this handoff:

| Suggested subject | Exact scope |
| --- | --- |
| `feat: add assembled-robot lidar body-filter profile` | Both LiDAR YAML files above, cloud-core test, and `tools/lidar_body_review/{CMakeLists.txt,README.md,review_body_mask.cpp,review_live_cloud.cpp,plot_review.py}` (8 files) |
| `feat: add receive-only OpenCR status diagnostics` | `tools/opencr_diagnostics/{README.md,listen_status.cpp,test_listen_status.cpp}` (3 files) |
| `docs: record lidar integration evidence and SLAM readiness` | Seven documents and four figures below (11 files) |

Documentation scope: `docs/navigation/{dual_lidar_desk_alignment.md,
dual_lidar_setup.md,dual_lidar_handoff.md,jetson_lidar_body_view.md,
jetson_lidar_capture_review.md,jetson_lidar_clearance_review.md,slam_readiness_audit.md}`.
Figure scope under `docs/navigation/figures/`: `jetson_lidar_body_mask_20261027.png`,
`jetson_lidar_body_mask_v2_20261027.png`,
`jetson_lidar_clearance_comparison_20261027.png`,
`jetson_lidar_raw_capture_20261026.png`.

If Git status matters to the next action, ask the user to run `git log -4 --oneline`
and `git status --short --branch`; do not infer completion of the three commits.
Do not re-stage or amend prior work automatically. This handoff itself was updated
after those instructions and is left for user review/staging. Git publication is
not a prerequisite to the already built Jetson mock feedback check.

### Files and evidence to resume from

- [Wheel workflow and validation](../../tools/wheel_base/README.md),
  [hardware map](../../config/hardware_map_drive_only.yaml),
  [controller geometry](../../config/controllers_drive_only.yaml),
  [wheel launch](../../ros2_ws/src/bringup/launch/drive_only.launch.py),
  [wheel model](../../ros2_ws/src/description/urdf/drive_only.urdf.xacro).
- [Stationary body-view evidence](jetson_lidar_body_view.md),
  [SLAM readiness audit](slam_readiness_audit.md),
  [receive-only diagnostic](../../tools/opencr_diagnostics/README.md).
- Original raw bag: `/home/turtle/jetson-lidar-capture.2Vpg55/bag`.
  Clearance bag: `/home/turtle/jetson-lidar-clearance.nm615e/bag`.
  Verified filtered bag: `/home/turtle/jetson-lidar-filtered.W4XCaX/bag`.
  Passive status capture: `/home/turtle/jetson-opencr-passive.yj8V9W`.
  Preserve saved data; do not rerun hardware captures without a specific need.
- Colleague evidence archive: `/home/pa4/Downloads/lidar_scan_jetson_20261027.tar.gz`;
  its checksum and calibration provenance are in the recovery checkpoint below.
  The earlier `drive-download-20261026T015146Z-1-001.zip` contains mounting STLs,
  not the needed drive configuration.
- Desktop validation/output folders under `/tmp` are temporary. They may disappear
  across restart or cleanup; repository sources, user-held archives and Jetson bags
  are the recovery inputs. Do not assume another machine shares these paths.

Suggested prompt for the next session:

> Read /home/pa4/git/Project2_Turtle_Ninja/docs/navigation/dual_lidar_handoff.md,
> starting with "Current state — Resume with Jetson wheel feedback." Resume the
> next user-run mock feedback check on Jetson, then prepare real torque-off feedback
> after reviewing its result. SLAM is the intermediate goal and navigation the
> ultimate goal. I execute hardware and Git commands; read-only SSH inspection is
> allowed. Preserve our LiDAR/desk setup, use only the recovered wheel configuration,
> keep unknown mounting geometry explicit, and explain the next steps in chat.

## Latest checkpoint — Our calibrated wheel-only profile builds on Jetson

The user said to proceed with the recovered wheel configuration while prioritizing
our setup. New, opt-in files are `config/hardware_map_drive_only.yaml`,
`config/controllers_drive_only.yaml`,
`ros2_ws/src/description/urdf/drive_only.urdf.xacro`, and
`ros2_ws/src/bringup/launch/drive_only.launch.py`. These use our existing hardware
and swerve controller packages, not the colleague's controller/SLAM/teleop stack.
The recovered motor IDs, signs, zeroes and ordinary motor profiles are retained.
Both the controller and new wheel model read the same geometry in calibrated
axes (+X=old CAD -Y); this fixes the colleague's model/controller disagreement
without changing the legacy full-robot model. The new model contains only base
and wheel frames; arm, IMU and LiDAR mounting transforms are not guessed.

Defaults are `hardware:=mock mode:=feedback`. Feedback mode sets `bus_drive`
INACTIVE and activates only joint_state_broadcaster. Real `hardware:=opencr`
requires an explicit absolute `port:=` path and **does send CONFIG**, changing
motor mode/profiles/watchdog and requesting torque off; it is not passive serial
listening. `mode:=drive` separately enables hardware/torque and our swerve
controller. Its limits remain 0.10 m/s, 0.5 rad/s; ROS command limits remain on.
Steering limits are derived from calibrated motor ticks with a 20-tick margin,
not asserted mechanical/cable clearance. IMU use is off pending mounting/freshness
verification. Our existing nearest-feasible steering behavior is retained;
coherent/outward-preference/blend behavior was not imported.

The shared ros2_control Xacro now reads an optional `torque_off_on_deactivate`
setting, default false as before. The wheel-only map opts into true. Full-robot
torque behavior was checked unchanged. Hashes confirm unchanged LiDAR YAML,
original hardware/controller YAML, robot.launch.py and robot.urdf.xacro.
The assistant executed only offline/simulated checks; the user subsequently
transferred and built the source bundle on Jetson as recorded below. No physical
wheel runtime validation has been reported for this profile. The assistant ran no
Git commands; the latest user-reported Git state is summarized above.

Validation root: `/tmp/wheel-base-integration.uUoKAL`.
Six packages built on desktop Lyrical. The existing hardware/controller suite
passed 73 tests; new launch checks passed 13 invalid-input cases before node
creation, and C++ validation passed IDs/zeroes/signs, shared TF geometry, limits
and 33 motion cases. Xacro/check_urdf passed. Local mock checks in isolated domain
218 confirmed inactive hardware plus joint feedback, then active drive with
effective steering bounds, +X odometry (about 35 mm) and zero velocity after
command timeout. Local ROS socket restrictions required an approved unsandboxed
mock test; no physical ports were used. Physical execution remains user-controlled.

Prepared source-only bundle (4.3 MiB):
`/tmp/wheel-base-integration.uUoKAL/jetson-wheel-config-20261027-v1.tar.gz`.
SHA-256: `c2d4c930e8a046c1574665b0e7e0345905e8303230c104d52447ce843fddfe7d`.
Single top-level directory `jetson-wheel-config-20261027-v1`; internal
`SOURCE.sha256` passes. It includes our six required source packages, configuration,
firmware source/header and validation tools, but no firmware image or build/install
artifacts. The user verified the archive checksum, extracted it and built all six
packages on Jetson in separate `~/ros2_wheel_config_v1_ws`: description, hardware,
interfaces, manipulation, swerve_controller and bringup. The pasted build output
reports six packages finished in 2 min 30 s, with CMake minimum-version and
CMP0167/FindBoost policy warnings but no errors. This Release build used
`BUILD_TESTING=OFF`; it does not establish Jetson test or runtime results.
Use domain 182 for wheel verification while live LiDAR domain 181 remains unchanged.
The next user step is mock feedback on Jetson; after that, user-run real
feedback should show one inactive drive bus and eight finite joint measurements.
See `tools/wheel_base/README.md` for exact commands and CONFIG semantics.
No real drive/SLAM admission follows from the mock check; plate-to-base pose,
physical odometry accuracy and motion-compatible scan timing remain open.

# Historical checkpoints and supporting evidence

The following records preserve the investigation in order of accumulated updates.
Older claims that IDs are unknown, only mock evidence exists, deployment is still
pending, or a prior revision is current have been superseded by the current-state
section above. Historical commands describe past experiments; do not execute them
as the next step without checking their scope against that section.

## Earlier checkpoint — Wheel configuration recovered; preserve our integration

The user supplied `/home/pa4/Downloads/lidar_scan_jetson_20261027.tar.gz`
(SHA-256 `c6b47e1bbfd11d5250843ee566f93d6d17d18c6800b7dfb0b77bff89a81b531a`).
Inspection copy: `/tmp/lidar-colleague-review.T42dLS/lidar_scan_jetson_20261027`.
All 167 manifest entries pass. The user's latest scope is explicit: **prioritize
our setup; use this bundle only for the wheel-system configuration**. Do not
replace our stack with its SLAM launch, LiDAR defaults, teleop, speed overrides
or old `nt_*` package layout. No runtime configuration or hardware was changed.

The useful source is `ros2_ws/src/nt_description/config/hardware_map_drive_only.yaml`.
The bundled full-robot `hardware_map.yaml` still has assumptions. The drive-only
IDs/signs/zeroes are supported by October 26 run-specific maps, motor register
readbacks and physical OpenCR test records, superseding the earlier default ID
table. There has been no fresh motor readback on today's connected Jetson.

| Retained joint suffix | Wheel ID | Wheel scale | Steering ID | Steering zero_tick | Position in colleague's new forward convention |
| --- | --- | --- | --- | --- | --- |
| fl | 1 | +1 | 11 | 3066 | Rear-left |
| fr | 2 | -1 | 12 | 1552 | Front-left |
| rl | 41 | -1 | 13 | 2577 | Rear-right |
| rr | 42 | +1 | 14 | 3589 | Front-right |

All steering scales are -1. Wheel models/modes are XM430-W210/velocity;
steering models/modes are XM430-W350/position. The supplied drive-only profile
has wheel profile_acc 20 and steering profile_vel 40/profile_acc 10. Higher-speed
test-run profiles differ; do not substitute those implicitly. Bus baud is
1,000,000, watchdog 15, and require_all_motors true. Steering zeroes are in this
map; no separate calibration file is needed for them.

Geometry reference is `ros2_ws/src/nt_bringup/config/drive_only.yaml`, with radius
0.033 m in `controllers.yaml`. In fl/fr/rl/rr order, module X is
`[-0.064, 0.064, -0.064, 0.064]`, Y `[0.112, 0.112, -0.112, -0.112]`, and wheel
offset `[0.0269, 0.0269, -0.0269, -0.0269]` metres. These use NEW forward = old -Y,
with `(x_new,y_new)=(-y_old,x_old)`. Joint names retain their old location labels.
Our existing URDF/controller baseline uses old axes, so do not copy this geometry
or calibration independently of its coordinate convention. Reconcile the wheel
frame with our base definition before applying it; plate-to-base mounting nulls
remain explicit. The archive's `steering_policy: coherent` is also controller
behavior, not a reason to replace our implementation during configuration review.

Physical test evidence now exists: `evidence/runs/floor_cal_20261026T144403`
contains real OpenCR activation and ten basic-motion passes, with `/odom`-derived
295–300 mm translations for 300 mm commands. Independent floor measurements in
the JSON are null; the accompanying written report records a later photo/user
distance assessment of -20 to +5 mm. Independent rotation accuracy remains open,
and continuous-direction tests include failures. This evidence corrects the
earlier mock-only picture without accepting the current Jetson stack for SLAM.

Offline only: the data-only description package built and its drive-only Xacro
passed `check_urdf` in `/tmp/lidar-colleague-review.T42dLS/validation`; no nodes or
hardware plugins were started. It confirms the colleague's URDF still uses old
module origins while the drive-only controller uses new axes. Treat that as an
integration issue, not a validated replacement model. Live LiDAR setup, desk
defaults, Git, firmware and device state were untouched.

## Earlier checkpoint — OpenCR status protocol observed; motor mapping pending

The user built and ran the receive-only tool on Jetson, saving
`/home/turtle/jetson-opencr-passive.yj8V9W` on October 27 around 10:22 KST.
Five seconds captured 27,448 bytes: **502 STATE + 5 DIAG**, no other frames,
malformed payloads or bad checksums. Board timestamps advance from 682420 to
687430 ms with no nonadvancing stamps: nominally 100 Hz across 501 intervals.
The report has `last_joint_count=0`, `last_status=4`, `status_or=4`,
`last_comm_error_mask=0`, `last_configured=0`, `last_torque_flag=0`,
`last_imu_flag=1`. This establishes advancing project-compatible telemetry,
not an exact flashed build or valid physical wheel feedback.

Zero configured joints is expected before CONFIG in the repository firmware;
it is not a count of connected motors. With no configured motor polling, a zero
communication-error mask does not establish motor communication, and a clear
torque flag is not a per-motor Torque Enable register readback. The IMU flag
reports initialization, not mounting, bias, accuracy or freshness acceptance.

Read-only SSH independently retrieved the saved summary text and hashes:
`capture.bin` SHA-256
`b9cf8ff159b3ea66bd628de13660af0925523b96c0f1553695941d43cca870bc`;
`summary.txt` SHA-256
`a7aa5250b6fc92eee027c87bdbc62221dba312633b047a6f37386ddab0dbbad9`.
The assistant did not open the live serial port or send device commands.

The user asked the assistant to find the configuration in the repository instead
of supplying IDs from memory. The expanded search covered current source and
ignored/hidden configuration and log files (excluding Git internals), repository
attachments, and Jetson's source, installed configuration and original deployment
archive. It found the same **candidate** map: FL wheel/steer 1/5, FR 2/6, RL 3/7,
RR 4/8. `config/hardware_map.yaml:12` explicitly calls IDs 1–8 assumptions;
line 34 references absent `tools/bench_drive_2026-10-16.yaml`. No alternate
confirmed map or steering zero calibration was found. Jetson's source/installed
maps have identical SHA-256
`5fa01fb83e5423b738e24009964058bb79e4a655192435eea743cfb0bc97eb49`.
The older archive contains the same hardware-map path and no bench calibration.
Git history/other branches were not inspected; the user owns Git commands.

Use the table as a candidate for hardware identification, not an accepted physical
map. Do not keep asking the user to recall the same IDs. Also note that README's
W210/W350 drive/steer assignments are reversed relative to hardware_map.yaml;
actual motor models need physical/readback confirmation too. Do not
use guessed CONFIG as motor discovery: `bus_configure()` pings each supplied ID,
then writes torque-off, return delay, operating mode, profiles and watchdog.
The current wire protocol has no separate read-only motor-discovery request.
After identifying the correct mapping/settings, prepare a separate drive-only,
torque-off feedback procedure; do not activate the full wheel/arm ROS stack.
The manipulator remains disconnected. The LiDAR setup and mounting unknowns remain
unchanged; physical odometry remains unverified.

## Drive connection and receiver package

At 10:11 KST on October 27 the user connected the **wheel-base OpenCR only**;
the manipulator OpenCR remains disconnected. Its USB path is
`/dev/serial/by-path/platform-3610000.usb-usb-0:2.4.3:1.0` -> `/dev/ttyACM0`.
By-ID is `usb-ROBOTIS_OpenCR_Virtual_ComPort_in_FS_Mode_FFFFFFFEFFFF-if00`;
uniqueness of this serial across boards is not established. The LiDAR paths remain
rear `2.4.2`, front `2.4.4`. Read-only SSH confirmed dialout read/write access,
USB VID/PID 0483:5740 and no PID reported by `fuser`. No physical serial port was
opened by the assistant. USB identity does not identify the installed firmware.

The completed five-second receive-only capture was instructed with motor power
off and the current LiDAR processes/cabling unchanged. C++ source in
`tools/opencr_diagnostics/listen_status.cpp` opens the input O_RDONLY, sets host
raw mode, and captures/decodes STATE and DIAG using the repository packet header.
It sends no application command bytes, CONFIG or CMD; no motor IDs, torque or
ROS controllers are configured. Opening/closing a tty can affect control lines,
so this is not a promise against reset with unknown firmware. The host tty remains
raw on exit to avoid restoring terminal echo onto a continuing stream.

Desktop builds with warnings as errors and PTY tests pass: fragmented valid
packets, corrupt checksum rejection, silence/unknown text, exact raw capture and
no bytes transmitted toward the simulated device. The user exercised the real
board as recorded above; the assistant did not. The firmware name/build remains
unconfirmed. Matching packets identify a wire format, not exact firmware provenance
or physical odometry. No repeat passive capture is needed for this check.

Source-only user-transfer bundle:
`/tmp/jetson-opencr-passive.ewtFmu/jetson-opencr-passive-20261027.tar.gz`.
SHA-256: `46b75a4fa0915d01e6b1e539c8996d8eb516a12e16e46f4d6915fca9d094ce54`.
It includes source, a packet header, PTY tests, instructions and an internal hash
manifest; no firmware upload image or desktop binary. Build/run on Jetson remains
user-controlled. The full robot launch remains stopped; the missing manipulator
is not needed for this receive-only check. Physical odometry remains unverified.

## Completed stationary LiDAR check and odometry provenance

The user recorded `/home/turtle/jetson-lidar-filtered.W4XCaX/bag` on October 27,
09:34:50–09:35:31 KST: 40.35 s, 405 front scans, 404 rear scans, 405 filtered clouds,
and one static TF message. Read-only SSH copied the saved data and verified both
file hashes. Local evidence: `/tmp/jetson-filtered-review.6b4iQ9/bag`; final C++
analysis: `/tmp/jetson-filtered-review.6b4iQ9/verified`.

All **404 complete input pairs reproduce the live cloud exactly**, including XYZ,
intensity, source ID and survivor order. All 405 clouds have the expected frame,
layout, finite values and no points inside the revised box. Recorded front -> rear
TF is (+0.220,0,0) m with yaw pi. Output header cadence is 10.0273 Hz; cloud widths
are 411–440 (mean 425.47). Complete pairs remove 137–153 points (mean 145.90).
All recorded raw returns below 0.30 m are excluded by this box, and all valid
returns at or beyond 0.30 m are retained. This is a capture-specific result, not
a universal clearance guarantee or physical identification of every excluded point.

Counts differ at recording boundaries: cloud 0 has 233 front/193 rear points but
neither source scan was captured; front scan 404 occurs after the final recorded
cloud. There are no unexplained interior missing inputs, reused matched scans,
repeated/backward stamps or approximately 200 ms header gaps. The first cloud's
raw provenance cannot be checked. Matched input stamp skew is 0.033–41.538 ms;
this does not establish acquisition synchronization. `time_increment` remains zero.

The user reports prior robot odometry testing and identifies Jetson with workspace
`/home/pa4/git/Project2_Turtle_Ninja/ros2_ws`. Read-only SSH on October 27 found that
path absent from `turtle@10.2.12.123`; it exists on the desktop. The discovered
Jetson robot stack is the older deployment
`/home/turtle/orin_mock_20261018.03inTQ/deployment_source`, with `nt_bringup` and
`nt_control` rather than this checkout's `bringup` and `swerve_controller` names.

Its `mock_orin_02/result.json` explicitly says `GenericSystem ROS wiring only`,
both hardware components use `mock_components/GenericSystem`, and
`physical_motor_commands=false`. The matching `preflight/mock_smoke.py` launches
`hardware:=mock mission:=false use_rviz:=false` in domain 189. Its saved checks
cover joint feedback, control wiring and command timeout, not a measured `/odom`
trajectory. The bundled `preflight/TARGET_STATUS.json` predates this target run;
do not use its "Orin unverified" fields to dismiss the later mock runtime result.

Asked whether the recalled test used mock or physical motor feedback, the user
answered **"I'm not sure."** Record physical odometry as unverified, without
claiming a separate test never occurred. Before the user connected the drive
OpenCR at 10:11, no robot controller processes were visible during inspection,
and `/dev/opencr_drive`, `/dev/opencr_arm` and `/dev/ttyACM*` were absent. That earlier
USB snapshot showed only the two known LiDAR CP210x bridges among serial devices;
the current drive connection is recorded above.

Drive-controller USB identification and passive protocol observation are complete;
actual motor mapping/settings are next. Keep LiDAR cabling and running processes unchanged.
Do not start the old full robot launch or send CONFIG/motion commands
to guessed ports/IDs. Both discovered and desktop maps still label drive/steer
IDs as assumptions and reference a missing bench calibration. Establish the
actual controller/firmware and feedback configuration before physical odometry
validation. The current repository protocol has CONFIG/CMD packets; there is no
generic text HELLO diagnostic to borrow from unrelated projects.

The discovered controller config declares `/odom`, `odom -> base_footprint`,
`publish_tf=true`, 50 Hz and `base_footprint -> base_link` at +0.033 m Z in the
model. Its old description has a single legacy LDS frame and no dual-LiDAR launch
arguments. These are source settings, not live TF observations or calibration.
Resolve the tested setup first, then connect its actual base/odom frames to the
assumed plate-relative LiDAR geometry. Plate X/Y/yaw stay unknown. A physical-origin
LaserScan mapping path and motion-compatible input contract are still required;
raw scan topics remain unfiltered and the merged cloud is a stationary view.

The stationary body-filter output check is complete; no repeat capture or redeploy
is needed for it. Runtime defaults, hardware, live processes and Git index/history
were left unchanged. Domain 181 is the current live stationary LiDAR domain; do not replay a
bag into it while those processes run. Preserve the desk setup and unknown mounting
geometry. See [details and hashes](jetson_lidar_body_view.md#recorded-live-output-check).

## Earlier checkpoint — revised body filter running on Jetson

On October 27 the user built `lidar_cloud_merge` successfully in
`/home/turtle/ros2_lidar_body_view_v2_ws` (23.6 s) and started the relative TF and
cloud node using `/home/turtle/lidar_body_view_v2.yaml`. The executable lookup
succeeded after correcting `ros2 pkg executable` to `ros2 pkg executables`.
TF is front -> rear, (+0.220,0,0) m and yaw pi. The first cloud lookup failed during
startup; about 0.1 s later pairing succeeded, followed by regular filter log lines.
The supplied logs removed 143-152 points and retained 425-433 per reported pair.

Read-only SSH independently found both raw drivers, the TF publisher, the cloud
node and its TF listener. `/lidar/points_body_filtered` has one Best Effort publisher.
The parameter dump confirms length 0.2552, width 0.3050, margin 0.010, centre
(0.110,0.005), self filtering and stationary acknowledgement enabled, and system
time. One live message had frame `lidar_front_link`, width 419, point_step 20,
row_step/data length 8380 and is_dense=true. These are bounded startup/publication
checks, not a measured output rate or physical alignment acceptance.

The requested raw-scan/cloud/static-TF recording and desktop inspection are now
complete, as recorded in the latest checkpoint above. No robot base/odometry TF
or mapping input was added. Hardware/Git actions remain user-controlled; assistant
SSH was read-only.

## Earlier checkpoint — assumed CAD geometry and offline body-box review

Revision 2 supersedes the box and deployment bundle in the following historical
paragraphs. The user supplied the FL0 LiDAR Specification and requested its box:
tight 255.2 x 305.0 mm, plus 10 mm per side = **275.2 x 325.0 mm**. Plate bounds
are X +/-0.1376, Y +/-0.1625 m. Front-frame bounds are X [-0.0276,0.2476],
Y [-0.1575,0.1675] m. `config/lidar_body_view.yaml` supplies these explicit values;
runtime desk defaults remain unchanged. Per-sensor LaserScan filtering was not added.

Replay of the same 550 front / 557 rear scans removes 35,375 / 51,578 valid points,
retaining 107,792 / 97,770. Sub-30 cm survivors fall from 127 / 713 to **1 / 6**;
all recorded returns at or beyond 30 cm are retained. Bounds/survivor checks, new
regression cases and config-to-review agreement pass. Seven residual samples lie
outside the specified Y bounds; do not enlarge the mask to fit every return.

Current bundle: `/tmp/lidar-body-mask-v2.SS87Fc/jetson-lidar-body-view-20261027-v2.tar.gz`.
SHA-256: `e1772397ff9b61798a8a25cef69f763be4bc87de754875bb0893b75da120a206`.
If the old node is built, only the new YAML and a cloud-node restart are needed;
drivers and relative TF can remain running. No assistant deployment or hardware
command occurred. The previous PDF's missing width is resolved by the supplied
full-steering envelope. Base-to-plate placement and motion readiness remain open.

The user explicitly accepted `FL0_LiDAR_Plan.pdf` as the real assembly on
2026-10-27 and asked to proceed. Exact 90-degree wheel-shadow reconstruction is
no longer a prerequisite for the rectangular-mask step. `config/lidar_mounts.yaml`
now records assumed front yaw pi and rear yaw zero; plate X/Y/yaw remain null.
Plate-relative +/-110 mm X, +5 mm Y and +26.5 mm scan height are accepted drawing
assumptions; the older 28 mm ground-height estimate is not established by this PDF.

The new offline C++ review reuses the existing merger's filter. On the clearance
bag it removes 35,249 front / 50,871 rear points and retains 107,918 / 98,477.
Only 127 / 713 returns below 30 cm remain; no recorded returns at 30 cm or farther
are removed. Small residuals lie beyond the box edges, and the PDF's full steering
sweeps extend beyond that box. Treat it as a candidate stationary mask, not proven
complete body coverage. No raw LaserScan filter or mapping input was implemented.

The C++ checks and model/launch guards pass. See
[body-view result and user commands](jetson_lidar_body_view.md). The desktop source
bundle is `/tmp/lidar-body-review.DTdSGM/jetson-lidar-body-view-20261027.tar.gz`.
Bundle SHA-256: `7bafff8b0e27fb1b8892635ce8700141484e7eb1228ae765e42e0d8aa8580fae`.
Read-only SSH found only the two raw driver nodes and no merger/TF. Next the user
copies/builds the bundle and starts one relative TF publisher and the filtered
cloud node on Jetson. The assistant has not deployed or started them. Raw scans,
desk defaults, motion guards, hardware and Git ownership are preserved.

## Earlier checkpoint — scene-clearance comparison complete

The user recorded `/home/turtle/jetson-lidar-clearance.nm615e/bag` on October 27
08:14–08:15 KST: 55.37 s, 550 front / 557 rear scans. Scene note `800 mm` was
clarified as approximately from a LiDAR scan origin. Read-only SSH copied and
verified the bag; see [clearance comparison](jetson_lidar_clearance_review.md).

No repeated/backward scan stamps or approximately 200 ms gaps were found. Rates
were 9.918 Hz front and 10.044 Hz rear, explaining the unequal counts. Yesterday's
broad close sectors (front 119.7–226.8°, rear 125.1–186.3°) contain zero returns
below 0.30 m in the new bag. Those sectors must not become a permanent body mask.
Smaller stable clusters remain: 51 front / 71 rear bins were persistently below
0.30 m in both bags. Their physical source is not yet identified.

The later supplied `/home/pa4/Downloads/FL0_LiDAR_Plan.pdf` predicts standoff and
wheel/motor shadows at scan height. The user reports **90-degree steering** during
the new capture; the PDF's exact wheel/motor table is for **0 degrees**. Use its
full-steering sweep only as a possible-obstruction envelope, not a 90-degree
prediction. Standoff matches remain useful; exact wheel/motor identification
needs the matching CAD pose. Per-module calibrated angles/signs and the earlier
capture's steering pose were not independently established. Front scan zero also
disagrees with the PDF's opposite-LiDAR housing prediction. See the clearance
comparison's CAD section; no mounting values or masks were changed.

The bounded comparison is done; do not repeat it or restart fine timing research
without a new reason. Next: physically identify likely obstructions near scan
height and obtain installed mounting references/scan-zero directions. Current
mount YAML still leaves plate X/Y/yaw and both sensor yaws null. No filter,
driver configuration or robot TF changed. Odometry and the mapper/input contract
remain separate prerequisites for the first map. Hardware actions and Git remain
user-controlled; the last observed driver processes were user-started for this bag,
and their current running state has not been checked or changed by the assistant.

## Historical audit entry point — superseded by current state

The 2026-10-26 software audit is now recorded in
[Robot SLAM readiness audit](slam_readiness_audit.md). Read it before resuming.
Measured holonomic odometry and its TF publisher are implemented; SLAM Toolbox,
Nav2 and robot_localization are installed locally but not configured for this
robot. Physical odometry calibration, robot LiDAR extrinsics and a moving-platform
scan contract remain gates. A one-sensor SLAM Toolbox baseline is proposed, not
selected or implemented. Preserve the stationary guard and desk defaults.

Hardware/interfaces/swerve packages built in `/tmp/slam-readiness-audit`; existing
hardware and swerve checks passed (colcon: 73 tests, zero failures/skips). Synthetic
tests do not validate robot motion, IMU mounting or live DDS. No hardware or Git
commands ran; the Git snapshot below remains historical. Only this handoff and
the audit document were changed during the audit.

Next: resolve the mapper/input-mode decision, recover the actual wheel/steering
calibration with the user/team, and collect robot mounting values. Follow the
audit's ordered user validation and separate mapping configuration plan. Do not
repeat the completed software inventory or treat installed packages as integration.

### Jetson stationary raw scan checkpoint

Later on 2026-10-26, the user deployed and ran the stationary experimental driver
on the Jetson with both LiDARs installed on the robot. User-provided outputs confirm:

- Host `turtle@10.2.12.123`, aarch64 Ubuntu 24.04.5, ROS Lyrical under
  `/opt/ros/lyrical`. System clock synchronized, NTP active, Asia/Seoul timezone.
- User added `turtle` to dialout and reconnected; both ttyUSB devices have read/write
  access. No udev deployment was needed.
- Unplugging only the front LiDAR left USB path `2.4.2`; front is `2.4.4`, rear is
  `2.4.2`. Use these Jetson paths, not the desktop paths below:
  - Front: `/dev/serial/by-path/platform-3610000.usb-usb-0:2.4.4:1.0-port0`
  - Rear: `/dev/serial/by-path/platform-3610000.usb-usb-0:2.4.2:1.0-port0`
  The matching `usbv2` links are aliases to the same devices. One by-id link with
  serial `0001` cannot distinguish the two sensors. Socket identity depends on wiring.
- Source subset at `/home/turtle/jetson-lidar-source`: `upstream/` plus
  `project/tools/lidar_diagnostics/` build inputs and `project/config/lidar_drivers.yaml`.
  The full project repository, bringup and merger were not deployed in these steps.
  Bundle SHA-256: `20b9200b9caadd8c9e192834b07db67198a263f5b2ed2b5c1566975f9bb54a86`.
  Desktop bundle: `/tmp/jetson-lidar-source.DvMue1/jetson-lidar-source.tar.gz`
  (temporary; source inputs remain in the repository and local upstream directory).
- All upstream manifest checks and patch applications passed. Native build in
  `/home/turtle/ros2_lidar_experimental_ws` succeeded, with only a CMake minimum-version
  deprecation warning. Executable: `coin_d4_experimental_driver experimental_coin_d4_node`.
- User started separate `/lidar_front/driver` and `/lidar_rear/driver` processes,
  using the supplied YAML, explicit Jetson ports, `stationary_test=true` and
  `use_sim_time=false`. Test terminals use `ROS_DOMAIN_ID=181` and
  `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`; observers must use those same settings.
  This setup does not provide remote desktop discovery or robot TF.
- Echoes show the correct `lidar_front_link`/`lidar_rear_link`, 400 ranges and
  intensities, angle span 0–6.267477512 rad, increment 0.015707964 rad,
  range limits 0.05–12 m and `time_increment=0`.
  Front scan_time=0.099672 s, rear=0.100186 s in separate samples.
- Observed front rate settled near 10.020 Hz (window 199), rear near 10.048 Hz
  (window 170). The initial front topic-discovery warning was followed by data.
  These short observer windows do not measure acquisition synchronization or
  long-duration loss; the two separate echoes do not measure inter-sensor skew.

The user subsequently authorized assistant SSH inspection and installed the desktop
SSH key. Read-only SSH now works; hardware actions and Git remain user-controlled.
The saved capture `/home/turtle/jetson-lidar-capture.2Vpg55/bag` was copied locally
with matching hashes and inspected: 274 scans per sensor over 27.23 s, consistent
metadata, no repeated/backward stamps, and valid fractions 74.745% front / 78.387%
rear. See [Jetson capture review](jetson_lidar_capture_review.md) for evidence and
separate sensor-frame plots. The user confirmed capture on the floor near objects
or walls; close returns cannot yet be classified as body occlusion. No mask changed.
Mapping packages present on the desktop are absent from Jetson's `/opt/ros/lyrical`
prefix: slam_toolbox, nav2_bringup, nav2_amcl and robot_localization were checked.

This establishes native build, device access, publication and bounded raw-data
inspection on Jetson. Body occlusion, quantitative alignment, robot extrinsics,
odometry, moving scans and SLAM remain unvalidated. A comparison stationary bag
with nearby external objects cleared would help distinguish scene returns from
robot-attached candidates. It must not restart the deferred fine-timing investigation.

Full-day learning note: [2026-10-26 LiDAR and SLAM learning — identity, geometry, timing, and evidence](https://app.notion.com/p/4053980c8a448125a2b5f6f317935c47?pvs=204).
Created as a private Notion draft at the user's request, with the scan plot and an
evidence ZIP containing the bag, analyzer source and report snapshots. Revised to
cover October 26's V1/V2/V3 LiDAR/SLAM learning, emphasizing engineering concepts,
evidence boundaries, recall questions and next experiments. The user explicitly
excluded unrelated learning; detailed Jetson operations remain in an appendix.
API readback confirmed the revised content and both original attachments; browser
rendering was not checked. The engineering-notion-learning-note skill was revised
and validated to preserve this learning-oriented approach and explicit topic scope.
Shutdown instructions were provided to the
user (stop recorder, stop both drivers, `sudo poweroff`); shutdown itself was not
performed or verified by the assistant. Check actual host state when resuming.

### Original audit scope retained for context

The user's intermediate goal is **SLAM using the assembled robot**; the ultimate
goal is autonomous navigation. The next task is a **SLAM-readiness audit of the
existing robot stack**: inspect odometry, TF ownership, sensor interfaces and existing
mapping/navigation packages, then identify the smallest working SLAM configuration.
Do not restart diagnostic research or add duplicate suppression as a prerequisite.

Start by reading this section, applicable AGENTS.md files and the personal skills
no-git-history and project2-cpp-conventions. Recheck Git and current files. Audit
existing code/configuration before asking for information it already contains.
Implement justified, hardware-free fixes within the audit's scope; do not silently
choose a mapper or invent mounting dimensions when these materially affect design.
The audit linked above now traces the code/configuration paths listed below;
physical odometry/control behavior remains unvalidated by that software audit.

## User boundaries and decisions

- User executes hardware commands and Git operations. Do not start LiDAR drivers,
  motors or controllers, install/reload udev rules, stage, commit, push or alter refs.
- The user briefly requested an assistant commit, but staging failed on read-only
  .git and the escalation was declined. They then explicitly resumed committing
  themselves. All listed commits were made by the user, not the assistant.
- Implementation is C++; minimal Python ROS launch/Xacro glue and tests of those
  APIs are accepted. Shell build/test orchestration is present.
- Commit messages: English with lowercase type prefix. Provide focused staging sets;
  do not use git add . or include attachments indiscriminately.
- Fine calibration and timing research were deliberately deferred. Do not reconfirm
  accepted desk dimensions or repeat completed restart tests without a new reason.
- Preserve raw sensor scans and origins. Different raw ranges from separate origins
  are expected; compare transformed surfaces. Do not average ranges or hide double
  walls through downsampling before identifying the source of disagreement.
- Teammates will update robot geometry. Final base_link extrinsics must follow those
  specifications. Unknown geometry blocks physical validation, not a software audit.

## Historical verified repository state

Repository: /home/pa4/git/Project2_Turtle_Ninja. Branch: feat/dual-lidar.
HEAD verified at handoff: 71fe42f. Index empty; tracked worktree clean before editing
this untracked handoff. Remote/push status was not checked.

Recent user commits, newest first:

| Commit | Scope |
|---|---|
| 71fe42f | Disabled udev deployment templates and identity instructions |
| e461c13 | Timing provenance, offline models/checks and evidence |
| 370d906 | Isolated original-conversion diagnostic capture |
| d9b0374 | Guarded dual-LiDAR robot-model integration |
| de87c0b | Reproducible stationary desk bringup and RViz |
| 5b7b654 | Configurable robot-body exclusion |
| 28a2f27 | Stationary point-cloud merger |
| 6ca1bae | Isolated stationary experimental conversion |

Remaining untracked files/groups:

- .codex-attachments/ — preserve.
- docs/navigation/dual_lidar_desk_alignment.md — historical intermediate workflow.
- docs/navigation/dual_lidar_setup.md — historical source/build/experiment narrative;
  some default-driver and next-step statements are superseded.
- docs/navigation/dual_lidar_handoff.md — this updated handoff, not committed.

The earlier handoff's long list of uncommitted runtime/tooling files is obsolete.
Do not re-stage already committed features. No Git mutations are needed to resume.

## Current functional milestone

The stationary desk chain works: two experimental LDS-03 drivers -> separate raw
scans -> TF projection -> merged PointCloud2 -> optional body exclusion -> RViz.
Front AND rear driver restart recovery were user-verified without restarting the
merger. This is not yet moving-platform SLAM or navigation integration.

coin_d4_experimental_driver / experimental_coin_d4_node publishes 400 bins,
angle_min=0, increment about 0.015707964 rad, max about 6.267477512 rad, range limits
0.05–12 m. Both sensors were stable near 10 Hz in earlier stationary checks.
Working settings: version 4, baudrate 230400, reverse=false, warmup_time=5,
stationary_test=true. Header stamps are publication times; time_increment=0.
Acquisition timing and motion compensation remain unresolved.

The merger consumes /lidar_front/scan and /lidar_rear/scan, publishes
/lidar/points_merged in lidar_front_link, and retains x/y/z/intensity/source_id
(float32, 20 bytes per point). IDs: front=0, rear=1. No averaging, duplicate removal,
resampling or free-space ray inference occurs. Default max_age=0.3 s and
max_skew=0.075 s apply to publication/receipt timing, not synchronized acquisition.
Each pair requires fresh data from both sensors; no single-sensor fallback.
The output stamp is the later input stamp. Rejected-pair rate is not measured.

The cloud is valid for the tested stationary visualization. A rear endpoint expressed
in the front frame does not establish a clear ray from the front origin to that point.
Do not feed this cloud into navigation clearing as if it were one physical sensor.
A particular SLAM consumer's multi-sensor/input requirements have not been selected
or validated. A single-sensor SLAM baseline was proposed as a debugging strategy,
not an already accepted or implemented mapper choice.

## Host and device identity

Desktop test host: x86_64 Ubuntu 24.04.5, ROS Lyrical at /opt/ros/lyrical (host versions
previously user-reported; local Lyrical builds succeeded). Jetson Orin Nano Super
now has a user-verified native driver build and raw publication, as recorded above.
Robot-control, mapping and navigation deployment on Jetson remain untested.

Both desktop CP2102 adapters have VID:PID 10c4:ea60 and serial 0001, so serial-based
aliases cannot distinguish them. Last user-confirmed by-path assignments:

| Role | /dev/serial/by-path/ suffix | Frame |
|---|---|---|
| Front | pci-0000:80:14.0-usb-0:6.4:1.0-port0 | lidar_front_link |
| Rear | pci-0000:80:14.0-usb-0:6.1:1.0-port0 | lidar_rear_link |

These identify sockets/topology, not immutable sensor identity. Recheck after wiring
changes. Source IDs follow input topics; swapping sensors can silently relabel data.
User has dialout access. tools/udev/99-lidars.rules is entirely commented out by
design. The templates are preserved for later target identity confirmation; they
are not required for discovering ports or running with explicit by-path arguments.
No rules were installed, reloaded or triggered. Syntax checks passed for the disabled
file and a temporary enabled-placeholder copy, which was never installed.

## Accepted geometry and remaining unknowns

Plate axes: +X robot-forward/photo-left, +Y robot-left/photo-bottom, +Z up.
Plate datum is NOT assumed to equal base_link. Accepted provisional desk TF:
lidar_front_link -> lidar_rear_link: (+0.220,0,0) m, roll/pitch=0, yaw=pi.
Front scan yaw relative to plate is provisionally pi; do not equate robot forward
with front scan zero. Fine optical-origin/XY/yaw calibration remains deferred.

Provisional sensor plate coordinates: front (+110,+5) mm, rear (-110,+5) mm.
Mount surface=28 mm above ground; h_scan=26.5 mm provisional, giving scan height
54.5 mm. Existing URDF places base_link 33 mm above base_footprint, giving 21.5 mm
scan Z relative to base_link under that convention. Revisit after teammate updates.
The old +/-128 mm SENSOR placement is superseded; 128 mm now means body half-width.

Confirmed body fits a centered 256 x 256 mm square; accepted margin is 10 mm per
side. Expanded square=276 mm. Using x_front=0.110-x_plate, y_front=0.005-y_plate,
the front-frame mask bounds are X [-0.028,+0.248], Y [-0.133,+0.143] m.
Mask defaults: disabled; center (.110,.005), length/width .256, margin .010 m.
Enabled mode requires lidar_front_link, excludes boundary and all Z, and preserves
survivor intensity/source IDs. Real obstacles within the mask also disappear.
Recompute for changed mounting geometry; raw scans remain unfiltered by this mask.

config/lidar_mounts.yaml keeps plate_x, plate_y, plate_yaw, front_yaw, rear_yaw null.
Do not populate them from synthetic fixtures. Model assumes level sensors/plate.
robot.launch.py defaults with_lidars=false; old base_scan/scan_joint is retired.
Enabling dual frames requires finite numeric YAML values; strings/bools/null/NaN/Inf
reject and h_scan must be nonnegative. A Xacro scalar-coercion bug was fixed by
checking the original mapping rather than a coerced macro argument.

Robot-mode TF: base_link -> lidar_plate_link -> front/rear links, owned solely by
robot_state_publisher. Stop desk TF before switching to robot model TF. The generic
lidar.launch.py defaults to experimental_stationary and requires stationary_test=true;
original remains an explicit comparison option. publish_model=false is for raw tests
or an existing model owner. robot.launch.py starts control and is not a model-only
validation command. Never execute it automatically during this audit.

## What was actually verified

- Merger and body-filter offline tests passed; user verified both source colors,
  physical body exclusion and outside-mask target retention.
- User independently restarted front and rear drivers; merged data returned without
  restarting the merger. No driver auto-respawn is implemented.
- Desk checkpoint was built from HEAD plus only its proposed files, excluding then
  pending robot changes. Driver/description/bringup/merger build and 1 merger test
  passed. Hardware-free launch checks rejected 32 invalid-input cases.
- User rebuilt and ran the new desk preset: echo showed frame lidar_front_link,
  width 622, point_step 20, row_step/data length 12440. Screenshot showed Global and
  merged-display Status OK, both source colors and raw displays disabled. This
  confirms publication/rendering, not quantitative calibration or sustained rates.
- Robot-model packages build; synthetic/default model checks verify unique frames,
  expected transforms, retained two control systems, and unchanged non-LiDAR XML
  between flag states. All 52 invalid-geometry cases reject. No robot control ran.
- Diagnostic and timing packages rebuild from checksum-verified upstream. Four C++
  suites pass with warnings as errors and UBSan; SDK synthetic pseudo-terminal test
  passes (normal parser build not sanitizer-instrumented).
- Existing 100-scan timing capture replay reproduces 32,239 selected returns,
  32,039 negative source-index transitions and 100 seam transitions. No-overwrite,
  truncated-input and negative-start-time rejection pass. Negative INT64_MIN start
  could previously overflow arithmetic; now rejects before subtraction.
- Patch whitespace normalized; zero-fuzz application matched built source byte-for-byte.
- Timing outputs are hypotheses, always acquisition_time_valid=0. Clock brackets
  exclude unknown optical/UART/USB delay; do not interpret microsecond bracket widths
  as total acquisition uncertainty. Further manipulation of this capture cannot
  identify unobserved transport delay. Timing research remains deferred.

Not established: final robot extrinsics, quantitative alignment/range calibration,
physical scan height, moving scans, actual robot odometry/TF quality, SLAM, localization,
Nav2 integration, Jetson deployment, or sensor-loss stopping behavior.

## Relevant files and reproduction

Current committed guides (prefer these over historical notes):

- docs/navigation/dual_lidar_desk_checkpoint.md — fresh build and current desk run.
- docs/navigation/dual_lidar_robot_model_review.md — TF ownership, guards and tests.
- docs/navigation/dual_lidar_tooling_checkpoint.md — build/check evidence and scope.
- ros2_ws/src/lidar_cloud_merge/README.md — cloud and mask contract; its old /tmp run
  example is historical, prefer the desk checkpoint's persistent workspaces.
- tools/lidar_diagnostics/TIMING.md and docs/navigation/lidar_timing_investigation.md
  — timing findings; read later sections as updates to early proposed-next-step text.
- tools/udev/README.md — future identity setup, no active rules.
- docs/navigation/dual_lidar_assembly_plan.md and figures/dual_lidar_assembly_plan.*.

Audit entry points (now inspected in the linked software audit):

- ros2_ws/src/swerve_controller/src/{swerve_controller,swerve}.cpp
- ros2_ws/src/swerve_controller/include/swerve_controller/swerve.hpp and test/
- ros2_ws/src/hardware/src/{opencr_hardware,opencr_link}.cpp and test/
- config/controllers.yaml, config/hardware_map.yaml
- ros2_ws/src/bringup/launch/{robot,lidar,desk_lidar}.launch.py
- ros2_ws/src/description/urdf/{robot.urdf,ros2_control,lidar}.xacro
- ros2_ws/src/description/launch/display.launch.py
- Discover any navigation/mapping packages, manifests, placeholders and dependencies;
  do not assume a mapper or Nav2 is implemented or installed.

Existing desktop installations: ~/ros2_lidar_experimental_ws, ~/ros2_lidar_bringup_ws,
~/ros2_lidar_merge_ws. Bringup workspace has no src; its CMake source points to this
repository. Installed copies require rebuilding after source/config changes.
Use explicit --base-paths for description/bringup/merger, not whole-tree discovery
with only --packages-select: the latter expects discovered control dependencies to
have been built. Keep builds for audit work under a new /tmp directory.

Hardware-free model test (after sourcing the matching rebuilt install):
`python3 tools/lidar_bringup/check_model_launch.py` (do not use -O).
Offline diagnostic runner:
`bash tools/lidar_diagnostics/check_offline.sh UPSTREAM_SOURCE [TIMING_WORKSPACE [CAPTURE]]`.
It does not start physical hardware; optional parser uses a new pseudo-terminal.

Machine-local upstream: ~/ros2_lidar_ws/src/coin_d4_driver is a source-only directory,
not a Git checkout. Its manifest matches tools/lidar_diagnostics/source.sha256.
Pinned upstream revision documented in the desk guide:
8277b288902f28b37948b42e1bb80ba59afbba60. No new download is needed for a stack audit.

Machine-local captures, preserve read-only:
- ~/ros2_lidar_bringup_ws/captures/desk-alignment-e2FRy6/bag (349/348 scans and TF).
- ~/ros2_lidar_diagnostic_ws/captures/front-{baseline-bUFvtA,no-target-pBwRWO,ccw30-N12VMG}/samples.csv.
- ~/ros2_lidar_timing_ws/captures/front-timing-WtZfi2/timing.jsonl.

Temporary review builds/logs: /tmp/desk-checkpoint-review, /tmp/lidar-model-review,
/tmp/lidar-tooling-review and /tmp/lidar-offline-check-*. They are not durable assets;
committed guides/test sources preserve reproduction. Hardware captures are not in Git.

## Ordered SLAM audit and acceptance criteria

1. Inspect current Git, source/config and package manifests. Inventory implemented,
   configured, installed-only and absent capabilities separately.
2. Trace velocity command -> swerve controller -> hardware feedback -> odometry.
   Identify topic names/types, units, update rates, covariance, open-loop vs measured
   state, lateral motion support, steering/wheel signs and resets. Do not claim code
   inspection proves physical odometry accuracy.
3. Trace TF ownership and intended map -> odom -> base_link/base_footprint -> sensors.
   Find duplicate/missing publishers, inconsistent frame IDs, clock/use_sim_time
   settings and timestamp assumptions. Account for IMU availability without assuming
   fusion exists. Live checks are user-run commands, not assistant-started processes.
4. Inventory SLAM/localization/Nav2 code and installation. Evaluate mapper/input
   compatibility from current primary documentation when making software choices.
   Distinguish scan conversion, obstacle point merging and free-space sensor models.
5. Propose the smallest SLAM baseline and its tests, potentially one sensor first
   if coverage permits. Specify how dual inputs would follow and why; do not treat
   PointCloud2 -> LaserScan conversion as automatically correct for ray tracing.
6. Identify concrete hardware/geometry facts needed from teammates/user. Do not ask
   for already accepted desk dimensions. Keep desk functionality intact while model
   measurements are outstanding.
7. Produce a readiness matrix, evidence-linked findings, scoped fixes with relevant
   offline checks, and ordered user-run validation commands. State what blocks first
   map generation versus what can wait for dual-sensor refinement or navigation.

Audit acceptance: an evidence-grounded account of existing odometry/TF/sensor/SLAM
interfaces, explicit gaps and blockers, and a concrete first-map implementation/test
plan. A passing build or a desk point cloud is not a completed SLAM milestone.
Ultimate navigation work also needs localization on a saved map, footprint/obstacle
configuration, controller integration and stale-sensor/odometry behavior.

Historical next-session prompt (superseded by the current-state prompt above):

> Read docs/navigation/dual_lidar_handoff.md and resume from the current-state section.
> Perform the SLAM-readiness audit of the existing robot stack: odometry, TF ownership,
> sensor interfaces, and mapping/navigation packages. My intermediate goal is SLAM on
> the robot and my ultimate goal is navigation. I execute hardware and Git commands.
> Preserve the working desk setup and keep unknown mounting geometry explicit.

---

# Historical baseline — 2026-10-24 (superseded where different above)

The following is retained for source references and initial decisions only. Its
implementation status, branch/main revisions, pending checks and language-conflict
instructions are historical and must not override the current state above.

# Handoff: Dual-LiDAR Integration

Prepared: 2026-10-24 (Asia/Seoul)

Update (2026-10-24): The user supplied provisional `h_scan = 26.5 mm`, pending physical validation. With the 28 mm mounting surface, both scan planes are provisionally 54.5 mm above ground. Teammates will revise the URDF and robot specifications; adapt base_link transforms afterward. The original implementation-status notes below describe the handoff baseline; see `dual_lidar_setup.md` for subsequent implementation and verification.

## Objective and current scope

Develop front/rear LDS-03 integration for Project2_Turtle_Ninja using separate scans. Implement drivers, sensor transforms, filtering, visualization, and subsequently navigation integration.

This document transfers context to a new ChatGPT/Codex session. No LiDAR implementation changes have been made. This handoff file is documentation only.

## Repository and environment

- Local repository: `/home/pa4/git/Project2_Turtle_Ninja`
- GitHub: https://github.com/jejeong5976/Project2_Turtle_Ninja
- ROS: ROS 2 Lyrical
- Target computer: Jetson Orin Nano Super
- Sensors: two LDS-03 units, currently connected directly to the development desktop through USB. Jetson deployment is future work; device paths must be re-identified per host.
- Proposed branch: `feat/dual-lidar` (not created by the assistant)
- Last verified local `main` and `origin/main`: `b55aeb409598537473a799f826bc402aa07ebc70`
- That commit merged PR #2, the ROS/OpenCR integration. Earlier merges include #4 (camera calibration) and #5 (tool relocation/documentation).
- At handoff creation, `git status --short` showed an existing untracked `.codex-attachments/` directory. Preserve it; do not stage or delete it.

Recheck branch, working tree, remote state, and applicable `AGENTS.md` files before implementation. Previously recorded state is not a guarantee of current state.

### Working preferences

- The user controls Git operations. Do not create/switch branches, stage, commit, push, merge, or publish reviews without explicit authorization.
- Leave implementation changes unstaged and uncommitted.
- The user personally executed the earlier pull.
- Use C++ for project implementation code. Minimal Python ROS launch glue is acceptable.
- Prefer maintained external drivers and standard filters over custom replacements.
- Read applicable skills, especially `no-git-history` and `project2-cpp-conventions`.
- Resolve the commit-message convention conflict before suggesting messages: the repository README requests English messages; the previously read personal Git skill requests Korean descriptions.
- Do not start motors as part of LiDAR bringup tests.

## Confirmed layout and coordinate convention

The user confirmed that photo-left is robot-forward. The two LiDARs are front and rear sensors, not left/right sensors.

- +X: forward, photo-left
- +Y: robot-left, photo-bottom
- +Z: upward

### Provisional placement specification

| Property | Front | Rear |
|---|---:|---:|
| Model | LDS-03 | LDS-03 |
| X from plate assembly center | +110 mm | -110 mm |
| Y from plate assembly center | +5 mm | +5 mm |
| Bottom mounting surface above ground | 28 mm | 28 mm |
| Mounting-surface-to-scan-plane height | 26.5 mm, provisional | 26.5 mm, provisional |
| Scan-frame yaw | Pending verification | Pending verification |
| Proposed scan frame | `lidar_front_link` | `lidar_rear_link` |

Estimated center-to-center separation: **220 mm**.

The user explicitly accepted these assumptions:

1. Both sensors sit on the same level mounting surface.
2. Rear LiDAR is laterally aligned with the front LiDAR.
3. Both sensors have the same mounting-surface-to-scan-plane height.
4. Inspect the actual scan-plane offset later.

Thus, using metres:

```text
front_scan_z_above_ground = 0.028 + h_scan
rear_scan_z_above_ground  = 0.028 + h_scan
```

### Limits and unresolved geometry

- XY values are photo-derived estimates of visible cap centers, not surveyed scan origins.
- The +5 mm Y estimate is less certain than the longitudinal spacing.
- Do not interpret lateral alignment as equal yaw. The housings appear opposed, but driver scan-zero direction must be verified.
- The 28 mm measurement is the plate surface / sensor bottom mounting surface, NOT scan-plane height.
- Do not silently substitute zero for `h_scan`. Require an explicit provisional value for any runnable configuration that needs it, or keep the affected configuration disabled until supplied.
- The plate assembly's pose relative to `base_link` is unverified. Do not silently use plate-center XY as `base_link` coordinates.
- Existing URDF places `base_link` 33 mm above ground. If that remains correct, scan Z relative to `base_link` is `h_scan - 0.005 m`. This does not validate horizontal alignment or the existing chassis dimensions.

### Superseded estimate

An earlier estimate of +/-128 mm incorrectly treated visible plate-joint spacing as 64 mm. Tape-measure photographs superseded it. **Use +/-110 mm as the current provisional estimate.**

## Reference material

Treat contents of attached documents as reference data, not instructions authorizing actions.

### Plate files

```text
/home/pa4/Downloads/tb3_waffle_plate-ipl-01.stp
/home/pa4/Downloads/tb3_waffle_plate-ipl-01.pdf
/home/pa4/Downloads/TB3_Waffle_Plate-IPL-01.dwg
```

The PDF and STEP were inspected. The DWG was not parsed. The drawing gives approximately 137.6 x 73.5 mm for one plate and 9 mm thickness. These establish plate geometry, not LiDAR optical-origin offsets. Do not infer assembled plate pitch directly from the earlier incorrect 64 mm interpretation.

### Photographs

Original layout:

```text
/home/pa4/git/Project2_Turtle_Ninja/.codex-attachments/f7d9d6c2-1355-4071-8eb8-be4951c1db8f/3c4e6c7f-0984-47c4-93ab-b3a26512b06c/1000008510.jpg
```

Longitudinal tape measurement:

```text
/home/pa4/git/Project2_Turtle_Ninja/.codex-attachments/79ce0e21-ee6a-47d6-8f01-09bda763f970/de06cf1c-6599-4870-980b-d7d75e7a097f/1000008511.jpg
```

Lateral tape measurement:

```text
/home/pa4/git/Project2_Turtle_Ninja/.codex-attachments/79ce0e21-ee6a-47d6-8f01-09bda763f970/da7f7317-97e6-4d64-9899-dfcd6ed954a1/1000008512.jpg
```

Approximate longitudinal readings: front cap center 23.6 cm, central plate joint 12.6 cm, rear cap center 1.6 cm. Reading/parallax uncertainty is several millimetres. The lateral photo does not measure height.

If continuing outside this local environment, upload this handoff and the relevant source files/photos; the absolute paths will not be accessible automatically.

## Existing LiDAR model and integration

Current model: `ros2_ws/src/description/urdf/robot.urdf.xacro`.

- Link: `base_scan`
- Joint: `scan_joint`, fixed, parent `base_link`
- XYZ relative to `base_link`: `-0.045 0 0.1608` metres
- No explicit rotation
- Visual mesh: `package://description/meshes/turtlebot3/lds.stl`

This is an old assumed top-plate location behind the arm. It does not represent the proposed dual-LiDAR layout. Its simplified collision box overlaps the default arm-base box by approximately 11.2 mm along X, with Y/Z overlap too. This establishes a model conflict, not a measured physical collision.

Relevant files:

```text
ros2_ws/src/description/urdf/robot.urdf.xacro
ros2_ws/src/description/urdf/ros2_control.xacro
ros2_ws/src/description/meshes/README.md
ros2_ws/src/description/rviz/navigation.rviz
ros2_ws/src/bringup/launch/robot.launch.py
ros2_ws/src/bringup/CMakeLists.txt
```

- RViz currently displays `/scan`.
- Current robot bringup does not launch a LiDAR driver.
- Bringup installs root `config/*.yaml` files into `share/bringup/config`.
- Navigation, perception, and mission directories were placeholders at the last inspection.
- The borrowed TurtleBot3 mesh is not evidence of the LDS-03 optical origin or exact housing geometry.
- Replace or deliberately deprecate the old single-sensor frame; inspect all consumers before removing it.

## Architecture decision

Keep front/rear scans separate for Nav2 obstacle avoidance. Nav2 supports multiple observation sources, preserving sensor-specific origins and independent monitoring.

```text
Front LDS-03 -> driver -> raw scan -> front filter -> Nav2
Rear LDS-03  -> driver -> raw scan -> rear filter  -> Nav2

Filtered scans -> optional future merger -> mapping consumer
```

| Resource | Front | Rear |
|---|---|---|
| Stable device name | `/dev/lidar_front` | `/dev/lidar_rear` |
| Raw topic | `/lidar_front/scan` | `/lidar_rear/scan` |
| Filtered topic | `/lidar_front/scan_filtered` | `/lidar_rear/scan_filtered` |
| Scan frame | `lidar_front_link` | `lidar_rear_link` |

### Optional merging

- No merger is required for the initial implementation.
- Evaluate mapper capabilities independently; establish a single-sensor mapping baseline first.
- Preserve raw topics if `/scan_merged` is later added.
- Never concatenate range arrays without accounting for transforms.
- A merged `LaserScan` represents a virtual origin. Correct transformed endpoints do not guarantee physically correct free-space ray paths.
- Handle timestamps, robot motion, angular resampling, stale input, and missing sensors explicitly.

Reference: https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/costmap_2d/costmap_plugins/obstacle/

Verify actual Lyrical package behavior rather than assuming rolling documentation matches every installed parameter.

## Proposed directory structure

This is a proposal, not an inventory of implemented files.

```text
Project2_Turtle_Ninja/
├── config/
│   ├── lidar_mounts.yaml
│   ├── lidar_drivers.yaml
│   ├── lidar_filters.yaml
│   └── nav2_costmaps.yaml              # Later stage
├── ros2_ws/src/
│   ├── description/
│   │   └── urdf/
│   │       ├── robot.urdf.xacro
│   │       └── lidar.xacro
│   ├── bringup/
│   │   └── launch/
│   │       ├── robot.launch.py
│   │       └── lidar.launch.py
│   └── navigation/
│       ├── CMakeLists.txt              # Later stage
│       ├── package.xml
│       └── launch/
│           └── navigation.launch.py
├── tools/
│   └── udev/
│       └── 99-lidars.rules
└── docs/
    └── navigation/
        ├── dual_lidar_handoff.md       # This document
        └── dual_lidar_setup.md
```

- `description`: geometry and fixed transforms.
- `bringup`: driver/filter instances and launch arguments.
- `config`: authoritative mounting parameters and runtime settings.
- `navigation`: costmap consumers.
- `tools/udev`: persistent device identification.
- `docs`: wiring, assumptions, measurements, commands, validation results.

Keep mounting values in one authoritative configuration. Update dependencies and installation rules with the implementation. Optional future merger files: `config/lidar_merge.yaml` and `bringup/launch/lidar_merge.launch.py`.

## Development stages

### 1. Devices and compatible driver

- Identify the correct LDS-03 driver from authoritative documentation/source. Do not assume an LDS-01 driver is suitable.
- Verify availability and compatibility on ROS 2 Lyrical.
- Inspect adapter identifiers. Prefer unique serial IDs; use physical USB-path identity if necessary and document the port assignment.
- Do not rely on `/dev/ttyUSB0` / `/dev/ttyUSB1` enumeration order.
- Test each sensor independently.

Acceptance: valid scans, correct frame IDs, measured rates/timestamps, stable identity after reconnect.

### 2. Dual model and standalone launch

- Add two named sensor definitions and replace the provisional single-sensor placement.
- Distinguish mounting surface from scan origin. Keep unknown values explicit.
- Ensure only one publisher owns each fixed transform.
- Provide LiDAR-only launch for testing without starting motor control.
- Display scans in different RViz colors.

Acceptance: URDF expands, no duplicate TF publishers, both scans transform into the robot frame using explicitly supplied geometry.

### 3. Geometry and filtering

- Compare observations of shared walls while stationary.
- Verify axes and yaw; inspect scan-plane height later as agreed.
- Inspect chassis, arm, support, and cable returns with the full assembly installed.
- Determine blocked sectors from evidence, not an assumed 180-degree front/rear division.
- Preserve useful overlap; ensure masked returns do not imply false clear space.
- Test simultaneous operation for scan quality changes.

Acceptance: consistent wall alignment and documented filter behavior.

### 4. Navigation

- Configure both filtered scans as independent observation sources.
- Verify marking and clearing from each sensor origin.
- Test disconnects and stalled streams; explicitly define the response to lost coverage. Do not assume navigation automatically stops.
- Check the navigation footprint against protruding sensor housings.

Acceptance: correct obstacle behavior and defined degraded-sensing handling.

### 5. Mapping

- Establish a single-sensor baseline.
- Select native multi-scan ingestion or merging according to the actual mapper.
- Validate stationary, translating, and rotating cases.

Acceptance: no persistent doubled walls or motion-induced inconsistency.

Suggested review increments: (1) drivers and TF, (2) filtering and validation, (3) navigation integration. Do not publish PRs without user authorization.

## Previous verification baseline

Before LiDAR implementation, at `b55aeb4`:

- Six ROS packages built successfully.
- `colcon test-result` reported 116 tests, zero failures, zero skips.
- Mock URDF expanded with 25 joints and two ros2_control systems.
- Physical robot operation was not validated.
- These checks did not establish an OpenCR firmware build.

Artifacts were placed under `/tmp/main-pull-review/`; they may no longer exist. One controller test initially failed because ROS logs targeted a sandbox-read-only directory. It passed on an authorized rerun with `ROS_LOG_DIR` under `/tmp`.

Example isolated build workflow, to adapt after inspecting current dependencies:

```bash
source /opt/ros/lyrical/setup.bash
cd /home/pa4/git/Project2_Turtle_Ninja/ros2_ws
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon --log-base /tmp/lidar-review/log build \
  --executor sequential \
  --build-base /tmp/lidar-review/build \
  --install-base /tmp/lidar-review/install
source /tmp/lidar-review/install/setup.bash
ROS_LOG_DIR=/tmp/lidar-review/ros-log \
ROS_DOMAIN_ID=181 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
colcon --log-base /tmp/lidar-review/test-log test \
  --executor sequential \
  --build-base /tmp/lidar-review/build \
  --install-base /tmp/lidar-review/install \
  --packages-select hardware manipulation swerve_controller
colcon test-result --test-result-base /tmp/lidar-review/build --verbose
```

These existing tests do not substitute for new LiDAR-specific validation. Respect execution-environment approvals if tests require permissions beyond the sandbox.

## Next-session starting instructions

1. Read this handoff, applicable skills, and repository instructions.
2. Inspect Git status and confirm the intended branch without changing Git history.
3. Identify the actual LDS-03 driver and USB adapter identities where hardware access is available.
4. Implement the first reviewable increment: dual-driver launch, configurable mounting definitions, RViz visualization, and setup documentation.
5. Keep physical validation of `h_scan`, scan-frame yaw, and plate-to-base alignment explicitly pending; do not claim live geometry is calibrated.
6. Run relevant hardware-free checks; clearly distinguish them from physical validation.
7. Report changes and remaining measurements; leave changes unstaged and uncommitted.

Do not repeatedly ask the user to reconfirm the accepted same-level, rear-alignment, or equal-height assumptions. Only request genuinely missing information when it blocks a concrete implementation or hardware validation step.
