# Wheel-base teammate delivery and console v2

Prepared 2026-10-27, locally only. The user operates all hardware and Git commands.

## Final update — vCrfzl in the revised layout

The user built v2 on Jetson and completed the final 29-segment round trip. Two
ordinary-input batches were discarded without halting the run; all stops were
confirmed automatic. The independent recorder closed after the final map/graph
save. No further physical experiment is requested.

New separate archive: `wheelbase-final-roundtrip-20261027-v1.tar.gz`, in the same
durable directory below. It contains the 8,002-scan independent bag (including
robot description), console bag/logs, final 335-scan / 414-edge graph, pixel-verified
5 cm map, offline 2.5 cm map, analysis and operator observations. SQLite integrity,
metadata counts, command-field equivalence and file hashes pass. Earlier archives
remain unchanged. The following G0I8Np results describe the previous layout.

The user reports approximate passage width 800–810 mm but no exact cross-section
or long-wall length. A loose object moved during return. An illustrative fine-grid
opening is 775–800 mm; absolute error cannot be assigned without matching the
physical section. Orientation remains physically unverified, with configured yaw
pi preserved. See [final review](jetson_mapping_final_roundtrip_review.md).

The final dataset is prepared for the teammate; no message/upload was sent.
Console and recorder have exited. User shutdown instructions were supplied for
mapping launch, wheel launch and then motor power. Further shutdown is unconfirmed.

## What the teammate is asking for

The supplied Korean instructions describe an independently recorded mapping run:
keep scan/odometry/TF/feedback together, save the occupancy map and serialized
graph before stopping the recorder, and report a physical scan-direction check
so the teammate can set the URDF default. This does not require replacing the
current stack with their separate bringup package.

Their original `lidar_scan_jetson_20261027.tar.gz` was inspected read-only from
`/home/pa4/Downloads`. It expects `/scan`, `base_scan` and an absolute
`NT_SCAN_YAW` (default pi/2) from `base_link`, with different mounting xyz.
Our current corrected topic/frame/mount and lower motion limits are preserved.
The new helper recorder uses the actual current topics and disables rosbag's
keyboard pause controls. It records independently of console lifetime.

## Console change

`run_mapping_console.cpp` v2 discards ordinary input while not READY, including
input collected immediately before the transition to READY. Invalid/overlong
READY commands remain recoverable. Arrow/function keys and accidental terminal
flow-control/suspend keys do not terminate or freeze the session. Commands are
never queued as a route. `q` at READY remains a deliberate finish; Ctrl+C remains
an explicit abort. Actual hangup/EOF, signals and health/bounds failures still stop.
Motion speeds, bounds, five-zero stops, settling checks and sensor guards are
unchanged. The console now allows one passive `/team_mapping_recorder` command
subscriber alongside the one mandatory wheel controller. Missing wheels, unexpected
consumers and any additional command publisher still prevent motion. Discarded
busy-input batches are logged in events/result.

C++17 warnings-as-errors build, 18 mock ROS motion/fault scenarios, input-editor
checks and terminal input/hangup checks passed. Tests use domain 193, localhost
discovery and empty static peers; no hardware. The restricted-sandbox attempt
had DDS interface errors and failed; the complete outside-sandbox run passed.
The final terminal queue-flush addition was separately rechecked with long-paste
PTY coverage and the accidental-input, recorder and operator-stop scenarios.
The user subsequently built/selected v2 on Jetson; the installed v1 was not changed. The
helper scripts passed syntax and mock CLI tests for normal saving, serializer
failure, missing graph files, map saver failure, recorder arguments and explicit
teleop selection. These are not physical hardware tests.

## Deliverables

Durable local output directory:
`/home/pa4/.codex/visualizations/2026/10/27/01a17c84-05ab-71a3-b568-0894083ce378/wheelbase-handoff/`.

- `jetson-mapping-console-20261027-v2.tar.gz`: source, build definition, header,
  tests, helper scripts, Korean instructions, frame/orientation report and source
  checksums. A convenience copy is also at `/tmp/jetson-mapping-console-20261027-v2.tar.gz`.
- `wheelbase-mapping-data-20261027-v1.tar.gz`: G0I8Np bag and original CSV/results,
  seven explicitly incomplete earlier attempts, before/after saved maps/graphs,
  exact-graph 5 cm / 2.5 cm rasters, analysis tables/reports, runtime launch
  parameters/URDF, configuration references and checksums. Original data is unchanged.
- `wheelbase-final-roundtrip-20261027-v1.tar.gz`: separate final revised-layout dataset described above.
- `SHA256SUMS`: checksums for the delivery archives. Each archive contains its
  own file manifest too. Extract to a new directory and check before using.

The data archive includes `README_KO.md`, `ORIENTATION.md`, `frame_contract.yaml`
and a machine-readable `delivery.json` with provenance and topic counts. This is
a prepared local handoff, not a message/upload to the teammate.

## Results and limits to communicate

G0I8Np completed 48 segments with user-confirmed automatic stops and an earlier
passage revisit: 7,610 corrected scans, 37,967 odometry messages, 759.32 seconds,
5.757 m summed straight travel. All scans are bracketed by odometry. Its starting
map already includes earlier commanded movement. Those seven earlier bags are
included as incomplete attempts, not relabeled completed runs. The full mapper
session also includes older motion; the 18:29 before graph preserves its prior
state. These bags alone are not a from-empty replay of the entire NJWQJ7 session.

Recorded scan is corrected front input; raw packets, moving rear scans and IMU
are not present. Existing bag lacks robot_description; the running launch's
URDF/parameters were copied separately through read-only SSH. `/map` includes
metadata in its `info` member. Nominal timing with zero assumed delay remains
provisional (`acquisition_time_valid=0`). No new wheel-calibration claim is made.

The saved final graph loads offline; its 5 cm raster matches the saved map and
last bag map. The 2.5 cm raster uses the same graph, without new optimization.
Live resolution is unchanged. Full arena coverage, loop-closure success, live
restore/relocalization and absolute accuracy are not established. Physical wall/
passage measurements are still needed for absolute error; map->odom changes are
not ground-truth error.

**Physical orientation check remains pending:** the user says the front box
was not checked in RViz with the base fixed frame. Active yaw is pi, confirmed
from the running process/configuration. Under the same axis convention, the
teammate's equivalent absolute `NT_SCAN_YAW` would be `3.141592653589793`.
Do not label this physically verified or integrate a duplicate live TF. The
front-object +X and left-object +Y checks are detailed in
[ORIENTATION.md](../../tools/lidar_diagnostics/team_capture/ORIENTATION.md).

At 19:22 KST, read-only SSH still found the same wheel and NJWQJ7 processes and
190 GB available. This was configuration/provenance inspection, not a new motion
readiness certification. Continuous mapping, LiDAR setup, assumed wheel calibration
and zero plate offsets were preserved. No assistant hardware or Git command ran.

## Original user-run workflow (completed; retained for reference)

The earlier procedure was to transfer/build v2 using [MAPPING_AREA.md](../../tools/lidar_diagnostics/MAPPING_AREA.md#user-run-transfer-and-build).
For another recording, leave existing wheel/input/SLAM processes running; in the
v2 directory run `./scripts/record_bag.sh` in C and then
`./scripts/teleop.sh --supervised` in D. At a full stop save with
`./scripts/save_map.sh` from another terminal, finish the console with `q`, then
Ctrl+C the recorder. Keep NJWQJ7 running. Check current physical clearances before
every new movement. The teammate's full `run_scan.sh` and second driver are not
needed in this already-running session.

The prepared data packages can be shared with the explicit orientation-pending
label. The physical direction result remains unresolved; do not infer it or request
another run after the user ended experimentation.
