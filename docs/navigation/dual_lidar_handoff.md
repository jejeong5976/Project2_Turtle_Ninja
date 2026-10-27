# Handoff: Dual LiDAR integration -> SLAM-readiness audit

Updated: 2026-10-27 (Asia/Seoul). This current-state section supersedes all earlier
resume instructions and the historical baseline retained below.

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
wheel runtime validation or Git actions have been performed for this profile.

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

## Start here — audit completed and next implementation boundary

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

## Verified repository state

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

Suggested next-session prompt:

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
