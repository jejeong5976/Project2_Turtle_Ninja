# Dual LDS-03 bringup

## Scope and driver

The first increment provides two independent raw scans, configurable fixed frames, and RViz displays. Filtering, Nav2, and mapping remain later stages. No filtered topics are published yet. The standalone launch does not start OpenCR, controller_manager, manipulation, or wheel/arm controllers. LDS units themselves rotate during scanning.

ROBOTIS identifies LDS-03 as COIN-D4. Use `coin_d4_driver` and one `single_coin_d4_node` process per sensor. Do not substitute LDS-01/02 drivers.

Sources inspected on 2026-10-24:

- [ROBOTIS LDS-03 manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/appendix_lds_03/)
- [Pinned official driver source](https://github.com/ROBOTIS-GIT/coin_d4_driver/tree/8277b288902f28b37948b42e1bb80ba59afbba60)
- [Upstream parameters](https://github.com/ROBOTIS-GIT/coin_d4_driver/blob/8277b288902f28b37948b42e1bb80ba59afbba60/params/single_lidar_node.yaml)
- [Lyrical distribution index](https://github.com/ros/rosdistro/blob/master/lyrical/distribution.yaml): no coin_d4_driver entry at inspection; do not assume an apt package exists.

Install/build the pinned upstream source in an external workspace and source that overlay before building this project. Do not vendor the driver into this repository. Its C++ source was selected for a local Lyrical build check; this is not Jetson hardware certification. Use `colcon build --executor sequential` in each workspace after sourcing `/opt/ros/lyrical/setup.bash` and prerequisite overlays.

## Hosts and configurable device paths

The two sensors are currently connected directly to the development desktop. The deployment target is Jetson Orin Nano Super. Desktop build results do not establish compatibility with the eventual Jetson OS/JetPack/ROS installation; verify those versions and build the driver on the target before deployment.

USB device paths are host-specific runtime settings, not robot geometry. `config/lidar_drivers.yaml` contains separate `port` parameters. `/dev/lidar_front` and `/dev/lidar_rear` are proposed udev aliases, not discovered device names. For desktop tests, copy that YAML to a local file, set each `port` to its verified current device path (prefer a unique `/dev/serial/by-id/...` path when available), and pass:

```bash
ros2 launch bringup lidar.launch.py publish_model:=false driver_config:=/absolute/path/desktop_lidar_drivers.yaml
```

Keep the node keys, topic names, and frame IDs unchanged when changing device paths. Temporary `/dev/ttyUSB*` paths may be used after identifying each sensor for the current test, but must be rechecked after reconnecting. Moving to Jetson requires a fresh device-identity check and a target-specific config or udev rules; desktop USB topology paths must not be assumed valid there. No code or geometry changes are required solely because device paths change.

## Device identity

On the current host (desktop now, Jetson later), connect and label one sensor at a time. Record `udevadm info --query=property --name=/dev/ttyUSB0` (the temporary enumerated name is for discovery only), especially ID_SERIAL_SHORT and ID_PATH. Use the commented template in `tools/udev/99-lidars.rules`. Never install unchanged placeholders or match both adapters solely by common vendor/product IDs.

After filling and reviewing the rules, install them under `/etc/udev/rules.d/`, reload udev rules, reconnect the sensors, and verify:

```bash
readlink -f /dev/lidar_front
readlink -f /dev/lidar_rear
```

The two links must resolve to different intended devices, including after disconnect/reconnect and reboot. With path-based identity, label the physical USB sockets and keep assignments fixed. Ensure the operator has serial access through the device group. Desktop adapter identities and confirmed socket assignments are recorded in the 2026-10-25 results below; Jetson identities remain pending.

## Geometry and TF ownership

`config/lidar_mounts.yaml` is the authoritative mount configuration. Null values intentionally reject model expansion. Supply explicitly accepted provisional values or measurements for `front_yaw`, `rear_yaw`, `plate_x`, `plate_y`, and `plate_yaw`. Units are metres/radians. Yaws are rotations about +Z; sensor yaw is relative to the plate, plate yaw relative to base_link. Verify scan zero direction with a stationary target; opposed housings alone do not establish yaw.

The plate frame is its assembly center at the sensor bottom mounting surface. Level mounting is the accepted assumption. Front/rear scan origins are provisionally (+0.110, +0.005, h_scan) and (-0.110, +0.005, h_scan) in that frame. The surface is 0.028 m above ground. The user supplied provisional `h_scan = 0.0265 m` on 2026-10-24; physical validation is pending. Both scan planes are therefore provisionally 0.0545 m (54.5 mm) above ground. Teammates will revise the robot specifications; derive base_link transforms from those updated specifications rather than treating the current base height as confirmed. Plate XY/yaw are deliberately unknown. The cap-center XY estimates still need optical-origin validation.

No housing visual/collision mesh is asserted. The old `base_scan`/`scan_joint` and `/scan` RViz display are retired. Default robot URDF has no LiDAR frames; enable `with_lidars:=true` only with populated geometry. The same macro is used by robot and standalone launch. Only robot_state_publisher owns these fixed transforms; no static-transform nodes are added.

When adjusting a copy of the mount YAML, pass its absolute path. Rebuild after changing repository config to update installed defaults. Do not insert made-up values simply to bypass the guard.

## Launch and inspect

First test each sensor in its own frame without geometry or robot control:

```bash
ros2 launch bringup lidar.launch.py publish_model:=false enable_rear:=false
# In a second terminal with the same overlays:
ros2 topic echo /lidar_front/scan --once
ros2 topic hz /lidar_front/scan
```

Repeat with `enable_front:=false enable_rear:=true` and `/lidar_rear/scan`. In raw-only mode there is no robot TF: set RViz Fixed Frame to the selected sensor frame if viewing it. Check frame IDs, timestamp progression/age, scan_time, angle coverage, range limits, and stationary wall quality. Record observations instead of inferring rates from nominal specs.

After supplying geometry, stationary dual-sensor bringup is:

```bash
ros2 launch bringup lidar.launch.py lidar_mounts:=/absolute/path/lidar_mounts.yaml use_rviz:=true
```

Front points are red, rear points blue. RViz uses base_link for this launch. Only fixed model transforms are available: movable arm/wheel links have no joint-state source in standalone mode, and their RViz warnings are expected. Map/Nav2 displays also remain empty.

For an existing robot bringup, enable its `with_lidars:=true` with the same `lidar_mounts` file and run the sensor launch with `publish_model:=false`. Do not run two robot_state_publishers. Starting `robot.launch.py` is a separate full-control action and is not part of the LiDAR-only test procedure.

The driver config is scoped to `/lidar_front/driver` and `/lidar_rear/driver`, with relative `scan` topics and distinct frame IDs. Override it via `driver_config:=/absolute/path/lidar_drivers.yaml` if needed; preserve these node keys. Raw topics are `/lidar_front/scan` and `/lidar_rear/scan`.

## Remaining acceptance work

- Measure actual scan-plane offset and plate/base alignment; verify yaw and shared-wall agreement.
- Test simultaneous scanning for interference or quality changes, reconnect identity, and missing/stalled input behavior on Jetson.
- Determine self-occlusion sectors from evidence with the full assembly. Add standard laser_filters only after those observations; do not assume 180-degree masks or publish masked returns as artificial free space.
- Configure separate Nav2 marking/clearing sources and define loss-of-coverage behavior explicitly before autonomous motion.
- Verify physical footprint, including housings, and establish single-sensor mapping baseline before any merger.

See `dual_lidar_handoff.md` for measurements and the remaining stage acceptance criteria.

## Hardware-free verification (2026-10-24)

- Read-only remote check: local HEAD, origin/main, and remote main were all `b55aeb409598537473a799f826bc402aa07ebc70` before edits. At that time work was on main, unstaged/uncommitted; see the later desktop results for the updated branch.
- Built upstream coin_d4_driver `8277b288902f28b37948b42e1bb80ba59afbba60` (package 1.0.1), description, and bringup successfully against local ROS 2 Lyrical. No upstream source modifications were needed. Build outputs: `/tmp/dual-lidar-check/`.
- Expanded default URDF and a dual URDF using a synthetic fixture under `/tmp`; verified unique joints, expected plate/front/rear translations, and preserved two ros2_control definitions. Synthetic values are test inputs only.
- All 50 null/non-finite/string/boolean mount-value cases and a negative h_scan were rejected. Installed default launch failed on missing plate_x before starting a node.
- Launch argument discovery and both-drivers-disabled raw mode passed. The valid synthetic model initialized robot_state_publisher and shut down cleanly after five seconds with both drivers disabled. ROS socket restrictions required running that smoke test outside the sandbox, restricted to localhost/domain 181.
- Python launch syntax and `git diff --check` passed. No serial ports, live scans, real TF calibration, or motors were exercised. RViz configuration was inspected structurally; rendered/live visualization is pending.

Upstream stamps scans with node time at publication (`get_node_time()`), rather than demonstrating hardware acquisition-time synchronization. Measure latency and assess motion distortion before mapping/merging; a successful build does not establish timestamp accuracy.


## Desktop hardware results (updated 2026-10-25, Asia/Seoul)

Evidence: user-executed commands, pasted terminal results, an RViz screenshot, and user-reported visual checks. These results do not establish Jetson compatibility or final robot calibration.

- Host: x86_64 desktop, Ubuntu 24.04.5 LTS, ROS 2 Lyrical as reported by the user. Sensors sit on a desk, not on the robot. The provisional 54.5 mm robot ground-relative height does not describe this test placement.
- Both adapters identify as Silicon Labs CP2102, VID:PID 10c4:ea60, serial 0001. Their by-id name collides; use the confirmed distinct by-path names below. The operator has dialout access.
- Front: `/dev/serial/by-path/pci-0000:80:14.0-usb-0:6.4:1.0-port0` (observed ttyUSB1).
- Rear: `/dev/serial/by-path/pci-0000:80:14.0-usb-0:6.1:1.0-port0` (observed ttyUSB0).
- Physical assignment was explicitly confirmed by the user. Paths were observed again on October 25. Recheck if hub topology, sockets, or host changes; enumeration names are not identity.
- The user created `feat/dual-lidar` and fast-forwarded it to `f872159` after fetching upstream. LiDAR edits remain unstaged/uncommitted. No upstream robot-model changes were present in that update.
- Previous `/tmp/dual-lidar-check` build artifacts disappeared. Persistent driver installation is now `/home/pa4/ros2_lidar_ws/install/coin_d4_driver`; the user confirmed both driver executables are discoverable after following the pinned-source build instructions.
- Project bringup/description installation: `/home/pa4/ros2_lidar_bringup_ws/install`. Desktop parameters: `/home/pa4/ros2_lidar_bringup_ws/config/desktop_lidars.yaml`. These are machine-local paths outside the repository.

| Test | Observed result |
|---|---|
| Front alone | Approximately 10.05 Hz, 99–100 ms intervals; frame lidar_front_link |
| Rear alone | Approximately 10.02 Hz, 98–101 ms intervals; frame lidar_rear_link |
| Both operating, front | Latest supplied average 10.011 Hz; 99–101 ms intervals |
| Both operating, rear | Latest supplied average 10.028 Hz; 98–101 ms intervals |
| Individual RViz target response | User reported both checks completed successfully |
| Alone versus simultaneous visual comparison | User reported no obvious difference; no visible degradation in this desk arrangement |
| One launch for both drivers | Both intended USB paths opened; node list contained only the two driver nodes |
| Topic identity | Each scan topic had one publisher, type sensor_msgs/msg/LaserScan, and the intended frame ID |

The publication rates are subscriber-observed arrival statistics, not precision timing or range calibration. Separate one-shot header captures do not measure inter-sensor synchronization. Absence of visible degradation is a qualitative observation, not proof that interference is impossible.

Verified desktop launch (stop any individually launched drivers first):

```bash
source /opt/ros/lyrical/setup.bash
source ~/ros2_lidar_ws/install/setup.bash
source ~/ros2_lidar_bringup_ws/install/setup.bash
ros2 launch bringup lidar.launch.py \
  publish_model:=false \
  driver_config:="$HOME/ros2_lidar_bringup_ws/config/desktop_lidars.yaml"
```

Keep model publishing disabled until robot geometry is ready. For RViz, use each sensor's own fixed frame and topic separately; no common desk transform has been established.

Still pending: scan metadata consistency; quantitative range checks and physical scan-zero identification; optical-origin and h_scan validation; installed robot alignment, occlusions, Jetson validation, disconnect/stall behavior, filtering, navigation, and mapping.


### Restart and metadata follow-up (2026-10-25)

The user confirmed the dual-driver shutdown/restart check passed. One-shot metadata:

| Field | Front | Rear |
|---|---|---|
| range/intensity array lengths | 399 / 399 | 401 / 401 |
| angle_min | 0 | 0 |
| angle_max | 6.283185482 rad | 6.283185482 rad |
| angle_increment | 0.01570796408 rad | 0.01562981494 rad |
| scan_time | 0.09964899719 s | 0.10027399659 s |
| time_increment | 0.00024974687 s | 0.00025005985 s |
| declared range limits | 0.10–100 m | 0.10–100 m |

Inspection of the installed workspace source at `/home/pa4/ros2_lidar_ws/src/coin_d4_driver/src/coin_d4_base_handler.cpp` found a driver-side inconsistency: `parse_lidar_serial_data` sets angle_increment to 2*pi/count and angle_max to 2*pi, but its loop emits count-1 points. The publisher copies those points into ranges without recomputing angular metadata. The inferred counts are 400 and 402; the last array sample represents about 358.20 and 358.21 degrees by the published index formula, not angle_max. This is larger than floating-point roundoff and is explained by the source, not evidence of USB packet loss.

The driver also hard-codes range_max to 100.0; this is not a demonstrated sensor capability. The sample conversion reads angle and range from different indices, and the publisher uses range order without publishing point.angle. The physical index-to-angle mapping requires further review/target checks before a correction can be chosen. Do not simply stretch the published angle increment to force the endpoints to match. No upstream source or runtime behavior was changed in this investigation. Quantitative angular calibration remains pending; current successful bringup checks remain valid within their stated scope.


### Conversion investigation (2026-10-25)

Traced packet decoding, revolution buffering, conversion, and ROS publication in the installed external driver. The upstream main source retrieved during this investigation has the same relevant conversion logic; no verified upstream fix was found in the inspected sources. References: [conversion source](https://github.com/ROBOTIS-GIT/coin_d4_driver/blob/8277b288902f28b37948b42e1bb80ba59afbba60/src/coin_d4_base_handler.cpp), [LaserScan contract](https://raw.githubusercontent.com/ros2/common_interfaces/lyrical/sensor_msgs/msg/LaserScan.msg), and [manufacturer specification](https://emanual.robotis.com/docs/en/platform/turtlebot3/appendix_lds_03/).

Confirmed by source and an isolated C++ reproduction:

- Extracted the exact `parse_lidar_serial_data` method into a temporary harness with the upstream data types and synthetic buffer/state inputs. No serial connections or ROS nodes were created. The test does not exercise packet decoding, actual hardware, or threaded timing.
- Input counts 400 and 402 produce 399 and 401 points, respectively, reproducing the observed angular endpoint discrepancy. Input element zero is omitted; determining whether a physical synchronization sample should be retained requires packet/protocol validation.
- In an eight-sample fixture, output element zero contains the angle from input 1 and range from input 7. Making input 1's angle invalid suppresses input 7's range. This confirms cross-sample validity checking, independent of the still-unverified physical clockwise/counterclockwise convention.
- The parser initially reconstructs per-sample angles from packet endpoints, but publication ignores `point.angle` and assigns positions solely through uniform index spacing. Reversing array order may be intended as a coordinate convention conversion; it does not demonstrate that the actual decoded angles align with the assumed grid. A mounting yaw cannot repair nonuniform angular errors.
- A synthetic 0.14 m return at intensity 65 becomes NaN; the same range at intensity 100 is retained. Thus the external driver already masks some near returns, even though this project has not added a filter chain. Do not convert these missing returns to free-space evidence.
- A count of one yields an empty output and non-finite time_increment. This is an edge-case reproduction, not an observed hardware event.

Other confirmed source behavior:

- The header uses node time at publication, not the first ray's acquisition time required by LaserScan. The buffer is already reversed before the optional `reverse` operation, yet the metadata still gives positive time_increment. Actual ray acquisition order and clock mapping must be established before motion compensation. Do not copy `outscan.stamp` blindly into a ROS timestamp: the internal clock implementation has compile-time-dependent clock behavior.
- Manufacturer documentation lists 0.05–12 m, nominal 4 kHz sampling, and 10 Hz scans. The driver's hard-coded 0.10–100 m limits do not match that specification. Around 400 input samples per turn and modest count variation are not themselves evidence of packet loss. The manual also lists a baud rate that differs from the working version-4 driver's 230400 setting; retain the demonstrated working setting pending hardware/protocol clarification.

Temporary reproduction: `/tmp/lds03-investigation/conversion_probe.cpp` and executable `conversion_probe`. Built with C++17 against `/home/pa4/ros2_lidar_ws/src/coin_d4_driver/include`; all assertions passed. These temporary artifacts may disappear. The external driver source, installed binaries, launch parameters, and Git state were not changed. Only these investigation notes were updated in the project.

Recommended next increment: prepare a reproducible C++ diagnostic build in an isolated workspace that exposes raw sample index, decoded angle, distance, and scan boundary timing for a stationary known-target test. Confirm which sample is a synchronization marker, physical angular direction/zero, and actual sampling order before designing angle-aware conversion. Correct metadata endpoint consistency alone is a limited fix; it cannot restore discarded angles or acquisition times from existing LaserScan messages. Then test a focused driver patch offline and on stationary hardware before navigation. Keep the working installation available for comparison; do not silently replace it.

### Isolated diagnostic build (2026-10-25)

Prepared a separate workspace at `/home/pa4/ros2_lidar_diagnostic_ws`. Its ROS package is `coin_d4_diagnostic_driver`, executable `diagnostic_coin_d4_node`, and shared library `libcoin_d4_diagnostic_driver_lib.so`. The original `/home/pa4/ros2_lidar_ws` source and installation remain unchanged. This is instrumentation, not a corrected driver. No sensors were opened during preparation or testing.

Reproducible project resources are in `tools/lidar_diagnostics/`:

- `build_isolated.sh`: verifies the inspected source checksums, refuses an existing destination, copies source into a new workspace, applies the patch without fuzz, and builds against only `/opt/ros/lyrical`.
- `source.sha256`: fingerprint of the inspected original source, including CMake/package files, parser, headers, and license. This checks reproduction against the inspected local source; it is not an independent upstream authenticity check.
- `instrumentation.patch`: adds capture hooks and gives the package/library/executable separate names. It omits installation of upstream launchers and the multi-sensor executable to avoid accidentally starting the normal package.
- `diagnostic_recorder.hpp`: bounded in-memory capture with CSV output; no angle/range/timestamp correction.
- `test_recorder.cpp`: hardware-free checks of mapping, completion, exclusive output, partial capture, timestamps, and NaN preservation.

To reproduce in a NEW workspace (the prepared workspace already exists, so do not repeat this command for that destination):

```bash
bash tools/lidar_diagnostics/build_isolated.sh \
  "$HOME/ros2_lidar_ws/src/coin_d4_driver" \
  "$HOME/ros2_lidar_diagnostic_ws_new"
```

Parameters added: `diagnostic_csv` (required, absolute path recommended, file must not exist) and `diagnostic_scans` (default 100, allowed 1–1000). Recording starts on converted scans after the normal warmup. Snapshots are copied while the scan buffer is locked; CSV formatting and writing occur after publication once the requested capture is complete. This adds processing overhead and is not a latency benchmark. Normal Ctrl+C writes any partial capture; forced termination may lose buffered data. A full capture prints `DIAGNOSTIC CAPTURE COMPLETE`; a write error prints `DIAGNOSTIC WRITE FAILED`. Do not treat an empty/partial CSV as a completed test. After capture the driver continues running until Ctrl+C.

CSV rows describe each decoded input sample before conversion, not raw UART bytes. Fields include raw index, sync flag, decoded angle, decoded distance (interpreted as millimetres by upstream), intensity/exposure, the separate index supplying the conversion angle, published index/angle/range, original scan metadata, internal scan-boundary times, host steady snapshot time, and ROS publication stamp. An omitted sample has published_index=-1 with blank published values. NaNs are preserved. Scan IDs are recorder-local; missing revolutions cannot be inferred solely from consecutive IDs. Internal/steady/ROS times are distinct clock domains; do not subtract across them without verifying clock mapping. The records are not per-ray hardware acquisition timestamps.

First user-run capture: stop BOTH normal drivers, leave the front sensor stationary, and place a broad upright target across its scan plane about 0.5–1 m away. Record the target's direction relative to an identifiable housing feature; do not assume that feature is scan zero. Keep the target and sensor stationary throughout the capture. Use a fresh terminal:

```bash
source /opt/ros/lyrical/setup.bash
source ~/ros2_lidar_diagnostic_ws/install/setup.bash
mkdir -p ~/ros2_lidar_diagnostic_ws/captures
capture_dir=$(mktemp -d "$HOME/ros2_lidar_diagnostic_ws/captures/front-baseline-XXXXXX")
echo "Capture directory: $capture_dir"
ros2 run coin_d4_diagnostic_driver diagnostic_coin_d4_node --ros-args \
  -r __ns:=/lidar_front_diag -r __node:=driver \
  -p port:=/dev/serial/by-path/pci-0000:80:14.0-usb-0:6.4:1.0-port0 \
  -p frame_id:=lidar_front_link -p topic_name:=scan_diagnostic \
  -p baudrate:=230400 -p version:=4 -p reverse:=false -p warmup_time:=5 \
  -p diagnostic_scans:=100 -p diagnostic_csv:="$capture_dir/samples.csv"
```

Wait for completion (roughly five-second warmup plus ten seconds of scans), then Ctrl+C. Report the CSV path, target/housing orientation, and any warnings. The diagnostic topic is `/lidar_front_diag/scan_diagnostic`. Do not run another process against the front serial device at the same time. Later captures at changed, documented target directions will be needed to establish physical angular direction and zero; one stationary capture alone cannot establish them. Return to the normal dual launch in a fresh terminal with the normal overlays after stopping diagnostics.

Validation completed: isolated package built on local Lyrical; package discovery and dynamic-library resolution point to the isolated installation; original source checksums still match; standalone recorder tests passed with warnings treated as errors. A synthetic harness containing the instrumented conversion method reproduced the same count/index/near-range/one-sample behavior as the original, under AddressSanitizer and UndefinedBehaviorSanitizer. The sanitizer run required execution outside the sandbox because LeakSanitizer could not run under its tracing environment. Runtime capture with real hardware remains pending and is executed by the user. No Git commits, staging, or ref updates were performed.


### First hardware diagnostic capture (2026-10-25)

User ran the isolated front diagnostic driver with an approximately 500 mm target described as physically in front. Capture completed and shutdown logs show both threads deactivated and the serial port closed. Source: `/home/pa4/ros2_lidar_diagnostic_ws/captures/front-baseline-bUFvtA/samples.csv` (machine-local, preserved unchanged).

- 100 scans (IDs 0–99), 40,099 decoded sample rows. Raw counts span 399–406, median 401; every scan emits exactly one fewer point.
- All 100 omitted index-zero entries have sync_flag=1 and nonzero decoded range/intensity. Ranges span 269–291 mm (median 279 mm), intensities 76–142, angles 358.6875–359.5625 degrees. Other entries have sync_flag=2. These are measurement-bearing decoded records, not empty entries; this alone does not prove protocol semantics or measurement accuracy.
- The signed circular residual `published_angle_deg + raw_angle_deg`, expressing departure from exact angular reversal, has median -1.956 degrees and 5th–95th percentiles -2.487 to -1.416 degrees. It is not a measured physical bearing error. Per-scan median residuals span -2.419 to -1.577 degrees; after removing each scan's median, the aggregate 5th–95th percentiles are -0.301 to +0.484 degrees. A fixed mounting-yaw adjustment alone cannot make the published grid equal the decoded angles for every sample.
- A candidate panel appears around decoded angles 160–200 degrees. In the narrower 177–183 degree window, 680 samples in the exploratory 400–650 mm range band have a 541 mm median (539–543 mm total span); per-scan medians span 540–542 mm. This shows a stable candidate return, not a +41 mm calibration error: target identity, tape reference, face orientation, and optical origin are not yet established. The candidate is near 180 degrees, not verified +X/zero. Do not assign mounting yaw from the word “front.”
- Of 39,999 emitted rows, 6,311 have NaN published ranges and 3,741 have finite ranges below the declared minimum, including 1,782 exact zeros. Consumers must respect invalid/out-of-range values; this does not establish which environmental surfaces caused them.
- Median reported scan_time is 99.765 ms. One initial publication interval is about 56 ms; the capture begins after the warmup buffer wait. This short initial interval is not enough to diagnose loss, and instrumentation timing is not a latency benchmark.

Next discriminating experiment: remove only the panel, leave the sensor and other desk objects fixed, and capture another 100 scans to a new CSV. Compare angular sectors between captures to identify the panel before testing a deliberate physical left/right displacement. Physical “front” must be tied to a reproducible housing feature. No driver correction or mounting transform was applied from this dataset.

Capture SHA-256: `5e6f1ba1ce941e08911f6527ff32a9b0c7e3f2c37de93d26c3f4ca4d93a1cb0b`.

### Panel-removal comparison (2026-10-25)

User completed the no-target capture and clean shutdown. Source: `/home/pa4/ros2_lidar_diagnostic_ws/captures/front-no-target-pBwRWO/samples.csv` (preserved unchanged). It contains 100 scans and 40,019 decoded rows; raw counts span 398–406, median 400.

In decoded angles 160–200 degrees and the exploratory 400–650 mm distance band, the baseline contains 3,943 returns across all 100 scans; the no-target capture contains zero. This strongly identifies the removed panel as the baseline cluster. In the narrower 177–183 degree sector, the baseline has 680 returns spanning 539–543 mm, median 541 mm. After removal, there are 672 samples: 100 zero-distance entries and 572 positive returns spanning 824–1,042 mm, median 827 mm. The positive-return median excludes zero readings; it does not describe a single flat background surface.

This comparison identifies the target, but does not establish absolute range accuracy or physical angular zero/direction. The approximate 500 mm placement is insufficient to infer a +41 mm bias. Next, return the panel and move it about 30 degrees counterclockwise around the stationary sensor as viewed from above, at approximately the same radial distance and facing the sensor. Record the housing feature defining the original “front.” A new capture will test whether decoded and published bearings change in the expected directions. No driver correction or mounting transform has been applied.

No-target capture SHA-256: `c9c33d632b2d1271ea14b932d65b93504fda2e0e76ed56f490cfb1852cb63c8c`.

### Counterclockwise target movement (2026-10-25)

User clarified that the original “front” means the robot's forward direction, not a named sensor-housing feature, and supplied `/home/pa4/ros2_lidar_diagnostic_ws/captures/front-ccw30-N12VMG/samples.csv`. Capture and shutdown completed. CSV integrity checks found 100 scans, 40,101 rows, raw counts 399–405, matching per-scan row counts and contiguous raw indices.

Using the same exploratory 400–650 mm distance band within decoded angles 100–240 degrees, the baseline panel occupies 162.797–198.234 degrees (3,943 samples) and the moved panel occupies 135.625–170.109 degrees (3,827 samples). The aggregate decoded-angle median changes from 180.250 to 152.984 degrees (-27.266 degrees); the corresponding published-angle median changes from 177.327 to 204.812 degrees (+27.485 degrees). These are cluster medians, not fitted panel-center bearings or a calibrated rotation measurement.

For the requested approximately 30-degree counterclockwise movement viewed from above, the decoded angle decreases and the published angle increases with reverse=false. Thus the current sign reversal is consistent with that physical test; removing the reversal wholesale would invert the observed direction. The unresolved conversion defects concern decoded-angle placement, omitted measurements, metadata, invalid ranges, and timestamp semantics. The robot-forward reference is near 180 degrees in this desk setup, but this does not establish a calibrated mounting yaw for the future robot. Preserve reverse=false and leave final mount coordinates/yaw pending physical installation. Next software work should replay these preserved captures offline to validate an angle-aware conversion before a separate hardware comparison.

Counterclockwise capture SHA-256: `e41fc550e837eba55dc99cbf0f881f8259279f1e5b7e3f08869e625d6baf7938`.

### Offline angle-grid candidate (2026-10-25)

Implemented standalone C++ conversion and capture replay in `tools/lidar_diagnostics/angle_grid.hpp`, `replay_angles.cpp`, and `test_angle_grid.cpp`; see that directory's README for reproduction and policy details. Both programs compile with C++17 and warnings treated as errors. Synthetic tests and replay of all three captures (300 scans, 120,219 decoded records) pass. The candidate includes index zero, maps each range using its own negated decoded angle, uses 400 circular bins with consistent endpoints, preserves the weak-close-return filter, rejects out-of-range values, keeps the nearest valid return on collisions, and leaves missing bins NaN.

Baseline/no-target/counterclockwise captures yield respectively 29,846/29,838/30,989 selected returns and 201/170/188 collision losers. Index zero is selected in 88 scans per capture. Maximum binning error is 0.45 degrees, a numerical quantization bound rather than physical bearing accuracy. The 400-bin choice and collision policy are explicit experimental choices. Range limits remain the upstream 0.10–100 m for comparison, pending specification reconciliation. The candidate has no timestamp model or ROS publisher and does not claim to fix acquisition timing. No installed driver or hardware process was changed. Next stage is a separate experimental build and stationary comparison, with timing resolved before navigation use.

### Separate experimental ROS build (2026-10-25)

Prepared `/home/pa4/ros2_lidar_experimental_ws` with package `coin_d4_experimental_driver` and executable `experimental_coin_d4_node`. Reproduction, exact user-run commands, and expected metadata are in `tools/lidar_diagnostics/README.md`. The build uses `experimental_scan.hpp` to adapt decoded SDK samples into the tested grid. The publisher no longer reverses the already-converted grid a second time. RAII locking protects conversion. The normal and diagnostic driver installations were not modified; no sensor process was started by the assistant.

The experimental adapter uses 0.05–12 m from the [ROBOTIS LDS-03 specification](https://emanual.robotis.com/docs/en/platform/turtlebot3/appendix_lds_03/), checked on 2026-10-25. The original weak-close-return filter is retained. Working baudrate 230400 is retained despite the manual's conflicting 115200 value. The offline replay accepts `--lds03` to test these range limits while preserving the earlier baseline defaults. All 300 captured scans pass replay with the new limits. SDK adapter boundary tests pass with warnings treated as errors and UndefinedBehaviorSanitizer; the ROS package builds and resolves its separate library correctly. Upstream source hashes remain unchanged.

This experiment requires `stationary_test:=true` and prints a timing warning: header stamps remain publication time and time_increment=0. It does not satisfy the first-ray acquisition timestamp semantics needed for a fully corrected navigation driver. The next user step is stationary front-sensor launch, checking 400 points, range limits, metadata, rate, RViz appearance, and shutdown. Jetson and live experimental hardware validation remain pending. No commits or staging were performed.

### User hardware validation and timing investigation (2026-10-25)

User subsequently confirmed both experimental sensors pass individual metadata,
visual stability, shutdown, and restart checks. Both publish 400 ranges/intensities
with 0.05–12 m limits and the intended frames. With both drivers running, each topic
has one publisher; sequential rate observations show front 10.036–10.044 Hz and rear
10.006–10.028 Hz. User reports both views visually stable. This is a stationary smoke
test, not proof of no dropped scans or physical calibration. Rear sensor is approximately
180 degrees relative to front; preserve that as a provisional mounting observation,
not a calibrated TF. Keep reverse=false for both. User committed the offline candidate
as `1636d21` and the isolated experiment as `6ca1bae`.

Source and existing-capture timing investigation is recorded in
`lidar_timing_investigation.md`. Internal ring timestamps are host parser timestamps
in the local monotonic clock domain; publication uses ROS time. The queued-byte delay
estimate is unused by the publisher and contains an unvalidated 5 kHz assumption.
The reordered grid cannot obtain correct acquisition timing by assigning a positive
scan_time/400 increment. Next work is bounded packet/clock provenance instrumentation
before choosing a time-aware output strategy. Existing installed drivers remain unchanged.
