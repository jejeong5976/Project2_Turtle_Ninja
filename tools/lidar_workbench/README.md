# Desktop integrated LiDAR workbench

This is the configurable desktop foundation for later mission work. It uses the
existing drive-only wheel stack, the verified LDS-03 driver source plus the existing
mapping patches, and continuous `slam_toolbox` mapping. Firmware, wheel calibration,
plate offsets, controller limits, and baseline configuration files are unchanged.

The user executes hardware and Git commands. Build and mock checks are hardware-free.
No Jetson connection is needed. The desktop runs the host software role; mock mode
does not reproduce Jetson scheduling, USB behavior, sensor noise, slip, or mechanics.

## Build once

ROS 2 Lyrical, the project's existing build dependencies, `slam_toolbox`,
`rosbag2_storage_sqlite3`, `yaml-cpp`, `patch`, and `fuser` must already be installed.
The build does not download anything. Run from the repository:

```bash
tools/lidar_workbench/build.sh \
  /home/pa4/ros2_lidar_ws/src/coin_d4_driver \
  /tmp/lidar-workbench-build-NEW
```

Choose a new workspace path. The script checks the upstream source hashes, copies
and patches the driver in that workspace, snapshots the host wheel/configuration
sources, compiles, and runs core tests. The protocol header is copied for host-side
compilation; firmware is neither compiled nor flashed. Existing workspaces are refused.
All subsequent parameter experiments use the same build and runtime options.

## One terminal

```bash
tools/lidar_workbench/run.sh /tmp/lidar-workbench-build-NEW --hardware mock --rviz
```

This launches mock wheels, two analytic room sensors, transforms, moving-frame
fusion, continuous SLAM, and a bag recorder. RViz opens as an optional window.
Wait for `READY`, then enter **one command at a time**, waiting for `READY` again:

```text
forward 0.05
backward 0.05
left 0.05
right 0.05
ccw 10
cw 10
save room_baseline
quit
```

Translation is in metres in the robot frame at the start of the segment; rotation
is in degrees. +X is forward, +Y left, positive yaw counterclockwise according to
the retained frame convention. Physical signs must be checked on the actual robot.
Commands are separate bounded segments, not a queued route or simultaneous compound
motion. Hardware scripts are refused. `stop`, `quit`, and Ctrl-C stop command output;
`status` reports health. Invalid command syntax is rejected. Runtime faults end the
session without automatic retry. A power emergency stop remains a physical action.

The launcher rejects an occupied local ROS domain and supervises only its own child
process groups. It requires fresh wheel odometry, both sensor streams, merged data,
mapping and expected TF/source ownership before admitting motion. It commands zero
and shuts down on health loss, process failure, terminal loss, departure from the
segment envelope, or timeout. The existing wheel command watchdog remains enabled.
These are software checks, not an obstacle-avoidance controller or a verified
physical stopping-distance guarantee.

For a noninteractive **mock-only** integration check:

```bash
tools/lidar_workbench/validate_mock.sh /tmp/lidar-workbench-build-NEW
tools/lidar_workbench/validate_mock.sh /tmp/lidar-workbench-build-NEW --sensor-loss
```

The first check records all six directions and checks the synthetic endpoints against
the known room geometry. The second pauses only its own simulated front sensor during
mock motion and checks for a recorded zero-command response and process cleanup.
These checks do not measure real sensor accuracy or the robot's physical stop time.

## Runtime settings

`run.sh WORKSPACE --help` lists all settings and units. Examples:

```bash
tools/lidar_workbench/run.sh /tmp/lidar-workbench-build-NEW \
  --hardware mock --speed 0.025 --turn-rate 0.10 \
  --scan-bins 1600 --map-resolution 0.05 \
  --filter-neighbors 1 --filter-radius 0.05 --protect-radius 0.30 --rviz
```

| Setting | Default and meaning |
|---|---|
| `--speed`, `--turn-rate` | 0.025 m/s, 0.10 rad/s; console `speed` and `turn-rate` can also change them while READY |
| `--max-linear`, `--max-angular` | Scan admission envelope 0.04 m/s, 0.15 rad/s; explicit experimental increases remain capped by unchanged controller limits 0.10 m/s and 0.5 rad/s |
| `--scan-bins` | 1600 angular output bins; 400–6400 supported. This does **not** increase physical angular resolution or native returns |
| `--map-resolution` | 0.05 m map cells; 0.01–0.10 supported. Cell size is not proven map accuracy |
| `--min-range`, `--max-range` | 0.05–12 m; can narrow the admitted range |
| `--filter-neighbors` | 0 disables the additional filter; 1–20 requires nearby other endpoints |
| `--filter-radius`, `--protect-radius` | 0.05 m neighbor radius; isolated returns within 0.30 m of their sensor origin are preserved |
| `--max-age`, `--max-skew`, `--max-odom-gap` | 0.25 s age, 0.075 s sensor-pair skew, 0.04 s maximum odometry gap |
| `--max-endpoint-error`, `--max-origin-shift` | 0.025 m and 0.005 m projection bounds; can only be tightened |
| `--max-distance`, `--max-turn` | 0.30 m, 30 degrees per command |
| `--settle`, `--hold` | 0.5 s settling plus 2 s stopped observation between segments |
| `--domain`, `--output`, `--session-seconds` | Local ROS domain 215, new session directory, 1800 s maximum session |

Native sensor sampling, serial baud rate, firmware settings, assumed wheel geometry,
plate offsets and nominal timing model are deliberately retained. There is no CLI
timing-offset tuning. Source time is nominal with delay zero and
`acquisition_time_valid=false`; deskew is conditional on that model and wheel odometry.
Increasing speed can cause rejection under the retained projection bounds. A CLI
setting does not establish an accurate operating speed.

## Data paths

Each sensor is corrected using bracketed wheel odometry and its own mounting pose.
Fusion transforms both clouds through `odom` to a common reference time, retaining
each return's actual ray origin under the timing model. It never silently substitutes
one sensor when the other is missing.

| Output | Use |
|---|---|
| `/lidar_front/scan_mapping` | Preserved guarded front input to continuous SLAM |
| `/lidar_rear/scan_mapping` | Rear projected scan for inspection; not fed to SLAM |
| `/lidar_front/raw_rays`, `/lidar_rear/raw_rays` | Selected SDK returns before deskew/body masking; not every raw packet/sample |
| `/lidar_front/rays`, `/lidar_rear/rays` | Deskewed observations with source ID, input bin, acquisition offset and ray origin |
| `/lidar/points_merged` | Unfiltered paired observations at a common time in `base_footprint` |
| `/lidar/points_filtered`, `/lidar/points_rejected` | Diagnostic spatial selection and its complement |
| `/map` | Continuous front-scan SLAM map |

Filtering identifies spatially isolated observations, **not proven sensor noise**.
Real thin or distant obstacles can be isolated too. The new filter is off by default;
the unfiltered and rejected observations are always retained. It does not alter
SLAM input or invent free space. Merged endpoints are not emitted as a synthetic
single-origin LaserScan, because the sensors observe from different origins.

The session directory contains the exact runtime settings, baseline configurations,
child command/log files, odometry and segment CSVs, ROS bag, and saved maps/pose graphs.
SDK timing traces retain the existing bounded capture mechanism for provenance;
they do not trigger a timing experiment or estimate a new timing offset.

## First user-run hardware checkpoint

1. Connect the desktop to the drive OpenCR and both LiDAR interfaces. Identify three
   distinct stable `/dev/serial/by-id/...` paths; no names are guessed. Keep the robot
   and cable sweep clear, with the physical stop accessible. The Jetson is unnecessary.
2. Inspect a dry run using the actual paths. `--dry-run` starts no nodes or devices:

   ```bash
   tools/lidar_workbench/run.sh /tmp/lidar-workbench-build-NEW \
     --hardware opencr --drive-port /dev/serial/by-id/OPENCR_PATH \
     --front-port /dev/serial/by-id/FRONT_LIDAR_PATH \
     --rear-port /dev/serial/by-id/REAR_LIDAR_PATH --dry-run
   ```

3. Replace `--dry-run` with `--execute --rviz` yourself. This configures hardware and
   enables wheel torque. Inspect stationary overlap and stable mapping first; use
   `save stationary_baseline`. Do not move until both streams and frame directions
   are credible. A failed startup is investigated rather than bypassed.
4. In open floor space, enter the six short commands above separately. Check physical
   direction, stopping, cables and wall consistency after every command. Measured
   start/end marks and headings provide independent references; wheel odometry alone
   is not a calibration measurement. Save `motion_baseline` and quit.
5. Review recordings and independent measurements before declaring the low-speed
   baseline accepted. Then use one unchanged scene and route for an optional filter
   comparison, and later increase speed one step at a time. Record map consistency,
   obstacle retention, scan gaps/rejections and measured physical error at each speed.

This is a proposed commissioning procedure, not an assignment rule. The supplied
dice-memory mission does not require a column calibration trial or a speed increase.
A three-face column scan is an optional later geometric checkpoint. Accuracy targets
for maze navigation and workstation approach must be agreed and physically measured;
the mission's puck-placement tolerance is not a claim about this SLAM map.
