# Offline angular conversion experiment

For the separate bounded packet/clock capture build and repeatable offline checks,
see [TIMING.md](TIMING.md). For the reviewed staging groups, see
[tooling checkpoint](../../docs/navigation/dual_lidar_tooling_checkpoint.md).

`angle_grid.hpp` is a standalone C++ candidate, not installed into either ROS driver.
`replay_angles.cpp` reads diagnostic captures without modifying them. It validates
row counts and raw indices, then checks sample accounting and selected-bin geometry.
It accepts the recorder's numeric, unquoted CSV format and reverse=false captures.

## Candidate rules

- Use each sample's own decoded angle, distance, and intensity, including index zero.
- Negate decoded angle for reverse=false; the physical counterclockwise panel test
  supports this sign. No mounting-yaw correction is applied.
- Round to the nearest of 400 bins covering [0, 2π), wrapping across zero. This
  explicit experimental choice gives 0.9-degree spacing, independent of input count.
  It is not a claim of hardware angular accuracy. The function accepts other sizes.
- Set angle_max=(bin_count-1)*angle_increment; do not duplicate the zero-degree ray.
- Keep the nearest valid return when multiple returns occupy one bin; first input
  wins equal-distance ties. This can favor a near outlier, so retain collision counts
  and review physical behavior before deployment. Do not interpolate across gaps.
- Empty bins have NaN range and zero intensity. Reject nonfinite/invalid angles,
  nonfinite ranges, and ranges outside the configured limits. Retain the upstream
  weak-close-return filter and exposure/intensity handling.
- Defaults remain 0.10–100 m for comparison with the current driver; these are not
  validated sensor specifications. Zero distances now fail the minimum-range check.
- No ROS message or timestamp is produced. Sorting into angular bins does not
  establish acquisition order, first-ray time, or time_increment. Those need a
  separate design before moving-platform use. Do not transfer upstream timing
  metadata into a reordered scan without validation.

## Build and run

From the repository root (no ROS installation needed):

```bash
replay_build=$(mktemp -d /tmp/lidar-angle-replay-XXXXXX)
g++ -std=c++17 -Wall -Wextra -Werror -pedantic \
  tools/lidar_diagnostics/test_angle_grid.cpp -o "$replay_build/test_angle_grid"
g++ -std=c++17 -Wall -Wextra -Werror -pedantic \
  tools/lidar_diagnostics/replay_angles.cpp -o "$replay_build/replay_angles"
"$replay_build/test_angle_grid"
"$replay_build/replay_angles" \
  "$HOME/ros2_lidar_diagnostic_ws/captures/front-baseline-bUFvtA/samples.csv" \
  "$HOME/ros2_lidar_diagnostic_ws/captures/front-no-target-pBwRWO/samples.csv" \
  "$HOME/ros2_lidar_diagnostic_ws/captures/front-ccw30-N12VMG/samples.csv"
```

`test_angle_grid.cpp` covers cardinal/irregular bearings, index zero, reverse mode,
wraparound, nearest-return collisions and ties, invalid ranges/angles, the weak-return
filter, exposure handling, empty input, invalid configuration, and endpoint consistency.
Replay failures return a nonzero exit status. Recorder-local scan IDs are not proof
that every hardware revolution was captured.

## Replay results, 2026-10-25

Each capture contains 100 scans. Every input is accounted for as rejected, a selected
bin winner, or a collision loser. The maximum selected-return quantization error is
0.45 degrees in each capture; this measures only binning error against decoded angles.

| Capture | Raw rows | Rejected | Collision losers | Selected returns | Empty bins | Index-zero winners |
|---|---:|---:|---:|---:|---:|---:|
| baseline | 40,099 | 10,052 | 201 | 29,846 | 10,154 | 88 |
| no target | 40,019 | 10,011 | 170 | 29,838 | 10,162 | 88 |
| counterclockwise | 40,101 | 8,924 | 188 | 30,989 | 9,011 | 88 |

Empty bins include gaps left by invalid readings and discretization. They are not
packet-loss counts. Source paths, hashes, physical setup, and earlier driver findings
are recorded in `docs/navigation/dual_lidar_setup.md`.

## Separate stationary ROS experiment (2026-10-25)

Built at `/home/pa4/ros2_lidar_experimental_ws`, package
`coin_d4_experimental_driver`, executable `experimental_coin_d4_node`, library
`libcoin_d4_experimental_driver_lib.so`. The working and diagnostic builds are unchanged.
Source modifications are reproducible with `experimental.patch`, `experimental_scan.hpp`,
`angle_grid.hpp`, and `build_experimental.sh`. The script verifies upstream checksums,
refuses an existing destination, snapshots its experiment inputs, and builds against
the ROS Lyrical underlay without the normal driver overlay. No upstream launchers are installed.

The SDK adapter explicitly uses the [ROBOTIS LDS-03 specification](https://emanual.robotis.com/docs/en/platform/turtlebot3/appendix_lds_03/)
range of **0.05–12 m**. The earlier offline defaults above remain available for baseline
comparison. Replay with `--lds03` selects these experimental limits. The upstream
filter rejecting ranges <=0.15 m with intensity <=65 is retained; the specification
does not guarantee every close return will survive this software filter. ROBOTIS also
lists 10 Hz nominal scanning, 4 kHz sampling, and 0.9-degree nominal resolution, with
frequency/resolution variation. Keep the working 230400 baud setting: the manual's
115200 value conflicts with the demonstrated version-4 setup and remains unresolved.

The build requires `stationary_test:=true` before opening the serial device. It keeps
publication-time header stamps and sets time_increment=0. These are explicit experimental
limitations: the [ROS LaserScan contract](https://raw.githubusercontent.com/ros2/common_interfaces/lyrical/sensor_msgs/msg/LaserScan.msg)
expects first-ray acquisition time. This build is for stationary visualization and
comparison, not motion compensation or navigation. No acquisition-time correction is claimed.

To reproduce in a new destination:

```bash
bash tools/lidar_diagnostics/build_experimental.sh \
  "$HOME/ros2_lidar_ws/src/coin_d4_driver" "$HOME/ros2_lidar_experimental_ws_new"
```

Stop existing sensor drivers first. Keep sensor and target stationary. In a fresh terminal:

```bash
source /opt/ros/lyrical/setup.bash
source ~/ros2_lidar_experimental_ws/install/setup.bash
ros2 run coin_d4_experimental_driver experimental_coin_d4_node --ros-args \
  -r __ns:=/lidar_front_experiment -r __node:=driver \
  -p stationary_test:=true \
  -p port:=/dev/serial/by-path/pci-0000:80:14.0-usb-0:6.4:1.0-port0 \
  -p frame_id:=lidar_front_link -p topic_name:=scan \
  -p baudrate:=230400 -p version:=4 -p reverse:=false -p warmup_time:=5
```

In a second terminal, source the same underlay and overlay, then:

```bash
ros2 topic info /lidar_front_experiment/scan
ros2 topic echo /lidar_front_experiment/scan --once --no-arr --timeout 10
ros2 topic hz /lidar_front_experiment/scan
```

Expected: one publisher, frame lidar_front_link, 400 ranges/intensities, angle_min=0,
angle_increment approximately 0.015707963 rad, angle_max approximately 6.267477 rad,
range_min approximately 0.05, range_max=12, time_increment=0, and approximately 10 Hz.
In RViz use fixed frame lidar_front_link, the experiment topic, and Best Effort.
Compare against the normal driver sequentially without moving sensor/target. Stop this
driver with Ctrl+C before starting another driver on the same serial device.
There is no diagnostic_csv parameter or automatic capture in this build.

Validation: C++ adapter tests cover range boundaries, index zero, direction, endpoint
consistency, the weak-return filter, and timing placeholders; they pass with warnings
treated as errors and UndefinedBehaviorSanitizer. Replay with the specified range limits
passes all 300 scans: baseline/no-target/counterclockwise selected returns are
31,135/31,113/32,218, with 211/182/199 collision losers. The ROS build, package lookup,
and separate shared-library resolution pass on the desktop; original upstream hashes
still match. Hardware behavior and the Jetson environment remain untested.
