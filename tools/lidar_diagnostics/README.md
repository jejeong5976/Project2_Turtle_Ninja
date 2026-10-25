# Offline angular conversion experiment

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

Next: integrate the candidate into a separately named experimental ROS build for a
stationary hardware comparison. Resolve timestamp semantics and range specifications
before claiming a corrected navigation driver. Preserve the working driver.
