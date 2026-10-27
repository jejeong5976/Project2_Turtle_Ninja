# Offline FL0 body-box review

This C++ tool reads the two raw LaserScan topics from a saved bag and applies the
existing cloud merger's projection and exclusion functions without starting ROS
nodes. It reviews the FL0 drawing geometry accepted by the user on 2026-10-27:
front (+110,+5) mm with yaw pi; rear (-110,+5) mm with yaw zero, in plate axes.
No base-to-plate transform or per-ray timing is inferred.

The revised FL0 specification supplies a tight 255.2 x 305.0 mm rectangle
containing the scan-plane CAD envelope over 360-degree steering. Adding 10 mm
per side gives 275.2 x 325.0 mm: plate X +/-0.1376, Y +/-0.1625 m. The explicit
Jetson profile supersedes its earlier 276 mm square; runtime desk defaults stay
unchanged. The report separates points inside the tight CAD box, points inside
only the margin, and retained points. Geometric membership does not prove that
a point came from the robot or establish calibrated mounting geometry.

From the repository root, use a fresh output directory:

```bash
source /opt/ros/lyrical/setup.bash
cmake -S tools/lidar_body_review -B /tmp/lidar-body-review-build
cmake --build /tmp/lidar-body-review-build -j2
ctest --test-dir /tmp/lidar-body-review-build --output-on-failure
/tmp/lidar-body-review-build/review_body_mask /path/to/bag /tmp/lidar-body-results
python3 tools/lidar_body_review/plot_review.py \
  /tmp/lidar-body-results/points.csv /tmp/lidar-body-results/comparison.png
```

The output directory must not already exist. Outputs are `summary.txt`, `scans.csv`
and `points.csv`, plus `geometry.csv` for the profile used. The plot reads the box
dimensions from that file. Invalid input ranges are excluded by the same projection code as
the merger. The tool independently checks plate-coordinate projections and box
membership away from float rounding boundaries, verifies all point accounting,
and checks exact survivor order/intensity/source fields. It does not reproduce
the live merger's timestamp eligibility or pair selection. Plotting Python only
displays the C++ results.

For the October 27 clearance bag, the tool checked 550 front / 557 rear scans.
The revised box removed 35,375 / 51,578 points and retained 107,792 / 97,770. Of the
35,376 / 51,584 valid returns below 0.30 m, 1 / 6 remained. All recorded
returns at 0.30 m or farther were retained. These are properties of this capture,
not acceptance thresholds or proof of complete physical body exclusion. The seven
remaining samples are outside the specified Y bounds; the mask is not enlarged
to remove them. Front-frame bounds are X [-0.0276,0.2476],
Y [-0.1575,0.1675] m, inclusive, at all Z.

Live use remains a separate user-run stationary cloud view. Its explicit config
is `config/lidar_body_view.yaml`. Keep raw scan topics unchanged, use a separate
filtered-cloud topic, and provide exactly one front-to-rear TF owner. Do not use
the merged cloud as a single-origin SLAM scan or a free-space clearing sensor.

## Recorded live-output verification

`review_live_cloud` checks a saved bag containing both raw scans,
`/lidar/points_body_filtered` and `/tf_static`. It deliberately checks the same
revision-2 profile, 400-bin stationary scan contract and 75 ms maximum pairing
skew; it is not a general-purpose validator for other profiles.

```bash
/tmp/lidar-body-review-build/review_live_cloud /path/to/bag /tmp/lidar-live-results
```

The output directory must be new. The reader checks the recorded relative TF,
cloud layout, finite values, source ordering and exclusion bounds. It reconstructs
filtered points using that TF, associates raw scans within the output stamp's
75 ms window, and verifies survivor coordinates/order, exact intensities/source
IDs, nonreuse, and the later-input output stamp. Coordinate comparison allows
2 micrometres for ARM/x86 floating-point differences; the October 27 recording
matched exactly. Ambiguous candidate timestamps fail rather than being guessed.

Outputs are `summary.txt`, `pairs.csv` and `cloud_points.csv`. A cloud missing raw
input can be identified as a recording-boundary case only at the first/last
cloud and outside that source's captured header interval. Its missing-source
points are checked for layout/bounds but cannot be reconstructed. Unused raw
scans are reported separately. Bag receipt minus cloud stamp is recorded as an
observation, not acquisition latency or proof of the node's steady-clock guards.

The 09:34 October 27 bag had 405 clouds: 404 complete pairs matched exactly, while
the first cloud preceded both captured raw-scan sequences. One final front scan
followed the last cloud. Every recorded cloud obeyed the box and source layout;
no missing inputs or scan reuse were found among the 404 complete pairs. This
closes the stationary live-output check, not physical mounting or moving SLAM.
