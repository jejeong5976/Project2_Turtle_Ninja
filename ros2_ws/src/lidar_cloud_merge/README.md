# Stationary merged LiDAR point cloud

This C++ node combines `/lidar_front/scan` and `/lidar_rear/scan` into
`/lidar/points_merged` (sensor_msgs/PointCloud2). Run alongside the existing desk
launch; it does not open serial devices, publish TF, or modify the original topics.
The required `stationary_test=true` guard reflects the experimental drivers'
publication timestamps and absent per-ray motion compensation. This is a desk
visualization, not a navigation-clearing observation source.

## Run

A verified temporary build is installed at `/tmp/lidar-cloud-install`. With the
existing desk launch still running, use a separate terminal:

```bash
source /opt/ros/lyrical/setup.bash
source /tmp/lidar-cloud-install/share/lidar_cloud_merge/local_setup.bash
ros2 run lidar_cloud_merge stationary_cloud_merge --ros-args \
  -p stationary_test:=true
```

For a persistent build (run by the user):

```bash
source /opt/ros/lyrical/setup.bash
mkdir -p ~/ros2_lidar_merge_ws
cd ~/ros2_lidar_merge_ws
colcon build --base-paths ~/git/Project2_Turtle_Ninja/ros2_ws/src/lidar_cloud_merge \
  --packages-select lidar_cloud_merge --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
ros2 run lidar_cloud_merge stationary_cloud_merge --ros-args -p stationary_test:=true
```

In the existing RViz window, retain fixed frame `lidar_front_link`, add a
PointCloud2 display for `/lidar/points_merged`, choose Best Effort reliability,
Points style and a 3-pixel size. Set Decay Time to 0.5 seconds so old clouds disappear
if publication stops. For provenance coloring choose Intensity and channel
`source_id`, with bounds 0 and 1; these colors need not match the raw scan displays.
Disable the two LaserScan displays when inspecting the merged display alone.

## Data contract

- Default target_frame: `lidar_front_link`. TF is queried at each input stamp.
- Default max_age: 0.3 seconds, checked against ROS stamp and steady-clock receipt
  age. Default max_skew: 0.075 seconds between input publication stamps.
- Each output consumes one new input from each sensor. Pending older inputs can
  be dropped when timestamps are too far apart. Both sensors must keep publishing;
  there is no one-sensor fallback. Startup may skip pairs until static TF arrives.
- The output stamp is the later input stamp. Pairing is approximate; it does not
  imply synchronized acquisition or deskew. The output rate is normally near the
  slower input rate but can be lower if pairs are rejected.
- Fields are float32 x/y/z (metres), intensity, and source_id (0 front, 1 rear).
  Finite returns within each scan's advertised inclusive limits are transformed.
  Missing/nonfinite intensity becomes zero. No resampling, averaging, duplicate
  removal or free-space ray inference occurs. Overlap can contain two returns.
- Existing 0.220 m X / pi yaw desk TF remains provisional. Calibration is deferred.

## Verification

The standalone CMake build and core tests pass: rigid projection, invalid/out-of-range
filtering, cloud layout, empty clouds, timestamp age, future stamps and pair skew.
A hardware-free bag check against `desk-alignment-e2FRy6` independently checked
246,506 projected points from 349 front and 348 rear scans against the recorded
180-degree transform. The user confirmed the live merged view on 2026-10-25:
RViz displayed both source colors on /lidar/points_merged with Global Status OK.
The desk driver/TF launch must remain running alongside the merger; running only
the merger and RViz produces no points. At this initial checkpoint, live sensor-loss/
recovery behavior had not been validated; see the 2026-10-26 results below.
Moving-platform operation remains unvalidated. Calibration remains deferred.

## Optional robot-body exclusion (2026-10-26)

The user confirmed a 256 x 256 mm body footprint centered on the plate datum and
10 mm outward margin on every side. This gives a 276 x 276 mm XY exclusion region,
including its boundary, at all heights. It is a geometric exclusion, not proof that
every removed return came from the robot; obstacles inside the margin disappear too.

With the provisional front origin at plate (+110,+5) mm and front yaw pi,
`x_front=0.110-x_plate`, `y_front=0.005-y_plate`. Consequently the mask center in
lidar_front_link is (+0.110,+0.005) m, and its limits are X [-0.028,+0.248] m,
Y [-0.133,+0.143] m. Do not use a square centered on the front sensor itself.
Recompute these values when mounting geometry changes. Current filtered mode requires
`target_frame=lidar_front_link`; the box is axis-aligned in that frame.

Startup parameters (metres): `body_length=0.256`, `body_width=0.256`,
`body_margin=0.010`, `body_center_x=0.110`, `body_center_y=0.005`.
`self_filter_enabled` defaults to false. Parameters are read at startup; restart the
node to change the filter. Use this after rebuilding the persistent workspace:

```bash
ros2 run lidar_cloud_merge stationary_cloud_merge --ros-args \
  -p stationary_test:=true -p self_filter_enabled:=true
```

The filter acts after both scans are transformed and before cloud serialization.
It preserves survivor intensity/source IDs and both original scan topics. A periodic
log reports removed/retained points in the latest pair; an all-excluded pair produces
an empty cloud. Disable by restarting with `self_filter_enabled:=false`.

Core tests cover transformed rear points, all boundary corners, outside points on
each edge, coordinate conversion, negative-margin rejection and retained point fields.
Live verification reported by the user on 2026-10-26:
- The enabled filter logged 23-76 removed points per reported pair and retained
  577-657 points. These are observations, not acceptance thresholds.
- The user confirmed the physical exclusion check: body-region returns disappeared
  while targets outside the expanded boundary remained visible.
- The user confirmed front-driver and rear-driver recovery after separately
  stopping and restarting each driver; merged output resumed without restarting
  the merger.

For repeat verification, compare enabled/disabled views and place a stationary
target just outside an accessible mask edge (and beyond the sensor's minimum
range). Confirm it remains visible. Fine calibration and moving-platform operation
remain deferred; these checks validate the current stationary arrangement only.
# Front scan preparation (2026-10-27)

`stationary_front_scan_filter` is a separate C++ executable. It reads
`/lidar_front/scan` and writes `/lidar_front/scan_body_filtered` using sensor-data
QoS. It requires explicit `stationary_test:=true`; this acknowledges stationary
use, it does not detect motion. It opens no device, publishes no TF and leaves
the existing merger and raw scans unchanged.

The fixed front-frame body box matches the accepted full steering envelope:
X [-0.0276, 0.2476], Y [-0.1575, 0.1675] metres, inclusive. Body endpoints and
invalid/out-of-range beams become NaN, never clear-space infinity. Valid exterior
returns retain their ranges/intensities. Header, physical frame, angle grid and
timing are preserved. This is endpoint exclusion, not a calibrated angular
occlusion model; exterior returns still need physical inspection. No synthetic
origin, merged-cloud projection, retimestamping or deskew is performed.

This is a stationary inspection component, **not a moving-SLAM input contract**.
It deliberately provides no scan freshness or settled-motion admission gate.
Do not feed it to a moving mapper until the selected timing workflow is validated.
The experimental driver still publishes at receipt/publication time with unknown
per-ray timing. NaN handling must also be verified with the chosen mapper.

Offline tests cover the inclusive boundary, invalid/unknown rays, preservation
of metadata and input, and rejection of mismatched frames or invalid metadata.
Deployment and live validation remain user-run and pending.
