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
the merger and RViz produces no points. Live sensor-loss/recovery behavior and
moving-platform operation have not been validated. Calibration remains deferred.
