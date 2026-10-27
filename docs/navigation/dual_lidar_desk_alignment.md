# Temporary desk alignment before merging

Timing research is paused. The user chose a measured temporary desk arrangement,
separate from the future robot URDF. The practical sequence is:

1. Start both tested stationary experimental drivers with one launch.
2. Publish the provisional rear scan origin pose relative to the front scan frame.
3. Overlay the two colored scans and check shared surfaces at multiple directions.
4. Add a C++ merger only after spatial alignment is checked. First combine transformed
   points in a common PointCloud2; if a consumer needs one LaserScan, project into
   angular bins with an explicit nearest-return policy and freshness policy.

Overlaying two LaserScan displays is an alignment check, not yet a merged topic.
Do not concatenate ranges: both sensors have different origins and occlusions. A
projected scan is a virtual sensor; clearing rays for navigation cannot automatically
be treated as rays from either physical origin. Moving-platform timing remains outside
the validated stationary scope.

## Common driver launch

`lidar.launch.py` now supports driver_variant=original (default) or
experimental_stationary. The latter requires stationary_test=true and its separately
built package to be sourced. Both use /lidar_front/scan and /lidar_rear/scan, matching
the existing YAML and RViz displays. front_port/rear_port override only their respective
YAML ports; keep host-specific USB paths out of robot geometry. Diagnostics' *_experiment
namespaces are not used by this combined launch.

The prepared `/home/pa4/ros2_lidar_bringup_ws` contains the updated launch files.
Stop all existing LiDAR processes before starting either combined launch.

```bash
source /opt/ros/lyrical/setup.bash
source ~/ros2_lidar_experimental_ws/install/setup.bash
source ~/ros2_lidar_bringup_ws/install/setup.bash

ros2 launch bringup lidar.launch.py \
  driver_variant:=experimental_stationary stationary_test:=true publish_model:=false \
  front_port:=/dev/serial/by-path/pci-0000:80:14.0-usb-0:6.4:1.0-port0 \
  rear_port:=/dev/serial/by-path/pci-0000:80:14.0-usb-0:6.1:1.0-port0
```

This starts drivers only. Each can be inspected in its own scan frame. No robot model,
motor controller, or assumed mounting transform is started.

## Measurements needed

Update (2026-10-25): the user confirmed the desk arrangement preserves the handoff's
220 mm longitudinal separation, lateral alignment, and equal height. Reuse these
accepted estimates. The robot-forward target previously appeared near 180 degrees
in the front scan, giving an initial rear pose in lidar_front_link of
x=+0.220 m, y=z=0, roll=pitch=0, yaw=pi. The sign of X follows this provisional
front scan orientation, not an assumption that robot-forward equals scan-zero.
Both scans are visible together in RViz; shared contours approximately align.
This is not a precise optical-origin or yaw calibration.

The parent frame is lidar_front_link at the front scan origin. Measure the rear scan
origin relative to it, in metres: rear_x along front +X, rear_y along front +Y, and
rear_z along front +Z. Supply rear roll/pitch/yaw in radians about those frame axes.
For a verified upright, level pair, roll/pitch can initially be zero. The reported
rear yaw of approximately π is an initial estimate, not calibrated geometry.

Use scan origins, not arbitrary housing edges or the desk height. Equal sensor
heights suggest a near-zero relative z only when both units are upright and supported
at the same height. The provisional robot h_scan/mount-surface numbers do not define
desk geometry. Physical optical-center uncertainty must be retained when using
housing centers as estimates.

To identify the front frame's physical axes, display RViz Axes at lidar_front_link
(red +X, green +Y, blue +Z) and use a nearby isolated target to associate its physical
direction with the plotted direction. Do not equate the robot's intended forward
direction with lidar_front_link +X: the earlier target described as robot-forward
appeared near 180 degrees in this sensor's published scan.

## Desk transform launch

`desk_lidar.launch.py` requires all six pose arguments explicitly; there are no
guessed geometry defaults. It starts the two stationary drivers and one temporary
static TF, lidar_front_link -> lidar_rear_link. It never expands the robot URDF.
Do not run another TF publisher owning lidar_rear_link at the same time.

After supplying measured variables, the command is:

```bash
ros2 launch bringup desk_lidar.launch.py stationary_test:=true \
  front_port:=/dev/serial/by-path/pci-0000:80:14.0-usb-0:6.4:1.0-port0 \
  rear_port:=/dev/serial/by-path/pci-0000:80:14.0-usb-0:6.1:1.0-port0 \
  rear_x:="$rear_x" rear_y:="$rear_y" rear_z:="$rear_z" \
  rear_roll:="$rear_roll" rear_pitch:="$rear_pitch" rear_yaw:="$rear_yaw"
```

The variables above are placeholders for the user's measurements, not shell values
provided by this document. Missing/nonfinite values fail before starting processes.
The existing RViz config opens in lidar_front_link, showing front red and rear blue.
Disable RobotModel during desk checks because this launch intentionally publishes no
robot_description. Both sensor views should agree on shared wall/board surfaces;
occluded areas can legitimately differ. Agreement at one surface is insufficient to
calibrate all six pose components. Begin with level mounting and check multiple
nonparallel surfaces before treating the approximate transform as aligned.

## Record the two shared surfaces

Leave the desk launch running with the current transform. Keep both sensors and
the diagonal and orthogonal target surfaces stationary and visible to both sensors.
Avoid walking through the scan plane during recording. In another terminal:

```bash
source /opt/ros/lyrical/setup.bash
source ~/ros2_lidar_experimental_ws/install/setup.bash
source ~/ros2_lidar_bringup_ws/install/setup.bash

mkdir -p ~/ros2_lidar_bringup_ws/captures
capture_dir=$(mktemp -d "$HOME/ros2_lidar_bringup_ws/captures/desk-alignment-XXXXXX")
echo "Capture directory: $capture_dir"

ros2 bag record -s mcap -o "$capture_dir/bag" \
  --qos-profile-overrides-path "$HOME/git/Project2_Turtle_Ninja/tools/lidar_diagnostics/desk_capture_qos.yaml" \
  --topics /lidar_front/scan /lidar_rear/scan /tf_static
```

After subscriptions are established, record for about 30 seconds, then press Ctrl+C
in the recorder terminal only. Wait for it to finish writing, then run:

```bash
ros2 bag info "$capture_dir/bag"
echo "Capture directory: $capture_dir"
```

Expect roughly 300 messages per scan topic for 30 seconds at 10 Hz and at least one
/tf_static message. Counts need not match exactly: this captures both streams
together but does not hardware-synchronize their measurements. Best-effort scan
subscriptions accept the sensor publishers; transient-local TF subscription receives
the existing static transform. If either scan is absent or TF has zero messages,
investigate before using the recording for alignment.

The next offline step is to select the same physical face of each target in both
scans, transform rear points using the recorded TF, and compare fitted line normals
and perpendicular offsets over multiple scans. Separate temporal scatter from a
persistent offset. Do not infer an exact correction from screenshot pixels or fit
unrelated/occluded surfaces together. Two nonparallel planar targets can constrain
planar translation and yaw; they do not validate the assumed height, roll, or pitch.
Keep the current TF unchanged until these measurements support an adjustment.

## Validation and remaining work

Hardware-free launch-declaration checks pass for driver selection, namespaces, YAML
and port overrides, the stationary guard, finite pose validation, and TF parent/child
direction. These checks instantiate launch declarations without starting processes.
The bringup/description install was rebuilt; the user ran the combined desk launch
and supplied an overlay with both scans visible. Quantitative desk alignment remains
pending the shared-surface capture. The recording CLI options were checked against
the installed ROS Lyrical CLI; no hardware recording was started by the assistant.
This step adds no merger and changes no robot URDF.
Desktop paths are temporary; discover and configure paths again on the Jetson.

## Stationary merged view implementation

The `lidar_cloud_merge` package now provides a separate C++ node for
`/lidar/points_merged`. Keep the provisional desk TF; calibration is deferred by
user choice. See `ros2_ws/src/lidar_cloud_merge/README.md` for the run command,
RViz settings, freshness policy and test scope. No additional hardware capture is
required to begin the merged visualization. The raw scan topics remain available.
