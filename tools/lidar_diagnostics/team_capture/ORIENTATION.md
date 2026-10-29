# LiDAR orientation report — 2026-10-27

**Physical front-box test: pending.** The user explicitly reports that the box
has not yet been checked in RViz with `base_footprint` (or aligned `base_link`) as
Fixed Frame. The arena photograph establishes which way the robot faces; it does
not establish the displayed scan direction. Do not label the yaw physically verified.

The active NJWQJ7 transform is `base_footprint -> lidar_front_link`:
`xyz = 0.110, 0.005, 0.0545 m`, `rpy = 0, 0, pi rad`. The scan topic is
`/lidar_front/scan_mapping`, with `header.frame_id = lidar_front_link`.
These are the accepted mounting assumptions, confirmed in configuration/runtime,
not a newly measured correction. Plate x/y/yaw stay zero. Height remains estimated.

The current stack does not use an environment variable named `NT_SCAN_YAW`.
For the teammate's launch, **pi = 3.141592653589793 is the equivalent absolute
yaw only if the base axes, scan zero and angle direction are identical**. Their
archive defaults to pi/2, `base_link -> base_scan`, and different xyz offsets;
do not copy that default or add pi on top of it. Topic remapping alone does not
rename a scan frame or change its axes. A new driver convention needs a new check.

Current `base_link` is 0.033 m above `base_footprint`, without rotation. Thus the
equivalent direct URDF joint from `base_link` would be:

```xml
<!-- Reference only: integrate after the physical direction check. -->
<link name="lidar_front_link"/>
<joint name="lidar_front_joint" type="fixed">
  <parent link="base_link"/>
  <child link="lidar_front_link"/>
  <origin xyz="0.110 0.005 0.0215" rpy="0 0 3.141592653589793"/>
</joint>
```

Do not run this joint and the existing static publisher for the same child at
the same time. Do not install it into the live NJWQJ7 session. The teammate can
integrate the equivalent transform once the check is reported; the existing
plate-based URDF is also valid if it composes to this same transform.

## User-performed check, with robot stationary

1. Keep the current drivers, wheel state and mapper running. Use an existing,
   clearly identifiable object in front if possible, without changing the scene.
   If a new test object is necessary, record that scene change explicitly.
2. In RViz set Fixed Frame to `base_footprint`. Add TF/axes and LaserScan on
   `/lidar_front/scan_mapping`; use Best Effort reliability for the scan.
3. Identify the object's returns relative to the robot axes. A box directly in
   front must appear at positive X. A known object on the left must appear at
   positive Y. The latter checks mirroring as well as yaw. Measure distance from
   the base origin for comparisons; bumper clearance is a different distance.
4. Record observed direction (+X/+Y/-X/-Y), approximate measured and displayed
   distances, screenshot, frame, topic and active yaw. Report a mismatch before
   changing TF; do not rotate the active map or apply an additional correction.

Ready-to-complete report:

```text
Fixed Frame: base_footprint
LaserScan: /lidar_front/scan_mapping (frame lidar_front_link)
Front object appeared: PENDING
Left-side object appeared: PENDING
Physical / displayed distance from base origin: PENDING
Active absolute yaw: 3.141592653589793 rad
Physically verified NT_SCAN_YAW-equivalent: PENDING
Scene change / screenshot / check time: PENDING
```
