# Corridor timing diagnostic sources — 2026-10-28 v1

This is an opt-in source overlay for the existing front LiDAR and wheel stack.
It prepares evidence for software timing measurements and relative LiDAR–odometry
alignment. It does not apply a timing calibration or change firmware, wheel
calibration, mount geometry, zero plate offsets, scan correction, motion limits,
controller safeguards, or continuous SLAM configuration.

The user executes all deployment, hardware and Git commands. `stage.sh` only
copies and patches source into a **new** workspace. It refuses an existing
destination and checks exact hashes of the baseline files before patching.
Original installed workspaces remain available. No teammate archive is required.

## Build only

On the Jetson, after copying this directory to
`~/nt-timing-diagnostics-20261028-v1`, use a fresh terminal:

```bash
cd ~/nt-timing-diagnostics-20261028-v1 &&
sha256sum --check SOURCE.sha256 &&
bash stage.sh \
  "$HOME/jetson-wheel-config-20261027-v1" \
  "$HOME/ros2_front_mapping_v1_ws/src/coin_d4_mapping_driver" \
  "$HOME/ros2_timing_diagnostic_v1_ws" &&
source /opt/ros/lyrical/setup.bash &&
cd ~/ros2_timing_diagnostic_v1_ws &&
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --executor sequential \
  --packages-select interfaces hardware swerve_controller coin_d4_mapping_driver \
  --cmake-args -DNT_FIRMWARE_DIR="$PWD/firmware/nt_opencr" &&
cmake -S timing_tools -B timing_tools/build &&
cmake --build timing_tools/build -j2 &&
ctest --test-dir timing_tools/build --output-on-failure
```

The copied firmware directory supplies the unchanged shared packet header; this
does not flash anything. If a hash, staging or build step fails, stop and retain
its output. Do not bypass the hash check or overwrite a partial workspace.

These commands do not start robot nodes. Jetson execution overhead and physical
trace coverage remain unverified until a stationary capture. Corridor dimensions
may be supplied later, before a bounded movement plan is selected.

## Activation contract for the subsequent supervised stationary check

Do not launch hardware based on this file alone. First establish present power,
manipulator connection, robot position, command ownership and serial ownership.
Use the existing launch/configuration with the diagnostic overlay sourced last;
check resolved package prefixes and loaded libraries before collecting evidence.
All participating processes must inherit the **same new, existing absolute**
`NT_TIMING_DIR`. Without it the diagnostic logger is disabled. Use system time
(`use_sim_time=false`) and the existing live ROS domain. Create the directory
before launching. Do not run two wheel or front-driver instances.

Expected files are `opencr_PID.nttrace`, `hardware_PID.nttrace`,
`swerve_PID.nttrace`, `lidar_parser_PID.nttrace`, `lidar_source_PID.nttrace`,
`mapping_PID.nttrace`, and `observer_PID.nttrace`. Some share the same PID.
`timing_tools/build/observe_timing` is a passive subscriber. It does not subscribe
to `/cmd_vel`, publish commands/TF, open serial ports, or call services. Retain
the normal bag alongside these traces, including `/odom`, corrected scans,
status, TF, description and commands. The binary traces do not replace that bag.

After the user stops motion, save the map/graph while SLAM is available, finish
the console and bag, then shut down the instrumented processes normally. Keep
terminal logs. Trace footers are written only after producer threads stop and
the writer queue drains; a live trace or forcibly killed process is incomplete.
Do not inspect an open file and call it a passed capture. Run `inspect_trace`
once per closed file; this verifies container counts/clock brackets, not physical
calibration quality. A final flush/fsync error can be reported only on stderr,
so a passing inspector alone is insufficient if terminal output reports failure.

## Recorded evidence and its limits

- LiDAR serial-read brackets, packet checksums/IDs, every ring handoff boundary,
  raw samples and selected rays for frames consumed by the source converter.
  Raw sample IDs retain their packet and within-packet index. Handoff/frame ID
  gaps expose frames overwritten before conversion; these are not logger losses.
- Mapping callback arrival, queue drops, each processing attempt, waiting for
  odometry, and the actual corrected-scan publish call. A frame may have several
  processing attempts. Publish-call duration is not subscriber receipt latency.
- OpenCR host reads, decoded board-state time/sequence and raw joint values;
  hardware read/use, controller update, odometry production and publication.
  Join hardware use to decoded states by board time/sequence and host ordering;
  reset/wrap or ambiguous repeated keys must be rejected or segmented explicitly.
- Passive odometry/scan/map/map-to-odom receipt times. Join odometry and scan
  receipt to producer records by header stamp, checking duplicates and gaps.
  Map and TF receipt **do not measure SLAM internal processing duration**.

All events bracket `CLOCK_REALTIME` with `CLOCK_MONOTONIC` on the same host.
Use monotonic intervals for durations. Retain the ROS header stamp separately;
do not substitute it for arrival/publication time. Check clock offset stability
and bracket width across the run. NTP does not synchronize the OpenCR counter
with optical acquisition. OpenCR `State.ms` is a firmware state-construction
time after motor reads; it is not the encoder acquisition instant. The inspected
LiDAR protocol does not supply a true optical timestamp.

Known corridor geometry and informative motion can constrain relative alignment.
Without an independent timed motion/acquisition reference, both absolute sensor
delays are not uniquely identifiable. Keep `acquisition_time_valid=0` and current
nominal timing/delay settings until analysis supports a specific change. A flat
timing objective is an inconclusive result, not justification for a correction.
The offline fitter and a hardware motion runner are not part of these sources.

## Local validation

On 2026-10-28 the four overlay packages compiled against ROS Lyrical. All 73
existing hardware/controller tests passed. A synthetic serial parser run captured
306 packets, including 300 after the legacy recorder sealed. The hardware-free
mapping fixture retained stale-input rejection, recovery and regression latching;
the passive observer matched both published fixture scans by header stamp.
Mock OpenCR and controller traces closed with consistent counts. Logger tests
cover queue wrap/full, concurrent IDs/counts, cap/loss reporting, exclusive creation
and disabled mode. The inspector rejected missing and truncated footers. A UBSan
logger test passed before the final event-enum-only addition. Final staged patched
sources and logger headers were byte-compared against those compiled locally.
This does not validate Jetson overhead, serial hardware behavior or calibration.

## Binary schema v1

Each little-endian record is 104 bytes: two uint64 (`kind,id`), eight int64
(`mono_before,system,mono_after,stamp,a,b,c,d`), three doubles (`x,y,z`). Times
are nanoseconds unless stated otherwise. Field names/order are defined in
`nt_timing_trace.hpp`; the patch specifies every producer field.

| Event | Payload |
| --- | --- |
| Header 1 | id=schema 1, a=104, b=PID, c=record-attempt limit |
| End 2 | id=accepted, a=written, b=lost, c=I/O error, d=max producer emit duration ns |
| Read 10, LiDAR | id=packet, a/b=read bracket, c/d=queued bytes, x=requested, y=upstream read result, z=stage |
| Read 10, OpenCR | id=last decoded local state ID, a/b=read bracket, c=result bytes, d=capacity |
| Packet 11 | id=packet, a=ring header anchor, b=validation time, c=checksum OK, d=sample count, x/y=calculated/received checksum |
| Raw 12 | id=scan, a=raw index, b=packet ID, c=packet sample index, d=checksum flag plus exp_m shifted 8 and sync_flag shifted 16, x=angle degrees, y=decoded range mm, z=quality |
| Frame 13 | id=scan, stamp=handoff ROS clock; a/b=start/end, c=raw count, d=source valid; second record c=-1 carries exact clock-pair before/after in a/b |
| Beam 14 | id=scan, a=bin, b=raw source index, x=range metres, y=angle radians before motion correction |
| Handoff 15 | id=scan, stamp=handoff ROS clock, a/b=ring start/end, c=raw count, d=closing packet ID |
| Mapping 20–24 | arrival / processing begin / processing end / publish begin / publish end; id=scan, publication stamp=corrected scan reference |
| OdomReceived 25 | id/stamp=odom header stamp at mapping callback |
| QueueDrop 26 / OdomWait 27 | id=scan; wait stamp=desired reference, a=latest odom stamp |
| Rejected 28 | id/stamp=last accepted scan/reference, a=fault latch; reason is in normal ROS status/log, not this record |
| StateDecoded 30 | id=local state ID, a=board ms, b=command sequence echo, c=communication errors, d=status |
| StateJoint 31 | id=local state ID, a=joint index, b=board ms, x/y/z=raw position/velocity/current |
| Hardware 32/33 | read begin/end; id/stamp=controller-supplied time |
| StateUsed 34 | id/stamp=controller-supplied time, a=board ms, b=sequence, c=communication errors, d=status, x=host state age seconds |
| Controller 40/41 | update begin/end; id/stamp=update time |
| OdomMade 42 | id/stamp=update time, x/y/z=pose x/y/yaw |
| OdomPublish 43/44 | publish-call begin/end; id/stamp=odom header stamp |
| Observer 50/51/52/53 | scan/odom/map/map-to-odom receipt; id/stamp=header stamp |

Each logger has a 16,384-record bounded queue and separate disk writer thread.
Producer calls do no disk I/O. Queue contention/fullness, I/O failure, or the
20,000,000 attempted-record cap are counted as loss; any loss invalidates that
file for complete timing analysis. Each file is bounded to approximately 2.08 GB.
Files use exclusive creation and cannot be overwritten. Inspect per-event
associations, clock jumps, source losses and logger overhead in addition to
container counts. Bench overhead is not a Jetson real-time guarantee.
