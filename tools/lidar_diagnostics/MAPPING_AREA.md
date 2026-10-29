# Supervised whole-area mapping

Prepared 2026-10-27 after the caGdIe revisit. The user requested whole-arena
mapping preparation and an offline 5 cm / 2.5 cm comparison. The user's first
console attempt, 1URnqw, refused preflight after a confirmed motor-power interruption;
it published no motion. The user recovered the drive in place at about 18:37;
18:38–18:39 read-only checks confirmed healthy input in the same map session.
The subsequent G0I8Np run completed 48 segments with confirmed automatic stops;
see [area review](../../docs/navigation/jetson_mapping_area_review.md). All hardware and Git commands remain user-operated;
assistant SSH is read-only. No extra agent or autonomous navigation is used.

## What changes and what stays

A separate C++ console records continuously while the user requests finite
straight/turn segments. It does not start the wheels by merely launching it.
The existing NJWQJ7 wheel/input/SLAM processes continue. Preserve front-only mapping,
LiDAR settings/sockets/geometry/body mask, assumed wheel calibration and zero plate
offsets. The completed pulse and route executables remain unchanged.

The 16.929 mm revisit residual is a useful diagnostic, not an automatic gate on
whole-area coverage. There is no requirement to reduce it to stationary-holdout
levels before this supervised mapping trial. Coherent walls/corridors, broad
coverage, meaningful revisits and graph evidence are the next useful assessment.
This is not autonomous navigation or obstacle avoidance.

## Console behavior

At each READY prompt, enter exactly one command and press Enter:

| Command | Meaning | Allowed range |
| --- | --- | --- |
| `f 0.20` | Forward, target 20 cm | 0.05–0.30 m |
| `b 0.10` | Backward, target 10 cm | 0.05–0.30 m |
| `l 30` | Turn left, target 30 degrees | 10–30 degrees |
| `r 30` | Turn right, target 30 degrees | 10–30 degrees |
| `q` | Finish recording while stopped | READY only |

These are examples, not a route to paste in sequence. Speeds remain 0.025 m/s
straight and 0.10 rad/s turning, with no combined translation/rotation or lateral
command. Targets are maximum desired excursions, not promises of exact travel.
Steering settling can shorten the movement; the console reports measured odometry.
It stops at the target or finite publication count and never extends automatically.

Each segment has five zero commands, stationary odometry verification and two
seconds of stationary capture before another READY prompt. In **console v2**,
ordinary input during preflight/motion/settling is discarded, never queued for
another movement. Mistyped or overlong READY commands produce an explanation and
keep recording. Arrow/function keys, Escape, Ctrl+D, Ctrl+S and Ctrl+Z do not stop
the run. **Ctrl+C is the explicit stop**; actual terminal EOF/hangup, SIGTERM and
SIGHUP also request five abort zeros after motion. Faults end the session without
retry. The previously deployed v1 still uses any-key abort until the user builds
and selects v2. A network stall can delay terminal events; finite bounds remain,
but software cannot guarantee physical braking. Keep the physical cutoff accessible.
Do not type ahead, paste multiple movement lines or run a second command source.

The same input gates apply: <=40 ms odometry gaps; <=0.04 m/s translation and
<=0.15 rad/s rotation; fresh scan/status/odom/map/TF; nominal timing/zero hypothetical
delay with acquisition_time_valid=0; active mapper; sole matched TwistStamped
command ownership; no new rejected input; no clock discontinuity. Map resolution
may be 0.05 or 0.025 m, fixed throughout a console session. This acceptance does
not change the mapper resolution. V2 permits one additional passive command
subscriber named `/team_mapping_recorder`, for the separate bag script. Exactly
one matched wheel subscriber is still mandatory; unexpected consumers and extra
publishers still prevent motion. Start the recorder before the console.

Straight limits relative to each segment start: no >10 mm wrong-way motion,
<target+30 mm travel, <20 mm lateral and <0.050 rad yaw. Turn limits: <30 mm
translation, <0.020 rad wrong-way yaw and <target+0.050 rad yaw. Command gaps must
remain <150 ms. Publication count is ceil(target/speed/0.1), max 120 straight or
53 turning, with a count*0.11+0.1 second deadline. A no-progress segment fails;
no retry is automatic. Stationary holds remain within 0.5 mm / 0.005 rad of their
anchor. Session bounds are 15 minutes, 100 segments, 500,000 records and 1 GiB of
serialized data, with 3 GiB free required initially. Finish with `q` before a limit;
inspect any limit/fault stop before starting another recording.

The bag records existing odometry, corrected scan/status, map, TF, commands and
joint states. It also subscribes to the observed `/slam_toolbox/pose_graph`,
`/slam_toolbox/new_node_event` and `/slam_toolbox/loop_closure_event`. Optional topic
absence is not treated as proof of no loop closure. `segments.csv` records completed
segments; events/commands preserve partial segments on an abort. Preserve raw
physical_stop_confirmed=false and record human observations separately.

## Recovering the confirmed motor-power interruption

**Completed on 2026-10-27 around 18:37. Do not repeat this section while healthy.**
User hardware/controller activation succeeded. Subsequent read-only checks found
the same processes, active mapper, fresh scans with published=47400 then 47420 and
rejected=12749 unchanged, battery about 12.1 V, no command publisher and 190 GB free.
User odometry remained near the prior caGdIe pose (0.0285 mm position difference,
-0.10464 degrees yaw difference from that historical recording). Continue at
"Choosing the path" / "Save before, explore, save after" using the existing console.
Retain `jetson-area-checkpoints.xFPjny` as the saved pre-coverage checkpoint.

The 18:16:32 motor-read failure left `bus_drive` unconfigured and the existing
`joint_state_broadcaster` / `swerve_controller` inactive. The robot was not moved;
one physical object was moved. Preserve the live NJWQJ7 mapper and its input.
The 18:29 checkpoint in `/home/turtle/jetson-area-checkpoints.xFPjny` contains
occupancy and serialized graph/data; restoration is still untested.

For this observed state, the user restores a charged motor supply with the base
stationary and manipulator disconnected, keeps the steering envelope clear and
leaves the existing launches running. Configuration briefly disables motor torque;
activation enables it. These commands are hardware operations, not passive checks.
No HOME/arm sequence belongs in this drive-only session.

The installed ros2_control version is 6.9.0. Its
[resource manager](https://github.com/ros-controls/ros2_control/blob/6.9.0/hardware_interface/src/resource_manager.cpp#L598)
performs configure then activate when unconfigured hardware is requested active.
Use one request, rather than dwelling in inactive between separate commands.
The deployed swerve activation resets the command, retains `pose_` and uses zero
integration time for its first update. Unloading/reconfiguring that controller or
restarting the wheel launch instead recreates the pose object and breaks this
continuity assumption. The user has now executed this recovery successfully;
it does not establish recovery after base relocation or a process restart.

User runs on Jetson, without any teleop or other command publisher:

```bash
source /opt/ros/lyrical/setup.bash
export ROS_DOMAIN_ID=182
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

ros2 control set_hardware_component_state bus_drive active &&
ros2 control switch_controllers \
  --activate joint_state_broadcaster swerve_controller --strict
```

Stop on failure; do not retry or restart either launch blindly. Then inspect:

```bash
ros2 control list_hardware_components
ros2 control list_controllers
timeout 8 ros2 topic echo /odom nav_msgs/msg/Odometry --once --no-daemon
timeout 8 ros2 topic echo /lidar_front/mapping_status std_msgs/msg/String \
  --once --full-length --no-daemon
```

Require hardware and both controllers active, fresh finite odometry and mapping
status `fault=0 fresh=1 reason=publishing`. The accumulated rejection count need
not return to four: it increased while power was off. New rejections should stop
and published scans should advance after recovery. caGdIe's final recorded odom
was approximately x=0.28234 m, y=0.01114 m, yaw=0.13096 rad; this is a historical
reference, not a sample immediately before power-off. Continuity was checked
read-only at 18:38–18:39. Keep the map at 5 cm and recheck clearances after the object
move; then resume the supervised coverage procedure without repeating caGdIe.

## Choosing the path

The user chooses each movement from the real floor, not from unknown map cells.
Before straight motion, check at least target+100 mm of clear level travel from
the outermost robot part in that direction and at least 50 mm along both sides of
the steering envelope. Before turning, check the entire swept robot envelope plus
50 mm; straight-path side clearance does not establish turn clearance. Cables and
protrusions count. Stop if the clearance cannot be confirmed.

Start with a short section of the current enclosure, then traverse connected
passages at their centers, cover accessible branches and return to an earlier
junction/view. Choose new headings only while stopped. Do not manually lift/move
the robot or rearrange the scene to bypass a narrow passage during the map session.
Do not interpret the earlier 250 mm front / 450 mm rear clearance as applying
after each new movement. There is no prescribed blind route around the pictured
partitions. Save when the accessible area has been covered or before 15 minutes.
A subsequent recording may continue the same live map after review; no map reset
is needed merely because the finite recording ended.

## User-run transfer and build

Desktop:

```bash
scp /tmp/jetson-mapping-console-20261027-v2.tar.gz turtle@10.2.12.123:/home/turtle/
```

Jetson, new interactive terminal, existing wheel/mapper terminals left running:

```bash
mkdir ~/jetson-mapping-console-v2 &&
tar -xzf ~/jetson-mapping-console-20261027-v2.tar.gz -C ~/jetson-mapping-console-v2 &&
cd ~/jetson-mapping-console-v2 &&
sha256sum --check SOURCE.sha256 &&
source /opt/ros/lyrical/setup.bash &&
cmake -S mapping_console -B build &&
cmake --build build --target run_mapping_console -j2
```

No existing directory is overwritten. This builds only the separate console.
The installed slam_toolbox message type support is used for optional graph events.

## Save before, explore, save after — user runs

Keep the current map session and robot placement. In the same terminal:

```bash
source /opt/ros/lyrical/setup.bash
export ROS_DOMAIN_ID=182
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
area_dir=$(mktemp -d "$HOME/jetson-area-checkpoints.XXXXXX")
echo "$area_dir"

ros2 run nav2_map_server map_saver_cli -f "$area_dir/before" \
  --ros-args -p save_map_timeout:=10.0 -p map_subscribe_transient_local:=true &&
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '$area_dir/before_graph'}" &&
test -s "$area_dir/before_graph.posegraph" &&
test -s "$area_dir/before_graph.data" &&
echo "BEFORE CHECKPOINT SAVED"
```

Require map save success, serializer result=0 and the checkpoint message. These
save the occupancy view and graph/data without restarting the live mapper.
Saving is not a tested restoration or relocalization result.

```bash
~/jetson-mapping-console-v2/build/run_mapping_console --supervised
```

Starting this command only records and waits. At READY, choose one movement whose
clearance you have checked; observe its complete stop before choosing another.
Finish using `q` at READY. On failure retain output/data and do not retry blindly.
After the robot is stopped, save from the same terminal:

```bash
ros2 run nav2_map_server map_saver_cli \
  -f "${area_dir:?Missing checkpoint directory}/after" \
  --ros-args -p save_map_timeout:=10.0 -p map_subscribe_transient_local:=true &&
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '$area_dir/after_graph'}" &&
test -s "$area_dir/after_graph.posegraph" &&
test -s "$area_dir/after_graph.data" &&
echo "AFTER CHECKPOINT SAVED"
```

Share session/checkpoint directories, the console output, stops/interventions and
which passages/views were covered. Keep NJWQJ7 running. Assistant retrieval remains
read-only. Next review should use each segment interval and full recording, graph
changes/events, wall/corridor consistency and coverage; single-pulse/route review
modes are not general-session certification.

## Resolution comparison and live configuration

See [resolution comparison](../../docs/navigation/jetson_mapping_resolution_review.md).
The offline result uses all 226 caGdIe scans, frozen odometry-derived poses and one
final recorded map transform, with the same Karto thresholds in both grids.
It isolates cell size; it does not reconstruct the optimized live pose graph.
A 2.5 cm grid is feasible and gives finer boundaries, also exposing uneven/sparse
coverage. It does not prove better sensor or pose accuracy.

Keep NJWQJ7 at 5 cm for this first broad coverage pass. The installed-version source
loads resolution_ during configuration and uses that cached member for rendering;
a plain `ros2 param set resolution` is not a verified live rerasterization method.
Do not restart or reconfigure NJWQJ7 just to change it. The saved xFPjny graph/data
now render offline at both resolutions, with exact 5 cm pixel equality against
the saved maps. Validate live continuation/restore before changing the live profile.
The console already accepts either
resolution on a fresh console session. No AMCL/fixed-map mode is introduced.

## Local validation

C++17 warnings-as-errors build passed. Eighteen ROS fixture scenarios cover a
five-segment sequence (30 cm forward, reverse, left/right 30 degree turns and
translation after turns), 2.5 cm map acceptance, delayed response and injected
odom/scan/status/lifecycle/ownership/clock-schedule/progress/direction failures.
Each completed segment had five zeros and no automatic next command; post-motion
faults attempted five abort zeros; preflight competition issued no command.
The v2 accidental-input scenario checks a pasted motion/quit command during a
segment, input crossing the READY transition, recoverable invalid input and no
extra movement. Editor and real terminal-adapter pseudoterminal checks cover
split arrow sequences, overlong input, control keys and actual hangup. Recorder
scenarios verify complete command reception, refusal without a wheel subscriber
and refusal of unexpected consumers. Recorded commands were checked against
reopened bags. All 18 scenarios passed on domain
193 with localhost discovery and empty peers outside the network-restricted
sandbox; the sandbox run had DDS interface errors and failed, so is not counted.
The final terminal queue flush was rechecked using long-paste PTY input and the
accidental-input, recorder and operator-stop scenarios. Logs:
`/tmp/mapping-console-input-review/{build,test-final,test-input-final}.log`. G0I8Np validates v1
physical operation; **v2 is built/tested locally, not deployed or physically run**.

```bash
source /opt/ros/lyrical/setup.bash
cmake -S tools/lidar_diagnostics/mapping_console -B /tmp/lidar-mapping-console-build
cmake --build /tmp/lidar-mapping-console-build -j2
ROS_DOMAIN_ID=193 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_STATIC_PEERS= \
ROS_LOG_DIR=/tmp/lidar-mapping-console-build/ros-log \
  /tmp/lidar-mapping-console-build/test_mapping_console
```

## Independent recording and teammate delivery

The v2 bundle also contains `scripts/record_bag.sh`, `scripts/teleop.sh` and
`scripts/save_map.sh`, matching the teammate's recording/save intent while keeping
our current wheel/input/SLAM stack. From the bundle root, run the recorder in a
separate terminal, wait for subscriptions, then run `./scripts/teleop.sh --supervised`.
When stopped, save with `./scripts/save_map.sh` before ending the recorder with
Ctrl+C. It writes PGM/YAML and graph/data, verifies serializer result=0 and creates
checksums. Leave NJWQJ7 running. No original teammate `run_scan.sh` or second LiDAR
driver should be started in this session. Recordings persist across console exits.

See [teammate instructions](team_capture/README_KO.md) and
[orientation report](team_capture/ORIENTATION.md). G0I8Np already provides a useful
bag for offline consistency analysis; independent physical measurements are still
needed for absolute accuracy. The active yaw is pi, but the user confirms the
front-box check in base-frame RViz has not yet been performed. Do not report it as
physically verified or change the running TF. Dataset delivery is described in
[handoff preparation](../../docs/navigation/wheelbase_mapping_delivery.md).
