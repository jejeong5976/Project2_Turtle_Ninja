# Maze mapping laboratory

Offline C++ benchmark using MuJoCo 3D geometry, the confirmed October 26 URDF in
C1_P0, the existing LiDAR deskew/projection/filter code, and real continuous
`slam_toolbox`. No serial API, wheel controller, firmware operation or hardware
fallback exists in this runner. The original workbench and baseline configs are
unchanged. All Git and hardware commands remain the user's responsibility.

## Evidence and assumptions

* Assignment evidence: `datasets/mission/20261028/mission_v5.pdf`, page 2, and the
  supplied arena photo. Specified outer dimensions, box dimensions and station
  centers are retained. The mission start is (0.734, 0.950), facing -X.
* Provisional geometry: modular wall endpoints digitized from the drawing;
  6 mm wall thickness, 300 mm wall height, 60 mm post diameter; central platform
  center (2.202, 3.500). These are not physical survey measurements. Boundary
  post counts follow the drawing, not a measured post spacing. All geometry is
  described in `arena.json`. The traffic signal/barcode fixtures have no supplied
  physical envelopes and are not collision objects in this model.
* Confirmed source model:
  `/home/pa4/Downloads/Learning/turtle_ninja_model_lab/releases/nt_urdf_release_20261026/urdf/robot.urdf`.
  C1_P0 is taken from that release's `config/initial_pose.json`. Its test pucks
  remain in the holders as a conservative model envelope, not actual inventory.
* The CAD base origin is 72 mm above its footprint. The working ROS description
  uses 33 mm. These are separate model conventions. Optical rays use the retained
  working transforms in `base_footprint`: front (+0.110,+0.005,0.0545,pi), rear
  (-0.110,+0.005,0.0545,0). No CAD LiDAR TF is published. CAD housings are excluded
  from self-occlusion because their optical conventions differ; other URDF visual
  triangles participate. This occlusion model is a sensitivity assumption.
* The route envelope is the larger of the full C1_P0 radial bound and the retained
  body-mask diagonal, plus 40 mm margin. It is conservative in this model; real
  steering sweep, cables, deformation, carried load and traction are unverified.
* Motion is kinematic with acceleration ramps, not a motor/contact dynamics test.
  Synthetic odometry optionally has 1% longitudinal scale error, -0.5% lateral
  error and 0.002 rad/m heading drift; range noise is 5 mm standard deviation.
  These are proposed sensitivity values, not measured calibration corrections.
* Both scanners use 400 selected rays at 10 Hz, nominal 250 us spacing, zero
  timing correction. Their simulated reference times are synchronized. Real
  inter-sensor phase, material effects, multipath, vibration and wheel slip are
  not validated. An ideal-data control sets the three noise/drift terms to zero.

The 54.5 mm rays intersect the 70 mm boxes. They pass below pucks at 70-130 mm.
Separate 90 mm ray checks confirm that the model is genuinely three-dimensional.

## Build and one-terminal execution

```bash
cd /home/pa4/git/Project2_Turtle_Ninja
tools/maze_mapping_lab/build.sh

# Create a fresh scene from the confirmed release; the source URDF is unchanged.
tools/maze_mapping_lab/run.sh scene build \
  --urdf /home/pa4/Downloads/Learning/turtle_ninja_model_lab/releases/nt_urdf_release_20261026/urdf/robot.urdf \
  --arena tools/maze_mapping_lab/arena.json \
  --output output/maze_mapping_lab/my_scene --render 1

# Render the retained scene and full robot.
tools/maze_mapping_lab/run.sh scene render \
  --scene output/maze_mapping_lab/scene

# Replay a retained dataset through the actual mapper. OUTPUT must be new.
tools/maze_mapping_lab/run.sh constrained six_core_6g \
  --dataset output/maze_mapping_lab/datasets_v2/coverage040 \
  --output output/maze_mapping_lab/trials/my_trial \
  --baseline config/slam_front_experiment.yaml --rate 10

# Prime the map faster, then measure its last 120 seconds at real time.
tools/maze_mapping_lab/run.sh constrained one_core_2g \
  --dataset output/maze_mapping_lab/datasets_v2/coverage040 \
  --output output/maze_mapping_lab/trials/my_resource_trial \
  --baseline config/slam_front_experiment.yaml --rate 10 --realtime-tail 120
```

ROS is isolated to domain 221 and localhost. Run trials sequentially. The launcher
owns and cleans up only its own SLAM child process groups. Results include the
map, serialized pose graph, scan/fusion counters, sampled estimated poses, CPU and
memory measurements, and the exact configuration. Generated data and assets are
under `output/maze_mapping_lab`; they are not firmware or deployment artifacts.

Generate a different dataset without editing source:

```bash
tools/maze_mapping_lab/run.sh scene generate \
  --scene output/maze_mapping_lab/scene \
  --output output/maze_mapping_lab/datasets_v2/my_route \
  --route coverage_closed --heading fixed --speed 0.025 --turn-rate 0.1 \
  --range-noise 0.005 --odom-scale 0.01 --yaw-drift 0.002 --seed 3 --margin 0.04
```

Available routes: `perimeter_closed`, `coverage_closed`, `coverage_open`,
`north_closed`. `fixed` heading uses forward/backward/lateral translation;
`tangent` turns to face each leg. North start is a mapping comparison, not a
replacement for the assignment's initial observation pose. Routes are planned
with known simulated geometry; this is not unknown-maze exploration or Nav2.

Replay controls: `--scan-bins` (400-6400 output bins; does not add native returns),
`--map-resolution` (0.01-0.10 m), `--filter-neighbors` (0-20), `--delay-ms`,
`--jitter-ms`, `--odom-delay-ms`, `--drop-every`, `--rate` (0.1-30),
`--limit-seconds`, `--realtime-tail`, `--domain` (0-232), `--lockstep` (0 or 1),
`--loop-search-distance` (0.1-5 m), and `--loop-closing` (0 or 1). The loop options
write only the trial's config copy. Communication delays defer delivery while
preserving acquisition timestamps; they are not calibration offsets.
There is a provisional 30 ms base delivery delay, rounded up to the next 20 ms
replay tick. `--delay-ms` and jitter add to that transport delay. Neither is a
measured USB/Jetson delay, and neither changes a beam's acquisition timestamp.

Front scans remain the sole SLAM input. Both scanners are deskewed and merged
with their distinct ray origins retained. Spatial filtering applies only to
diagnostic merged clouds; it is not an obstacle/noise classifier and does not
silently change the established mapping input.

## Resource interpretation

Profiles constrain an isolated desktop process group, with six allowed desktop
CPU IDs (0-5), no swap, and an aggregate CPU quota:

| Profile | CPU quota | RAM cap |
|---|---:|---:|
| six_core_6g | 600% | 6 GiB |
| two_core_4g | 200% | 4 GiB |
| one_core_2g | 100% | 2 GiB |
| quarter_core_1g | 25% | 1 GiB |

Caps can also be set without editing source:

```bash
tools/maze_mapping_lab/run.sh constrained custom \
  --cpu-quota-percent 100 --memory-max 2G --cpu-list 0-5 \
  --dataset output/maze_mapping_lab/datasets_v2/holonomic040 \
  --output output/maze_mapping_lab/resources/my_custom_trial \
  --baseline config/slam_front_experiment.yaml --rate 10 --realtime-tail 60
```

`--cpu-quota-percent`, `--memory-max` and `--cpu-list` can override a named profile
too. The actual cgroup limits and affinity are recorded with every trial. Memory
units are `M` or `G`; swap remains disabled for the process group.
The default affinity still permits CPU IDs 0-5 even with a 100% quota. That quota
can produce burst/throttle behavior; it is not identical to pinning one physical
core. Use `--cpu-list 0` for that distinct experiment. Cgroup throttled time may
sum time across runnable threads and exceed elapsed wall time.

These are **desktop stress envelopes**, not calibrated Jetson performance modes.
Six desktop cores are not six ARM cores, and GHz ratios cannot establish an
equivalent throughput. The 6 GiB cap provisionally leaves 2 GiB of an 8 GiB target
for the OS/other tasks. Jetson CPU and GPU share memory; there is no independent
8 GiB RAM plus 8 GiB VRAM allowance. The current SLAM/deskew/fusion pipeline uses
CPU computation. Its GPU compute allocation is zero. This does not benchmark
future camera, CUDA or TensorRT workloads, shared-memory bandwidth or GPU clocks.
The 3D world and rendering are outside the constrained target workload.
The replay process includes prerecorded input buffers and their memory overhead.
So are physical USB/serial drivers, OpenCR, rosbag recording, RViz and future
camera/perception processing. A desktop quota cannot reproduce ARM instructions,
Jetson memory bandwidth, cache behavior, power/thermal throttling or real USB
contention. The 19 V supply is the user's physical configuration, not an emulated
performance mode. No Jetson was contacted or assumed powered for these trials.

There are two replay methods. `--lockstep 1` subclasses the installed asynchronous
mapper only to acknowledge completion of its original laser callback. The clock
waits for every acknowledgement, including scans skipped by the mapper's motion
thresholds. This controls accelerated geometry comparisons; it is not a resource
test. The installed SLAM algorithms and the working baseline are unchanged.
`controlled-suite` runs the route/speed/start comparisons, an ideal control,
a second noise seed, a same-input repeat, and two experimental loop settings.
The same-input repeat checks the limits of reproducibility.
Use `controlled-suite output/maze_mapping_lab/my_new_comparison` for a new full
run. Existing output directories are refused; no earlier evidence is overwritten.

`--lockstep 0` (default) runs the normal installed asynchronous launch with timed
delivery. At rates above 1 this is an accelerated screening test, not a real-time
capacity claim or a scheduling-independent route ranking. Earlier `trials/`
screening results are superseded by `controlled/` for route comparison.
Real-time tails reset the wall-clock schedule after map
priming and report their own scheduling lag. A 100 ms lag limit is a proposed
benchmark threshold, not a proven hardware safety margin. Full raw delivery-age
limits remain 250 ms. Large injected delay produces explicit rejected scans and
an unusable-map result; prerecorded truth continues and is not a motor-stop test.
Tail CPU and wall measurements exclude priming, the final drain and graph saving.
The tail scheduling check requires both lag below 100 ms and simulated delivery
age plus wall lag below 250 ms. Accepted-pose latency separately measures from
the scheduled synthetic scan reference to receipt of the SLAM pose. It excludes
intentionally skipped scans and is not occupancy-map publication latency or
complete message-drop accounting. Physical sensor-to-map latency remains unverified.

Map metrics use only the known initial pose to align map and arena. No ICP or
best-fit postprocessing hides drift. Surface recall and occupied-cell precision
use a tolerance of 1.5 occupancy cells (75 mm at the baseline 50 mm resolution).
This is a mapping comparison tolerance, not the assignment's 10 mm puck-placement
requirement. Coverage includes all non-obstacle floor cells, including occluded
regions. Report all three metrics rather than interpreting coverage alone as
accuracy. Pose RMSE is sampled when SLAM publishes a pose, not continuous-time
trajectory ground truth. Process-tree RSS double-counts some shared mappings;
the cgroup peak is also saved.

## Validation and results

```bash
tools/maze_mapping_lab/run.sh scene validate \
  --scene output/maze_mapping_lab/scene \
  --dataset output/maze_mapping_lab/datasets_v2/coverage040
tools/maze_mapping_lab/run.sh scene summarize \
  --trials output/maze_mapping_lab/controlled \
  --output output/maze_mapping_lab/controlled_comparison
```

Validation checks independent known ray/height intersections, full-arm import,
the conservative robot envelope, every 20 ms trajectory sample for clearance and
speed, and scan cadence. `experiments.json` records the proposed comparison matrix.
See the resulting `comparison.csv` and per-trial JSON for actual measured results.
The physical trial must use measured maze clearances and the existing bounded
one-terminal workbench. These simulated paths are not a hardware motion script.

### Completed geometry comparison, October 29

Fourteen controlled runs completed. The same-input repeat produced byte-identical
`map.pgm` and `slam_poses.csv`; this is a verified result for this pair, not a
cross-platform determinism guarantee. A supplementary low-speed fixed-heading run
first timed out waiting for a ROS callback acknowledgement; that failed attempt
is retained in `controlled/holonomic025`, and its successful isolated retry is
`controlled/holonomic025_retry`. It is not counted as a successful result.

The 0.04 m/s fixed-heading interior route is the current physical-comparison
candidate. It completed in 16.3 simulated minutes, covered 93.4-93.6% of the arena's
free floor, and achieved 82.3-87.5% occupied-cell precision across two synthetic
noise seeds. Return errors were 3.6-6.0 cm. The corresponding turning route took
26.4 minutes and had 67.8-83.0% precision. The north start and open finish supplied
no clear advantage in this matrix. These are single trials per setting except
the documented seed pair/repeat, not statistical certification.

At 0.025 m/s, fixed heading completed in 25.5 minutes with 89.6% coverage and
80.5% precision; going slower did not ensure a more accurate map. The 0.08 m/s
turning run rejected 896,269 rays under the unchanged projection guards, whereas
the 0.025 and 0.04 m/s cases rejected none. More CPU alone cannot remove that
geometric limitation. Reducing loop-search distance to 0.75 m or disabling loop
closure produced the same baseline map in the controlled turning comparison;
those tests do not justify changing the working mapper settings.

The ideal-data control reached 95.6% precision; realistic material, slip and
timing behavior are still unmodeled. The simulation is a repeatable comparison
baseline, not evidence that a physically accurate full-maze map is achieved.

For physical commissioning, use the existing workbench at 0.025 m/s and 0.10 rad/s.
First verify the six bounded directions and stopping response. Then compare a
small marked closed rectangle while holding heading against the same rectangle
with turns, stopping at independently measured checkpoints. Inspect both scans,
merged/filtered diagnostics, wall spacing, doubled walls and return error; save
the bag and map. Only then extend the fixed-heading candidate to surveyed maze
sections. Cables, actual robot clearance and thin-obstacle retention need physical
checks before interpreting this synthetic route as usable travel space.

### Completed resource checks, October 29

Each normal asynchronous run replayed the whole fixed-heading 0.04 m/s dataset,
priming at up to 10x and measuring the final 60 seconds at 1x. This verifies a
real-time window with an established graph, not an entire mission at real time.
Accelerated priming can change which scans the asynchronous mapper processes;
these maps must not replace the controlled geometry comparisons above.

| Desktop stress profile | Peak cgroup memory | Mean CPU cores during 1x window | Accepted-pose latency p95 / max |
|---|---:|---:|---:|
| 600% CPU quota / 6 GiB | 375 MiB | 0.607 | 140 / 160 ms |
| 100% CPU quota / 2 GiB | 324 MiB | 0.566 | 320 / 520 ms |
| 200% CPU quota / 4 GiB, combined communication/filter stress | 348 MiB | 0.584 | 320 / 340 ms |

The combined case added 100 +/- 40 ms transport delay, delayed odometry by 40 ms,
dropped every twentieth scan pair (489 total), and enabled one-neighbor filtering.
All three measured tails met the producer scheduling/delivery-age checks and had
98 accepted pose samples. The 100% quota nevertheless delayed pose results beyond
the raw-input age limit: freshness of incoming scans is not proof of timely pose
estimation. The combined case removed 277,257 of 5,507,376 merged points; that is
not evidence they were sensor noise, because the synthetic model did not inject
spurious-object returns. Filtering still does not alter the front-only SLAM input.

The negative case added 300 ms transport delay and used custom CLI CPU/RAM caps.
All 299 examined scan pairs were rejected as stale; no usable map was produced,
and the runner returned its expected negative-result code 3. This verifies the
offline input guard, not physical stopping behavior. Results are under
`output/maze_mapping_lab/resources`; the failed supplementary geometry attempt
and earlier screening results remain available for audit.
