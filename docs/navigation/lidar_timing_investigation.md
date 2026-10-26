# LDS-03 acquisition timing investigation

Investigated 2026-10-25 against upstream source fingerprint in
`tools/lidar_diagnostics/source.sha256` and the existing diagnostic captures.
This is an engineering finding, not a timing correction or a session handoff.

## Findings

1. `lidar_data_processor.cpp`, `wait_package`, records `getTime()` when parsing the
   ring-start packet's type byte. This is host parser time after serial read, before
   that packet's full checksum is validated. It includes unknown sensor, UART, USB,
   buffering, and scheduling delay. No per-ray sensor acquisition timestamp is
   preserved by the inspected decoder.
2. The grab thread closes the previous scan when it encounters the next sync sample,
   copies `tim_scan_start`/`tim_scan_end` into the scan buffer's timing fields, and
   advances start to end. Thus their difference describes host-observed ring periods.
   It does not establish absolute first-ray acquisition time.
3. `timer.cpp` uses CLOCK_REALTIME only with HAS_CLOCK_GETTIME enabled. The inspected
   diagnostic and experimental compile definitions do not enable it. Their default
   implementation calls CLOCK_MONOTONIC, truncated to microseconds. A hardware-free
   C++ probe compiled with the SDK timer confirmed its value matches the local
   `std::chrono::steady_clock` epoch within the 1 microsecond truncation tolerance.
   This is local-build evidence; verify again on Jetson or after toolchain changes.
4. The publisher uses `node_->now()` at publication, ignoring `LaserScan::stamp`.
   ROS time can differ from the monotonic epoch and can follow simulation time.
   Directly assigning an internal timestamp to a ROS header is incorrect here.
5. `wait_scan_data` puts a queued-byte/delay estimate into the sync node's `stamp`.
   This is a duration estimate, not an absolute timestamp. It is not used for the
   published header. Its calculation uses `scan_time_increment = 1e9 / 5000` from
   `handling_info.hpp`, whereas the ROBOTIS manual specifies 4 kHz. Do not treat this
   dormant estimate as a validated latency correction.
6. The output grid uses increasing counterclockwise angles while raw decoded sample
   order is predominantly increasing clockwise angle. Bin wrapping, missing returns,
   and nearest-return collision selection also change which raw sample supplies each
   bin. A positive `scan_time / 400` would not describe those selected samples' times.
   Even a negative constant cannot exactly account for the seam and arbitrary source
   selection. Raw index is useful provenance, not a verified per-ray timestamp.

The [ROS LaserScan definition](https://raw.githubusercontent.com/ros2/common_interfaces/lyrical/sensor_msgs/msg/LaserScan.msg)
requires a first-ray acquisition timestamp and defines time_increment as time between
measurements. The current stationary experiment intentionally remains publication
stamped with time_increment=0. It is not ready for motion compensation/navigation.

## Capture evidence

Sources, all under `/home/pa4/ros2_lidar_diagnostic_ws/captures/`:

- `front-baseline-bUFvtA/samples.csv`
- `front-no-target-pBwRWO/samples.csv`
- `front-ccw30-N12VMG/samples.csv`

After verifying the local clock correspondence, subtracting internal end from the
steady-clock snapshot gives the following **host processing ages**, not optical or
USB latency. Each capture has 100 scans. The first recorded scan is reported separately
because recording begins after driver warmup; its larger age is consistent with an
older buffered scan but does not alone establish the cause.

| Capture | Median host ring period (ms) | First snapshot age (ms) | Remaining snapshot age min / median / max (ms) |
|---|---:|---:|---:|
| baseline | 99.7615 | 43.304344 | 0.026700 / 0.108852 / 0.229640 |
| no target | 99.5680 | 56.684876 | 0.008786 / 0.128811 / 0.228124 |
| counterclockwise | 99.7245 | 50.648107 | 0.046207 / 0.129466 / 0.219210 |

Median adjacent decoded-angle changes in reception order are +0.890625, +0.90625,
and +0.890625 degrees respectively. Small numbers of negative/zero differences exist;
the decoded angle includes distance-dependent correction, so angle differences alone
cannot validate uniform sampling times. Existing CSVs omit original packet bytes,
packet membership, byte arrival times, and ROS/steady clock pairs at a common event.
They cannot recover exact acquisition timestamps retrospectively.

## Next implementation increment

Prepare a separate timing diagnostic, without replacing the working installations:

1. Record bounded packet provenance: checksum validity, ring flag, sample count,
   packet sequence, and raw sample index. Commit ring timing only for validated packets.
2. Record monotonic timestamps around reads and decode completion, queued byte count,
   and paired monotonic/ROS timestamps at scan handoff and publication. Account for
   clock jumps and reject simulation-time use for this physical-hardware experiment.
3. Retain the winning raw index for every angular bin. This permits explicit analysis
   of time order and seam crossings rather than assigning times by bin index.
4. Measure host buffering separately from any estimated transport delay. Validate
   packet sampling assumptions before applying the nominal 4 kHz rate as a time model.
5. Choose the final output strategy based on that evidence: an acquisition-ordered
   scan with downstream compatibility checks, or timed points that are motion-corrected
   before producing a uniform scan. Any host-derived acquisition estimate must state
   its uncertainty; it must not be described as a hardware timestamp.

The [ROBOTIS specification](https://emanual.robotis.com/docs/en/platform/turtlebot3/appendix_lds_03/)
provides nominal sampling/frequency information, not a validated end-to-end timestamp
or transport-latency model. No timing constants or installed driver binaries were
changed during this investigation.

## Timing diagnostic implementation

Prepared a separately named `coin_d4_timing_driver` in `/home/pa4/ros2_lidar_timing_ws`.
See `tools/lidar_diagnostics/TIMING.md` for schema, limitations, reproducible build,
offline tests, and the exact user-run capture command. It records host read/packet
timing, checksum-gated ring boundaries, clock pairs, and selected-bin raw provenance.
It deliberately retains publication-time headers and time_increment=0. No claim of
acquisition-time correction is made. Existing installations remain unchanged.

## First live timing capture (2026-10-25)

User completed front capture and clean shutdown:
`/home/pa4/ros2_lidar_timing_ws/captures/front-timing-WtZfi2/timing.jsonl`.
SHA-256: `ef8c82881ba50e85c770ef2a237024ff3eb0de91be3c7be2f02c8b69b42bdb20`.
Source is preserved unchanged. Footer reports 100 published scans, complete=1,
overflow=0, clock_bad=0.

Read-only analysis parsed all JSON records and independently reconstructed the
nearest-return bin selection from decoded sample angle/range/intensity/exposure.
All 40,000 bin source references match that reconstruction. There are 40,144 decoded
sample records and 32,239 selected valid returns. Every sample's packet reference,
checksum flag, and within-packet sample index is consistent with its packet record;
every published scan has exactly its declared raw count and 400 unique output bins.

- 2,338 decoded packets, all checksums valid; 4,699 read calls, all reporting the
  requested byte count. This describes the recorded calls, not a proof of no UART loss.
- 135 handoffs include warmup; published IDs are 36–135 with no gaps or duplicates.
  Handoffs 1–35 occurred before the first captured publication. They should not be
  reported as post-warmup losses.
- Paired ROS/monotonic clock brackets: median width 2.210 microseconds, maximum
  15.150 microseconds. Midpoint offset estimates span 6.800 microseconds. No clock
  discontinuity was detected; this is not a sensor clock synchronization result.
- Host-observed ring periods: median 99.700959 ms, range 99.015166–104.110275 ms.
- First scan: ring-header time to publication-stamp sampling is 46.669696 ms,
  of which 46.655251 ms follows handoff. Warmup-buffer waiting accounts for the
  location of this delay in the pipeline; exact scheduling cause was not measured.
- Remaining 99 scans: ring-header time to publication-stamp sampling is median
  1.636356 ms, 95th percentile 1.963525 ms, maximum 7.214713 ms. The capture performs
  per-sample/bin serialization before publication, so these are instrumented host
  processing delays, not a production-driver benchmark or optical/transport latency.
- In increasing output-bin order, selected raw-index steps comprise 32,039 negative
  steps and 100 positive wrap jumps: exactly one large wrap jump in each scan.
  Median step is -1; negative steps range down to -16 and positive jumps reach 412.
  Missing/rejected/colliding samples explain why selected indices are not a uniform
  sequence. This confirms that a positive scan_time/400 increment cannot represent
  the output's source order, and a constant negative increment also misses the seam
  and variable source-index gaps.

Decision: do not subtract 1.64 ms from all headers or assign a guessed uniform
time_increment. Neither would recover optical acquisition time. The next software
increment should retain source-index time estimates and explicitly model scan wrap
before selecting an acquisition-ordered scan or timed-point output. Any nominal-4-kHz
estimate remains an assumption pending validation of sampling order and transport
delay. Hardware measurement accuracy and moving-platform compensation remain untested.

## Offline relative-time model implemented

Added `sample_time_model.hpp`, `test_sample_time_model.cpp`, and `replay_timing.cpp`
under `tools/lidar_diagnostics/`. Each selected bin retains its original raw index and
packet/sample provenance. Two hypotheses assign offsets from raw index zero: i/4000 s
and i*T/N, where T is the observed host ring period and N the decoded sample count.
Empty bins carry NaNs. No absolute timestamp is generated, and wrap jumps remain
explicit instead of moving samples into another revolution.

Replay of the 100 published scans in front-timing-WtZfi2 gives a median per-scan maximum
offset disagreement of 0.423146 ms, maximum 1.050688 ms. Inferred N/T ranges from
4000.244473 to 4041.839534 samples/s (median 4017.019139); this is not a calibrated
hardware sampling-rate measurement. Period statistics here exclude the warmup-only
handoffs included in the earlier 135-boundary statistics.

Synthetic tests, sanitizer replay, independent checks of all 40,000 derived rows,
incomplete-input rejection, and no-overwrite checks pass. Detailed reproduction and
assumptions are in `tools/lidar_diagnostics/TIMING.md`. The next design decision is how
to anchor relative sample times with measured uncertainty and carry them downstream;
neither model establishes the optical time of raw sample zero. No live timing behavior
or installed binaries were changed.

## Conditional host-clock anchor implemented

Added checked integer clock-bracket mapping in `host_time_anchor.hpp` and extended
`replay_timing.cpp`. Raw sample zero matches the opening validated ring packet in
all 100 published scans, and each closing packet matches its end boundary. The model
maps that host parser event into ROS-clock coordinates using the handoff clock pair,
assuming a constant offset, then adds nominal-rate or period-scaled relative offsets.
Handoff/publication offset brackets overlap in every scan. Handoff bracket width is
median 2.4165 microseconds, maximum 5.590 microseconds. The mapping looks back about
100 ms from handoff to the earlier ring-header event.

All derived rows explicitly mark acquisition_time_valid=0. Bracket widths quantify
clock-read bracketing only, not sensor/USB latency or total acquisition uncertainty.
Exact integer arithmetic preserves nanosecond epoch precision and outward rounding.
Synthetic/sanitizer tests, independent verification of 40,000 derived rows, malformed
anchor rejection, and no-overwrite tests pass. See `tools/lidar_diagnostics/TIMING.md`
for formulas, output columns, and reproduction. Existing live drivers remain unchanged.

The remaining optical-to-host delay is unobservable from this capture. Further
receive-time arithmetic cannot calibrate it; doing so requires protocol evidence or
an independently timed physical reference. Any practical estimated-time implementation
must state its assumptions and undergo motion validation before replacing the stationary
experiment. No calibrated acquisition-time correction has been claimed or deployed.
