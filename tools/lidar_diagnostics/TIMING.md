# Bounded timing diagnostic

Prepared 2026-10-25 in `/home/pa4/ros2_lidar_timing_ws`:

- Package: `coin_d4_timing_driver`
- Executable: `timing_coin_d4_node`
- Library: `libcoin_d4_timing_driver_lib.so`

This is a separate stationary diagnostic, not a corrected acquisition-time driver.
It uses the experimental 400-bin conversion, 0.05–12 m limits, original weak-return
filter, publication-time stamp, and time_increment=0. No normal, diagnostic, or
experimental installation is overwritten. The modified SDK layout is private to
this separately linked build; do not mix its headers with another driver's library.

## Recorded evidence

The output is newline-delimited JSON (one flat object per line, integer values):

| `kind` | Evidence |
|---|---|
| `metadata` | Schema version, requested scan count, byte cap |
| `read` | Local packet-attempt ID, stage (0 header / 1 payload), monotonic before/after read, requested bytes, upstream read result, queued bytes before/after |
| `packet` | Packet ID, checksum validity and values, sample count, ring-header parse time (0 for ordinary packet), validation time, decoded header angles in Q6 degrees |
| `handoff` | Locally generated scan ID, paired monotonic-before / ROS / monotonic-after clocks when handing a completed scan to the publisher |
| `boundary` | Scan ID, raw count, validated ring start/end host times, closing packet ID |
| `sample` | Scan ID, raw index, packet ID and packet sample index, checksum/sync flags, decoded angle in Q6 degrees, distance in mm, intensity and exposure |
| `bin` | Scan ID, output bin, selected raw index (-1 means empty) |
| `conversion` | Scan ID, monotonic conversion completion time, reverse setting |
| `publish` | Scan ID, paired clocks around the actual ROS header-stamp reading, clock-quality flag |
| `published` | Scan ID and monotonic time after the publish API returns; not subscriber reception time |
| `simulation_time_rejected` | Simulation time was enabled during capture; capture marked invalid |
| `end` | Published count, complete/overflow/clock_bad flags |

Packet IDs count local parse attempts, not hardware sequence numbers. Aborted attempts
can have read records without a completed packet record. Read timestamps bracket the
read operation after waitForData, not optical acquisition or individual byte arrival.
The read_result is the upstream return value; requested_bytes must not be interpreted
as a separately measured actual byte count. Queue queries occur outside that bracket.
All `mono_*`, ring, and validation times use CLOCK_MONOTONIC nanoseconds. ROS times
are recorded in their own domain, with brackets for estimating the offset uncertainty.
No optical/transport delay is subtracted.

Ring timing is committed only after a full packet passes its checksum, using the
preserved earlier ring-header parse time. Invalid packets remain observable. Sample
exposure is initialized to zero before decoding so invalid packets cannot contribute
an uninitialized diagnostic field. No new acquisition model is applied.

## Bounds and validity

`timing_jsonl` is required and must name a new file; creation uses O_EXCL and mode 0600.
`timing_scans` defaults to 100 and accepts 1–1000. Read/packet recording begins before
warmup; counting ends after the requested number of published scans, not hardware
revolutions. Scan handoff IDs let us detect skipped buffer handoffs or repeated reads,
but do not prove the absence of hardware/serial loss.

Records are buffered in memory with caps of 64 MiB serialized data and 250,000 records;
container/string overhead consumes additional bounded memory. When a cap is hit, later
records are dropped and the file is sealed at the next publication or normal shutdown
with complete=0 and overflow=1. Completion freezes recording, writes all records, and
flushes the file after publishing. The node continues scanning until Ctrl+C. Capture
overhead changes processing time; these measurements are not an uninstrumented latency
benchmark. A forced kill may leave an empty or incomplete file.

The build requires stationary_test=true, version=4, and use_sim_time=false at startup.
Enabling simulation time during capture marks it invalid at handoff/publication checks.
Clock-offset brackets separated by more than 5 ms flag clock_bad=1, subject to timestamp
ordering; this is a coarse discontinuity detector, not clock calibration. Offline
analysis must still inspect clock brackets, gradual drift, and event order. Broad or
overlapping brackets can conceal jumps. Complete=1 describes recorder completion only,
not guaranteed sensor quality, packet integrity, or absence of latency.

## Capture command (user executes)

Stop both other LiDAR drivers. Keep front sensor and surrounding objects stationary.
No particular panel angle is required. In a fresh terminal:

```bash
source /opt/ros/lyrical/setup.bash
source ~/ros2_lidar_timing_ws/install/setup.bash
mkdir -p ~/ros2_lidar_timing_ws/captures
capture_dir=$(mktemp -d "$HOME/ros2_lidar_timing_ws/captures/front-timing-XXXXXX")
echo "Capture directory: $capture_dir"

ros2 run coin_d4_timing_driver timing_coin_d4_node --ros-args \
  -r __ns:=/lidar_front_timing -r __node:=driver \
  -p stationary_test:=true -p use_sim_time:=false \
  -p port:=/dev/serial/by-path/pci-0000:80:14.0-usb-0:6.4:1.0-port0 \
  -p frame_id:=lidar_front_link -p topic_name:=scan \
  -p baudrate:=230400 -p version:=4 -p reverse:=false -p warmup_time:=5 \
  -p timing_scans:=100 -p timing_jsonl:="$capture_dir/timing.jsonl"
```

Wait for `TIMING CAPTURE SEALED: file flushed; inspect end.complete and stop with Ctrl+C`,
then Ctrl+C. Expected duration is warmup plus about ten seconds and final file writing.
In the same terminal:

```bash
tail -n 1 "$capture_dir/timing.jsonl"
```

Expected footer: kind=end, published_scans=100, complete=1, overflow=0, clock_bad=0.
Report the capture directory and footer; include any `TIMING WRITE FAILED` messages.
Do not start another process on the same serial device until this driver stops.

## Reproduction and checks

From the repository root, choose a NEW workspace:

```bash
bash tools/lidar_diagnostics/build_timing.sh \
  "$HOME/ros2_lidar_ws/src/coin_d4_driver" "$HOME/ros2_lidar_timing_ws_new"
```

The script verifies source.sha256, applies timing.patch with zero fuzz, copies the
conversion/recorder/test headers and source, snapshots the inputs, and builds against
the Lyrical underlay. It does not start hardware. The pseudo-terminal parser test
executable is built but not installed as a ROS executable.

Offline tests performed: recorder completion/partial capture, caps, concurrent writes,
exclusive file creation, and clock jumps under UndefinedBehaviorSanitizer; real SDK
parser fed synthetic packets through a fresh pseudo-terminal, checking checksum-gated
ring timing and packet/sample provenance including multi-sample packets. No physical
LiDAR was opened. This was the pre-capture verification; the completed front capture
and offline results are recorded below. Jetson build remains pending.

## Offline source-index time model

The first front hardware capture completed successfully; analysis is recorded in
`docs/navigation/lidar_timing_investigation.md`. `sample_time_model.hpp` and
`replay_timing.cpp` now model relative time for each selected raw sample without
changing any ROS driver. `test_sample_time_model.cpp` tests model boundaries and wrap.

For raw index i in a scan with N decoded samples and observed host ring period T:

- Nominal-rate hypothesis: relative time = i / 4000 seconds.
- Period-scaled hypothesis: relative time = i * T / N.

Both set decoded raw index zero to relative time zero. Neither assigns it an absolute
acquisition time. Both assume reception order represents sampling order without
unrepresented missing samples. The second additionally spreads host-period variation
uniformly over the samples. N intervals span the ring; dividing T by N-1 would incorrectly
place the last sample at the next ring boundary. Missing bins keep index=-1 and NaN
offsets. Collision winners inherit their selected source's offset.

Wrap is represented as a real jump in relative source time, not hidden by adding or
subtracting a ring period. Such unwrapping would assign some returns to a different
revolution. Offsets are attached to individual bins; a single LaserScan time_increment
is not generated. No sensor/USB latency estimate or ROS clock offset is applied.

Build requires C++17 and Boost.JSON headers/library (available on this desktop):

```bash
model_build=$(mktemp -d /tmp/lidar-time-model-XXXXXX)
g++ -std=c++17 -Wall -Wextra -Werror -pedantic \
  tools/lidar_diagnostics/test_sample_time_model.cpp -o "$model_build/test_model"
g++ -std=c++17 -Wall -Wextra -Werror -pedantic \
  tools/lidar_diagnostics/replay_timing.cpp -lboost_json -o "$model_build/replay_timing"
"$model_build/test_model"
"$model_build/replay_timing" \
  "$HOME/ros2_lidar_timing_ws/captures/front-timing-WtZfi2/timing.jsonl" \
  "$model_build/relative_times.csv"
```

The tool validates the completion footer, counts, sample/packet references, and bin
source bounds before creating a new output file exclusively. It refuses existing
outputs and incomplete/overflow/clock-invalid inputs. The CSV contains scan/bin IDs,
raw and packet provenance, and the two relative offsets in nanoseconds. These are
offline hypotheses, not calibrated timestamps. Partial output from a write failure
must be discarded. Input files are read-only.

First capture results (100 published scans, excluding warmup-only handoffs):

| Metric | Minimum | Median | Maximum |
|---|---:|---:|---:|
| Host ring period (ms) | 99.033516 | 99.684526 | 102.832267 |
| N / T (samples/s, inferred) | 4000.244473 | 4017.019139 | 4041.839534 |
| N × 250 µs minus host ring period (ms) | 0.006142 | 0.424205 | 1.053276 |
| Maximum selected-offset difference per scan (ms) | 0.006127 | 0.423146 | 1.050688 |

The two time models agree at raw index zero by construction, so their disagreement
grows with source index. These differences measure sensitivity to assumptions, not
accuracy or a measured hardware-rate correction. Do not tune the nominal rate to the
observed N/T merely to make the models agree.

Validation: synthetic model tests and capture replay pass with UndefinedBehaviorSanitizer
and compiler warnings treated as errors; all 40,000 CSV rows were independently checked
against source indices and formulas. Truncated and incomplete captures are rejected
before output creation, and an existing output remains unchanged after a refused run.
The model preserves 32,239 selected returns, 32,039 negative source-index transitions,
and 100 positive seam transitions. Existing drivers remain unchanged.

## Host-clock anchoring (offline only)

`host_time_anchor.hpp` extends replay with conditional ROS-clock envelopes around
the host ring-header event, then applies each relative-time hypothesis. It does not
estimate when the laser sampled the scene. The replay first verifies that raw sample
zero belongs to the validated packet whose ring-header time equals the scan start,
and that the closing packet matches the recorded end. These checks pass for all 100
published scans in the first front timing capture.

Let a ROS clock read R be bracketed by monotonic reads B and A (B <= A). Under a
constant clock-offset assumption, the offset lies in [R-A, R-B]. A host ring-header
event H consequently maps to [H+R-A, H+R-B]. Per-sample relative offsets are added to
this host reference. The handoff bracket is used; the publication bracket must overlap
it in offset space. Non-overlap rejects the run before creating an output file. This
is a strict hypothesis check, not proof that the offset was constant between reads.

All absolute arithmetic uses checked integers to retain nanosecond precision near
Unix-epoch magnitudes. Period-scaled offsets use exact integer fractions rounded
outward; floating-point relative offsets are not added directly to epoch timestamps.

The output CSV now additionally contains:

- host_nominal_lower_ns / host_nominal_upper_ns
- host_scaled_lower_ns / host_scaled_upper_ns
- acquisition_time_valid, always **0**

Host envelopes are blank for empty bins. The original relative-offset columns remain.
These envelopes cover clock-read bracketing only. They exclude unknown sensor/serial/
USB delay, sample-order/model error, drift, and unobserved clock discontinuities. The
start event is mapped backward from a handoff about 100 ms later; constant offset over
that interval is an assumption, not a measurement. Do not use these columns as ROS
LaserScan acquisition stamps or feed them to motion compensation as calibrated times.

Replay results: handoff clock-bracket widths span 1.546–5.590 microseconds (median
2.4165 microseconds), with overlapping handoff/publication offset brackets in all 100
scans. Mapping lookback spans 99.052424–102.848219 ms. This microsecond-scale bracket
does not bound optical acquisition uncertainty. The latter remains unquantified.

Additional standalone test:

```bash
g++ -std=c++17 -Wall -Wextra -Werror -pedantic \
  tools/lidar_diagnostics/test_host_time_anchor.cpp -o "$model_build/test_anchor"
"$model_build/test_anchor"
```

Rebuild replay_timing using the preceding instructions to emit the new columns.
Anchor tests and replay pass with UndefinedBehaviorSanitizer. An independent exact-
integer check verified all 40,000 output rows. Tests also reject missing handoffs,
mismatched opening-packet anchors, non-overlapping clock brackets, truncated input,
invalid arithmetic, and existing outputs. No installed driver or capture file changed.

Practical limit: further manipulation of this receive-time capture cannot identify
the unobserved optical-to-host delay. Calibrated acquisition timing needs additional
protocol evidence or an independently timed physical reference. A future estimated-
time mode could make explicit assumptions, but must not claim the above brackets as
its total error bound. The stationary experiment remains the validated operating mode.


## Checkpoint review (2026-10-26)

The diagnostic and timing builds were reproduced from checksum-verified upstream
source in new /tmp/lidar-tooling-review workspaces. Calibration/timing investigation
was not resumed and no hardware was started. Use check_offline.sh for repeatable
C++17 warning-as-error and UndefinedBehaviorSanitizer checks:

```bash
bash tools/lidar_diagnostics/check_offline.sh \
  "$HOME/ros2_lidar_ws/src/coin_d4_driver" \
  "$HOME/ros2_lidar_timing_ws" \
  "$HOME/ros2_lidar_timing_ws/captures/front-timing-WtZfi2/timing.jsonl"
```

Only the first argument is required. The second runs the built synthetic
pseudo-terminal parser test (never a physical serial device). The optional third
replays an existing complete capture and checks no-overwrite, truncated-input and
negative-timestamp rejection. Output goes to a new printed /tmp directory. Tests
write new files only; input captures and working driver installations are unchanged.
Boost.JSON development files and a C++17 compiler are required; the parser test
requires the separately built timing workspace. Shell scripts are invoked with bash.

Review fixed acceptance of negative ring-start timestamps before end-start arithmetic;
malformed INT64_MIN input now rejects normally instead of causing signed overflow.
The diagnostic recorder is scan-count bounded (1–1000 scans); the timing recorder
additionally has explicit byte/record caps. Neither tool calibrates acquisition time.
