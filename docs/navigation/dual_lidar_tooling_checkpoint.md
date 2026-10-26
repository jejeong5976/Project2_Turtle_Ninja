# Diagnostic and timing tooling checkpoint

Reviewed 2026-10-26 after robot-model commit d9b0374. This preserves existing research
and makes its checks repeatable. It does not resume calibration, change the running
stationary driver, establish acquisition timestamps, or install udev rules.

## Reproduction and review results

Both builders verify the tracked source.sha256 manifest before copying upstream
source, refuse an existing destination, apply their patch with zero fuzz, and build
against only /opt/ros/lyrical. Package, library and executable names are isolated
from the working driver. New builds passed at:

- /tmp/lidar-tooling-review/diagnostic: coin_d4_diagnostic_driver
- /tmp/lidar-tooling-review/timing: coin_d4_timing_driver

Both patch files now use GNU diff-style suppressed blank context lines so Git
whitespace checks pass. Applying them with zero fuzz produced byte-identical source
to the successfully built versions; no patch behavior changed.

The normal diagnostic records the original converter's behavior. The timing build
uses the experimental conversion and preserves raw sample/packet/bin provenance.
The original source is unchanged. Neither build script starts a sensor process.

```bash
bash tools/lidar_diagnostics/build_isolated.sh \
  "$HOME/ros2_lidar_ws/src/coin_d4_driver" "$HOME/ros2_lidar_diagnostic_review_ws"
bash tools/lidar_diagnostics/build_timing.sh \
  "$HOME/ros2_lidar_ws/src/coin_d4_driver" "$HOME/ros2_lidar_timing_review_ws"
bash tools/lidar_diagnostics/check_offline.sh \
  "$HOME/ros2_lidar_ws/src/coin_d4_driver" "$HOME/ros2_lidar_timing_review_ws" \
  "$HOME/ros2_lidar_timing_ws/captures/front-timing-WtZfi2/timing.jsonl"
```

Choose new build destinations. The existing capture argument is optional; captures
are machine-local evidence, not repository assets. Upstream retrieval and its pinned
revision are documented in dual_lidar_desk_checkpoint.md. Exact source hashes remain
the build gate.

Four standalone C++ test suites passed with warnings as errors and UndefinedBehaviorSanitizer:
original-conversion recorder, timing recorder, relative-time model and host-clock
anchor. The separately built SDK parser test passed using synthetic packets through
a new pseudo-terminal. The parser test is not sanitizer-instrumented by its normal
build. No physical serial device was accessed.

Replay of the preserved front timing capture passed: 100 scans, 32,239 selected
returns, 32,039 negative source-index steps, 100 positive seam transitions; statistics
match the existing TIMING.md results. Existing-output refusal preserves output bytes;
truncated captures and negative ring-start timestamps reject before output creation.
The negative-timestamp check fixes a signed-overflow path for malformed input.
All derived rows retain acquisition_time_valid=0. This is not motion-ready timing.

The test runner prints a fresh /tmp artifacts directory and keeps its outputs for
inspection. Expected rejection cases are intentional; only its final PASS establishes
success. The first review version passed a directory where the parser test needed a
new filename; that runner error was corrected and the complete suite passed afterward.

The desk capture QoS file was reviewed: both scans request Best Effort/Volatile;
/tf_static requests Reliable/Transient Local so an already-published static transform
can be recorded. No new bag capture was performed.

Udev review is separate: tools/udev/README.md documents disabled rules, duplicate
serials and deployment checks. Syntax verification passed, but identity matching and
Jetson installation remain untested.

## User-controlled commit groups

Run each review/commit separately. Never blanket-stage the repository.

Original-conversion diagnostic capture:

```bash
git add -- tools/lidar_diagnostics/build_isolated.sh \
  tools/lidar_diagnostics/diagnostic_recorder.hpp \
  tools/lidar_diagnostics/instrumentation.patch \
  tools/lidar_diagnostics/test_recorder.cpp
git diff --cached --check
git diff --cached
git commit -m "feat: preserve isolated lidar diagnostic capture tooling"
```

Timing provenance, offline models, repeatable checks and evidence (after the above):

```bash
git add -- tools/lidar_diagnostics/README.md \
  tools/lidar_diagnostics/TIMING.md \
  tools/lidar_diagnostics/build_timing.sh \
  tools/lidar_diagnostics/check_offline.sh \
  tools/lidar_diagnostics/desk_capture_qos.yaml \
  tools/lidar_diagnostics/host_time_anchor.hpp \
  tools/lidar_diagnostics/replay_timing.cpp \
  tools/lidar_diagnostics/sample_time_model.hpp \
  tools/lidar_diagnostics/test_host_time_anchor.cpp \
  tools/lidar_diagnostics/test_sample_time_model.cpp \
  tools/lidar_diagnostics/test_timing_parser.cpp \
  tools/lidar_diagnostics/test_timing_recorder.cpp \
  tools/lidar_diagnostics/timing.patch \
  tools/lidar_diagnostics/timing_recorder.hpp \
  docs/navigation/lidar_timing_investigation.md \
  docs/navigation/dual_lidar_tooling_checkpoint.md
git diff --cached --check
git diff --cached
git commit -m "feat: preserve lidar timing provenance and offline checks"
```

Udev templates: use the separate two-file checkpoint in tools/udev/README.md.
Historical dual_lidar_setup.md, dual_lidar_desk_alignment.md and dual_lidar_handoff.md
remain outside these groups; they contain superseded workflow instructions and need
reconciliation before a documentation checkpoint. Preserve all attachments.
