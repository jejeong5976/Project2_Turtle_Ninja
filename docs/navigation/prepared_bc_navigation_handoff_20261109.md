# Prepared B/C navigation — handoff for a new ChatGPT session

Prepared **November 9, 2026, Asia/Seoul (KST)**, after physical session PB01 closed.
This is the current standalone continuation brief. Older preparation documents retain historical proposals and approvals; they do not authorize new actions.

## Resume here

**Continue offline first.** Prepared B/C is implemented, and the native compute-time problem is fixed. The latest held physical session passed fresh localization and dual-lidar timing/transform checks. **No movement was approved or commanded.** The actual planner selected a different route from the earlier successful replay, and the fresh route/short-segment reviews did not qualify movement.

The immediate task is to explain the fresh planner's reverse detour, prepare the intended forward L2 approach using current recorded geometry, and establish positive body and stopping clearance before proposing another physical experiment. Preserve the **0.08 m/s physical cruise cap**, with lower local speed where required. Evaluate C after the shared passage behavior works for B.

### Current physical state and authorization

| Item | Last verified state |
|---|---|
| Motor power | **OFF**, explicitly confirmed by the operator after software shutdown |
| Placement | Stationary at the agreed start, facing arena **+Y**; no driving occurred |
| Robot processes | Closed; no matching robot processes or serial owners at **11:02:23 KST** |
| Recording | Closed SQLite bag; integrity `ok`, **245,079 messages**, approximately 3.9 GB uncompressed |
| Viewer | Browser tab and owned SSH tunnel closed; remote viewer port no longer listening |
| Evidence | Copied locally; archive and all **27 per-file hashes** verified at **11:06:10 KST** |
| Jetson shutdown | User was told normal OS shutdown may proceed, followed by power disconnection after completion; **actual shutdown is not reported** |
| Approvals | Prior exact deployments completed; the one PB01 held-start approval is consumed. **No new deployment, hardware-start or movement approval exists** |

Do not contact or restart Jetson merely to resume this offline work. Its current availability/power is unknown after shutdown release. Do not reuse historical startup approvals or a native-review marker as permission to start hardware.

## User's objective and fixed decisions

- Make B/C negotiate tight turns proactively: choose a checked curve or turning position before clearance rejection forces recovery beside a wall.
- Retain the passage entry, crossing tangent and exit. Small route/localization changes should adjust tracking without repeatedly initiating alignment.
- Preserve B's smooth bounded diagonal corrections. Meaningful obstacle/route changes must still invalidate the maneuver and stop or replan.
- Temporary holds retain valid geometric intent only; stale observations, an active hold or an unsafe path never retain permission to move.
- C's final-heading preview is subordinate to passage clearance; compare its final-arrival benefit later.
- Use route geometry, **not hard-coded L2 coordinates in controller logic**. Explicit experiment approach/exit checkpoints are a separate route-design choice.
- **No Git commands at all**, including status/diff/log. Do not stage, commit, switch branches or use workflows that implicitly perform Git operations. Git state/branch were not inspected.
- Ask separately before **deployment**, **hardware startup**, and **movement**. Finish the concrete reviewable preparation before asking.
- Do not weaken hold, scan freshness, transform, collision, unknown-space or physical-limit guards. Retain the fixed ahead-left object and all unexplained scan returns.
- The user confirmed that the ahead-left returns are a stationary object to pass. Do not repeatedly ask what it is or ask again for established start-area wall distances.
- Project implementation is C++; read the project's C++ skill before code changes. Python was used for ROS orchestration and offline scientific analysis, not as a replacement controller.

## Workspace and source identity

All paths below are machine-local. A plain ChatGPT session may not have filesystem access: upload this document and the relevant evidence files when necessary. Do not claim to have inspected a path that is unavailable.

| Purpose | Path |
|---|---|
| Current workspace and evidence | `/home/pa4/git/Project2_Turtle_Ninja` |
| **Canonical B/C implementation** | `/home/pa4/git/Project2_Turtle_Ninja-body-route/ros2_ws/src/body_route_controller` |
| Initial prepared-passage implementation/comparison | `/home/pa4/git/Project2_Turtle_Ninja/output/prepared_passage_20261109_v1` |
| **Latest source snapshot and timing revision** | `/home/pa4/git/Project2_Turtle_Ninja/output/prepared_passage_timing_20261109_v2` |
| Frozen physical v2 packet and validation | `/home/pa4/git/Project2_Turtle_Ninja/output/prepared_b_physical_20261109_v2` |
| **Latest physical session evidence** | `/home/pa4/git/Project2_Turtle_Ninja/output/prepared_b_physical_20261109_v2/physical/PB01-20261109` |

The current workspace's `ros2_ws/src/body_route_controller` is an older A-only runtime. **Do not overwrite it or mistake it for the canonical B/C source.** The initial implementation was synchronized to the canonical directory, followed by six v2 timing changes with hash preconditions. During handoff preparation, these six canonical files again matched the v2 snapshot: `PASSAGE.md`, `include/body_route_controller/passage.hpp`, `src/passage.cpp`, `src/controller.cpp`, `test/test_adapter.cpp`, `test/test_passage.cpp`.

Check current files and applicable `AGENTS.md`/skills before editing; later changes by the user or another session are possible. The canonical directory is outside the previous session's writable roots, so tool permission may be needed to edit it. Do not interpret that as deployment or hardware authorization.

## What is implemented

The opt-in shared `PreparedPassage` behavior:

1. Detects significant upcoming bends and checks a continuous curve or an early stop/turn/cross maneuver using padded footprint, stopping envelopes and explicit uncertainty assumptions.
2. Retains a stable crossing direction while allowing bounded lateral correction; small path jitter does not select a new alignment target.
3. Invalidates on meaningful route, goal, obstacle, unknown-cell or map-geometry change.
4. Schedules a local speed cap at or below cruise. The historical successful passage selected **0.063125 m/s** within the 0.08 m/s cap.
5. Preserves geometry across temporary motion-state resets, with fresh validation before resumption. Cancel/reset/lifecycle cleanup forget the maneuver.
6. Defers C's final-heading preview until passage-compatible clearance allows it.

Normal B tracking outside a selected maneuver remains substantially unchanged; early-turn handling does not qualify the entire route automatically.

The v2 compute revision reuses body/turn checks only within an already checked locked-map speed search, translates equal-speed stopping envelopes, and skips expensive polygon work only when an entire cell bounding box is proven free of lethal/unknown cells. Exact blocking-cell snapshots include map geometry. A prepared-controller computation exceeding **50 ms returns zero**, resets motion state, retains recent issued commands for stopping checks and requires fresh validation before resuming.

Settings in the physical prepared profile:

| Parameter | Value |
|---|---:|
| Translation / body yaw caps | 0.08 m/s / 0.10 rad/s |
| Linear / angular acceleration settings | 0.05 m/s² / 0.20 rad/s² |
| Diagonal correction cap | 0.015 m/s |
| Travel cone / lookahead | 45° / 0.24 m |
| Steering profile velocity / acceleration | Existing hardware values 20 / 5 |
| Prepared allowance assumptions | 0.05 m position, 5° yaw, 0.5 s response delay |
| Profile identity | `fnv1a64-fd9b4cf58123240a` |

`prepare_passages` defaults false; the physical v2 profile explicitly enables it. Assumptions are included in profile generation, identity, trial logs and live parameter verification. **Response and uncertainty values remain hypotheses, not measured physical bounds.**

Read implementation details in [PASSAGE.md](/home/pa4/git/Project2_Turtle_Ninja/output/prepared_passage_timing_20261109_v2/source/body_route_controller/PASSAGE.md), [changes.patch](/home/pa4/git/Project2_Turtle_Ninja/output/prepared_passage_timing_20261109_v2/changes.patch), and [canonical-sync.json](/home/pa4/git/Project2_Turtle_Ninja/output/prepared_passage_timing_20261109_v2/canonical-sync.json).

## Verification already completed

### Historical same-route comparison

See [comparison report](/home/pa4/git/Project2_Turtle_Ninja/output/prepared_passage_20261109_v1/REPORT.md) and [COMPARISON.json](/home/pa4/git/Project2_Turtle_Ninja/output/prepared_passage_20261109_v1/COMPARISON.json).

| Response model at 20 Hz | Baseline B | Prepared B |
|---|---|---|
| Gain 0.25, 50 ms delay | Complete, 76.85 s | Complete, 80.80 s |
| Gain 0.10, 50 ms delay | Complete, 86.90 s | Complete, 81.25 s |
| Gain 0.10, 200 ms delay | Progress timeout, 57.35 s | Complete, 84.00 s |
| Gain 0.10, 400 ms delay | Progress timeout, 59.40 s | Complete, 94.10 s |

Prepared B improved completion from 2/4 to 4/4 and reduced full translation stops from four to one in the completed baseline comparisons. It was not uniformly faster or pause-free: 10–17 additional uncertainty-guard zero-command ticks remained. Minimum residual stopping clearance was only **about 4.85–4.99 mm** after the illustrative 5 cm/5° allowance; residual body clearance was about 12.5 mm.

This is first-order body-velocity replay, **not Isaac Sim**, wheel/steering physics or measured braking. It does not model live scan/costmap changes, replanning, scheduling delays or full hardware dynamics. Never turn its archived route or pose into live initialization automatically.

### Native v2 validation

See [native review](/home/pa4/git/Project2_Turtle_Ninja/output/prepared_b_physical_20261109_v2/NATIVE_REVIEW.md), [test/timing summary](/home/pa4/git/Project2_Turtle_Ninja/output/prepared_b_physical_20261109_v2/native-evidence/SUMMARY.json) and [trace comparison](/home/pa4/git/Project2_Turtle_Ninja/output/prepared_b_physical_20261109_v2/native-evidence/TRACE_COMPARISON.json).

- Native ARM64 build exited 0. **224 XML tests passed:** 94 controller/core/profile, **4 adapter**, 88 swerve, 14 navigation, 5 position/rate, 7 scan diagnostics, 5 localization, 2 full-map files, 5 launch tests. Two lidar regressions, viewer freshness, profile reproduction and mocked launch construction also passed.
- A synthetic 70 ms map-lock delay produced a zero command with deadline status, followed by fresh validated resumption. Holds, stale feedback, changed unknown cells and rolling maps remain guarded.
- Four native replay maxima: **22.307, 21.771, 21.694 and 21.905 ms**, versus the 50 ms cycle. Native v1 took about 223–224 ms and must not be used for movement.
- All four v2 native trajectory/event/stopping-envelope files match native v1 byte-for-byte. Host comparison differs only by floating-point rounding, at most about 1.01e-13.
- Native replay timing is not evidence of controller performance under actual moving-robot load.

**Historical wording correction:** older v2 source reports say “local only,” “build running,” or “3 adapter tests.” Deployment and native validation subsequently completed, with **4 adapter tests / 224 total**. Older native-review startup proposals are consumed historical proposals.

## Latest real-world evidence: PB01

Read [PB01 report](/home/pa4/git/Project2_Turtle_Ninja/output/prepared_b_physical_20261109_v2/physical/PB01-20261109/REPORT.md) and [archive verification](/home/pa4/git/Project2_Turtle_Ninja/output/prepared_b_physical_20261109_v2/physical/PB01-20261109/ARCHIVE_VERIFIED.json).

The single held session ran approximately **10:53–11:02 KST**, with wheel hardware, both lidars, recording and a visible live viewer. Hold remained engaged throughout.

- Two fresh dual-scan localization fits agreed **7.07 mm / 0.40°**. Withheld p90 residuals were **1.89 / 2.08 cm**, about 99.28% within 5 cm. Independent front/rear fits differed 1.25/1.00 cm and 1.7/1.5°. Reviewed wall overlays supported initialization.
- Localization and navigation lifecycle activation succeeded. The viewer showed LIVE/HELD. A 20-second stationary window had zero translation and about 0.029° heading drift. AMCL sigmas around 8.4/9.0 cm and 4.5° are recorded covariance, not a surveyed error bound.
- Original timing/TF probe passed: 5 s warmup + 20 s measurement; **200 scans per lidar, 55,298 front rays and 56,690 rear rays**, all negative time increments, zero timing/TF failures. Oldest-ray age stayed below 0.141 s. All 33 observed-free grids were nonempty. Scan/TF limits stayed 0.3/0.05 s.

### What blocked movement

1. **Planner discovery:** installed `route_trial` planning preview failed its 2-second server-discovery wait. Direct inspection showed `/planner_server` active. One planning-only ROS client with 5-second discovery warmup succeeded. That diagnostic created no hold publisher, navigation client or follow-path client. Discovery timing is a plausible explanation; its root cause is not established.
2. **Wrong route for the intended experiment:** the fresh global route has **278 poses / 5.486 m**, initially heads backward, then travels below the nearby wall and around to the right of the destination. It does not follow the previously evaluated forward L2 approach. Prepared B stays at the start with `no_checked_turn_or_crossing`, then modeled progress timeout, in all four response cases on the fresh strict map/scan snapshot.
3. **Short alternatives also unqualified:** a 20 cm forward candidate completes nominally but has **−55 to −56 mm** residual stopping clearance with the original 5 cm/5° allowance. A 10 cm right + 25 cm forward candidate has about **−80 mm** at 10 cm/5° allowance, still about **−30 mm** at 5 cm. These were local-only simulations, never movement requests or robot actions.

Nominal body/envelope overlap counts are zero for those short alternatives. Negative uncertainty-adjusted clearance is a qualification failure, **not an observed physical collision**. Preserve the failed cases and investigate geometry/assumptions rather than dropping returns or reducing allowances to force a pass.

Important PB01 files, relative to the physical-session directory listed above:

| Files | Purpose |
|---|---|
| `fit1.json`, `fit2.json`, `fresh-fit-review.png` | Fresh-fit evidence; now historical |
| `ray-timing-detail.txt`, `stationary-review.json` | Held readiness measurements |
| `route-review-frame.json`, `strict-unknown.bin`, `strict-unknown-metadata.json` | Current scan endpoints plus static occupied/unknown map for offline analysis |
| `l2-planning-review.json`, `.path`, `live-route-review.png` | Fresh planner detour |
| `live-route-replay/` | Four rejected full-route hypotheses |
| `response-candidate.path`, `response-replay/`, `RESPONSE_REVIEW.json` | Rejected forward response segment |
| `right-forward-candidate.path`, `right-forward-replay/`, `RIGHT_FORWARD_REVIEW.json` | Rejected diagonal response segment |
| `prepare_geometry.py`, `analyze_response.py`, `analyze_right_forward.py` | Offline scientific analysis; no robot actions |
| `evidence/runs/PB01-20261109/bag/` | Closed ROS bag for offline replay/inspection |
| `evidence/PB01-launch.log`, `evidence/ros-log/` | Startup, discovery, hardware and closure logs |

The local strict snapshot preserves unknown cells and current scan returns; it is not the live costmap or a complete future free-space observation. `geometry.bin` is an optimistic diagnostic variant and must not be substituted for strict qualification.

## Hardware identity for a later separately approved session

Last owned target: `turtle@10.2.10.231`, ARM64 Jetson, ROS 2 Lyrical under `/opt/ros/lyrical`. Availability is not currently assumed.

| Device | Verified physical path |
|---|---|
| Wheel OpenCR | `/dev/serial/by-path/platform-3610000.usb-usb-0:1.3:1.0` |
| Front lidar | `/dev/serial/by-path/platform-3610000.usb-usb-0:2.1:1.0-port0` |
| Rear lidar | `/dev/serial/by-path/platform-3610000.usb-usb-0:2.2:1.0-port0` |
| Arm controller | USB1.4, disconnected |

The user once called an unplugged board “wheel” and then corrected it to **arm**. USB1.3 remains the wheel board. Earlier lidar-swap speculation was resolved by unplug proof; **front is USB2.1, rear USB2.2**.

Retained physical lidar extrinsics: front x=+0.1145 m, y=0, yaw=π; rear x=−0.1145 m, y=0, yaw=0. Do not substitute this checkout's different newer plate model. The corrected COIN-D4 timing driver was deployed and passed PB01 live timing; do not restore the earlier raw timing behavior.

Runtime is isolated to loopback ROS domain **227**; synthetic adapter tests used **231**. Viewer is loopback port **8767**, reached through an owned SSH tunnel. Never manufacture viewer heartbeat with HTTP automation; actual visible browser freshness is required for movement.

## Deployment and evidence identities

| Artifact | SHA256 |
|---|---|
| Approved v2 source packet: 8,205,944 bytes, 651 hashed payload files | `050b34d9af51c0088808ea5f4304cb5e94eda6e00b94bf7e5812341c2db3a738` |
| Native v2 validation archive | `a95c3259042e945b424911fb95125200cc907184346ab3362265ba0aee71d691` |
| PB01 physical archive: 115,299,948 bytes, 27 file hashes | `9a380e93c717b7accbbeaf9b38a2992d103bd279c873eeb9a1e766201455d933` |

Last verified deployment: `/home/turtle/nt_prepared_b_20261109_v2`. It copied verified unchanged v1 dependencies/CLIs into its own install, rebuilt changed core/plugin/tests, and reran validation. Preserve v1 and the earlier baseline as historical evidence. Do not rerun deployment scripts into existing destinations; new revisions require new concrete review and separate approval.

The native-review marker in v2 records successful native validation only. It does not authorize a new startup. Some state files retain `deployment_approved: true` as a historical fact about that exact completed packet; it is not open-ended permission.

## Ordered next work and acceptance criteria

1. Read this handoff, the PB01 report, current source and applicable instructions. Preserve existing changes. Confirm local evidence access; **no Git commands and no Jetson dependency for initial analysis**.
2. Reproduce the fresh planner detour from archived inputs. Inspect `src/maze_planner.cpp`, planner costs/unknown-space policy, route goals and raw bag costmap/scan data. Distinguish what the global planner knew from what the strict local review adds. Identify whether route selection, waypoint specification, observed obstacles or another cause explains the detour; do not assume which one is responsible.
3. Prepare the intended forward approach using explicit experiment checkpoints or a justified general planning change. Keep controller logic independent of L2 coordinates. Review the fixed object's clearance before the turn and the entire body's exit from the opening.
4. Evaluate candidate routes with the **same limits and several response delays**, reporting completion, full translation/correction stops, total time, minimum body and stopping clearance, and uncertainty assumptions. Require positive adjusted clearance throughout the whole maneuver, not just centered in the opening. Include current rejected cases as regressions. A merely positive few-millimeter modeled margin with unmeasured assumptions is not robust physical qualification.
5. Investigate planner discovery/startup readiness independently. Preserve finite failure behavior and motion gates; do not hide failed attempts. Add meaningful regression checks if changing code.
6. Measure or justify response and localization assumptions before relying on them for narrow passage clearance. PB01 supplies stationary scans/feedback only; **it contains no moving response measurement**. If a safe bounded response experiment is needed, make its route, limits, stop boundary and acceptance criteria concrete first.
7. Only then prepare any new deployment request. After separately approved deployment/native tests, request one bounded held startup; obtain fresh localization and unchanged timing/observation readiness. Ask separately for the exact movement scope. No automatic transition, retry or hold release.
8. After a future trial, record measured stop, close software/bag, request physical motor OFF, verify evidence and process/device closure, then release Jetson for shutdown. Software shutdown alone retains motor torque.

Existing local executables for offline work are under
`/home/pa4/git/Project2_Turtle_Ninja/output/prepared_passage_timing_20261109_v2/build/`:
`test_body_route`, `test_body_route_adapter`, `passage_replay`. Rebuild if source changes; do not present stale binaries as validation of new edits. Adapter tests require isolated ROS setup; core/replay tools use files. Do not blindly rerun scripts that write into existing evidence directories.

Example **offline-only** reproduction of the rejected PB01 quick-response route, writing a new temporary result:

```bash
workspace=/home/pa4/git/Project2_Turtle_Ninja
pb01="$workspace/output/prepared_b_physical_20261109_v2/physical/PB01-20261109"
review_dir=$(mktemp -d /tmp/prepared-b-review.XXXXXX)
"$workspace/output/prepared_passage_timing_20261109_v2/build/passage_replay" \
  "$pb01/l2-planning-review.path" 1.59535839 "$review_dir/quick-prepared" \
  "$workspace/output/prepared_b_physical_20261109_v2/packet/profiles/prepared-continuous-reactive/nav2.yaml" \
  "$pb01/strict-unknown-metadata.json" "$pb01/strict-unknown.bin" \
  .25 prepared 1 .05 .08726646259971647
```

The yaw above is an archived offline input, never a live pose seed. Other response cases use gain/delay-ticks `.10/1`, `.10/4`, `.10/8` at 20 Hz. No ROS initialization, hardware access or command publication occurs in this replay executable.

## Separate unfinished Isaac Sim work

The earlier **0.25 m/s** request was simulation-only and does not change the physical cap. Its B experiment failed before the turn on stale scan timing; a subsequent timing/stop-recording repair remained incomplete at the last record. The user reported no Claude changes before the physical work. This handoff did not revalidate or finish that simulation repair.

If explicitly returning to Isaac, read [Isaac handoff](/home/pa4/git/Project2_Turtle_Ninja/docs/navigation/isaac_bc_sim_handoff_20261106.md) and [Claude handoff](/home/pa4/git/Project2_Turtle_Ninja/docs/navigation/claude_code_handoff_20261109.md). Treat their physical-state sections as historical; this document and PB01 evidence supersede them. Do not confuse the prepared-passage body-response replays with successful Isaac runs.

## Prompt to paste into the next session

> Read the attached prepared_bc_navigation_handoff_20261109.md, or read `/home/pa4/git/Project2_Turtle_Ninja/docs/navigation/prepared_bc_navigation_handoff_20261109.md` if you have this workspace. Continue offline from PB01: investigate why the fresh planner selected the reverse detour, then prepare and validate the intended forward L2 approach with stable passage tracking and positive body/stopping clearance. Keep the physical cap at 0.08 m/s. No Git commands. Ask separately before deployment, hardware startup and movement. Motor power was confirmed OFF and Jetson was released for shutdown; do not assume it is online or reuse any earlier startup approval. Read current files and preserve existing work. If local files are unavailable, identify the specific evidence files you need rather than claiming access.
