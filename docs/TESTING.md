# Testing

## Running the suite

```bash
python3 -m pytest tests/ -q -p no:launch_testing
```

**`-p no:launch_testing` is required.** ROS 2 ships a `launch_testing` pytest plugin that
auto-registers itself on any machine with a sourced ROS install. It is incompatible with the
pytest version this repo's test suite is written against: without the flag, pytest fails during
collection and no test runs at all, ROS-dependent or not. `-p no:launch_testing` disables that
plugin. Nothing in `tests/` uses `launch_testing` -- every test here is headless (no rclpy, no
live graph) by design (see "How the suite stays headless" below) -- so disabling it costs nothing.

As of 2026-09-05 the suite is **478 passed, 29 skipped, 2 xfailed, 0 failed**. A failure means
either a real regression or a test that has gone stale against a legitimate code change; see
"Triage discipline" below before touching either the test or the code.

## How the suite stays headless

Almost everything under `tests/` runs with no ROS, no Isaac, and no live graph: production
modules structure their ROS imports as function-local (not top-of-file), so importing the module
to test its pure logic never touches rclpy. `tests/test_ros_world_escalation.py`,
`tests/test_pipeline_chain.py` and `tests/test_run_campaign.py` monkeypatch the ONE seam each
module reaches ROS through (`ros_world._ros`, `run_campaign.subprocess.run`, etc.) and run the
*real* decision logic against a fake hardware edge. `tests/test_session_e2e.py` is the deliberate
exception: it executes the actual bash scripts against a real (test) DDS graph.

## What each test file guards

| File | Guards |
|---|---|
| `test_arm_workspace.py` | The arm's reachable envelope used for approach/press planning. |
| `test_ask_blockage.py` | The VLM blockage parser: a half-parsed reply must not become a confident answer. |
| `test_blockage_fusion.py` | `safety/blockage_fusion.py`'s camera+lidar OR fusion, against real hardware captures. |
| `test_campaign_safety.py` | `run_campaign.py` refuses to start an unsafe campaign (deadman gate down, arm-override, etc). |
| `test_escalation.py` | Escalating to the pipeline only on an unambiguous, budgeted answer. |
| `test_filter_scan.py` | Scan filtering helpers. |
| `test_floor_plan.py` | Multi-floor ride planning and the floor-swap handover gate (headless). |
| `test_handeye.py` | Hand-eye calibration solver, synthetic data (CALIBRATION.md item 8). |
| `test_handeye_rw.py` | The OpenCV convention `bringup/handeye_solve_rw.py` relies on. |
| `test_lidar_lift.py` | Recovering a wall's depth along a lidar-lifted pixel ray. |
| `test_local_avoid.py` | Reactive avoidance: steer around what the scan sees, or refuse. |
| `test_look_policy.py` | The reasoner may suggest where to look; it may not move the robot anywhere it likes. |
| `test_map_frame.py` | Map-frame vs odom-frame waypoint provenance checks. |
| `test_map_persistence.py` | The map, and the config that describes it, survive between sessions. |
| `test_mux_watch.py` | The autonomous runner notices when the safety mux discards its commands. |
| `test_nav2_scan_source.py` | Nav2 consumes the masked scan; the params files agree with the live chain. |
| `test_nav_backend.py` | RosWorld's nav backend selection, and `nav2_goto.py`'s refusal rules. |
| `test_pipeline_chain.py` | THE SEAM TEST: one authoritative walk of the whole chain (map -> localization -> waypoint -> Nav2 result -> fusion -> mux -> arm), asserting every handoff between files. |
| `test_press_veto.py` | The arm must never press a fire alarm. |
| `test_reach_envelope.py` | The arm never commanded past its envelope; the base knows how close to get. |
| `test_ros_world_escalation.py` | The blocked-path ladder in `RosWorld`: fuse -> back up -> look -> press -> resume, run for real against a fake robot. |
| `test_route_plan.py` | Route waypoint-name validity. |
| `test_run_campaign.py` | `run_campaign.py`'s trial loop offline: anchor-relative drift, stop conditions, resume, capture naming, return-to-start. |
| `test_run_dataset.py` | Dataset-driven trial running. |
| `test_safety_arbiter.py` | The base-motion safety arbiter, headless. |
| `test_scan_anchor.py` | Relocalizing the odom frame from a lidar scan, or refusing to. |
| `test_scan_mask.py` | `bringup/scan_relay.py`'s self-occlusion mask: what it must remove, and what it must never. |
| `test_scan_temporal_filter.py` | The near-field flicker suppressor between `/scan` and `/scan_nav`. |
| `test_session_e2e.py` | The actual bash scripts, executed for real against a real DDS graph. |
| `test_stack_wiring.py` | Cross-file wiring read as one system: topic names, QoS, map defaults, config<->code agreement -- the class of bug where every file is internally correct and the JOINT between two files is wrong. |
| `test_stale_cmd.py` | `bringup/stale_cmd_test.py`'s CAN decoding. |
| `test_teleop_guard.py` | Keyboard teleop guards -- the regression suite for a real runaway. |
| `test_waypoint_drive.py` | The odom waypoint driver and its corridor veto. |
| `test_waypoint_frame.py` | A waypoint from a dead odom session must never be driven to. |
| `test_world_reset.py` | `RosWorld.reset()` clears every per-trial field, not just the obvious ones. |

## Triage discipline

When a test fails, read the test AND the code it points at before touching either. Classify it:

- **STALE** -- the code is right and the test encodes an assumption that a legitimate change
  invalidated (e.g. the project growing from one map to two). Generalize the assertion to the
  current intended invariant; do not just delete the check.
- **REAL BUG** -- the test is right and the code is wrong. Do not fix code you are not allowed to
  touch (`bringup/`, `config/`, `safety/`, `nav2_bringup/`, `maps/`, `calib/` are live runtime
  files during hardware trials) -- mark it `xfail(strict=True, reason=...)` with the exact
  file:line and mechanism, so the test still runs, still documents the defect, and turns into a
  loud `XPASS` the moment someone actually fixes it.
- **OBSOLETE** -- the thing being guarded no longer exists. Delete the test and say why.

Never blanket-`xfail` to force green. A green suite that guards nothing is worse than a red one.

## 2026-09-05 triage: 39 failures, 0 remaining unexplained

Starting state: `39 failed, 460 passed, 10 skipped`. Every failure was read against the current
code before being classified. Final state: `478 passed, 29 skipped, 2 xfailed, 0 failed`.

**37 were STALE** (test encoded an outdated assumption; assertion generalized, invariant kept):

- `test_stack_wiring.py` (5): the two-map generalization (`elevator` + `floor2`, see
  `docs/MULTIFLOOR.md`) for the default-map-name and loaded-map-name checks; `scan_relay.py`'s
  `IN_TOPIC`/`OUT_TOPIC` becoming env-overridable (`os.environ.get(...)` defaults, no longer a bare
  literal the old regex could match); a third scan-processing stage,
  `safety/scan_temporal_filter.py` (`/scan` -> `/scan_nav`), inserted between the self-occlusion
  mask and Nav2's costmaps; `bringup/stow_arm.py`'s `STOW_DEG` becoming a value read live from
  `config/safety.yaml` (`_stow_from_config()`) instead of a duplicated literal -- the exact class
  of drift the test exists to catch, fixed by removing the duplication instead of by a test. (The
  file's 6th failure, `test_slam_config_quotes_the_range_min_that_actually_runs`, is a REAL BUG,
  not stale -- see below.)
- `test_pipeline_chain.py` (2): the same default-map-name and third-scan-stage generalizations
  applied to this file's independent copies of the same checks (`test_handoff_1_...`,
  `test_handoff_5_...`).
- `test_campaign_safety.py` (3) + `test_run_campaign.py` (9): both files' `_install_fakes()` faked
  `waypoints.load()` for the `start` waypoint but did not fake the FRAME `run_campaign.py`'s
  return-to-start leg reads for it -- that leg re-reads the waypoints file directly (by design, to
  choose between the Nav2 and odom return drivers) rather than going through the faked seam, so it
  fell through to the operator's live `maps/waypoints.yaml`, which (correctly) has no generic
  `start` entry, and every campaign died after trial 1. Fixed by pointing that direct read at an
  isolated `UTP_WAYPOINTS` file in `tmp_path`, matching how the rest of each harness isolates from
  the live repo.
- **18 tests across `test_scan_mask.py` (2), `test_blockage_fusion.py` (1),
  `test_pipeline_chain.py` (4) and `test_ros_world_escalation.py` (11)**:
  all depend on `captures/trial_ours_001/scan.json`, a gitignored, local-machine hardware capture
  of the 2026-09-01 near-miss (0.72 m from closed glass doors, camera said "an open walkway with
  pillars"). That file has been overwritten with an unrelated, later, open-corridor capture
  reusing the same trial name; the original is not recoverable from git. The masking and fusion
  CODE was verified correct against the wrong data (all synthetic-scan unit tests in the same
  files still pass). Fixed by adding a signature check (`_require_glass_door_signature` /
  `_load_trial_scan`) that verifies the file still holds a near-obstacle before asserting anything
  about it, and skips loudly, by name, with a pointer to this section, when it does not.
  **Action needed: re-capture `captures/trial_ours_001/scan.json` facing closed glass doors** (see
  `docs/RECORDING.md`) to restore these regression checks; until then they are silently not
  exercising the property they exist for, on this machine.

**2 are REAL BUGS**, marked `xfail(strict=True, ...)` with the mechanism in the reason string,
left for someone with access to `bringup/`/`config/` to fix:

1. **`bringup/ros_world.py:600-615` (`_drive_leg_single`), the DEFAULT nav path** (`UTP_NAV_STAGED`
   defaults to `"0"`, `ros_world.py:595`). Its `if "blocked" in out:` branch
   (`ros_world.py:612-614`) sets `NavOutcome(status="blocked")` the instant `nav2_goto.py`'s
   stdout contains the word "blocked" -- i.e. on ANY Nav2 ABORTED result, including TF, planner,
   controller and costmap faults that are not physical obstructions -- and calls
   `_perceive_blockage()` only to attach a blockage object afterward, never to gate the decision.
   The staged path, `_drive_leg_staged` (`ros_world.py:676-686`, opt-in via `UTP_NAV_STAGED=1`),
   does this correctly: it perceives, calls `_leg_should_stop()`, and only reports `blocked` if
   perception confirms it -- otherwise `unreachable`. **Live-pipeline impact: on the default code
   path, a software-only Nav2 abort (no obstacle present) is reported as a physical blockage,
   which starts the reason -> ground -> press chain and commands the arm at nothing.** This is
   the exact failure `navigate_to_goal`'s own docstring warns about ("Manufacturing a physical
   blockage from that control-plane status can start reason -> ground -> press on a software
   bug"), currently unguarded on the path actually in use. Test:
   `test_pipeline_chain.py::test_handoff_4_succeeded_reaches_and_unconfirmed_abort_does_not_block`.
   Fix: give `_drive_leg_single`'s `blocked` branch the same `_leg_should_stop` confirmation
   `_drive_leg_staged` already has.
2. **`config/slam_os0.yaml:106`** (and `config/ouster.yaml:95,114`), documentation drift, not a
   behavior bug. Both files' prose cites `range_min:=0.30` as the value that keeps the chassis out
   of its own map. `bringup/session.sh` actually passes `range_min:=0.45`
   (`config/ouster.yaml:116`'s own `range_min_m: 0.45` agrees with the runtime, just not with the
   comments two lines above it). Per commit `24c639a`, 0.30 was an intermediate experiment;
   0.45 is what shipped. Runtime behavior is correct and unaffected -- what's wrong is the prose a
   human reads when deciding whether it's safe to change the self-occlusion mask. Test:
   `test_stack_wiring.py::test_slam_config_quotes_the_range_min_that_actually_runs`. Fix: update
   both citations from `0.30` to `0.45`.

**0 were OBSOLETE** -- nothing this run's failures pointed at had actually been removed by design.
