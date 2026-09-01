# CONSOLIDATION — a dependency-ordered sequence to run AFTER the demo

Written 2026-09-01, with a demo pending the same night. Nothing in here except **step 0** has been
executed. Every step below is independently safe, independently revertable, and states its own
verification. Do them in order where the order is marked load-bearing; skip freely where it is not.

**Ground rule for every step:** the acceptance test is `bash bringup/session.sh nav` reaching
`Nav2 ok (navigate_to_pose available)` and `bash bringup/session.sh campaign 2 --dry-run`
completing. A unit suite that goes green while that path is broken has told you nothing — that is
the whole subject of step 7.

**Measured starting state (2026-09-01):** 59 scripts in `bringup/` (44 `.py`, 15 `.sh`),
18 modules in `safety/`, 31 test files with **873** assert statements, `bringup/ros_world.py` at
**1209** lines, and both a `sim/` and an `archive/` tree inside the hardware repo.

---

## Step 0 — ONE Nav2 config — **DONE 2026-09-01**

Three near-identical ~400-line param files collapsed to one.

- `nav2_bringup/nav2_params.yaml` → `archive/` (the sim mirror; nothing launched it, and as
  `ranger_nav.launch.py`'s `params_file` default it was actively dangerous — see below).
- `nav2_bringup/nav2_params_os0.yaml` → `archive/` (rolling-window variant; referenced only by one
  row of a table in `docs/NAV2.md`).
- `nav2_bringup/nav2_params_os0_map.yaml` is now the only Nav2 config in the repo.
- `ranger_nav.launch.py`'s `default_params` now points at the surviving file. Updated with it:
  `nav2_bringup/README.md`, `docs/NAV2.md`, `archive/README.md`.

**The evidence this was worth doing**, both from the same day:

1. Both costmaps were subscribed to `/scan_filtered`, which on the OS0 chain is the **raw**
   projection containing the robot's own arm and mast. The one-line fix had to be typed **three
   times**, once per copy, and the identical 22-line explanatory comment now appears verbatim
   three times in `git log`.
2. The copies had already silently diverged. Commit `cd3dcc1`, the same day, fixed
   `transform_tolerance` 0.2 → 1.0 in four places, `controller_frequency` 20.0 → 10.0, and the
   MPPI horizon — in `nav2_params_os0_map.yaml` **only**. Neither copy got any of it. So: the
   scan topic was propagated (by hand, three times); nothing else was. A copy that is only
   sometimes updated is worse than no copy, because it still reads as authoritative.
3. `nav2_params.yaml`, reachable as the launch default, still carried `sensor_frame: lidar_link`
   — there is no `base_link → lidar_link` TF on this robot; `docs/NAV2.md` gotcha 4 calls that
   "the single most expensive bug this stack has had" — and `motion_model: Omni`, which the
   Ranger firmware cannot execute. A bare `ros2 launch ranger_nav.launch.py` came up blind.

**Full semantic diff** of the three, by `yaml.safe_load` + flattened key comparison (213–215 leaf
keys each), 18 differing keys:

| key | `nav2_params.yaml` | `nav2_params_os0.yaml` | `nav2_params_os0_map.yaml` (LIVE) |
|---|---|---|---|
| `global_costmap…obstacle_layer.scan.sensor_frame` | `lidar_link` ⚠ | `base_link` | `base_link` |
| `local_costmap…obstacle_layer.scan.sensor_frame` | `lidar_link` ⚠ | `base_link` | `base_link` |
| `controller_server…FollowPath.motion_model` | `Omni` ⚠ | `DiffDrive` | `DiffDrive` |
| `global_costmap…plugins` | static+obstacle+inflation | obstacle+inflation | static+obstacle+inflation |
| `global_costmap…static_layer.plugin` | `StaticLayer` | *absent* | `StaticLayer` |
| `global_costmap…static_layer.map_subscribe_transient_local` | `true` | *absent* | `true` |
| `global_costmap…rolling_window` | *absent* | `true` | *absent* |
| `global_costmap…width` / `.height` | *absent* | `24` / `24` | *absent* |
| `global_costmap…resolution` | *absent* | `0.05` | *absent* |
| `global_costmap…transform_tolerance` | `0.2` ⚠ | `0.2` ⚠ | **`1.0`** |
| `local_costmap…transform_tolerance` | `0.2` ⚠ | `0.2` ⚠ | **`1.0`** |
| `controller_server…FollowPath.transform_tolerance` | `0.2` ⚠ | `0.2` ⚠ | **`1.0`** |
| `behavior_server…transform_tolerance` | `0.2` ⚠ | `0.2` ⚠ | **`1.0`** |
| `controller_server…controller_frequency` | `20.0` ⚠ | `20.0` ⚠ | **`10.0`** |
| `controller_server…FollowPath.model_dt` | `0.05` | `0.05` | `0.1` |
| `controller_server…FollowPath.time_steps` | `56` | `56` | `40` |
| `controller_server…FollowPath.batch_size` | `2000` | `2000` | `1000` |

⚠ = a value known to be wrong on this robot, still present in the copy at the moment it was
archived. Everything not listed was byte-equal after parsing. Note the **scan topic is not in this
table**: all three had been hand-fixed, which is precisely the labour this step removes.

**Rolling-window variant, if you ever want it back:** it is four keys, not a file. Drop
`static_layer` from `global_costmap.plugins` and add `rolling_window: true`, `width: 24`,
`height: 24`, `resolution: 0.05`. Pass them as `--ros-args -p` overrides. It was not turned into
a small override file because there was no caller to point at one, and an override file with no
caller is the same duplication with fewer lines.

**Proof the live path is unaffected** (read, not run — the robot is on charge):
`bringup/session.sh:237` `sed`s `$ROOT/nav2_bringup/nav2_params_os0_map.yaml` into
`/tmp/utp_nav2_params_runtime.yaml`, rewriting the two absolute behaviour-tree paths;
`bringup/session.sh:276` launches `ranger_nav.launch.py` with `params_file:="$RUNTIME_PARAMS"`.
Neither archived file appears anywhere on that path. `ranger_nav.launch.py` consults
`default_params` only when `params_file` is not given, and `session.sh` always gives it. No test
resolves either archived filename as a path; `tests/test_nav2_scan_source.py` and
`tests/test_stack_wiring.py` both reach the params by `glob("nav2_params*.yaml")`, which still
matches the one survivor and still satisfies the non-empty guard at `test_stack_wiring.py:152`.

**One stale reference left, in a file this pass was forbidden to edit:**
`tests/test_stack_wiring.py:52` — the comment *"The other two are reachable: nav2_params.yaml is
ranger_nav.launch.py's own default"* is now false. Comment only; no assertion depends on it.

---

## Step 1 — Retire `sim/` from the live path (ANALYSIS ONLY — do not delete)

`sim/` is 7 tracked files, 316 KB, dominated by `trial_server_patched.py` at **1864 lines** —
larger than `ros_world.py`. Contents: `trial_server_patched.py`, `build_robot_usd.py`,
`sim_press.py`, `make_sim_waypoints.py`, `trial_server.sh`, `safety_sim.sh`, `README.md`.

**What references it, in full:**

| reference | live? |
|---|---|
| `bringup/ros_world.py:1117` — `_ros([ROS_PY, REPO/"sim"/"sim_press.py", ...])` | **guarded by `os.environ["UTP_SIM"] == "1"`**; unreachable on hardware, but it is a live-module code path |
| `EXPERIMENT_LOG.md:1381, 1391` | prose |

Nothing else in `bringup/`, `safety/`, `nav2_bringup/`, `tests/` or `config/` touches it. It is not
importable as a package and no test collects from it.

**The hazard is real but it is not "unused code".** CLAUDE.md says the simulation repo is
authoritative and must never be edited from here. `sim/trial_server_patched.py` is, by its own
docstring, *a patched COPY* of that repo's server. A copy of an authoritative artefact, held in a
second repo, with a local patch on top, will diverge — that is what copies do — and the divergence
will be discovered during a sim campaign, at night, when nobody wants to debug it.

**What removing it would take, and why not to do it now.** `sim_press.py` is genuinely reachable
(under `UTP_SIM=1`) and `trial_server_patched.py` may be the only copy of that patch anywhere.
Deleting a subtree with a demo pending is exactly the move that turns a tidy-up into an outage.
The safe sequence, later:

1. Confirm each file's upstream still exists in the sim repo and diff it. Anything byte-equal to
   upstream: delete, replace with a one-line pointer in `sim/README.md`. ~1 h.
2. Anything that differs: the difference is the artefact. Turn the patch into a real patch file
   under `patches/` (that directory already exists for this purpose) and record the upstream
   commit it applies to. ~2 h.
3. `sim_press.py` stays until the `UTP_SIM` branch in `ros_world.py` is removed or moved with it.
   It is the sim's arm backend and it has no hardware equivalent.
4. Verify: `UTP_SIM=1` dry-run of a trial on domain 42 — **never** domain 9.

**`archive/`: leave it exactly where it is.** 31 files, 276 KB. That is the correct cost for a
designated graveyard with a README that explains each resident, and it is what let step 0 be an
archive rather than a delete. Do not "clean up" `archive/`; its entire value is that things are
recoverable and the reason they left is written down.

---

## Step 2 — ONE navigation CLI, ONE pose source, ONE provenance validator

**Order matters: this must come after step 0** (the config the CLI plans against must be
unambiguous first) **and before step 5** (`ros_world.py` shells out to four of these scripts, so
splitting it while its subprocess targets are moving means two moving parts at once).

The family as it actually stands, corrected from the review brief:

| script | lines | what it really is |
|---|---|---|
| `nav2_goto.py` | 216 | the Nav2 action client. **Live** — `ros_world.py` shells to it |
| `goto_clicked.py` | 139 | RViz-click → goal. Own TF listener |
| `waypoints.py` | 587 | record/list/drive waypoints. **Live** — `session.sh` and `ros_world.py` |
| `twopoint.py` | 343 | odom-frame two-point drive |
| `turn_by.py` | 143 | odom-frame relative turn. **Live** — `ros_world.py` |
| `scan_compass.py` | 139 | yaw from scan geometry |
| `pose_source.py` | 238 | the pose/session/provenance library. **Live** — `session.sh` imports `slam_session_id` |
| `odom_session.py` | 25 | one function: DDS GID of the `/odom` publisher |
| `whereami.sh` | 44 | **not navigation at all** — prints the laptop's IP addresses and writes `.last_address` |

**Two corrections to the brief.** `whereami.sh` is a network-address reporter; it has nothing to do
with pose and must not be folded into a navigation CLI. `odom_session.py` is 25 lines and one
function — it is not a CLI, it is a leaf of the provenance library and belongs *inside* the single
pose source, not replaced by it.

**Six independent TF-pose readers exist today:** `pose_source.py`, `goto_clicked.py`,
`face_target.py`, `grab_frame.py`, `reproject_target.py`, `check_scan_geometry.py`. Each opens its
own `TransformListener` with its own timeout and its own staleness rule. That is six places for the
`map`-vs-`odom` frame mistake to be made differently.

**Target shape:**

- `bringup/pose_source.py` becomes the *only* module that reads a pose. It already owns
  `slam_session_id`, `current_map_name`, `PoseSource`, and the `.loaded_map` contract. Absorb
  `odom_session.odom_session_id` into it (25 lines, one caller).
- Provenance validation stays split exactly as it is today and that split is correct, not
  accidental: `safety/map_frame.py` owns **map**-frame provenance, `safety/waypoint_frame.py` owns
  **odom**-session provenance. `test_map_frame.py:81` already asserts the boundary. Do **not**
  merge them into "one validator" — they answer different questions and fail closed for different
  reasons. The brief's "ONE provenance validator" is the wrong target; the right one is *one
  caller-facing entry point* over the two.
- `bringup/nav.py` with subcommands: `goto <waypoint>`, `goto --click`, `goto --xy`, `turn <deg>`,
  `record <name>`, `list`, `where`. Every subcommand a thin argparse shell over `pose_source` +
  the existing action client.

**Migrate in this order, one commit each:**

1. Fold `odom_session.py` into `pose_source.py`, leave a 3-line shim. Verify: `pytest tests/ -q`,
   plus `session.sh nav` reaching `maps/.loaded_map -> …`. **~30 min.**
2. Repoint the five ad-hoc TF readers at `PoseSource`. One script per commit. Verify each against
   the behaviour it feeds (`face_target` → a dry-run press approach; `grab_frame` →
   `check_scan_geometry.py --tf`). **~3 h.**
3. Build `nav.py` as a *dispatcher over the existing scripts* — no logic moves yet. Verify: every
   old invocation and its new equivalent produce identical output. **~2 h.**
4. Only then inline the bodies and delete the old entry points. **~3 h.**

**Do not do step 4 unless someone is actually confused by the old names.** Steps 1–2 remove real
divergence risk. Step 4 is renaming, and renaming is where working scripts go to die.

---

## Step 3 — ONE `robot doctor`

Independent of every other step. Do it whenever.

Today: `health.py` (382), `preflight.py` (160), `lab_gates.sh` (185), `map_watch.py` (134), and
the `check_*` family — `check_calib.py` (121), `check_depth_alignment.py` (96),
`check_marker.py` (122), `check_scan_geometry.py` (140), `check_llm.sh` (86). That is
**1426 lines across 9 entry points**, and a new person has no way to know which to run first.
`check_press_safe.py` is **not** in this family — it is a runtime veto called by `ros_world.py`
mid-trial and must keep its own name and exit code.

Target: `bringup/doctor.py` with `doctor topics`, `doctor tf`, `doctor calib`, `doctor scan`,
`doctor llm`, `doctor arm`, `doctor all`, plus `doctor watch` for the live map monitor. Each
subcommand is the existing script's `main()` moved into a function; the script becomes a shim.

- **Anything that moves the robot stays behind an explicit flag and keeps its own file.**
  `lab_gates.sh` drives the chassis. It must not become `doctor gate 3` where a tab-completion
  slip starts a motion. Leave it out of the doctor entirely, or gate it behind `--i-am-holding-the-rc`.
- Verify: each subcommand's output diffed against the old script's on the same live graph.
- **~4 h**, and it saves onboarding time rather than failure rate. Rank it below step 7.

---

## Step 4 — ONE hand-eye module

Independent. Do it whenever.

`handeye.py` (174), `handeye_auto.py` (244), `handeye_capture.py` (241), `handeye_collect.py` (189),
`handeye_solve_rw.py` (215), `handeye_verify.py` (193), plus `check_calib.py` (121) and
`reproject_target.py` (145) — **1522 lines**, seven entry points, for one calibration.

Target: `safety/` is the wrong home (it is not an interlock); make `bringup/handeye/` a package —
`solve.py` (the maths, importable, no argv, no I/O), `capture.py`, `verify.py`, and one
`__main__.py` that only parses arguments.

The maths must move first and alone: `tests/test_handeye.py` and `tests/test_handeye_rw.py` already
exercise the solver, and they should keep passing byte-for-byte across the move. If they do not,
the move changed behaviour and must be reverted.

Verify: re-solve from the archived captures in `calib/` and assert the resulting `handeye.json` is
numerically identical to the committed one. **~5 h.** This one genuinely reduces risk — the press
error budget is the solver's output, and it currently has seven ways in.

---

## Step 5 — Split `ros_world.py`

**Must come after step 2.** `ros_world.py` shells out to `nav2_goto.py`, `waypoints.py`,
`turn_by.py`, `face_target.py`, `approach_blockage.py`, `approach_target.py`, `grab_frame.py`,
`detect_frame.py`, `check_press_safe.py`, `stow_arm.py` and `sim/sim_press.py`. Splitting it while
those targets are being renamed is two refactors interleaved, and the failure mode is a
`FileNotFoundError` at trial 30 of 50.

It is **1209 lines**, not the 912 the review quoted — it has grown ~33% since. Proposed split:

| new module | from | what it owns |
|---|---|---|
| `bringup/world/proc.py` | `_ros`, `ROS_PY`, env plumbing | subprocess execution and timeouts |
| `bringup/world/navigation.py` | `navigate_to_goal`, `_drive_leg_staged`, `_drive_leg_odom`, `_nav2_unavailable`, `_stage_seconds`, `_distance_to_goal`, `_goal_waypoint` | the leg state machine and Nav2 result classification |
| `bringup/world/perception.py` | `_read_scan`, `_nearest_ahead_m`, `_fused_verdict`, `_perceive_blockage`, `current_blockage`, the four `*_view` methods, `last_look_info` | observation and blockage |
| `bringup/world/manipulation.py` | `act`, `_reground`, the press sequence | arm and press |
| `bringup/ros_world.py` | `RosWorld` only | a thin `World` adapter that delegates |

**On the blockage duplication — the brief is partly wrong, and the truth is worse.**
`ros_world._fused_verdict` is *not* a third implementation: it already delegates to
`safety.blockage_fusion.fuse` and only falls back to camera-only when the module will not import,
printing a loud warning when it does. That design is correct and should be kept.

The real duplication is one layer down, in the geometry, and the three copies **disagree**:

| where | "nearest thing ahead" is measured as |
|---|---|
| `ros_world._nearest_ahead_m` (line 217) | angular cone, **±20°** (`FORWARD_HALF_ANGLE_DEG`) |
| `approach_blockage.nearest_ahead` (line 108) | angular cone, **±15°** |
| `safety/blockage_fusion` (`DEFAULT_HALF_WIDTH_M`) | **rectangular corridor, 0.40 m half-width** — not an angle at all |

`nearest_ahead_m` is what the back-off reverse distance is computed from, and which of the three
produced it depends on whether `blockage_fusion` imported. A cone and a corridor give different
answers for the same scan at every range: at 2 m a ±20° cone is 0.73 m half-width, at 0.5 m it is
0.18 m. **`safety/blockage_fusion.py` should be the one that survives** — it is the newest, the
best documented, the only one with its own test file, and the corridor is the geometrically correct
model of a 0.5 m-wide robot. `ros_world._nearest_ahead_m` and `approach_blockage.nearest_ahead`
should both become calls into it, keeping the import-failure fallback exactly as it is.

**Do the geometry unification (~2 h) BEFORE the file split.** It is a real behavioural fix with a
small diff. The split is 1200 lines of movement that changes nothing observable, and if you only
have time for one, do the geometry.

Verify the split: `pytest tests/test_ros_world_escalation.py -q` (77 asserts, all dry-run), then a
2-trial `--dry-run` campaign, then a 2-trial live campaign with a hand on the RC. **~8 h** for the
split, and it is the single riskiest item in this document.

---

## Step 6 — `safety/`: classification only, MOVE NOTHING

Full table in the hand-back report; the operational conclusions are:

- **Nothing in `safety/` may be deleted on the strength of this classification.** An interlock
  whose only caller is a shell script looks dead to every static tool there is.
- Four modules have **no non-test importer**: `escalation.py`, `local_avoid.py`, `route_plan.py`,
  `scan_filter.py`. Three of those belong to the retired odom/route era (`route_run.py` is already
  in `archive/`). They can be *marked* retired in a header comment now, at zero risk. Consider
  moving them only after a campaign has run without them, and even then move to `archive/`, never
  delete.
- **`archive/README.md` currently contains a false statement that must be corrected:** it says
  *"`safety/scan_filter.py` is NOT retired — that is the pure-logic corridor veto."* It is not.
  `scan_filter.py` is the **A1M8-era self-occlusion sector mask** (`KEEP_HALF_ANGLE_DEG = 148.0`,
  its docstring measured against the A1M8 mount in August); the corridor veto is
  `safety/waypoint_drive.corridor_blocked`, a different function in a different file. The OS0-era
  replacement for `scan_filter.py` is `bringup/scan_relay.py`'s `mask_self_returns`, which
  reimplements the same idea with different constants (`MASK_MIN_DEG = 74.0`,
  `MASK_MAX_DEG = 180.0`, `MASK_MAX_M = 1.00`) and does not import it. So the graveyard README is
  telling a future reader that a retired module is live, and that is exactly how a retired module
  gets maintained forever — or, worse, how a live one gets deleted next time the README is trusted.
  **~10 min to fix, and it should be fixed before any `safety/` move is contemplated.**

---

## Step 7 — The test-quality problem. **Do this before steps 2–5.**

Not because it is elegant, but because steps 2–5 all move source text, and a suite that asserts on
source text will fail on every one of those moves for reasons that have nothing to do with
correctness. Fixing the tests first is what makes the refactors cheap; leaving them is what makes
the refactors look dangerous.

### The count

**873** assert statements across 31 test files. Of these:

- **45** assert *directly* on file text, a regex over file text, or an AST parse.
- **91** (10%) do so once one hop of dataflow is followed — `src = X.read_text()` on one line,
  `assert "..." in src` on the next. This is the number to use; it is close to the ~81 the review
  estimated.

Worst offenders:

| file | text asserts | total asserts |
|---|---|---|
| `tests/test_ros_world_escalation.py` | 21 | 77 |
| `tests/test_stack_wiring.py` | 19 | 77 |
| `tests/test_nav_backend.py` | 17 | 49 |
| `tests/test_map_persistence.py` | 16 | 41 |
| `tests/test_session_e2e.py` | 9 | 64 |
| `tests/test_scan_mask.py` | 4 | 61 |
| `tests/test_nav2_scan_source.py` | 3 | 5 |
| `tests/test_handeye_rw.py` | 2 | 16 |

### Why they are actively harmful, not merely useless

1. **They break harmless refactors.** Every one of steps 2–5 moves code. `test_nav_backend.py:154`
   reads `nav2_params_os0_map.yaml` as *text* and asserts on substrings; step 0 nearly tripped it.
   `test_stack_wiring.py:875` asserts on a regex over `safety/reach_envelope.py`'s source. Rename a
   constant and a green suite goes red while the robot behaves identically.
2. **They pass while runtime behaviour is wrong.** A test that asserts the string `/scan` appears
   in a YAML file cannot tell you that `scan_relay.py` is not running, that the QoS is
   incompatible, or that the mask is inverted. The `topic:` line was *correct in all three files*
   by the end of 2026-09-01, and the robot still could not plan, because the fault was live and
   the assertion was textual.
3. **They did not catch either of today's two real integration failures.** Both are integration
   faults between components that each passed their own tests:
   - Nav2's scan topic pointing at the unmasked `/scan_filtered`. `tests/test_nav2_scan_source.py`
     is **untracked in git** — it was written *after* the incident, in response to it.
   - A waypoint's map name never being compared to the loaded map.
     `tests/test_map_persistence.py` is **modified today**, for the same reason.

   A suite of 873 assertions that produces its coverage the day after the outage is a suite that is
   documenting history, not preventing it.

### What replaces them

Not "delete the text assertions". Most of them are trying to express a real cross-component
invariant — they just express it in the only medium available, which is the source file. The fix is
to make the invariant *executable*:

- Where the invariant is about a **config value**, parse the config and assert on the parsed value,
  not on the file's text. `tests/test_nav2_scan_source.py::test_every_costmap_observation_source_uses_the_masked_scan`
  already does this correctly (`yaml.safe_load` then a recursive walk) — it is the model. Its
  sibling `test_no_params_file_still_claims_scan_filtered_is_the_clean_one` asserts on a *comment*
  and is the anti-model, though it is a defensible short-term tripwire while the inverted comment
  is fresh.
- Where the invariant is about **two components agreeing**, write a contract test against both
  implementations, not a grep. The cone-vs-corridor divergence in step 5 is a one-line property
  test — same scan in, same `nearest_ahead_m` out — and no amount of source-text reading would
  ever have surfaced it.
- Where the invariant is genuinely about the **live graph** (publisher counts on `/cmd_vel`, QoS
  compatibility, TF existence), it is not a unit test. Move it into `doctor` (step 3) where it runs
  against a live graph and can actually fail for the right reason.

### The behavioural areas that must keep tests, whatever else is cut

Non-negotiable. If a consolidation step would reduce coverage in any of these, stop and rewrite the
test instead:

| area | why |
|---|---|
| **map provenance** | a waypoint recorded in map A, driven in map B, is a robot driving to coordinates that mean nothing |
| **Nav2 result classification** | "succeeded" vs "aborted" vs "timed out" decides whether a trial escalates or is scored a failure |
| **scan masking** | the robot seeing its own arm as the room — today's incident, twice over |
| **safety arbitration** | the twist mux is layer 2, and the only software layer between Nav2 and the wheels |
| **arm reach limits** | the tool tip sweeps ~0.88 m through space the costmap believes is empty |
| **blockage fusion** | fail-closed OR; an AND would have driven at both of 2026-09-01's glass doors |
| **trial record semantics** | if the record is wrong the experiment is wrong, and nothing downstream can detect it |

---

## What is NOT worth doing

Stated plainly, because a consolidation plan that recommends everything is a plan nobody follows.

- **Renaming working scripts for consistency.** Step 2 stage 4 and most of step 3 are renaming.
  They reduce confusion for a newcomer and reduce failure rate by approximately zero. Every one of
  them is a chance to break a path string inside a shell script that no test executes.
- **Splitting `ros_world.py` for its own sake.** 1209 lines is uncomfortable, but the file is
  densely commented with measured incident evidence and the split moves ~1200 lines without
  changing one observable behaviour. Do the ±20°/±15°/0.40 m geometry fix — that is a real bug —
  and treat the split as optional.
- **Merging `safety/map_frame.py` and `safety/waypoint_frame.py`.** They look like duplicates and
  are not. Map-frame and odom-session provenance fail closed for different reasons, and a merged
  validator would have one code path where there are correctly two.
- **Deleting anything from `safety/`.** Covered in step 6. The asymmetry is total: a wrongly-kept
  module costs disk, a wrongly-deleted interlock costs a person.
- **Deleting `sim/` before the demo, or arguably at all.** Step 1.
- **Reducing the assertion count as a goal.** The problem is not that there are 873 assertions.
  It is that 91 of them assert on the wrong thing. Converting those to behavioural assertions may
  well *raise* the count.
- **Consolidating `archive/`.** It is doing its job. Leave it.

---

## Suggested order, with rough time

| # | step | depends on | time |
|---|---|---|---|
| 0 | one Nav2 config | — | **done** |
| 6a | fix the false `scan_filter.py` claim in `archive/README.md` | — | 10 min |
| 5a | unify the three `nearest_ahead` geometries onto `blockage_fusion` | — | 2 h |
| 7 | convert the 91 source-text assertions to behavioural ones | — | 6–8 h |
| 4 | one hand-eye package | 7 | 5 h |
| 2 | one pose source (stages 1–2 only) | 0, 7 | 4 h |
| 3 | one `robot doctor` | 7 | 4 h |
| 5b | split `ros_world.py` | 2, 5a, 7 | 8 h |
| 1 | reconcile `sim/` against the sim repo | — | 3 h |
| 2' | `nav.py` dispatcher and the renames (stages 3–4) | 2 | 5 h — **optional** |

Steps 6a and 5a are the two that fix something real in under half a day. If the week gets eaten,
do those two and step 7, and let the rest wait for the next quiet fortnight.
