# The benchmark pipeline — what we actually run, and what changes on hardware

This is the testing pipeline validated in simulation. The physical robot runs **the same pipeline**;
only the `World` implementation changes. Read this before touching the runner, so you know what a
"trial" is and what is being measured.

Source of truth is the pipeline repo (`~/unlocking-the-path`); this document explains it and flags
every place hardware differs.

---

## 1. The thesis in one paragraph

The robot is given a goal it **cannot reach by navigating**, because the building is in the wrong
state — a door is closed, an elevator has to be called and ridden. Success requires *changing the
environment*. The claim is that this is best done by **decoupling** semantic reasoning ("press the
blue ADA plate left of the door") from geometric grounding ("that description is at these pixels"),
rather than asking one model to do both. No training: everything is a pre-trained VLM plus a
pre-trained open-vocabulary detector, wired together.

## 2. The loop

Every trial is a finite-state machine over four swappable modules (`utp/pipeline/fsm.py`):

```
        navigate ──► blocked? ──no──► goal reached ──► success
                        │
                       yes
                        ▼
   ┌───────► REASON ──► GROUND ──► ACT ──► VERIFY ──┐
   │      (what to do)  (where)   (do it) (did it   │
   │                                       work?)   │
   └──────────── retry / recover ◄──────────────────┘
```

| Module | Job | Implementations |
|---|---|---|
| `Reasoner` | next action + natural-language target description | `vlm` \| `heuristic` \| `none` |
| `Grounder` | description → pixel box → 3D point | `decoupled` \| `direct_vlm` \| `oracle` |
| `Executor` | carry the action to the world (arm press, elevator entry) | `on` \| `off` |
| `Verifier` | did the world actually change? | `on` \| `off` |
| `Navigator` | drive to the goal | `nav2` \| `scripted` |

**The `Reasoner` never emits coordinates.** That prohibition is the thesis, enforced in the system
prompt. If you ever "helpfully" let the VLM return a box, you have deleted the experiment.

`verify()` is **inside the control loop**, not just in the metrics — the FSM uses its result to
decide retry vs recover vs resume. This is why a human keypress is not an acceptable witness on
hardware: it would put a person inside the agent's decision loop and unblind every trial.

## 3. Methods are config rows, never code paths

From `config/methods.yaml`. Baselines and ablations are the same code with different modules
selected — this is what makes the comparison fair.

| Method | reasoning | grounding | execution | verification |
|---|---|---|---|---|
| `passive` | none | none | off | off |
| `heuristic` | heuristic | decoupled | on | off |
| `direct_vlm` | vlm | direct_vlm | on | off |
| **`ours`** | **vlm** | **decoupled** | **on** | **on** |
| `ours_no_decoupling` | vlm | direct_vlm | on | on |
| `ours_no_reasoning` | heuristic | decoupled | on | on |
| `ours_no_execution` | vlm | decoupled | **off** | on |
| `ours_no_verification` | vlm | decoupled | on | **off** |

`passive` is the load-bearing control: **a mission is only valid if `passive` provably fails it.**
On hardware this is the top risk — if the ADA door is motion-activated or push-open, `passive`
succeeds and the mission measures nothing. Check it on the first site visit.

Matrices (`matrices:` in `methods.yaml`): `smoke` = `[ours]`, `core` = the headline four,
`head2head` = `[direct_vlm, ours]` (the decisive decoupling test).

## 4. Mission tiers

The sim benchmark is 47 missions in one office building across three floors:

| Tier | What it tests |
|---|---|
| `M0_open` | no intervention needed — **negative control**, must NOT interact |
| `M1_door` | single button door — the clean reference loop |
| `M3_disambig` | decoys: several similar controls, only one correct |
| `M4_unreachable` | sealed goal — **success = correctly reporting unreachable**, inverted scoring |
| `M5_elevator` | call → enter → select floor → exit; cross-floor, long horizon |
| `M6a/M6b_keyfob` | badge reader; capability-grounded refusal when the badge lacks the zone |
| `M7_chained` | several interventions in sequence |

**Scoring is per-scene and explicit** (`success_criterion` in `config/scenes.yaml`):
`reach_goal`, `restraint_open` (success = drove through with **no** spurious interaction),
`restraint_unreachable` (success = gave up cleanly). The negative controls invert normal scoring —
verify them by hand on the first runs, because a mis-scored negative control silently invalidates
the restraint claim.

### The real-robot mission set

| | Sim analogue | Runnable on hardware? |
|---|---|---|
| **R0** | `M0_open` | yes — negative control, no interaction |
| **R1** | `M1_door` | yes — **the minimum viable result** |
| **R2** | `M3_disambig` | yes, if the site has decoy controls |
| **R3** | `M5_elevator` | yes — real elevator, no fob |
| ~~R4~~ | `M6a/M6b_keyfob` | **no** — the test site's elevator has no credential gating, so the keyfob tiers stay simulation-only |

Collect **R0 → R1 to completion → R2 → R3**, not in parallel. R1 alone — a mission where `passive`
provably cannot reach the goal and `ours` can, on hardware, with no human in the loop — is a
complete real-robot validation.

## 5. What a trial records

Per trial (`utp/common/schema.py`), beyond success/failure:

- **Outcome:** `success`, `interactions_required` vs `interactions_completed`, `total_time_s`,
  `path_length_m`, `failure_category` + `failure_detail`
- **Reasoning:** `reasoning_correct`, `reasoning_correct_rate` — did it choose the right action?
- **Grounding:** `target_correct`, `grounding_iou`, `grounding_center_error_px` — the decoupling
  evidence, and the numbers the whole thesis rests on
- **Cost:** `latency_vlm_s`, `latency_detector_s`, `num_vlm_calls`
- **Safety/physical:** `n_collisions`, `collided`, `arm_press_ok`, `arm_press_distance_m`,
  `arm_ee_final_pose`
- **Provenance:** `cfg_vlm_model`, `cfg_*` for every module, `scene_seed`, `world`

Raw VLM traces (exact prompts + verbatim responses, every attempt) go to the trial's artifacts
folder. Keep them: they are what lets a reviewer audit a claim instead of trusting a paraphrase.

**Two hardware gotchas in the analysis layer:**
- `analysis/tables.py::_real_only()` filters on `world == "isaac"` and will **silently discard every
  real-robot trial**. Fix before logging anything you care about.
- `arm_press_ok` should be reported as an *outcome*, not an error. On hardware a press that lands
  but does not open the door is data, not a bug.

## 6. Running it

The port seam is a single function — `make_world()` in `utp/runner/batch.py`. `registry.py` already
selects the real VLM, real GroundingDINO and `Nav2Navigator` for **any** non-mock backend, so
adding a `real` backend needs **zero registry changes**.

```bash
cd ~/unlocking-the-path && . env/.venv/bin/activate

# pure-python sanity check, no GPU, no network, no robot
python -m utp.runner.batch --matrix smoke --world mock --seeds smoke

# full test suite
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest -q

# hardware (once the real World backend exists)
source ~/utp_robot/bringup/env.sh          # ROS + ROS_DOMAIN_ID=9
python -m utp.runner.batch --matrix core --world real --seeds dev
```

Runs are **resumable** — already-logged `trial_id`s are skipped, so a crash costs nothing. Budgets
live in `config/runtime.yaml`: `trial_time_budget_s: 300`, `max_recovery_attempts: 2`,
`control_loop_hz: 30`.

## 7. What changes on hardware

| | Simulation | Real robot |
|---|---|---|
| Pose feedback | ground truth at 5 Hz | **odom** for the servo/press chain, **map**/AMCL only for goals, DTG, path length |
| Ground truth (`gt_*`) | from the simulator | instrumented door witness (reed switch → ESP32); AprilTag + lidar aperture as logged cross-checks |
| Elevator floor | published by the sim | AprilTags per floor lobby — observable only **after exiting**, a genuine change to the mission |
| Controller | MPPI, `motion_model: Omni` | **Regulated Pure Pursuit** — the Ranger cannot do simultaneous strafe+yaw |
| Approach servo | blends `linear.x` + `angular.z` | must **alternate** rotate-then-translate |
| `use_sim_time` | `true` | **`false`** everywhere |
| Seed | varies geometry | geometry is fixed, so seed varies the **start pose** (±0.3 m, ±15°) |
| Nav2 footprint | chassis only | must account for the arm, or the planner routes the base through space the arm occupies |

**On the seed change — this is methodology, not cosmetics.** With fixed real geometry, identical
start poses mean the only variation between trials is VLM nondeterminism, and n=8 is effectively
**n≈1**: eight replays of one initial condition. Jitter the start pose from the seed and record it.

**Annotate grounding boxes blind** — shuffled, method hidden. The decoupling comparison is what this
data exists to settle; letting the annotator see the condition poisons it.

## 8. The three ways this pipeline lies to you

1. **A plausible failure category.** `failure_category` is assigned by our code. If grounding
   returned a box on the wrong object with high confidence, the trial logs a clean "execution
   failure" and the real cause is invisible. Read the artifacts, not just the table.
2. **Negative controls scoring backwards.** `M0_open` and `M4_unreachable` succeed by *not* acting.
   A sign error there flatters the restraint claim rather than breaking it, so it will not announce
   itself.
3. **Success without the mechanism.** If the door happens to be open, `ours` "succeeds" and so does
   `passive`. Always check the `passive` row before believing an `ours` row.
