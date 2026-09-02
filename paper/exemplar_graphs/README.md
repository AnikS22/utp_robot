# Exemplar graphs — what the paper figures have to look like, and what a run must record

Two reference figures from a comparable paper set the bar. Drop the source images
into `reference/` as `fig12_trajectory_keyframes.png` and `fig13_skill_strip.png`
(they are referred to by those names below).

The point of this folder is to work backwards from the figure to the data, so that
**every trial we run from now on captures what the figure needs the first time.**
Re-running a trial to get a missing frame is how a week disappears.

---

## Reference A — trajectory + keyframes (their Fig. 12)

Two-panel layout, one row per scene.

**Left panel — the map and the paths.**
* Point cloud rendered as coloured points (height-coloured, blue low to red high)
* **Two trajectories overlaid**: baseline in red, ours in green, each with an arrowhead
* Numbered circular markers (1,2,3,4) on OUR trajectory at the decision points
* A gold star at the goal
* The obstacle circled in red and labelled in red text ("box", "door")
* A scale bar ("1m") — not axes. Clean, no gridlines, no legend clutter
* Legend top-centre naming the two methods

**Right panel — what the robot did at each numbered point.**
* One row per number, aligned to the markers on the left
* 2-3 images per row. A **camera icon** marks the ego-centric frame; the others are
  third-person
* A coloured callout box at the right of rows where a decision happened:
  blue `VLM Reasoning: Navigation`, blue `VLM Reasoning: Push box`,
  green `Goal reached!`

**What it is arguing:** the baseline detours around the obstacle (long red loop) and
ours goes through it, and you can see *why* at each step because the VLM's decision is
printed next to the frame that produced it.

## Reference B — skill execution strip (their Fig. 13)

* One row per object, 3-5 frames left-to-right showing the manipulation
* Rows for different object shapes and different door types
* No annotation at all — the strip alone shows the strategy adapting

**What it is arguing:** one policy, many objects. For us that is: ADA plate, elevator
call plate, elevator car button, and whatever else we get.

---

## What a run must record, to make either figure

Ranked by how hard it is to recover if we forget.

| # | Data | Where it comes from | Status |
|---|---|---|---|
| 1 | **Trajectory in map frame**, timestamped | `map -> base_link` TF, ~10 Hz | **MISSING for trials.** `maps/*/poses.jsonl` proves the format; it only runs during mapping |
| 2 | **Ego-centric frame at each decision point** | `/mast_cam/color/image_raw` | Partial — `grab_frame.py` saves `rgb.png` at each PRESS only, not at nav legs |
| 3 | **The VLM's decision text**, tied to the frame | the reasoner's plan | Partial — `detection.json` has the query and score, not the reasoning |
| 4 | **Third-person video** | operator's phone on a tripod | **MISSING.** Cannot be recovered later. See below |
| 5 | **Baseline trajectory for the same scene** | run heuristic / direct-VLM on the same waypoints | **MISSING.** The red line in Fig 12 |
| 6 | **The map/point cloud background** | `maps/elevator.pgm`, or accumulated `/ouster/points` | Have the grid; a prettier cloud needs one dedicated pass |
| 7 | Per-trial metrics | `captures/trials.jsonl` | **Already good** — 40 fields incl. path_length_m, num_vlm_calls, latency_vlm_s, success, failure_category |

### The third-person camera is the one that cannot be redone
Everything else is reconstructable from logs. A phone on a tripod, fixed position,
recording the whole run, is the only source for the third-person column, and there is
no second chance at a first successful elevator run. **Set it up before the run, not
after it works.**

---

## Recording without disturbing the VLM

This is a real constraint, not a courtesy. The detector runs on `cuda:0` and has
already been starved into CUDA OOM once — RViz was holding **14.3 GB** of GPU with the
full 131k-point cloud displayed. Rules:

* **No GPU.** The recorder must not open a CUDA context, and RViz's 3D point cloud
  display stays OFF during any run that grounds. Set `Decay Time` to 0 as well.
* **Do not subscribe to `/ouster/points`.** `/scan` already runs 4.6-6.4 Hz against
  the sensor's 10 because of DDS transport loss; another subscriber on the 1 MB cloud
  makes it worse. Record the map once, not the stream.
* **Throttle images.** 1-2 Hz JPEG, not 30 Hz PNG. `grab_frame.py`'s full-resolution
  PNG + depth `.npy` is for grounding, not for film strips.
* **Poses are free.** TF at 10 Hz is a few hundred bytes a second. Log continuously.
* **Separate process, `nice`d**, so it cannot take CPU from the detector, and writing
  to its own directory so a crash cannot corrupt a capture.

---

## Proposed layout

    runs/<utc-timestamp>_<method>_<scene>/
      meta.json          method, scene, waypoints, git sha, config hashes
      poses.jsonl        {stamp, map:{x,y,yaw}, odom:{...}}  ~10 Hz
      events.jsonl       {stamp, kind, detail}  leg start/end, ground, veto, press, stow
      frames/            ego-centric JPEG, 1-2 Hz, named by stamp
      decisions/         one dir per VLM call: the frame it saw + its text + detection.json
      thirdperson.mp4    dropped in by hand from the phone

`trials.jsonl` keeps its current schema and gains one field pointing at the run dir,
so the metrics table and the figures come from the same record.

---

## Open questions, to settle before the first recorded run

1. **Baseline runs.** Fig 12's red line needs the same scene driven by a comparison
   method. Which — heuristic, passive, direct-VLM? All three exist as `method_name`
   values in `trials.jsonl`. Running each on the elevator costs elevator time.
2. **Do we want the point cloud or the occupancy grid as the background?** The grid is
   free and already saved; the cloud looks better and needs a dedicated slow pass with
   nothing else running.
3. **How many objects for the Fig 13 strip?** Minimum three to show adaptation. We
   have ADA plate and two elevator buttons; a third door type would help.

---

## How to actually record a run (built 2026-09-02)

    # 1. start the recorder BEFORE the route, niced so it cannot starve the detector
    nice -n 10 python3 bringup/run_recorder.py --scene elevator --method ours &
    #    it prints the directory it chose; export it so the route can mark events
    export UTP_RUN_DIR=runs/<the-dir-it-printed>

    # 2. phone on the tripod, recording. This is the ONLY unrecoverable stream.

    # 3. run the route as normal -- it emits events automatically
    bash bringup/elevator_route.sh

    # 4. stop the recorder (Ctrl-C or kill), then build the figure
    python3 paper/make_figure.py runs/<the-dir>

    # 5. drop the phone video in as runs/<the-dir>/thirdperson.mp4

`bringup/run_event.sh` is a no-op when `UTP_RUN_DIR` is unset, so a recorded run and
an unrecorded run are the same code path. Nothing about recording changes what the
robot does.

### What comes out

* `figure_trajectory.png` — the map, the green path, numbered markers merged by PLACE
  (arriving somewhere and pressing there is one marker, not two), leader lines where
  markers would collide, a gold star at the goal, and a 1 m scale bar. No axes.
* `keyframes/NN_*.jpg` — the ego-centric frame nearest each numbered marker, named so
  the number matches. It warns when the nearest frame is more than 2 s from the event.
* The console prints what each number means, which is what the right-hand column's
  captions get written from.

### Still to do by hand
The VLM reasoning callouts (`VLM Reasoning: Push door`) are not captured yet — the
route knows the query string but the reasoner's decision text is not written into the
run. That is the next piece to wire.
