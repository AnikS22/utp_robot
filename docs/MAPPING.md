# Mapping the test site

Produces the `map` the mission runs localize against. Do this **after** the calibration items that
feed it — a map built on a wrong lidar pose or a mirrored scan looks perfectly plausible and
navigates catastrophically.

## Before you start

| Blocker | Why |
|---|---|
| `bringup/stale_cmd_test.py driver` **and** `firmware` both PASS | Mapping means driving. On 2026-08-20 the base ran away under teleop. Source says the *driver* cannot latch (`ranger_messenger.cpp:391` commands straight from the subscription callback; no repeat timer anywhere), but the *chassis firmware's* behaviour when commands stop arriving is unreadable from source and unmeasured. Two failure modes, two phases. |
| CALIBRATION ③ lidar mount pose | An unmeasured offset biases **every** obstacle by that offset. |
| CALIBRATION ④ scan direction / zero-angle | A mirrored scan builds a map that looks fine and is wrong everywhere. Physical check, cannot be skipped. |
| Gate **S1** — glass doors | 2D lidar sees *through* glass. It will not appear in the map, and the robot will drive at it. Safety, not data quality. |

Gate **S0** is not a mapping blocker but is the project's top risk: if the ADA doors are
motion-activated, `passive` succeeds and R1 measures nothing. Answer it on the same visit — one
walk-through settles it.

## Build the map

```bash
bash ~/utp_robot/bringup/mapping.sh --with-base        # lidar + odom + SLAM + RViz + teleop
```

That is the whole stack in one command, with RViz already showing `/map`, `/scan`, the TF tree and
the pose graph (`maps/mapping.rviz`). Without `--with-base` it starts only the read-only half — it
cannot move the robot even if every part of it fails — which is the right way to check the lidar
and the TF chain before handing CAN authority over. The pieces it starts, if you would rather run
them yourself:

```bash
source ~/utp_robot/bringup/env.sh
sg dialout -c 'bash ~/utp_robot/bringup/lidar.sh'          # /scan + base_link->lidar_link
ros2 launch ranger_bringup ranger_mini_v3.launch.py use_sim_time:=false publish_odom_tf:=true
ros2 launch slam_toolbox online_async_launch.py \
     use_sim_time:=false slam_params_file:=$HOME/utp_robot/config/slam.yaml
bash ~/utp_robot/bringup/teleop.sh                          # drive: http://127.0.0.1:8420
```

`publish_odom_tf:=true` is **not** the launch default. Without it there is no `odom` frame at all,
slam_toolbox has nothing to anchor to, and nothing says so. Likewise `config/slam.yaml` exists only
because stock slam_toolbox uses `base_frame: base_footprint`, which this stack does not have — with
the stock value it publishes `/map` and looks healthy while never emitting `map → odom`.

**Driving technique**, in rough order of how much it matters:
- **Use DualAckermann only.** `mapping_scan_gate.py` blocks `/scan_mapping` in spinning,
  parallel/crab, and side-slip modes. A physical run on 2026-08-24 showed that spin-mode odometry
  caused metre-scale scan-matching jumps. If RViz stops adding map data, check the RC mode.
- **Slowly.** The A1M8 spins at ~7 Hz, not the 10 Hz the driver claims. Fast rotation smears scans.
- **Close loops.** Return to somewhere you have already been, by a different route. Loop closure is
  what removes accumulated drift; a map from a single out-and-back is a spiral.
- **Use broad Ackermann turns and pause afterward.** Do not pivot or crab while mapping.
- Cover every space a mission enters, including the far side of each door.

Watch `/map` in RViz while driving. If corridors bend or a room appears twice, stop and re-drive
the loop rather than saving it.

## Save it

```bash
bash ~/utp_robot/bringup/save_map.sh          # -> maps/site.{pgm,yaml,posegraph,data}
```

Save after **every closed loop**, not once at the end: saving does not stop mapping, and a crash
between the last loop and the end of the drive costs the whole walk. The script checks the files
actually landed on disk, because both services return success even when nothing is written. Under
the hood:

```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{name: {data: '$HOME/utp_robot/maps/site'}}"                       # .pgm + .yaml, for Nav2
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '$HOME/utp_robot/maps/site'}"                            # .posegraph, to resume mapping
```

Save **both**. The `.pgm`/`.yaml` pair is what Nav2's map_server loads; the `.posegraph` is the only
thing that lets you continue mapping later instead of starting over.

## Hand the map to Nav2

`ranger_nav.launch.py` takes a `localization` argument, because **exactly one thing may publish
`/map`, and exactly one may publish `map -> odom`**. Running Nav2's `map_server` alongside
slam_toolbox puts two publishers on `/map`, and the costmap uses whichever it latched.

**A. Saved map + AMCL** — the normal way to run a mission on a map you already built:

```bash
ros2 launch $HOME/utp_robot/nav2_bringup/ranger_nav.launch.py \
     map:=$HOME/utp_robot/maps/site.yaml localization:=amcl      # amcl is the default
```

`map_server` serves `site.yaml`; **AMCL** provides `map -> odom`. Do not run slam_toolbox at all.
AMCL starts with no idea where it is: give it a pose with **2D Pose Estimate** in RViz (or publish
`/initialpose`) roughly where the robot actually stands, then drive a few metres so the filter
converges. Until it does, `map -> odom` does not exist and the global costmap logs
`Timed out waiting for transform` — which is the expected message, not a fault.

**B. slam_toolbox owns everything** — while mapping, or localizing off a `.posegraph`:

```bash
ros2 launch $HOME/utp_robot/nav2_bringup/ranger_nav.launch.py localization:=slam
```

No `map_server`, no AMCL: slam_toolbox supplies both `/map` and `map -> odom`. This is how you
navigate *while still mapping*, and no `map:=` argument is used.

AMCL was **added for hardware** (the `amcl:` block in `nav2_params.yaml`). The sim never needed it —
`docs/integration_contract.md` says "no AMCL here" because Isaac published `map -> odom` as ground
truth. Two settings there are worth knowing: `base_frame_id` is `base_link` (the stock
`base_footprint` default is the same trap that silenced slam_toolbox), and the odometry noise
`alpha1..alpha5` are conservative placeholders until CALIBRATION.md item 5 measures the real values.

## Switch slam_toolbox to localization mode

Mapping and localizing are different modes and must not both run.

```bash
# config/slam.yaml: mode: mapping -> localization, then
ros2 launch slam_toolbox localization_launch.py \
     use_sim_time:=false slam_params_file:=$HOME/utp_robot/config/slam.yaml \
     map_file_name:=$HOME/utp_robot/maps/site
```

AMCL is the alternative, and is what `localization:=amcl` above starts. Whichever you pick,
exactly one node may publish `map → odom`; two publishers on that edge is its own bug — which is
precisely why the launch file takes a `localization` argument instead of letting you start both.

**Expect degeneracy in a long featureless corridor**: uncertainty grows along the travel axis while
staying tight laterally. Because the press chain deliberately runs in the **odom** frame
(`PIPELINE.md` §7), that degrades goal scoring but cannot cause a missed press. Verify that
separation holds rather than assuming it.

## Check the map before trusting it

```
CHECK:  drive to a known landmark; the pose in RViz matches where the robot physically is
        a measured corridor width in the map matches a tape measure within ~5 cm
        no doorway is walled shut by the inflation layer (inflation_radius 0.30)
        glass doors: confirm by hand whether they appear at all -- they usually do not
```

Record the corridor measurement and the loop-closure outcome in `EXPERIMENT_LOG.md`. A map with no
recorded check is a map nobody has any reason to trust.
