# STARTUP — one command, and what to do when it says no

Bringing this robot up has been the most expensive part of every session. Not because any single
step is hard, but because when a step fails it usually fails **silently**: the node is alive, the
topic is advertised, `ros2 node list` is green, and nothing is printed anywhere. The symptom then
surfaces two or three layers downstream, wearing someone else's clothes.

This document is the fix. One command, in dependency order, every stage verified before the next,
and a `WHY` block that names the cause, gives the command that fixes it, and says what the symptom
would otherwise have looked like.

---

## The one command

```bash
bash bringup/bringup_all.sh --mode nav --map elevator
```

```bash
bash bringup/bringup_all.sh --mode map                 # sensing chain + slam in MAPPING mode
bash bringup/bringup_all.sh --mode nav --map floor2    # chain + slam LOCALIZING on floor2 + Nav2
bash bringup/bringup_all.sh --mode full --map floor2   # nav + camera + arm (the press chain)
bash bringup/bringup_all.sh --status                   # report only: starts, kills, changes NOTHING
```

| | |
|---|---|
| **Idempotent** | A component that probes healthy is left alone. Re-running costs a minute of probes and restarts nothing. Run it as often as you like. |
| **Never runs `sudo`** | `can0` needs a password. A bring-up that blocks on an invisible password prompt is indistinguishable from one that has hung, so this detects it, prints the exact line, and stops. |
| **Never kills by a loose pattern** | Every kill candidate must match on its full `/proc/<pid>/cmdline` **and** carry this repo's `UTP_ROBOT_STACK` marker and this `ROS_DOMAIN_ID` (both exported by `bringup/env.sh`). A matching process that is *not* ours is reported with its pids and left alone — never killed, never stacked on top of. |
| **Exit codes** | `0` everything the mode needs is up · `1` something required is down · `2` a human is needed before anything can start (`can0`, the cable) — and in that case **nothing is started at all**. |

The mode decides what is required. `map` needs the sensing chain and slam in mapping mode. `nav`
adds a saved map, slam in localization mode and Nav2. `full` adds the camera and the arm. **The
xArm is only fatal in `full`** (or with `UTP_NEED_ARM=1`); the lidar and the router are fatal
always.

---

## The dependency graph

Each arrow is a *data* dependency, and each stage is verified **by rate** before the next is
started. Starting a stage before its input exists produces a node that is alive and silent forever.

```text
  can0 up  ──▶ chassis (ranger_mini_v3, publish_odom_tf:=true)
                 │  /odom  +  TF odom->base_link
                 ▼
  bringup/lidar3d.sh ──┬──▶ TF base_link->os_sensor   (from config/ouster.yaml `mount`)
                       └──▶ ouster_ros driver         (TF os_sensor->os_lidar, /ouster/points)
                                  │
                    ╔═════════════╧══════════════════════════════════════════════╗
                    ║  base_link -> os_lidar MUST RESOLVE BEFORE THE NEXT STAGE  ║
                    ╚═════════════╤══════════════════════════════════════════════╝
                                  ▼
        safety/cloud_artifact_filter.py   ──▶ /ouster/points_clean
                                  │              (drops range < 1.4 m AND reflectivity <= 1)
                                  ▼
        pointcloud_to_laserscan           ──▶ /scan_filtered      BEST_EFFORT
          target_frame:=base_link                │
          height band 0.20–1.20 m                │
          range_min 0.45                         │
                                  ┌──────────────┴───────────────┐
                                  ▼                              ▼
        scan_relay.py  mask 0.90 m               scan_relay.py  mask 1.30 m
                /scan   RELIABLE                        /scan_nav   RELIABLE
                  │                                          │
                  ▼                                          ▼
            slam_toolbox                                 Nav2 costmaps
      (mapping, or localization on a saved map)      (global + local obstacle layer)
                  │  /map  +  TF map->odom                   │
                  └──────────────────┬────────────────────────┘
                                     ▼
                     Nav2 servers ACTIVE (lifecycle), navigate_to_pose
```

Alongside, and required before anything drives:

```text
  bringup/safety.sh ──▶ twist_mux_node.py   the ONLY publisher of /cmd_vel
                    └──▶ arm_monitor_node.py ──▶ /safety/arm_stowed  (fail-closed)
```

**The two relays are not optional and not interchangeable.** `pointcloud_to_laserscan` publishes
BEST_EFFORT; `slam_toolbox` subscribes RELIABLE; incompatible DDS QoS delivers zero messages with
no error anywhere. And the two masks differ on purpose: `/scan` keeps returns out to 0.90 m astern
because a 2 m lift car puts its side walls ~1.00–1.15 m behind the robot and slam needs them;
`/scan_nav` masks to 1.30 m because a costmap that can see the robot's own tail can never reverse
into anything.

**A map is only valid for the chain that built it.** Change any number in this graph — the height
band, `range_min`, the artifact filter's thresholds — and every map built before the change is no
longer matched against the geometry the live sensor now produces. Rebuild them, or expect the fit
to fall. See `bringup/sensing_chain.sh`, which is where these numbers live.

---

## Symptom → cause

The eight that have actually cost sessions. Every one of them is silent by default; the
`bringup_all.sh` row and `WHY` block that catches each is named in the last column.

| # | Symptom you actually see | Cause | Fix | Caught by |
|---|---|---|---|---|
| 1 | A node is alive, in `ros2 node list`, and publishes **nothing, forever**. No error, no retry, no log line. | **It was started before its input existed.** Happened three times in one day: the cloud filter, `pointcloud_to_laserscan` and both relays were each launched into a vacuum. A ROS subscriber does not complain about a topic nobody publishes. | Start in dependency order and verify each stage by rate before the next. | Rows marked `BLOCKED`: a stage whose input is down is **not started**, and the row says which input. |
| 2 | `/scan` at **exactly 0.00 Hz** while the cloud flows, every node reports healthy, and nothing is printed anywhere. ~30 minutes, once. | **`base_link->os_sensor` is missing.** It is published by `bringup/lidar3d.sh` from the `mount` block in `config/ouster.yaml`, and by *nothing else*. Launch `ros2 launch ouster_ros driver.launch.py` directly and it never appears — then `pointcloud_to_laserscan` cannot transform into `target_frame base_link` and **drops every cloud in silence**. | `bash bringup/lidar3d.sh` — never the driver launch file directly. | `mount_tf` row. `base_link->os_lidar` is verified **before** p2l is started; if it does not resolve, p2l is not started at all. |
| 3 | Nav2 plans a perfect path and the robot does not move. `/odom` streams at full rate with **every velocity sample identically zero**. | **`can0` is absent or down.** No `/odom`, so no `odom->base_link`, so `slam_toolbox` cannot publish `map->odom` — and it surfaces three layers away as "localization is wrong in RViz". Bringing it up needs a password. | `sudo ip link set can0 up type can bitrate 500000` then `python3 bringup/claim_can.py` | `can0` row. Detected **before any ROS**, printed with the exact command, and the script **stops without starting anything** (exit 2). It never runs `sudo` and never waits on a prompt. |
| 4 | `192.168.1.221 unreachable` blocks the entire bring-up — for a mapping run that never touches the arm. Reads like a cable fault on the cable that also carries the lidar. | **`session.sh:66-68` pings the xArm and `die`s for every session type**, before the command word is even looked at. The arm is *normally* powered off for mapping and nav-only work. | Use `bringup_all.sh`, or `lab_gates.sh` (now fixed the same way). | `net_arm` row: `WARN` in `--mode map`/`nav`, `FAIL` only in `--mode full` or with `UTP_NEED_ARM=1`. Lidar (`.119`) and router (`.1`) stay fatal in every mode. |
| 5 | A probe reports **0.00 Hz for a topic that is demonstrably healthy** — twice in one day. Or `ros2 topic hz` reports **1.7 Hz and 10.0 Hz** for the same topic, minutes apart, against a repeatable 6.4 Hz. | **A fresh node per topic measures DDS discovery, not rate.** The opening seconds of any window are empty by construction. And `ros2 topic hz` is simply not trustworthy on this stack. | One node, every topic, spin ~3 s so discovery completes, **reset the counters**, then count over a window. | The probe in `bringup_all.sh` (and now `lab_gates.sh` gate 1 and `health.py`, both of which used to get this wrong). `/scan` and `/scan_nav` are probed **RELIABLE**, exactly as their consumers subscribe, so the probe fails the same way the consumer fails. |
| 6 | Every Nav2 goal comes back **`rejected in 0.0s`** while `ros2 node list` shows a healthy Nav2, `ros2 action list` **does** show `/navigate_to_pose`, and RViz shows an empty world. | **The lifecycle nodes are inactive.** The action server is advertised *before* activation; inactive costmap nodes publish nothing, which reads as an RViz configuration problem and is not one. `slam_toolbox` is the same: it comes up `unconfigured` and is indistinguishable from a hung node. Underneath, usually **two Nav2 stacks** — two `lifecycle_manager`s contending, activation never completing. | `ros2 lifecycle get /bt_navigator`. Configure **then** activate. Tear down the **nodes**, not just the `ros2 launch` wrapper, and start exactly one. | `nav2` and `slam` rows require `active`, compared **exactly** — `grep -q active` matches the substring in `inactive` and reports a dead Nav2 as healthy. That is this repo's signature bug in one word. |
| 7 | A TF gate that "passes" every time, or "fails" every time, and is believed either way. | **`timeout N ros2 run tf2_ros tf2_echo A B \|\| die` can never fail.** `tf2_echo` never exits, so `timeout` always returns 124. A zero-timeout `can_transform` has the same shape: it measures the probe's own subscription setup, because a latched `/tf_static` arrives only *after* discovery. | Grep the output for `Translation:`, or poll `can_transform` with a real budget. | The probe uses `tf2_ros.Buffer.can_transform` polled with a 10 s budget — a question that can actually come back "no". |
| 8 | A mapping node launched in the background is **gone minutes later**, and the drive with it. | **A process started from a shell that later exits dies with it.** A plain `nohup … &` is not enough. | `setsid nohup … < /dev/null &` and `disown`. | Every process `bringup_all.sh` starts, via `start_bg`. |

### Three more that share the same shape

| Symptom | Cause | Fix |
|---|---|---|
| Nav2 plans, publishes, and the robot does not move — for the full leg timeout, then reports `leg timed out`. | The **`arm_stowed` gate is blocking**. It is fail-closed and it has no measured evidence, because the arm is off. A navigation symptom for an interlock cause; days have gone into the planner for this. | Stow the arm so the monitor *measures* it (`python3 bringup/stow_arm.py --go`), or, if no arm is fitted or powered: `UTP_ARM_BACKEND=absent bash bringup/safety.sh` — and record that against any trial, because a gate satisfied by **declaration** is not one satisfied by **measurement**. Reported on the `safety` row as a duty cycle, because a gate that *flaps* looks fine when sampled once and still blocks most ticks. |
| Localization looks healthy and every waypoint is meaningless. | The map has a `.pgm`/`.yaml` but **no `.posegraph`**. `slam_toolbox` `mode: localization` does not error — it starts a new empty graph at the robot's feet, publishes `/map`, and reports `active`. `map` is then a fresh-SLAM frame wearing the saved map's name. | `--mode nav` refuses a map that is missing any of `.yaml`, `.posegraph`, `.data`, and names which. `bash bringup/map_persist.sh save <name>` writes all of them or none. |
| Two publishers on one topic; two drivers racing for one device; the loser logs `No RealSense devices were found`, which reads as a cable fault. | **Launching a second copy of something is a failure mode, not a harmless retry.** | Count before starting. `bringup_all.sh` never starts a component that already has a live process it cannot verify as its own — it reports the foreign pids and stops. |
| Every Nav2 leg is refused: `recorded in map X but the map currently loaded is Y`. Loud and correct — and you find out with the robot already standing there. | **The waypoints on disk belong to a different map.** Two maps' origins are unrelated, so the same numbers name a different physical place in each. | The `waypoints` row checks `maps/waypoints.yaml` against `--map` **offline, before the stack comes up**, and lists which maps the waypoints were actually recorded in. |

---

## What it does *not* do

- **It does not run `sudo`,** ever. `can0` is reported, with the command, and the run stops.
- **It does not kill anything it cannot prove it started.** Ownership is `UTP_ROBOT_STACK` plus
  `ROS_DOMAIN_ID` read from `/proc/<pid>/environ`, not a name, a topic or a frame — a frame-name
  match once killed 22 of the sim campaign's TF publishers, and a `pkill -f` has taken out the
  calling shell twice.
- **It does not move the robot.** Nothing here commands a twist. Gate 3 and above in
  `bringup/lab_gates.sh` are where motion starts, and they stay manual.
- **It does not save your map.** `slam_toolbox` holds the pose graph in RAM and serializes only on
  request. Start `bash bringup/map_insurance.sh start <name>` **before** the drive, and finish with
  `bash bringup/map_persist.sh save <name>`.

## One divergence you should know about

`bringup_all.sh` builds `/scan_nav` with a **second `scan_relay.py`** off `/scan_filtered` at a
1.30 m rear mask — the definition in `bringup/sensing_chain.sh`, and the one this repo's Nav2
params document. `bringup/stack.sh` and `bringup/session.sh` instead publish `/scan_nav` from
`safety/scan_temporal_filter.py`, fed from `/scan`. **Both cannot own the topic**: two publishers
on one topic interleave. If `scan_temporal_filter.py` is running, `bringup_all.sh` refuses to start
its relay, says so on the `scan_nav` row, and tells you to pick one. Pick one deliberately — and
remember that whichever you pick is part of the chain a map is only valid against.

## After it says everything is up

```bash
# mapping
bash bringup/map_insurance.sh start <name>     # BEFORE the drive. A drive that lives only in RAM
python3 bringup/map_watch.py                   # is a walk you will repeat.
bash bringup/map_persist.sh save <name>

# navigating
python3 bringup/relocalise.py --check          # want >= 80%
python3 bringup/waypoints.py list              # must be recorded in the map that is loaded
```

Then `bash bringup/lab_gates.sh 0 2` for the checks that move nothing, and read `docs/MORNING.md`
for the rest of the session.
