# Nav2 on this robot — what it is, and the parts that will confuse you

Nav2 turns *"go to this pose"* into wheel commands: it plans a route, follows it, avoids what
appears, and retries when stuck. It is **not one program** — it is five cooperating lifecycle nodes,
which is why bringup can half-fail and why its failures are usually silent rather than loud.

Pair this with `MAPPING.md`, which produces the map Nav2 needs.

---

## The four inputs. Miss one and it says nothing useful.

| Input | Where ours comes from |
|---|---|
| a **map** | `map_server`, loading the `.yaml`/`.pgm` from `MAPPING.md` |
| **TF `map → odom → base_link`** | slam_toolbox gives `map→odom`; the Ranger driver gives `odom→base_link` **only with `publish_odom_tf:=true`** (not the default) |
| **`/scan`** | the RPLIDAR — how it sees what the map does not contain |
| **`/odom`** | the Ranger driver's wheel odometry |

The two TF edges mean different things and the distinction is load-bearing:

- **`odom → base_link`** — smooth, continuous, and *drifts*. "Where am I relative to a moment ago."
- **`map → odom`** — the correction pinning you to the map. It **jumps** when localization
  re-converges.

That jump is why `PIPELINE.md` §7 runs the grounding → approach → press chain in **odom** and uses
`map` only for goals and metrics: an AMCL correction mid-press would move the target under the arm.

---

## The pieces

```
  goal ──► bt_navigator ──► planner_server  ──► a path through the map
               │                                        │
               │                                controller_server
               │                                        │
               │                                 /cmd_vel_nav ──► twist mux ──► wheels
               ▼
        behavior_server   (back up / wait, when stuck)
```

| Node | Job | Ours |
|---|---|---|
| `planner_server` | **global** route over the whole map | NavFn with `use_astar: true` — straighter paths in corridors |
| `controller_server` | **local** following + live avoidance at 20 Hz. *This is what drives.* | MPPI, `motion_model: Omni` (see the warning below) |
| `behavior_server` | recoveries | back up / drive-on-heading / wait. **`spin` deliberately removed** — the high-CoM base with the arm flips when it spins in place |
| `bt_navigator` | orchestrator running a behavior-tree XML: plan → follow → recover → replan | the `_no_spin` trees, to match the missing `spin` plugin |
| `map_server` | serves the static map | |

### Costmaps — the idea that makes the rest click

Planner and controller each keep a **costmap**: the map re-rendered as *how bad is it to be here*.
The **global** costmap covers the whole map for planning; the **local** one is a rolling 4×4 m window
rebuilt from live `/scan` for dodging.

The layer that matters is **inflation** — obstacles get a cost halo so paths do not hug walls:

- `inflation_radius: 0.30` — just above the inscribed radius + padding (0.28 m), so the robot can
  plan through any doorway it physically fits. **0.45 was tried and rejected**: on a real
  0.75–0.80 m doorway the inflation from *both jambs overlapped and closed it*, and in-room goals
  landed inside inflation and became unplannable. Raise this and Nav2 quietly decides every doorway
  is a wall. (The README table still says 0.45 — the YAML is the truth.)
- `cost_scaling_factor: 3.0` — gentle decay, keeping doorway and elevator centres navigable.
- footprint is the **real rectangle** `0.72 × 0.50 m`, not a circle, so it can thread real gaps.

### Lifecycle nodes — why bringup aborts entirely

Nav2 nodes go `unconfigured → inactive → active`, driven by `lifecycle_manager`. **If any single node
fails to activate, the manager aborts the whole bringup.** On 2026-08-20 `bt_navigator` could not
open its behavior-tree XML (the path was hardcoded to another developer's home directory) and so
*nothing* came up.

The line to look for is **`Managed nodes are active`**. Anything else means it did not start.

---

## Sending a goal

RViz: **2D Goal Pose**, click and drag. Or:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{
  pose: { header: {frame_id: 'map'},
          pose: {position: {x: 2.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}} } }" --feedback
```

It is a ROS **action**, not a topic: it streams feedback and returns success/failure.
On this robot, `bringup/nav2_goto.py <waypoint> --go` wraps the same call and is what
`RosWorld.navigate_to_goal` runs for every leg. It reports **`blocked`** when Nav2 exhausts its
recoveries — and *that* is the signal that starts the reason → ground → act → verify loop, not a
failure. Its exit codes are a contract: `0` a real outcome (parse stdout), `6` timeout, `2..5`
this backend cannot serve the request so the caller falls back to odom waypoints, `1` it crashed.
`utp/pipeline/navigation/nav2.py::Nav2Navigator` is the sim's twin of it.

---

## Five things specific to this robot

**1. Nav2 does not publish `/cmd_vel` here.** Both `controller_server` **and** `behavior_server` are
remapped to **`/cmd_vel_nav`**, because `config/safety.yaml` makes the twist mux the only permitted
`/cmd_vel` publisher. Consequence: **the mux must be running or Nav2 commands nothing** — the robot
sits still and no component reports an error. (`behavior_server` publishes its own `/cmd_vel`;
remapping only the controller leaves recovery behaviours driving the base around every interlock.)

**2. Nav needs the deadman; teleop does not.** Nav is `requires_enable: true`, so it needs
`/safety/enable` held **and** `arm_stowed` satisfied. Three fail-closed gates.

**3. `motion_model` is `DiffDrive`, not `Omni`.** MPPI-Omni emits strafe and yaw in one twist, and
the Ranger firmware **drops `angular.z` whenever `linear.y` is non-zero**. The sim can run Omni
because `utp/control/ranger_4ws.py` drives the four wheel joints directly — Isaac can, CAN cannot
(the protocol offers a body twist and no per-wheel interface). Set in the `nav2_params_os0*.yaml`
files. If doorway transit misbehaves, this is the first thing to re-open.

**4. `sensor_frame` is `base_link`, not `lidar_link`.** `/scan_filtered` is the Ouster cloud
projected by `pointcloud_to_laserscan` with `target_frame:=base_link`, and **there is no
`base_link -> lidar_link` TF on this robot at all**. With the old value Nav2's ObservationBuffer
drops every scan and the obstacle layer never marks or clears — costmaps stay empty and every path
looks clear, so the robot drives into a door at full confidence. This is the single most expensive
bug this stack has had.

**5. The OS0 sits at 1.146 m, so `/scan_filtered` starts at 0.50 m.** Close-in ground obstacles are
in a blind cone that no software setting fixes.

---

## Two map modes, and which one a campaign needs

| | rolling window (`nav2_params_os0.yaml`) | saved map (`nav2_params_os0_map.yaml`) |
|---|---|---|
| needs a map first | no | yes — grid **and** pose graph |
| global costmap | 24 m window travelling with the robot | the whole saved floor |
| a goal behind an unseen wall | planned straight through it, replanned when the wall appears | planned around it |
| coordinates portable between sessions | **no** — origin is wherever the robot booted | yes |
| use it for | a quick RViz click-and-drag check | **all 50 trials** |

`session.sh nav` runs the second. The first is only worth using to confirm Nav2 is alive at all
before a map exists.

---

## When it does not work

Nav2 fails quietly. Check in this order:

```bash
ros2 run tf2_tools view_frames                   # is map -> odom -> base_link complete?
ros2 topic hz /scan                              # lidar alive?
ros2 topic echo /local_costmap/costmap --once    # any non-zero cells? all zero == blind
ros2 topic info /cmd_vel_nav                     # is the controller publishing at all?
ros2 topic echo /safety/status                   # is the mux blocking, and why?
grep "Managed nodes are active" <launch log>     # did bringup actually finish?
```

### One ERROR at startup that is expected, not a fault

```
The inflation radius (0.300000) is smaller than the circumscribed radius (0.480104)
Inflation layer either not found or inflation is not set sufficiently ...
```

Nav2 wants inflation >= the circumscribed radius so it can skip full-footprint collision checks
when far from obstacles. Ours is deliberately smaller, because sizing it that way closes real
doorways (above). The cost is **planning speed, not safety** — full-footprint checking still runs,
it just runs more often. Do not "fix" this by raising `inflation_radius`; that trades a working
robot for a faster planner.

**The most common failure is a missing TF.** The costmap's message filter then drops *every* scan and
logs nothing at normal verbosity, so the obstacle layer stays empty and the robot drives confidently
into walls. It presents as a planner bug and is not one.
