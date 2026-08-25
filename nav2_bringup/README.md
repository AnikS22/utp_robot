# nav2_bringup/ — Nav2 for the AgileX Ranger Mini 3 base (M1.2)

Standard Nav2 (ROS2 **Jazzy**, Nav2 1.3.x) bringup configured for the Ranger Mini 3 4WS base.
Runs as a separate py3.12 process that consumes what the Isaac worker publishes over the ROS2 / DDS
bridge and emits `/cmd_vel`. Built to `docs/integration_contract.md`.

## Files
- `nav2_params.yaml` — costmaps, planner, controller, behaviors tuned for the Ranger footprint.
- `ranger_nav.launch.py` — brings up the 5 lifecycle nodes + lifecycle manager + map server
  + the `lidar_link` alias TF (see gotchas).
- `behavior_trees/*_no_spin.xml` — copies of the default nav-to-pose / nav-through-poses BTs with the
  `<Spin>` recovery removed (the high-CoM base flips when Nav2 spins in place). `behavior_plugins`
  omits `spin` to match; `bt_navigator` points at these via `default_nav_*_bt_xml`.
- `map_to_odom_publisher.py` — broadcasts the ground-truth `map → odom` from `/scene/state`
  `robot_pose` (run alongside the launch; NOT auto-started by it).
- `maps/placeholder_map.yaml` (+ `.pgm`) — a 6 m × 6 m empty room so the stack activates standalone.
  The scene assembler exports the real maps to `runs/maps/<scene>.yaml`; override the `map` arg.

## GOTCHAS (load-bearing — fixed 2026-07-17)
- **`lidar_link` has no native TF.** Isaac stamps `/scan` `frame_id="lidar_link"`, but
  `ROS2PublishTransformTree` names the lidar TF after its USD prim (`RPLidar_S2E`). Without a bridge,
  the costmap `MessageFilter` drops **every** scan (`"timestamp earlier than transform cache"`), the
  obstacle layer stays empty, and the robot drives **blind into walls/doors**. The launch now
  publishes a static identity alias `RPLidar_S2E → lidar_link` (`lidar_link_tf`). Verify with:
  `ros2 topic echo /local_costmap/costmap` → non-zero lethal cells.
- **No `spin` recovery.** The default BTs' recovery RoundRobin calls the `spin` action; with `spin`
  removed from `behavior_plugins`, that XML fails to load and the whole bringup aborts. Use the
  `_no_spin` trees (already wired in `nav2_params.yaml`).

## What it consumes / produces
| direction | topic / interface | type | notes |
|---|---|---|---|
| consumes | `/scan` | sensor_msgs/LaserScan | RTX 2D lidar, obstacle layer (global+local), `lidar_link` frame |
| consumes | `/odom` | nav_msgs/Odometry | bt_navigator + local costmap |
| consumes | `/tf`, `/tf_static` | tf2_msgs/TFMessage | `map → odom → base_link` (published by Isaac; **no AMCL here**) |
| consumes | `map` | nav_msgs/OccupancyGrid | from the bundled map_server |
| **produces** | `/cmd_vel` | **geometry_msgs/Twist** | controller output (unstamped — see below) |
| produces | `/plan`, costmaps | nav_msgs/Path, OccupancyGrid | debug/introspection |
| **action** | `/navigate_to_pose` | nav2_msgs/NavigateToPose | primary goal interface (also `/navigate_through_poses`) |

Localization is **not** run here: per the contract the `map → odom` transform is published by Isaac
(`/scene/state` reports `"localized":true`), so the bringup omits AMCL and just serves the static map.

## Launch

This directory is not an installed ament package, so launch it by path:

```bash
source /opt/ros/jazzy/setup.bash          # Nav2 must be installed: sudo apt install ros-jazzy-navigation2
ros2 launch /path/to/Unlocking_the_path/nav2_bringup/ranger_nav.launch.py \
    map:=/path/to/Unlocking_the_path/runs/maps/<scene>.yaml
# standalone smoke test (placeholder map, wall clock):
ros2 launch .../ranger_nav.launch.py use_sim_time:=false
# or, without ros2 launch:
python3 .../ranger_nav.launch.py
```

Launch args: `map` (default placeholder), `params_file` (default `nav2_params.yaml`),
`use_sim_time` (default `true` — Isaac drives `/clock`), `autostart` (default `true`), `log_level`.

## Send a NavigateToPose goal (CLI)

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{
  pose: {
    header: { frame_id: 'map' },
    pose: {
      position: { x: 1.0, y: 0.0, z: 0.0 },
      orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }
    }
  }
}" --feedback
```

The pipeline side (`utp/pipeline/navigation/nav2.py`, `Nav2Navigator`) wraps this same action and
reports `blocked` when no path is found within a window.

## Non-default params and why (Ranger-specific)

| param | value | rationale |
|---|---|---|
| `global/local_costmap.footprint` | `[[0.36,0.25],[0.36,-0.25],[-0.36,-0.25],[-0.36,0.25]]` | exact 0.72 m (x) × 0.50 m (y) rectangular body, `base_link` at center (vs the default `robot_radius` circle) |
| `footprint_padding` | `0.03` | small uniform safety pad around the body in tight corridors |
| `inflation_layer.inflation_radius` | `0.45` | > inscribed radius (0.25 m) so the planner respects body width, but < ~0.5 m so ~1 m doorways stay passable |
| `inflation_layer.cost_scaling_factor` | `3.0` | gentle cost decay → keeps doorway/elevator centers navigable instead of walling them off |
| `controller FollowPath plugin` | RegulatedPurePursuit | Twist-friendly (vx + yaw only); the 4WS mode controller in Isaac (`utp/control/ranger_4ws.py`) maps that Twist to per-wheel (steer, speed) — Nav2 must not assume free holonomic motion |
| `use_rotate_to_heading` | `true` | exploits the Ranger **spin** mode (0-radius) to align in place before driving; `allow_reversing:false` (mutually exclusive) |
| `regulated_linear_scaling_min_radius` | `0.9` | ≈ the base's Ackermann min turn radius (`config/robot.yaml` 0.81 m) — slow down on tight curves |
| `desired_linear_vel` | `0.50` m/s | conservative indoor cruise, well under the 1.5 m/s operating cap from `config/ranger_kinematics.yaml` |
| `enable_stamped_cmd_vel` | `false` | **contract requires `/cmd_vel` = geometry_msgs/Twist (unstamped)**; Nav2 Jazzy defaults to TwistStamped. Set on `controller_server` and `behavior_server` |
| `obstacle_layer scan.sensor_frame` | `lidar_link` | matches the lidar frame in the contract TF tree |
| `planner GridBased.use_astar` | `true` | straighter global paths than the Dijkstra default in narrow corridors |
| `*.use_sim_time` | `true` | Isaac publishes `/clock`; overridable via the `use_sim_time` launch arg |
| local costmap `width/height` | `4 × 4` m | enough lookahead for 1 m corridors while staying cheap to roll |

Planner is `nav2_navfn_planner::NavfnPlanner` (robust, minimal config surface). For smoother feasible
paths, swap `GridBased.plugin` to `nav2_smac_planner::SmacPlanner2D` (requires `ros-jazzy-nav2-smac-planner`).

## Verification status (standalone, no live sim)

Verified on ROS2 Jazzy with `ros-jazzy-navigation2` available:
- `ros2 launch ranger_nav.launch.py use_sim_time:=false` brings up all 5 lifecycle nodes
  (`map_server`, `planner_server`, `controller_server`, `behavior_server`, `bt_navigator`) and the
  lifecycle manager auto-configures + activates them: every node reaches `active [3]`, manager logs
  **"Managed nodes are active"**, no plugin-load or config errors.
- `ros2 action list` shows `/navigate_to_pose` (and `/navigate_through_poses`).
- `ros2 topic info /cmd_vel` → `geometry_msgs/msg/Twist` (confirms the unstamped contract).
- Both costmaps subscribe to `/scan` (LaserScan); StaticLayer loads the placeholder map (120×120 @ 0.05 m).

**Integration step (needs another agent):** full path execution requires the Isaac trial server's live
`/scan`, `/odom`, `/tf`, and the 4WS `/cmd_vel` consumer. This bringup is the Nav2 half of that
contract and is validated standalone; end-to-end navigation is exercised once the Isaac worker is up.
