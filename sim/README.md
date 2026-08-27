# sim/ — running the benchmark workflow in Isaac Sim

Same executor, same routes, same safety mux, same grounder as hardware. Only the arm action
differs (sim IK via `/arm_reach` vs the xArm SDK). Sim is **ROS_DOMAIN_ID=42**; hardware is 9, and
they must stay apart so a sim test can never publish a twist at the real chassis.

    bash sim/trial_server.sh                    # 1. Isaac + bridge (~90 s to SERVER_UP)
    ROS_DOMAIN_ID=42 ros2 topic pub --once /scene/command std_msgs/msg/String \
        '{data: "{\"cmd\":\"build\",\"scene_type\":\"button_door\",\"seed\":1}"}'
    bash sim/safety_sim.sh                      # 2. the REAL mux + arm monitor (ONE instance!)
    UTP_WAYPOINTS=$PWD/maps/waypoints_sim.yaml ROS_DOMAIN_ID=42 \
        python3 sim/make_sim_waypoints.py       # 3. waypoints from scene geometry
    UTP_WAYPOINTS=$PWD/maps/waypoints_sim.yaml UTP_SIM=1 ROS_DOMAIN_ID=42 \
        python3 bringup/route_run.py benchmark_sim --go

`UTP_WAYPOINTS` is mandatory for sim: without it the sim run would read and CLOBBER the hardware
waypoints, and sim poses in the real building drive the robot into a wall.

## Files

| | |
|---|---|
| `trial_server.sh` | launcher. Sources system Jazzy FIRST (else the ROS2 bridge fails to start) and points `ISAAC_ASSETS_ROOT` at NVIDIA's CDN |
| `trial_server_patched.py` | COPY of the sim repo's trial_server with 2 deltas, both tagged `UTP-LAPTOP`: repo path, and `rep.orchestrator.run()` after play (without it every rendered frame is a blank buffer) |
| `build_robot_usd.py` | COPY of the sim repo's builder; rebuilds the gitignored `ranger_xarm6_full.usd` from the committed configuration USDs |
| `safety_sim.sh` | the real `twist_mux_node` + `arm_monitor_node --backend scene_state` on domain 42 |
| `make_sim_waypoints.py` | benchmark waypoints from the scene's layout constants, mapped world -> odom |
| `sim_press.py` | the press action, sim edition: same grab -> ground -> refuse-without-a-3D-point, then `/arm_reach` instead of the xArm SDK |

The sim repo itself is NEVER edited (CLAUDE.md) — everything here is a copy.

## Run exactly ONE safety stack

Two `safety_sim.sh` instances = two publishers on `/cmd_vel` = the same class of bug that cost
2026-08-26 on hardware. Check with `ros2 topic info /cmd_vel` — Publisher count must be 1.

## Known-open

`/safety/arm_stowed` flaps in sim: `stale_after_s: 0.5` vs a MEASURED `/scene/state` rate of
0.55 Hz under headless RTF, so the gate collapses between messages and the mux blocks with
`arm_not_stowed`. Fix is per-backend staleness. See EXPERIMENT_LOG.md 2026-08-27 — the same
failure mode is live on hardware whenever the xArm SDK session drops.
