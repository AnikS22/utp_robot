# Start here

Robot project: `~/utp_robot`. Startup runbook: [docs/STARTUP.md](docs/STARTUP.md).

```bash
cd ~/utp_robot
bash bringup/inputs.sh up   # hardware inputs only; no localization
```

In an unmapped room, use `--mode inputs` to check sensors, camera, arm and safety without
starting SLAM, localization or navigation.

For navigation, use `--mode nav --map floor1` and supply the actual `SEED_POSE=x,y,yaw`.
Read the runbook's seed instructions; never reuse a pose from a different map.

Save with `bash bringup/map_persist.sh save <new_name>`, verify all four files, then stop with
`python3 bringup/stop_stack.py`. Do not reset or stop unsaved SLAM as a startup repair.

For a startup task, read the runbook and run the appropriate command. Open longer historical
documents only when a specific reported fault requires them.

Already running: `bash ~/utp_robot/bringup/inputs.sh check` saves a fresh input audit.
Power on the Ranger base before enabling CAN; the arm/lidar being reachable is not proof that
chassis power is on. See the runbook for the one-time-per-connection sudo CAN command.
