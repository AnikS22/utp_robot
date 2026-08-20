# Real-robot experiment log

Running log for the physical Ranger Mini 3.0 + xArm6 benchmark. Append-only, newest entry at the
bottom. Deadline **2026-08-25**.

**Rules for this log**
- One entry per working session, dated. Record what was *observed*, not what was expected.
- A gate is only GREEN when someone watched it pass. "Should work" is not a result.
- Negative results are entries too — the four measurements that did not survive checking are worth
  more than the ones that did.
- If a number gets measured (riser height, stop distance, press tolerance), it goes here with units
  and how it was measured.

**Status key:** `OPEN` not started · `WIP` in progress · `GREEN` verified by observation ·
`RED` tried and failed · `CUT` descoped

---

## Gates

| ID  | Gate | Status | Notes |
|-----|------|--------|-------|
| S0  | Site survey: are the ADA doors motion-activated or push-open? | `OPEN` | **Top risk.** If they open without a button press, `passive` succeeds and R1 measures nothing. Falsifiable in one visit. |
| S1  | Site survey: glass doors? lidar-visible? | `OPEN` | 2D lidar sees through glass — safety issue, not just data quality. |
| S2  | Riser height measured | `OPEN` | Goes into `base_link → link_base`. Unmeasured = every press misses vertically by exactly this. |
| S3  | D455 returns valid depth on the ADA push plate | `OPEN` | Brushed metal is near worst-case for stereo. Failure looks like a calibration bug. |
| H0  | Which computer is the robot's brain | `OPEN` | Workstation (GPUs, tethered) vs onboard Jetson. Decides where every package installs. |
| H1  | `can0` up, `RangerStatus` clean, base moves on `/cmd_vel` | `WIP` | Also: **does the driver time out a stale twist?** Unverified. If it holds the last command, that is a runaway. |
| H2  | Lidar publishing, `lidar_link` TF correct, scan matches the room | `WIP` | **/scan is publishing** (2026-08-18). Still open: `base_link->lidar_link` TF, and scan direction / zero-angle vs the physical robot. |
| H3  | Arm reaches commanded Cartesian points repeatably | `RED` | xArm-Python-SDK direct, not MoveIt. SDK Cartesian API is **millimetres**; our stack is metres. |
| H4  | Safety stack verified in sim (mux + interlock + failsafes) | `WIP` | Arbiter logic done and unit-tested; not yet exercised against a running robot. |
| H5  | Hand-eye calibration | `OPEN` | |
| H6  | Empty-sim testbed: proposed motions + takeover rehearsed | `OPEN` | |

---

## 2026-08-17 — safety stack, first code

**Context.** Decision taken that the real-robot work lives in its own tree (`real_world/`) and does
not edit the simulation stack. Anything needed from `utp/` gets copied here, not modified in place.

**Finding: there was no base/arm interlock anywhere in the codebase.** Searched for it explicitly.
The only mutual exclusion is `isaac_worker/trial_server.py:1814`, which rejects a *new*
`/arm_reach/goal` while the arm is busy — that protects the arm from the arm. The `/cmd_vel`
handler at `trial_server.py:1964` processes twists unconditionally, and `arm_state` is published in
`/scene/state` but never read by anything able to stop the base.

Base motion never coincided with an extended arm only because `act()` is written sequentially
(approach → send goal → block on result → retreat). **Safety was emergent from control flow.** In
sim that holds. On hardware, control flow is exactly what stops being trustworthy: a Nav2 recovery
behaviour, a stale twist, a pipeline crash mid-press, or a Ctrl-C at the wrong moment all produce
base motion without passing through `act()`.

**Built.** `real_world/safety/` — a priority twist mux with fail-closed interlocks, plus the arm
monitor that feeds it. No Isaac dependency, no `utp` dependency; the same files run in sim and on
hardware. `real_world/config/safety.yaml` holds the wiring and the limits.

Layering, for the record. Software is layer 2:
- **Layer 0** — chassis + arm-controller hardware E-stops. Cut motor power; nothing overrides them.
- **Layer 1** — Ranger RC transmitter. Revokes CAN command authority at the driver, below anything
  software can touch. **Whoever stands next to the robot holds the RC.** This is the takeover path.
- **Layer 2** — this mux. Covers the failures that happen faster than a human reacts.

**Bug found and fixed during testing.** On the very first tick `dt = 0`, and the slew limiter
returned the target unchanged — so a mux restarting while Nav2 was already commanding full speed
would slam to full speed with no ramp. That is precisely the hard-acceleration case the raised CG
cannot take. `dt <= 0` now holds the current value, and the first tick assumes the nominal publish
period so slew limiting is in force from tick one. Regression test:
`test_first_tick_does_not_jump_to_full_speed`.

**Verified.** 23/23 in `real_world/tests/test_safety_arbiter.py`
(`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest real_world/tests/ -q`; the env var is needed
because ROS's `launch_testing` pytest plugin fails to import — missing `lark` — unrelated to us).
Covered: arm-extended blocks motion; stale arm gate blocks; never-seen gate blocks; interlock
applies to teleop too; override permits crawling recovery for teleop only; teleop preempts nav;
stale teleop yields back; deadman gates autonomy but not teleop; commander death stops the base;
E-stop latches through a topic flap and stops hard without ramping; speed and slew ceilings.

**Not yet verified — this gate is `WIP`, not `GREEN`.** The logic is unit-tested; the *nodes* have
never run. Nothing here has been exercised against a moving robot, in sim or otherwise. H4 stays
`WIP` until someone watches an interlock stop a robot that was actually moving.

**Environment note.** Nothing hardware-related is installed on this workstation: no `can0`, no
`ranger_ros2`/`realsense2_camera`/`rplidar_ros`, no `xarm` Python module. Only `nav2_bringup` and
`slam_toolbox` under `/opt/ros/jazzy`. All three driver chains are greenfield installs.

**Next.** Empty-sim testbed (H6) to rehearse the proposed motions, the takeover, and each failsafe
against a robot that is actually moving — physics + `/cmd_vel` path, *not* the kinematic drive in
`teleop_office.py`, which bypasses PhysX and so cannot show tipping, twist truncation, or whether
an interlock stopped anything.

---

## 2026-08-18 — hardware plugged in; first device-level bring-up

All three devices connected: arm via Ethernet, chassis via USB-CAN, lidar via microUSB.

### Enumeration

| Device | Bus ID | Kernel | Node | State |
|---|---|---|---|---|
| USB-CAN adapter | `1d50:606f` Geschwister Schneider (candlelight) | `gs_usb`, `can_dev` | `can0` | present, **DOWN/STOPPED** |
| RPLIDAR | `10c4:ea60` Silicon Labs CP2102 | `cp210x`, `usbserial` | `/dev/ttyUSB0` | **responding** |
| USB Ethernet (for arm) | `0bda:8153` Realtek RTL8153 | `r8152` | `enx00e04c674c60` | **NO-CARRIER** |

`can0` clock 48 MHz, `brp 1..1024` — fine for 500 kbps. `can-utils` (`candump`/`cansend`) already
installed. User is in `dialout`, so the lidar needs no permission work. Stable lidar path exists:
`/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0` — use
that in the launch, never `/dev/ttyUSB0`, which reorders as soon as a second serial device appears.

### H2 (partial) — lidar identified and healthy `GREEN` for the sensor itself

Probed directly over raw serial with `real_world/bringup/probe_rplidar.py` — no ROS, no driver, no
sudo. Repeated cleanly:

```
health   : GOOD  error_code=0
model    : 0x18          <- A1M8, as expected
firmware : 1.29
hardware : 7
serial   : 6FB2ED93C0EA98C9C2E29EF59D68406E
port     : /dev/ttyUSB0 @ 115200 baud  -> OK
```

**115200 confirmed as the correct baud.** This matters because the wrong baud on an RPLIDAR gives a
silent no-data start rather than an error, which would have looked like a dead sensor.

Still OPEN for H2: `lidar_link` TF, and scan direction / zero-angle convention against the physical
robot. Sensor health says nothing about either, and the missing `lidar_link` TF already caused a
door ram once.

**Caveat worth recording.** Probing is only ~70–90% reliable *across process restarts*; a bad
process reads zero bytes for its entire life, while within one open session it is 20/20. Forcing
DTR/RTS either way and settles up to 1 s changed nothing beyond noise (n=12). Root cause not
established — it is in the CP2102 reopen path, not the sensor, which reports identical health and
serial every time it answers. The ROS driver holds the port open continuously (the 20/20 regime),
so this affects hand-probing only. The probe now reopens on failure: 15/15.

Time spent chasing this: two wrong theories (`stty`-before-`open` race, DTR line state), both
disproved by measurement. Logged so nobody repeats it.

### H1 — blocked on sudo

`can0` exists but is STOPPED. Bringing it up needs root:

```
sudo ip link set can0 up type can bitrate 500000
candump can0            # expect Ranger heartbeat frames with the chassis powered
```

`candump` showing traffic verifies wiring, bitrate and chassis power in one step, before any
driver is installed.

### H3 — RED, physical problem

`enx00e04c674c60` reports **Link detected: no**, speed unknown, no carrier. The other onboard port
`enp37s0f0` is also NO-CARRIER (`enp37s0f1` is the workstation's live network). So no matter which
port the arm is patched into, there is no physical link. Cannot reach the control box until this
comes up.

To check, in order: is the arm control box powered on; is the cable fully seated at both ends; do
the link LEDs light. Once carrier appears, the host still needs a static address on the control
box's subnet (uFactory ships 192.168.1.x) before `XArmAPI` can connect.

---

## 2026-08-18b — lidar interfaced and publishing `/scan`; repo split out

### Repo split

This tree is now a standalone git repo at `~/Desktop/utp_robot`, so it can be committed and cloned
onto the laptop that rides on the rover. It no longer lives inside `Unlocking_the_path`. `ros2_ws/`
is gitignored — `bringup/setup_workspace.sh` rebuilds it from pinned commits, which is what keeps
the rover laptop and the workstation identical. Verified: a from-scratch run builds all 5 packages,
and a second run is a clean no-op.

### Driver workspace

Built against ROS 2 Jazzy: `rplidar_ros`, `ugv_sdk`, `ranger_msgs`, `ranger_base`, `ranger_bringup`.

**Correction to an earlier note.** It said `ranger_ros2` is "humble-branch-only". Wrong — a `jazzy`
branch exists. But the correct branch for us is still **humble**, for a different and more important
reason: **the jazzy branch does not support the Ranger Mini V3.** Its `RangerSubType` enum stops at
`kRangerMiniV2` and it has no `ranger_mini_v3.launch.py`; only humble has `kRangerMiniV3`. Building
the "right" branch for our ROS distro would have silently cost us V3 support.

**Two build obstacles, both solved without root:**

1. *conda shadows ROS's python.* colcon runs `package_xml_2_cmake.py` with the first `python3` on
   `PATH`; conda's has no `catkin_pkg`, so **every** package fails at `ament_package()` with an
   opaque "returned error code 1" that says nothing about python. Both scripts scrub conda from
   `PATH` themselves.
2. *`asio.hpp` missing.* `ugv_sdk` needs standalone asio (not boost::asio); Ubuntu ships it as
   `libasio-dev`, which needs `sudo`. It is header-only, so `setup_workspace.sh` vendors
   `asio-1-28-0` and passes the include path. No password needed.

### H2 — `/scan` is live

```
messages       : 15 over 1.96s  -> 7.14 Hz
frame_id       : lidar_link
angle span     : -179.0 .. 180.0 deg
beams          : 360  (incr 1.000 deg)
range_min/max  : 0.15 .. 12.00 m
valid returns  : 87/360   min=0.16  max=9.00 m
current scan mode: Standard, sample rate: 2 Khz, max_distance: 12.0 m
```

Note the rate is **7.14 Hz**, not the 10 Hz the driver is configured for — normal for an A1M8,
whose motor is unregulated. `config/sensors.yaml` in the sim repo assumes 5.5 Hz. Neither number is
wrong, but Nav2 costmap expectations should be set from the measured value, not the configured one.

**The bug that blocked this, and it was not the obvious one.** The driver connected fine (correct
S/N, firmware, health OK) and then failed at scan start with `0x80008000` (INVALID_DATA) or
`0x80008002` (TIMEOUT). The obvious reading — dead sensor, bad cable, unpowered motor, wrong baud —
was wrong in every case. Probing the raw serial protocol by hand showed the legacy `SCAN` command
(`0xA5 0x20`) streaming 17 KB of perfectly good measurements with distances 0.16–15.59 m. So the
sensor, motor and link were all fine.

The actual cause: **our A1M8 runs firmware 1.29, which predates the scan-mode negotiation
protocol.** `getAllSupportedScanModes()` returns `0x80008004` (NOT_SUPPORT) and the express-scan
path returns garbage, but the bundled SDK (2.1.0) assumes newer firmware and has no fallback.
`patches/rplidar_ros-legacy-scan.patch` adds a `legacy_scan` parameter that calls
`startScan(..., useTypicalScan=false)`, i.e. the legacy command. 13 lines.

Generalisable lesson: a device that answers `GET_INFO` and `GET_HEALTH` is *not* a device that will
scan. Identity, health, and capability are three separate questions, and the SDK conflated the third
with firmware age.

**Also fixed today, in our own tooling:** `probe_rplidar.py` was ~70–90% reliable across process
restarts, and I chased two wrong theories (an `stty`-before-`open` race, then DTR line state) before
measurement killed both — in-session it is 20/20, so the fault is purely in the reopen path and is
still not root-caused. The probe now reopens on failure: 15/15. Flaky diagnostics are worse than no
diagnostics, because they send you debugging the wrong subsystem.

### Still open for H2

- `base_link -> lidar_link` static TF. **`/scan` publishing is not the same as Nav2 being able to
  use it** — the missing TF is what once left the costmap blind and rammed a door.
- Scan direction and zero-angle convention against the physical robot. A mirrored scan builds a map
  that looks plausible and navigates catastrophically. Cannot be checked from a bench; needs the
  lidar mounted on the rover in a room with known geometry.
- The 24% valid-return rate (87/360) is unremarkable on a bench pointing into open space, but should
  be re-measured in the actual corridor.

### Hardware note

The **USB-CAN adapter was unplugged** during the lidar re-seat — `1d50:606f` is gone and `can0` with
it. It shared the hub. Needs re-plugging before any Ranger work. The lidar moved to a better spot in
the process: it is now behind a single hub on bus 003 rather than two chained hubs on bus 005.

---

## 2026-08-18c — isolation, process hygiene, and the laptop plan set

### Incident: I killed 22 of the sim campaign's TF publishers

While cleaning up my own orphaned lidar processes I matched every `static_transform_publisher`
whose `--child-frame-id` was `lidar_link` and killed them. The sim's Nav2 launches publish
`RPLidar_S2E -> lidar_link` — **same child frame**. 28 processes killed; only 6 were mine.

That alias is precisely the transform whose absence makes the costmap MessageFilter drop every scan
and leaves the obstacle layer empty. Trials in flight on domains 137–225 from ~15:0x are
compromised. Full write-up and restore commands: `INCIDENT_2026-08-18_tf_kill.md` in the sim repo.
The MAIN session was in `shell` state and unreachable via SendMessage, hence the file.

**Rule adopted:** never scope a kill by frame name, topic name, or any other shared identifier.
Scope by full command line AND by the executable living under this repo.

### Domain isolation

`ROS_DOMAIN_ID=9` is now reserved for hardware, set centrally in `bringup/env.sh`. My first pick
was 43 — wrong twice over: the sim campaign documents 42/43 as **poisoned** (`run_campaign_gpu.sh`
BUG #6), and it walks domains upward through the 137–225 band toward the 232 DDS ceiling. 9 is
low, free, and outside the walk.

`bringup/preflight.py` enforces it, turning a silent collision into a loud abort. It also detects a
stale process holding the lidar's serial port, because that failure presents as
`SL_RESULT_OPERATION_TIMEOUT` and looks exactly like a hardware fault.

**Verified:**
```
domain 9   : /utp_robot_lidar_tf, /utp_robot_rplidar
domain 208 : 18 nodes, all the sim's — no utp_robot node visible
shutdown   : no orphans, serial port released
```

### Two bugs of mine, both fixed

**`exec` killed the cleanup trap.** `lidar.sh` ended with `exec ros2 run ...`, which replaced the
shell, so the `trap` never fired and every run leaked a node holding the serial port. That is where
the orphans came from in the first place — and therefore, indirectly, the incident above.

**Killing `ros2 run` does not kill the node.** `ros2 run` is a python wrapper that execs the real
binary as a *child*; killing the pid we hold leaves the node alive. Children are now started with
`setsid` and cleanup kills the whole **process group**. Verified: no survivors.

**Also fixed:** the preflight guard was too strict — it counted its own invoking shell and the
per-domain `ros2` CLI daemon as foreign collisions and refused to let `lidar.sh` start at all.

### Laptop plan set

The rover laptop is a **Dell Pro Max 16 Plus** (mobile workstation, discrete NVIDIA RTX Pro). That
settles the architecture: **the full pipeline runs onboard**, so there is no wireless link inside
the reason→ground→act→verify loop.

Written for the laptop's Claude Code agent: `CLAUDE.md` (auto-loaded entry point) plus
`docs/AGENT_BRIEF.md`, `docs/LAPTOP_SETUP.md`, `docs/HARDWARE_SPECS.md`, `docs/CALIBRATION.md`.

Three findings that came out of writing them, each of which would have cost a day at the site:

1. **The pinned CUDA stack will not work on this laptop.** `env/requirements-perception-cuda.txt`
   pins `torch==2.5.1+cu121` for the workstation's RTX 6000 Ada (sm_89). Blackwell is sm_120 and
   cu121 builds do not support it — torch would fall back to CPU *silently*, turning a 0.2 s
   grounding call into ~20 s that looks like a network problem. The laptop needs cu128+.
2. **`config/detectors.yaml` assumes three GPUs** (`gdino: cuda:0`, `owlv2: cuda:1`,
   `clip: cuda:2`). The laptop has one. Every backend must be overridden to `cuda:0`.
3. **The VLM endpoint is a university HPC service** (`chat-llm.hpc.fau.edu`) and may need the
   campus network or a VPN. The test site is a building with unknown connectivity. This must be
   verified *from the test site* before it becomes a blocking discovery on the day.
