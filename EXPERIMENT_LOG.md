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
| S2  | Riser height measured | `GREEN` | **391.225 mm** (CAD, tape-checked), base plate horizontal so it is a pure translation. `link_base` at ~0.740 m off the floor. Makes the 1.067 m ADA hall call reachable. |
| S3  | **D435** returns valid depth on the ADA push plate | `OPEN` | Brushed metal is near worst-case for stereo. Failure looks like a calibration bug. **The camera is a D435, not the D455 in the docs** (measured FOV 70.2x43.2 deg, 2026-08-20c) — narrower view, but a closer minimum depth. |
| H0  | Which computer is the robot's brain | `GREEN` | **Decided 2026-08-20c: the rover laptop.** Perception installed and measured on it (gdino ~210 ms/call, sm_120 confirmed). |
| H1  | `can0` up, `RangerStatus` clean, base moves on `/cmd_vel` | `GREEN` | **Both stale-command gates measured 2026-08-21d.** Driver: PASS, holds nothing (`0x111` stops dead on publisher death). Firmware: chassis coasts **1.26 s / ~18 cm** before its own watchdog fires. Reached only on SIGKILL of the mux, since an explicit zero stops the base at once and the mux publishes every tick. Residual risk bounded and documented. |
| H2  | Lidar publishing, `lidar_link` TF correct, scan matches the room | `WIP` | `/scan` 6.6 Hz, 360 beams, `base_link->lidar_link` published, angle increment **positive (CCW, correct ROS convention)** (2026-08-21). Still open: scan direction / zero-angle against the physical robot, and a **33° dead arc at −102°..−69°** plus ~17 beams pinned at `range_min` behind — self-occlusion, unconfirmed. |
| H3  | Arm reaches commanded Cartesian points repeatably | `WIP` | Ethernet link up 2026-08-20c: `192.168.1.221`, fw v2.6.0, servos live. Not yet commanded to move. SDK Cartesian API is **millimetres**; our stack is metres. |
| H4  | Safety stack verified in sim (mux + interlock + failsafes) | `WIP` | Mux + interlock exercised against the real base 2026-08-20c (clamped 0.5 -> 0.15 m/s under override). The 2026-08-20 runaway is now **explained**: gates H1 show nothing latches, so it was continuous commanding from a stale held-key belief — the hold lease addresses exactly that. Still to do on the robot: kill the teleop UI mid-drive and measure the actual stop time end to end. |
| H5  | Hand-eye calibration | `GREEN` | **Solved and verified 2026-08-21e.** rms 3.0 mm, worst 8.2 mm. Independently validated: solved camera x -327.6 mm vs -324.2 mm off the CAD. End-to-end placement **4.3 mm mean / 9.7 mm worst** over 6 targets — 3x margin on the ±30 mm an ADA plate allows. |
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

---

## 2026-08-20c — rover laptop provisioned; all four devices up; base drove; **base ran away**

First session on the actual rover laptop (bare Ubuntu 24.04.4: no ROS, no git, no pip). Every
device brought up, the base driven under teleop, and one serious safety failure.

### Enumeration and bring-up

| Device | Result |
|---|---|
| RPLIDAR A1M8 | **GREEN.** fw 1.29, SN `6FB2ED93C0EA98C9C2E29EF59D68406E` — the same unit `HARDWARE_SPECS.md` documents. `/scan` measured **6.95 Hz**, 360 beams @ 1.000°, 0.15–12.0 m. The `legacy_scan` patch works exactly as written. |
| xArm6 | **Reachable.** `192.168.1.221`, SN `XI1305`, fw v2.6.0, 6 axes. TCP load 0.82 kg already configured. |
| RealSense | **Streaming**, but it is a **D435, not the D455** in every doc. |
| Ranger Mini 3 | **GREEN.** `can0` up at 500000, 28 CAN ids at ~48 Hz, battery 49.7 V, no faults. |

### Findings that cost time, so they are written down

**The USB-CAN adapter drops off a shared hub.** Three plug-ins today: on a **direct** laptop port it
enumerated and stayed; on the hub it enumerated correctly (`gs_usb` bound, "Configuring for 1
interfaces") and **disconnected 2–3 s later, twice** — once three seconds after the lidar joined the
same hub. `HARDWARE_SPECS.md` already warns "separate direct ports, not a shared hub"; this is that
warning reproducing. Not a software fault: enumeration succeeds completely every time.

**The camera is a D435.** Colour FOV **measured from the streamed intrinsics: 70.2° × 43.2°**, against
the 90° × 65° the docs assume for a D455 — a third less horizontal view, so a plate the sim assumed
was in frame may not be. It also has **no IMU**. Partly compensating: the D435's minimum depth is
closer (~0.28 m vs ~0.52 m), which helps at press range. Recorded in `config/camera.yaml`.

**`ranger_msgs/msg/SystemState`'s constants are WRONG.** It declares `CONTROL_MODE_RC=0,
CONTROL_MODE_CAN=1` and `MOTION_MODE_SIDE_SLIP=3`. The SDK the driver actually speaks
(`ugv_sdk/include/ugv_sdk/details/interface/agilex_types.h`) says `STANDBY=0, CAN=1, UART=2, **RC=3**`,
and `ranger_interface.hpp` says `kSpinning=2, **kPark=3**, kSideSlip=4`. `ranger_messenger.cpp:201`
passes the raw SDK value straight through, so anyone reading `/system_state` against the message's own
constants concludes "RC mode, side-slip" when the chassis is in **CAN mode, parked**. Decode with the
SDK enums.

**Handing control from RC to CAN is physical and one-shot.** The chassis ignores CAN motion commands
in `CONTROL_MODE_RC`; the [Ranger Mini 3.0 manual] says **SWB to the top**. Confirmed: flipping SWB
moved `control_mode` 3 → 1 on its own. But `EnableCommandedMode()` is called **once**, at driver
startup (`ranger_messenger.cpp:44`), and for protocol V2 `SendMotionCommand` does *not* carry the
control mode — so a driver started while the RC still holds authority never asks again. It sits there
publishing `/odom`, subscribed to `/cmd_vel`, forwarding commands the chassis discards, looking
healthy. **Flip SWB first, then start the driver.**

### H1 — base moves on `/cmd_vel`

Drove the base from a browser WASD UI through the full chain: keypress → `/cmd_vel_teleop` → twist mux
→ `/cmd_vel` → driver → `0x111` on CAN → wheels. The mux clamped a commanded 0.5 m/s to **0.15 m/s**
(`max_vx 0.6 × override_speed_factor 0.25`) and logged `base motion permitted, source=teleop`.

### **RED — the base ran away**

**Observed:** with the operator's hands off the keyboard the base kept driving, and was stopped only
by the hardware E-stop.

**Cause.** The teleop's heartbeat watchdog detects the *page dying*. The page did not die: it stayed
alive posting a stale belief that `W` was held after a `keyup` was dropped while the JS thread was
saturated rendering the camera at ~8 fps. The lesson, stated exactly: **a heartbeat proves the sender
is alive; it proves nothing about whether a human is still asking for motion.** Those are different
claims and need different evidence. Aggressive camera rendering on the same thread as the key
handlers is what made keyup unreliable — telemetry was allowed to compete with a safety mechanism.

**Fix.** `safety/teleop_guard.py` — pure logic, 23 unit tests — adds a second, independent guard: a
**hold lease**. Key autorepeat is the evidence (it continues while a key is physically down and stops
immediately on release, and unlike `keyup` a dropped autorepeat self-corrects milliseconds later).
The page reports its last-keydown age; **the node** refuses any non-zero command whose age exceeds
1.0 s. Enforced in the node, not the page, because the page is the component that proved
untrustworthy. Render rates cut and paused when the tab is hidden.

**Verified end-to-end against the exact failure:** heartbeat healthy + fresh key → drives (43
non-zero samples); heartbeat *still healthy* + key age 5 s → **0/70 non-zero, stopped**; key fresh
again → recovers. Gate stays **RED** until the hardware half is done.

**Still open, and it gates all driving:** the `/cmd_vel` stale-command test. It separates "the UI held
the command" from "the driver holds the last command" — different bugs. Zero-risk with the E-stop
engaged: the CAN bus still shows what the driver transmits.

### Nav2 ported and validated standalone (H-nav)

`nav2_bringup/` copied from the sim repo (never edited in place). Four hardware deltas, plus two
bugs found by actually launching it:

- `use_sim_time: false` in all 7 places; Isaac-only `RPLidar_S2E → lidar_link` alias removed;
  `map_to_odom_publisher.py` dropped (slam_toolbox owns `map → odom` here).
- **The behavior-tree paths are hardcoded to another developer's home directory**
  (`/home/minghanwei/...`). `bt_navigator` fails to activate with "Couldn't open input XML file", and
  because the lifecycle manager then aborts, **Nav2 does not come up at all**. Now resolved from the
  launch file's own location. This is a latent bug in the sim repo too — it fails on any machine.
- **`behavior_server` publishes `/cmd_vel` itself.** Remapping only `controller_server` left **three**
  publishers bypassing the mux (`ros2 topic info /cmd_vel -v`: all `behavior_server`). BackUp and
  DriveOnHeading are recovery behaviours that drive the base — exactly the case `arbiter.py`'s
  docstring names. Both servers now remap to `/cmd_vel_nav`; `/cmd_vel` publishers: **0**.

**Verified with no hardware attached:** all 5 lifecycle nodes reach active, "Managed nodes are
active", `/navigate_to_pose` present, `/cmd_vel_nav` is unstamped `Twist`, `spin` absent from
`behavior_plugins`.

### Three silent TF failures, none of which logged an error

1. `ranger_mini_v3.launch.py` defaults **`publish_odom_tf: false`** → no `odom` frame at all.
   (Its description string is also wrong — copy-pasted from `update_rate`.)
2. The realsense driver **prefixes every frame with `camera_name`**, so `base_frame_id:=mast_cam_link`
   produced `mast_cam_mast_cam_link` and detached the whole camera subtree from `base_link`. Images
   kept flowing; every TF lookup to an optical frame would have failed.
3. slam_toolbox defaults **`base_frame: base_footprint`**, which this stack does not have. It
   published `/map`, looked healthy, and **never emitted `map → odom`** — silently. Fixed in
   `config/slam.yaml`.

Also: an **optical** frame must not be written with body-frame rpy. The driver owns
`mast_cam_link → *_optical_frame` from factory extrinsics; this repo owns only the mount pose.
Verified `base_link → mast_cam_color_optical_frame` = rpy `(-80.2°, 0.07°, -89.8°)`, single-root tree.

### Pipeline installed and exercised offline

`torch 2.13.0+cu130`, `sm_120` **in `torch.cuda.get_arch_list()`**, 13.7 TFLOP/s fp32 — no CPU
fallback. Modern torch defaults to a CUDA build new enough for Blackwell, so the cu121 trap recorded
on 2026-08-19 does not arise via `pip install torch`.

- Sim suite: **277 passed, 3 skipped**. Two undeclared dependencies found: `networkx`
  (`test_m4_mission_geometry.py`) and `opencv-python` (the recorder's `cv2`); neither is in
  `pyproject.toml`.
- **The 47-mission benchmark ran offline** via the undocumented `--world graph` backend, using a new
  key-free matrix (`config/pipeline/`, a local override copy — the sim repo is never edited).
  141 cells, `passive`/`heuristic`/`ours_no_reasoning` all **21/47**.
- **That identical score is correct, not a bug.** `graph_world.py:189` returns
  `Observation(rgb=None, ...)`, and `DecoupledGrounder.locate` returns `None` immediately without
  pixels — so no intervention can complete and every interventional tier is 0 for every method. The
  `graph` backend validates mission structure, negative-control scoring and FSM plumbing; it
  **cannot** exercise the intervention path.
- **Validity check passes.** All 21 `passive` successes are legitimate negative controls
  (M0_open ×11, M4_unreachable ×5, restraint_open ×3, M6b_keyfob_exec ×2 — the last is
  `success_criterion: restraint_unreachable`, `interactions_required: 0`). No mission is invalidated
  by `passive` succeeding where it should fail.

**GroundingDINO measured on this GPU**, on the six `validation/audit/*_pov.png` renders:
**~210 ms per call** after a 3.3 s load (first call 3.2 s including CUDA context). That is the 0.2 s
`LAPTOP_SETUP.md` predicts for a working GPU, against ~20 s for a CPU fallback. To verify:
`A_negative_open_pov.png` is a negative control the harness expects to MISS, and gdino returned a
0.49-score box on it — check against `validation/detect_negative_open.py`'s own query before reading
anything into it.

### The sim's controller choice cannot transfer to hardware

`config/methods.yaml` keeps MPPI with `motion_model: Omni` (instructed: match the sim). Recording why
that is load-bearing rather than cosmetic. `utp/control/ranger_4ws.py` documents an `omni` mode —
*"GENERAL 4WIS: any planar twist at once. Constrains nothing… Needed because Nav2's MPPI controller
emits strafe and yaw in the SAME twist"* — i.e. Isaac makes MPPI-Omni work by driving the four wheel
joints **directly**. The Ranger CAN protocol exposes exactly four host commands (`0x111` motion, a
**body twist**; `0x121` light; `0x131` braking; `0x141` set-mode) and **no per-wheel interface of any
kind**. So the sim's `omni` mode is simulator-only and MPPI's strafe+yaw will be truncated by the
firmware's mode selection (GAP 1). Left as-is deliberately; re-open if doorway transit misbehaves.

### Gate deltas

| Gate | Was | Now |
|---|---|---|
| H1 | `WIP` | **base moves on `/cmd_vel`, verified.** Stale-command timeout still UNVERIFIED. |
| H2 | `WIP` | `/scan` + `base_link→lidar_link` verified; scan direction (④) still needs the physical check. |
| H3 | `RED` | arm reachable, servos live (real temperatures + encoder noise). Not yet moved. |
| H4 | `WIP` | mux + teleop exercised on real hardware; **new RED sub-item: teleop runaway**, fixed in software, unverified on the robot. |
| H0 | `OPEN` | **decided: the rover laptop is the brain.** Perception installed and measured on it. |

---

## 2026-08-21 — VLM endpoint wired up; the documented base URL was wrong

Decision taken that internet access is available for the whole test window, so the FAU OwlChat
endpoint is in the loop (rather than the key-free `heuristic` fallback). Key installed in the
pipeline repo's gitignored `.env` (mode 600, never tracked).

### The base URL in our docs rejects every key

`docs/LLM_ENDPOINT.md` and `docs/HARDWARE_SPECS.md` both specified
`https://chat-llm.hpc.fau.edu/v1`. Measured:

| base URL | result |
|---|---|
| `https://chat.hpc.fau.edu/api/v1` | **HTTP 200**, 70 models listed |
| `https://chat-llm.hpc.fau.edu/v1` | **HTTP 401** — "Authentication Error, Invalid proxy server token passed" |

The sim repo's older `docs/owlchat_llm_guide.md` (verified 2026-07-02) had it right and even names
the failure: that host is *"the raw LiteLLM backend [that] rejects your keys with
`token_not_found_in_db`"*. Both of our docs are now corrected. The lesson is the general one: two
docs disagreed and the newer one was wrong, so the endpoint got asked rather than believed.

`bringup/check_llm.sh` now tries every candidate base URL, prints the **server's own** reason for each
rejection, confirms the pinned model is still listed, and runs vision end to end. It prints no key
material. Run it **from the test site**, not the lab.

### Endpoint verified working

- `openai/gemma4-vibe` present among 70 models — the pin in `methods.yaml` is still valid.
- **Vision confirmed end to end** through our own client (`validation/vlm_smoke.py`): the model
  returned a well-formed `Plan` and correctly reported it could not see a door in the synthetic
  test frame, choosing `action="none"` with `abstain=true` (keep looking) rather than
  `report_unreachable`. That is the C2 distinction behaving correctly on the first real call.

### Note

The key was pasted into a chat transcript. `owlchat_llm_guide.md` §1 says to treat that as burned:
delete it in the Account → API Keys panel and mint a new one once testing is done.

---

## 2026-08-21b — AMCL added; Nav2 can now be given a real map

Closing a gap that would have stopped the first mission run: the ported `ranger_nav.launch.py`
started `map_server` but **no localization**, because in simulation `map -> odom` came from Isaac
ground truth (`docs/integration_contract.md`: "no AMCL here"). Loading a saved map on hardware
therefore published `/map` and left Nav2 blind, with no error from any component.

**Added** an `amcl:` block to `nav2_params.yaml` and a `localization` launch argument:

- `localization:=amcl` (default) — `map_server` + AMCL. For running on a saved map.
- `localization:=slam` — neither node starts; slam_toolbox supplies both `/map` and `map -> odom`.
  For navigating while still mapping.

The switch exists because **exactly one node may publish `/map`, and exactly one `map -> odom`**;
running Nav2's `map_server` next to slam_toolbox silently puts two publishers on `/map`.
`base_frame_id` is set to `base_link` — the stock `base_footprint` default is the same trap that
made slam_toolbox publish `/map` while never emitting `map -> odom`.

**Verified with no hardware attached.** `localization:=amcl`: `map_server` and `amcl` both reach
`active [3]`; bringup then stalls at `planner_server` because the global costmap waits on
`map -> base_link`, which AMCL cannot produce without `/scan`, `/odom` or an initial pose — expected
without a robot. `localization:=slam`: both are correctly absent and the lifecycle manager is told
exactly `['planner_server','controller_server','behavior_server','bt_navigator']`. The remap is
visible in the running process arguments (`-r /cmd_vel:=/cmd_vel_nav`) on **both**
`controller_server` and `behavior_server`.

**Correction to `docs/NAV2.md`.** It quoted `inflation_radius: 0.45` from the `nav2_bringup/README.md`
table; the YAML actually sets **0.30**, and its comment records why: 0.45 closed a real 0.75-0.80 m
doorway because inflation from both jambs overlapped, and in-room goals landed inside inflation and
became unplannable. 0.30 sits just above inscribed + padding (0.28). The README table is stale.

Consequence worth recording so nobody "fixes" it: Nav2 logs an ERROR at startup that inflation
(0.30) is below the circumscribed radius (0.480). That is a **planning-speed** optimization, not a
safety check — full-footprint collision checking still runs. Raising the radius to silence it trades
a robot that can pass doorways for a faster planner.

---

## 2026-08-21c — the driver cannot latch; mapping reduced to one command

### The runaway question splits in two, and source answers one of them

`HARDWARE_SPECS.md` has carried `/cmd_vel` stale-command behaviour as UNVERIFIED since bring-up,
and it gates all driving — teleop, mapping, Nav2. Read the driver instead of guessing:

- `ranger_messenger.cpp:391` — `TwistCmdCallback` calls `robot_->SetMotionCommand(...)` **directly
  inside the subscription callback**.
- `agilex_base.hpp:92` — `SendMotionCommand` writes exactly one `0x111` CAN frame per call.
- No repeat timer exists anywhere in `ugv_sdk`. The only loop in `agilex_base.hpp` (line 258) is
  the connect/version-detect handshake.

**So the driver cannot latch.** When its publisher dies, no callback fires, and `0x111`
transmission simply stops. That reframes the 2026-08-20 runaway completely: the driver was
faithfully relaying live commands the entire time. The browser was generating them, from a stale
held-key belief after a dropped `keyup` — which is what `safety/teleop_guard.py`'s hold lease
fixes. **The runaway was never evidence about the driver.**

It also exposes the question that was hiding behind the first one. The chassis now receives *no
commands at all* rather than *zero commands*, and whether firmware stops on that is a different
property — one not stated anywhere in `ugv_sdk` or the `ranger_ros2` README, and not readable from
source at any effort. A PASS on the driver and a FAIL on firmware is still a runaway.

### `bringup/stale_cmd_test.py` — two phases, because two failure modes

Reads `can0` with a raw `AF_CAN` socket (stdlib only, no python-can dependency).

- `driver` — **E-stop engaged, zero risk.** The E-stop makes a command unactionable while leaving
  it fully visible on the bus, so the whole thing can be measured without motion. Refuses to run
  unless `0x211` actually reports `vehicle_state=ESTOP`; it checks rather than trusting a flag.
  Publishes to `/cmd_vel` at 20 Hz, SIGKILLs the publisher, watches `0x111` for 5 s.
- `firmware` — **the base moves.** Requires `--i-am-holding-the-estop` and prints the preconditions
  it cannot check. Commands 0.15 m/s, cuts the publisher, watches the chassis' own reported
  velocity in `0x221` decay — or not.
- `listen` — decode-only, commands nothing. Useful for reading `control_mode` without ceremony.

SIGKILL rather than SIGTERM on purpose: a crashed node gets no destructor, publishes no zero twist
on the way out, and leaves its DDS participant undisposed. Testing the polite exit tests the case
that was never dangerous. The publisher is killed by **process group of a pid we started** — never
a pattern match; `pgrep -f` matching my own shell is exactly how the stack got killed on 2026-08-20.

The test deliberately publishes **straight to `/cmd_vel`, bypassing the twist mux**. The mux would
zero the command and the test would measure the mux, which is not the thing in question.

**Wire format, and why it gets its own tests.** `struct16_t` under `USE_LITTLE_ENDIAN`
(`agilex_protocol_v2.h:20-21`, the default) declares `high_byte` **first**, and `EncodeCanFrameV2`
memcpy's the struct straight into the frame — so despite the macro name the wire order is
**MSB-first**. Decoding LSB-first turns 0.15 m/s (`0x0096`) into `0x9600` = 38400, which overflows
int16 to **-27.136 m/s**: a gentle crawl forward reads as a violent reverse, and it is still
non-zero, so an `is_zero()`-based verdict looks perfectly healthy while every number a human reads
is nonsense. `tests/test_stale_cmd.py` pins this against an independently transcribed encoder.

### Mapping is now one command

`bringup/mapping.sh` starts lidar + ranger driver + slam_toolbox + RViz + teleop, with the same
setsid/process-group reaping the other bringup scripts use.

The base is **opt-in** (`--with-base`), and the default is a stack that physically cannot move the
robot no matter what fails inside it — the right way to check the lidar and the TF chain before
handing CAN authority over. `--with-base` prints the RC/arm/E-stop checklist and requires a typed
`yes`, because `EnableCommandedMode()` takes authority away from the RC transmitter and that is a
decision a person makes while looking at the robot.

`maps/mapping.rviz` shows `/map`, `/scan`, TF, the **pose graph**, and (off by default) raw
`/odom`. Two QoS settings are load-bearing and both fail as a blank screen: `Map` must be
**Transient Local** because slam_toolbox latches `/map`, and `LaserScan` must be **Best Effort**
because a Reliable subscriber receives nothing from a Best Effort publisher while the reverse works
fine. Verified by loading it in RViz with no hardware: all six displays resolve, log clean.

The pose graph display earns its place — after driving a circuit, **absence of a loop edge is the
clearest possible sign the loop did not close**, and that is invisible in the occupancy grid until
the map is finished and wrong.

`bringup/save_map.sh` writes both artefacts and then **checks the disk**, because `save_map` and
`serialize_map` both return success when nothing is written. It refuses to overwrite an existing
map without confirmation (a map costs a walk around a building) and prints the extent in metres —
a map far smaller than the floor you walked means the odom anchor was lost partway.

### Two documentation errors fixed

- `docs/MAPPING.md` still said `ranger_nav.launch.py` "omits localization entirely", contradicting
  the section directly above it that documents the `localization` argument added the same day.
- It also quoted `inflation_radius 0.45` in its map-acceptance checklist. The YAML sets **0.30**;
  0.45 is the value that closed a real 0.75-0.80 m doorway. Quoting it in a checklist is worse than
  quoting it in a table — a checklist invites someone to "fix" the config to match.

### Status

83 tests green (was 66). Nothing was plugged in for any of this: `can0` absent, no serial device,
no RealSense on USB, arm not answering. Everything above is source reading, offline validation, and
scripts staged for the moment hardware appears.

**Still blocking the map:** both `stale_cmd_test.py` phases, plus CALIBRATION ③ (lidar mount pose)
and ④ (scan direction — a mirrored scan builds a map that looks plausible and is wrong everywhere,
and it cannot be detected after the fact).

---

## 2026-08-21d — both stale-command gates measured on hardware; the chassis coasts 1.26 s

### GATE 2 (driver) — PASS

E-stop engaged, `control_mode=RC`, so the command was doubly unactionable while still fully
visible on CAN. Published 0.15 m/s to `/cmd_vel` at 20 Hz, SIGKILLed the publisher, watched `0x111`.

| phase | result |
|---|---|
| baseline, nothing publishing | **0** frames of `0x111` |
| commanding | 40 frames, all non-zero, decoded `lin=+0.150` |
| after SIGKILL | **0 frames.** Transmission stopped dead. |

Confirms the source reading exactly: `ranger_messenger.cpp:391` commands from inside the
subscription callback, `agilex_base.hpp:92` emits one frame per call, no repeat timer. The driver
relays and holds nothing. The exact `+0.150` also validates the MSB-first decode on real hardware.

### GATE 3 (firmware) — FAIL, but a quantified one

`vehicle_state=NORMAL`, `control_mode=CAN`, wheels centred. Commanded 0.15 m/s; chassis reported
0.144–0.147. SIGKILLed the publisher and watched `0x221`:

**The chassis held ~0.147 m/s for 1.263 s before reaching zero** — roughly **18 cm of uncommanded
travel**, scaling linearly with speed.

So the chassis *does* have a command watchdog and a lost commander is bounded, not an infinite
runaway. But 1.26 s is a **backstop, not a brake**.

**This closes 2026-08-20.** The runaway lasted far longer than 1.26 s, so it cannot have been a
latch of any kind — not in the driver (Gate 2: nothing to latch) and not in firmware (Gate 3: it
gives up after 1.26 s). It was continuous commanding from a browser holding a stale held-key
belief, exactly as the hold-lease fix assumed. Both gates now tell one consistent story.

### The design consequence, which is the useful part

The 1.26 s timeout applies only when commands stop **arriving**. An explicit zero is a *command*
and stops the base at once. `twist_mux_node.py:150` already publishes every tick, zeros included,
so the normal watchdog path — `teleop_guard.decide()` returns ZERO, the mux publishes zero, the
driver relays a zero `0x111` — never touches the firmware timeout at all.

The 1.26 s coast is reachable in exactly one way: the **mux process itself dies without running its
parting zero**, i.e. SIGKILL. Nothing in software can shorten that, and it is the residual risk.

Corrected a comment in `twist_mux_node.py` that called the parting zero "best-effort" and said "the
driver-side watchdog is the real guarantee (still UNVERIFIED)". Both halves were wrong: there is no
driver-side watchdog at all, and the parting zero is the difference between stopping now and
coasting 18 cm.

### Chassis authority is stickier and more fragile than the docs say

Four hours of this went into finding a handover sequence that works. What was learned:

- **`control_mode` only changes when the chassis ACCEPTS a mode-set.** It is not a live reflection
  of switch positions. Once `RC` has authority it keeps it, and `EnableCommandedMode()` is one-shot
  at driver startup, so nothing ever re-asks.
- **`STANDBY` is the only state where the CAN claim is uncontested.** Restarting the driver from
  `RC` does nothing. From `STANDBY` it takes `CAN` immediately.
- **Touching the RC sticks takes authority back**, dropping `CAN` to `RC` and requiring another
  driver restart.
- **Turning the transmitter OFF while the chassis is in `RC` faults it to `EXCEPTION`** — a
  lost-link failsafe. This was my suggestion and it was wrong; it made the state worse, because a
  chassis in `EXCEPTION` will not accept a mode-set either, so the driver can never claim CAN.

**Working recipe:** transmitter **on** but sticks untouched → power-cycle the rover (boots to
`STANDBY`) → restart the driver immediately → `CAN`.

### Correction to 2026-08-21c: EXCEPTION is not just the E-stop

`EXCEPTION` (0x02) with `error=0x0000` covers **both** "E-stop pressed" and "RC link lost while in
RC mode". Observed both. There is no separate state for either, and the error code does not
distinguish them — it is `0x0000` in both cases. `VEHICLE_STATE_ESTOP` (0x01) is never observed on
this chassis at all.

### Also measured

- The raw `0x111`/`0x221` steering field is **not radians**. `ranger_base.hpp:155/175` treats it as
  degrees ÷ 10 with the sign flipped, so a resting raw of `-0.935` is **+9.4°**, not −0.9 rad —
  wrong by 5.7× and pointing the other way. Both readings are plausible wheel angles, which is what
  makes the error survive inspection. `Motion.steer_deg` now does the conversion and `__str__`
  prints units on every field.
- Battery 49.6–49.7 V throughout, `error_code` 0x0000 at all times.
- A rover power cycle re-centres the wheels (+9.4° → 0.0°).

### Status

85 tests green. **H1 gate: the base moves on `/cmd_vel` and stops when told — the residual risk is
the 1.26 s SIGKILL coast, documented and bounded.** Mapping is no longer blocked by the
stale-command question. It is still blocked by CALIBRATION ③ (lidar mount pose) and ④ (scan
direction), and by the arm being unstowed.

Gate 3 was run with the arm **34.9° off stow**, a deliberate deviation from the CLAUDE.md rule,
authorised by the operator for a 45 cm creep at 0.15 m/s with a hand on the E-stop. Recorded
because the rule exists for tipping risk and the deviation should not become precedent.

---

## 2026-08-21e — hand-eye solved and verified end-to-end; 4.3 mm placement accuracy

### The detector finds a real ADA plate, with a real decoy beside it

First run of the shipped `DecoupledGrounder` on hardware, against a wall carrying an ADA push
plate **and a red FIRE pull station** — an unstaged, real-world `M3_disambig` condition.

| | score |
|---|---|
| ADA push plate (chosen) | **0.536** |
| FIRE pull station | 0.281 |
| "The Atrium" sign | 0.162 |

GDINO, `cuda:0`, 1081 ms. The margin is the result, not the winner: the decoy was *seen*, ranked,
and beaten by ~2x. That distinguishes "could not see it" from "saw it and preferred a decoy",
which is the whole reason `Detection.candidates` carries the ranking.

Depth was **100% valid** on the plate — gate **S3** answered. Brushed metal is near worst-case for
stereo and it returned clean.

### CALIBRATION item 7 — depth/colour alignment: PASS

Measured off one frame, no motion, using the plate's own protrusion from the wall:

    centre offset  dx -3.0 px / dy +4.5 px  =  -2.8 mm / +4.2 mm at 0.84 m   (accept +/-2 cm)
    diameter       17.0 cm from depth vs 16.5 cm from RGB

Three ways this measurement goes wrong, all of which produced wrong answers before the method
settled, and all now handled by `bringup/check_depth_alignment.py`: a flat depth threshold mistakes
a non-fronto-parallel wall for misalignment (0.92 m one side, 0.87 m the other); signs and fire
alarms also stand proud, so a bounding box over all protruding pixels measures the widest of them;
and the ROBOT'S OWN ARM appears at ~0.37 m, half a metre proud.

### CALIBRATION item 1 — riser: 391 mm, and the base plate is horizontal

From CAD, cross-checked against a tape measure: deck -> arm mounting plate **391.225 mm**, deck
0.345 m, so `link_base` sits at **~0.740 m** off the floor. The plate is horizontal, so item 1 is
a pure translation as `CALIBRATION.md` assumes.

Consequence for the science: with the flange at 0.74 m and 0.764 m of arm, the **1.067 m ADA
elevator hall call is comfortably reachable**. At the flush 0.345 m mount it would not be. The
elevator tier is physically possible, as `HARDWARE_SPECS.md:164` predicted.

### CALIBRATION item 8 — hand-eye: PASS, without the ruler measurement

`bringup/handeye.py`'s Kabsch needs the arm to REPORT the marker's position, i.e. somebody must
first measure flange -> marker and set it as the TCP offset. `cv2.calibrateRobotWorldHandEye`
removes that dependency: given the marker's full 6-DoF pose (solvePnP on the ArUco corners, scaled
by the 40 mm printed size) it solves for the camera pose AND the marker offset together.

10 poses, collected by `bringup/handeye_auto.py` driving the arm through bounded joint deltas.

    link_base <- camera_optical   t = (-0.3276, +0.0123, +0.7315) m
                                  rpy = (-95.39, +0.75, -87.75) deg
    marker on flange (SOLVED)     (+26.4, +0.6, -95.4) mm
    residual                      rms 3.0 mm, worst 8.2 mm      accept <20 / <40   PASS
    rotation spread               63.8 deg

**The check that actually validates it.** A residual only says the solution fits its own data; a
systematically wrong calibration does that perfectly. The independent check is that the solved
camera **x = -327.6 mm** against **-324.238 mm measured off the CAD** — 3.4 mm apart, and the
solver never saw that number. Camera y came out +12 mm, i.e. on the centreline, also matching.

It also resolved camera **z = 1.4715 m** in `base_link`, against a tape measurement of 1057 mm.
The tape was measuring a bracket, not the lens: the marker is only ~390 mm from the lens and the
arm's flange is at ~1.6 m, so a lens at 1.057 m was geometrically impossible. The calibration
found the lens on its own.

### End-to-end placement: 4.3 mm mean, 9.7 mm worst

`bringup/handeye_verify.py` closes the loop rather than trusting the residual: choose a target,
compute the flange pose that puts the MARKER there, command it, then measure where the marker
actually landed *through the camera, through the calibration*. Errors are end-to-end — calibration,
arm positioning, detection and depth all included.

    6 targets spread over +/-50 mm in all three axes
    mean |error| 4.3 mm     worst 9.7 mm
    bias   dx +2.3  dy +0.5  dz +0.5 mm      (no meaningful systematic offset)
    spread dx  3.3  dy  1.4  dz  2.9 mm

An ADA plate is 110-150 mm across, so a press needs roughly +/-30 mm. **4.3 mm is comfortable with
3x margin.**

### J5 is the constraint on this arm, twice

`XCONF.Robot.JOINT_LIMITS` for this device: **J5 runs -97 to +180 deg**, and in the working pose it
sits at -92.3 -- only **4.7 deg of downward headroom**.

- A plain -8 deg delta walked it past the stop and aborted collection after 3 poses. Fixed by
  reading the SDK's own limit table and clamping with a 3 deg margin, rather than hardcoding a
  table that would be wrong on another arm.
- Later a *Cartesian* goal 50 mm lower needed J5 below -97 and raised **error 23** at exactly
  -96.98 deg. Clamping cannot see this one: the joint solution is chosen by the IK, not by us.
  Cleared (a limit fault, not a collision -- error 22 would warrant eyes on the robot first) and
  the arm was returned in JOINT space, which cannot sneak past a stop the way a Cartesian goal can.

Practical rule for the press: **approach in joint space, or keep Cartesian goals above the current
height.** Downward Cartesian moves in this configuration are the ones that fault.

### The OpenCV convention was got wrong twice

`calibrateRobotWorldHandEye` is written for eye-IN-hand; we have eye-TO-hand. The first attempt
reasoned its way to inverting the robot poses -- wrong. The second misread a brute-force search
that found two valid mappings and kept the last one -- also wrong. Both produced answers off by
~0.8 m, a distance that still looks like plausible robot geometry.

The correct mapping: inputs go in **as measured**, and **both outputs are inverted**.

`tests/test_handeye_rw.py` (12 tests) pins it against synthetic ground truth and asserts that the
two wrong mappings *are* wrong. A comment would not have caught either mistake.

### Status

97 tests green. Gate **H5 GREEN**. CALIBRATION items 1, 7, 8 done and recorded.

Not yet done: item 2 (the arm still has `tcp_offset = [0,0,0,0,0,0]` and `tcp_load = 0` -- it knows
nothing about the gripper, which also means its collision thresholds are wrong), items 3/4/5, and
the press itself.

---

## 2026-08-21f — the day's real lesson: every failure was silent

### Six failures, one pattern

Counting them honestly, because the pattern matters more than any individual fix:

| failure | what it looked like |
|---|---|
| lidar USB re-enumerated (x3) | node alive, `/scan` at 0 Hz, no error logged |
| camera USB re-enumerated (x2) | node alive, no topics advertised, no error |
| CAN link dropped | driver alive, `/odom` publishing **zeros at 50 Hz** |
| SLAM running without `/scan` | a map "built" from every scan stacked at the origin |
| grounder on an absent target | a confident 0.369 box on a **fire alarm** |
| wrong hand-eye convention (x2) | a clean 3 mm residual, wrong by ~0.8 m |

**Not one of these errored.** Every one produced plausible output and was wrong, and every one was
found by eventually thinking to look. The common cause is using *"the process is running"* as a
proxy for *"the thing is working"*.

`bringup/health.py` is the structural answer: it checks USB presence, CAN frame RATE (not just
link state), arm error code, topic RATES (not existence), whether `/odom` carries anything but
zeros, and the TF chain. First thing after plugging in, and any time something feels off.

It caught one live while being written: `map -> odom` was still being published by slam_toolbox
with `/scan` dead at 0.0 Hz — confidently telling the stack where the robot was, based on nothing.

### Calibration validated properly, and one weakness found

Leave-one-out cross-validation of the 10 pairs, run offline with the robot unplugged:

    LOO error   mean 3.31 mm   worst 6.16 mm      vs in-sample residual 3.0 mm

Near-equal is the point: an overfit calibration shows a small in-sample residual and a much larger
held-out error. This does not. Combined with camera x agreeing with CAD to 3.4 mm, the calibration
is validated two independent ways.

**Weakness: camera lateral position is weakly determined.** Across the LOO sub-solves camera y
spans 55 mm (std 14.3 mm) against x's 16 mm — camera-y and marker-y trade off against one another.
Prediction is unaffected (that is exactly what LOO measures), but the reported y is not a physical
measurement, and the next collection should include more **lateral** pose spread.

### `bringup/check_calib.py` — recalibration is now a question, not an assumption

The hand-eye transform is voided by exactly two things: the camera moving on its column, or the
arm moving on its pedestal. It is **not** voided by driving, moving the arm, power-cycling,
unplugging USB, or restarting every node — all of which happened repeatedly today.

So rather than recalibrating on suspicion, one capture predicts where the marker should be from
the arm's pose and compares with where the camera sees it. Under 15 mm (the verified accuracy):
nothing moved. Over 25 mm: something did, and the dominant axis hints at which mount.

### `handeye_collect.py` retired

It cannot run here — needs `pyrealsense2` (installed nowhere) and OpenCV >= 4.7's `ArucoDetector`
(system is 4.6.0) — and hardcodes `DICT_4X4_50` while the fitted marker is `DICT_6X6` id 3, which
fails as a flat "no marker detected" on frames where the marker is plainly visible. It now exits
2 with an explanation and a pointer. The body is kept, unreachable, because its documentation of
the frame algebra is correct and worth reading.

### What blocks the press

One measurement: **flange face → gripper fingertips**. The calibration knows where the *marker*
is, not where the *fingers* end, and the fingers are what would touch. Plus the gripper's mass:
`tcp_load` is 0, so the arm's collision thresholds are set for a bare flange — it will either
nuisance-trip on its own gripper or miss a real contact.

Everything else for the press is done and verified.


---

## 2026-08-21g — will it actually hit the button? Quantified: marginal

Rather than answer from the residual, the leave-one-out spread of the transform was propagated out
to the plate's real position.

    calibration volume centre (camera)   0.457 m
    ADA plate                 (camera)   0.855 m      -> 502 mm outside it

    deviation at the plate      mean 10.3 mm   worst 35.1 mm
    in the VERIFIED volume      mean  4.5 mm   worst 12.4 mm
    extrapolation penalty       2.8x
    ADA tolerance               ~+/-30 mm

**Typical case hits comfortably; worst case misses.** The 4.3 mm placement accuracy measured by
`handeye_verify.py` is real but was measured within +/-50 mm of the calibration volume. Using it at
the plate is a half-metre extrapolation, and a rigid transform's rotation error grows with distance.

The spread is almost entirely in **y and z** (std 9.9 and 9.4 mm) with x tight at 1.7 mm — the
weakly-determined lateral axis from 2026-08-21f, amplified by range.

**This is the honest reason not to attempt the press on today's calibration**, separately from the
missing fingertip measurement. Both need fixing, and the calibration one is the less obvious of
the two: nothing in the reported numbers hints at it. A residual, a LOO error and a verified
placement accuracy can all look excellent while describing a volume the target is not in.

**Fix:** collect a second batch of poses with the arm extended to working distance.
`bringup/handeye_auto.py` now takes `--prefix`, so batches accumulate in `calib/pairs/` instead of
overwriting, and `handeye_solve_rw.py` merges them:

    python3 bringup/handeye_auto.py --go --prefix near
    ... reposition the arm out toward the plate ...
    python3 bringup/handeye_auto.py --go --prefix far
    python3 bringup/handeye_solve_rw.py

Then re-run `handeye_verify.py` **with targets near the plate**, not near the calibration volume —
today's verification would have passed either way, which is the point.

### Method note worth keeping

Propagating parameter uncertainty to the point of use is a better validation than any in-sample
statistic. It is what distinguished "the calibration is excellent" (true) from "the calibration is
excellent where we will use it" (not established). Do this before trusting any calibration at a new
working distance.

---

## 2026-08-21h — remote access for a robot-mounted laptop

`bringup/remote_access.sh` (needs sudo, run once) and `bringup/whereami.sh`. Written because the
laptop is the robot's brain and will ride on the chassis with the lid shut, so SSH becomes the only
way to reach it once it drives.

Starting state: **sshd was not installed at all**, lid-close suspended by default, and WiFi
power-save was on.

Three robot-specific problems, none of which apply to a desk machine:

- **Lid shut.** `HandleLidSwitch=ignore` *and* masking the sleep targets — both, because desktop
  power managers can request a suspend logind alone would not veto.
- **Nobody at the keyboard.** Driving a robot is not keyboard input, so default idle handling
  suspends a machine that is busy. `IdleAction=ignore`.
- **It moves.** Roaming drops TCP; WiFi power-save parks the radio between packets and reads as
  random unreachability. Power-save off, plus `mosh` (survives roams and IP changes) and `tmux`
  (so a drop costs a reconnect, not a 40-minute mapping run).

**Key auth only, deliberately.** This is a `/16` campus network, not a lab LAN. The script refuses
to enable password auth and prints how to install a key from the keyboard instead.

**Address discovery.** Campus DHCP re-leases and there is no screen to read an IP from. `env.sh`
now stamps `.last_address` every time it is sourced, so the last known address is always on disk.
`whereami.sh` prints and records on demand.

**Untested and worth testing early: campus client isolation.** Many university SSIDs stop clients
reaching each other, which blocks SSH regardless of sshd configuration and looks exactly like a
firewall problem on the robot. Test from the intended client machine *while the robot is still
within reach of a keyboard*. If blocked, the answer is an overlay network (Tailscale/ZeroTier),
not more sshd configuration — that also covers reaching it from off campus.

## 2026-08-25 — the map was never going to work: the lidar is 46% blind, and it is the sensor

Mapping had been producing noisy maps that lost their pose. Three causes were found; only one of
them is fixable in software, and it is not the one that matters most.

### The headline: only 44 valid points per scan were reaching slam_toolbox

| | measured |
|---|---|
| `/scan` | 6.9 Hz, 360 beams |
| beams returning anything | **23%** (75% `inf`) |
| return quality | median **0**, max **15** — the A1M8 reports 0-63 |
| valid points into slam_toolbox | **44 per scan** |

slam_toolbox's correlative matcher needs several hundred points to separate one corridor pose from
another. At 44 it cannot, so it falls back on odometry, and 4WS odometry is exactly what must not
be trusted. Noisy map and lost pose are one symptom, not two.

### It is the sensor, not the scene — the two-location test

Blind sectors were measured at one location, the robot was then moved to the middle of the
building, and they were measured again. The scene changed completely. The holes did not:

| | location 1 | location 2 |
|---|---|---|
| forward | -34..+4 | **-32..+4** |
| left-rear | +107..+150 | **+107..+150** |
| right | -124..-107 | **-122..-106** |
| right-rear | -149..-142 | **-149..-142** |
| rear | -176..-172 | **-176..-172** |
| mean hit rate | 23.2% | 24.7% |

A 36-degree hole pointing straight ahead, stable across two unrelated environments, with healthy
neighbours returning a wall at 1.7 m either side of it, is not geometry. **~46% of beams never
return and ~125 degrees of arc is permanently dead, fixed in the sensor frame.**

Corroborating, from the driver's own startup line: `scan mode: Standard, sample rate: 2 Khz`. At
the measured 7.1 Hz that is ~280 samples/rev, and the raw scan carries exactly 280 beams. The
sensor is taking every sample it should. ~70% of them come back empty.

**Conclusion: the optical window is obstructed or degraded.** Clean it. If the blind sectors
survive a proper clean, the unit is failing and the map quality ceiling is hardware.

### Ruled out: angle compensation was not manufacturing the holes

`angle_compensate:=true` bins ~280 real samples into 360 slots, so ~80 bins are empty by
construction. Tested with it off: 280 beams, **85 valid**. With it on: 360 beams, **92 valid**.
Identical return count. Compensation neither creates nor destroys returns, so it stays on — a
uniform 1.000 deg increment is friendlier to the scan matcher than a jittery 1.256 deg.

### CALIBRATION item 3 — lidar mount pose: CLOSED, from CAD

`Downloads/Ranger_mini_Xarm6_custom_box.stp` (STEP AP214, 67 assembly placements) was parsed
directly. The CAD frame is not the ROS frame: **CAD +X -> ROS -X, CAD +Y -> ROS +Z, CAD +Z -> ROS +Y**.

That mapping is not assumed, it is checked. Applying it to the D435f in the same assembly
reproduces the independently measured camera pose in `calib/handeye.json`:

| | CAD | measured | delta |
|---|---|---|---|
| camera x | -0.320 | -0.3269 | **7 mm** |
| camera y | 0.000 | -0.0026 | **2.6 mm** |

Two sensors, one transform, two independent measurements agreeing. The vertical datum is pinned
separately: the CAD point cloud bottoms out at -344 mm, i.e. the CAD origin lies on the chassis
deck, matching the 0.345 m deck height already measured for the riser.

```
              was (design, copied from the sim repo)      now (CAD)
    x_m       0.25                                        0.318
    y_m       0.0                                        -0.013
    z_m       0.08                                        0.379   (+-0.07, part origin)
```

**What the old value cost:** x was 68 mm short, past the +-2 cm gate. A lidar offset is a LEVER
ARM — invisible driving straight, and it swings every return during rotation, which is precisely
when mapping was observed to lose its pose. The z error was ~0.30 m, which is why self-hits arrive
at 0.16-0.19 m: the scan plane sits ABOVE the 0.345 m deck, so the chassis body never occludes at
all. The lidar looks over it and sees only the superstructure.

### The rear filter was discarding 85 degrees of live scan

The +-105 deg cut was a safe guess made before the mount pose was known. Two independent sources
now agree on where self-hits actually stop. Measured over 60 scans: robot-range returns
(0.16-0.19 m) appear only beyond |150| deg. From CAD: what blocks is the arm riser, the EcoFlow
battery, the mast and the two power mounts at +-162 deg, all clustered around 180 deg.

Widened to **+-148 deg**, keeping a 2 deg guard band inside the nearest measured self-hit.

| | valid points into SLAM |
|---|---|
| +-105 deg | 44 |
| +-148 deg | **65-70** |

**+50%, for free.** On a sensor already returning on a quarter of its beams that was not
affordable to leave on the floor. `tests/test_filter_scan.py` pins the 105..148 band explicitly so
it cannot silently regress.

### Still open

**CALIBRATION item 4 — scan direction — cannot be closed from this data.** The self-hits confirm
the zero-angle points forward, but they cannot confirm handedness: a mirrored scan puts the rear
structure at 180 deg too. The robot's occluders are near-symmetric about the centreline, so there
is no asymmetric feature to test against. It needs the physical check in
`bringup/check_scan_geometry.py` — object left of the robot must read **+90**, not -90.

### Process, twice, both mine

**Killing a child killed the stack.** `filter_scan.py` and the lidar TF were stopped by PID with
cwd verified — correct as far as it went, and not far enough. Both were children of a running
`bringup/mapping.sh`, whose EXIT trap reaps its whole `CHILDREN` array. Stopping one cascaded into
rplidar, slam_toolbox, ranger_base and RViz, and cost an in-progress mapping run. **Verifying a
PID is not the same as verifying nothing supervises it.** Check for a parent script before killing
anything that a bringup script started.

**`pkill -f "rplidar_node --ros-args"` killed the shell running it** — the pattern matched that
shell's own command line. Exit 144 = SIGTERM. This is the failure CLAUDE.md rule 5 exists to
prevent and it was walked into anyway. Scope by PID, or not at all.

A third, smaller one worth knowing: stopping the lidar by process group leaves an orphaned
`rplidar_node` holding `/dev/ttyUSB0`, and the next start fails with `SL_RESULT_OPERATION_TIMEOUT`
or `Failed to set scan mode` — which looks like a hardware fault and is not. `bringup/lidar.sh`
recovers on its own because it retries; a raw `ros2 run` does not.

## 2026-08-25b — the arm reached a real ADA button under vision, and three silent bugs nearly stopped it

First end-to-end perception-to-motion on real hardware: a photograph of a real ADA plate, through
the shipped grounder, into an arm that reached it. What follows is mostly the bugs, because the
successful part was short and the bugs are what would have wasted the day.

### The result

Two independent captures of the same wall, minutes apart, each through a separate GDINO inference:

| | run 1 | run 2 | delta |
|---|---|---|---|
| score | 0.555 | 0.548 | 0.007 |
| bbox | (350,453)-(524,626) | (351,452)-(525,625) | ~1 px |
| 3D target, camera frame | -0.203, 0.155, 0.881 | -0.201, 0.153, 0.879 | **2 mm** |
| FIRE pull station | #1 @ 0.365 | #1 @ 0.371 | stable |

**The decoy held.** A red FIRE pull station sits beside the ADA plate and was ranked second both
times. That is the negative control the experiment exists to measure, and it survived contact with
a real wall rather than a rendered one.

The arm then reached it: target 0.767 m from `link_base` (limit 0.884), stepping to a 150 mm
marker standoff, then 100, then 60, holding at each. Joint headroom went from 8.6 deg at the start
pose to 42-52 deg on J5 through the approach -- the approach improved the arm's condition rather
than degrading it. Operator confirmed the tool reached the button.

### BUG 1 — the arm was aiming at a constant

`approach_target.py` took `--capture`, printed `target (camera)`, and used a value that came from
NEITHER:

    p_cam = np.array(a.target_cam if a.target_cam else [0.019, 0.162, 0.839])

A hardcoded leftover from an earlier session. `--capture` was documented as "capture dir holding
the grounded detection" and was used only to load depth for the gripper-gap measurement. Today's
detection was ignored.

    hardcoded   target link_base [0.493, 0.029, 0.491]   0.696 m from base
    detected    target link_base [0.526, 0.252, 0.498]   0.767 m from base
                                          ^^^^^ 223 mm apart in y

The plate is 174 px, about 170 mm, across. **A clean miss, executed with total confidence, and
nothing printed anything wrong.** It survived because `detect_frame.py` wrote only an annotated
PNG -- a picture a human has to read is not a handoff. It now writes `detection.json`, and
`approach_target.py` reads it or REFUSES. There is no default target any more.

### BUG 2 — the safety measurement was fitting a wall to a car park

`gripper_gap_mm()` returned **14906 mm** and printed it as "nearest part of the gripper is 14906 mm
proud of the wall". Two independent faults:

  * `grab_frame.py` maps a 0 mm reading to NaN, but the D435 also reports **65535 mm** for out of
    range. That is finite, sailed through `np.isfinite`, and 3.8% of the frame read over 20 m.
  * the wall mask was `(xx < W*0.28) | (xx > W*0.78)` then `& (xx > 380)`. At W=1280,
    `W*0.28 = 358`, so `xx > 380` deleted the ENTIRE left band and left only `xx > 998` -- which
    in this scene is the glass door and the trees outside. The plane was fitted to the car park.

Result: `plane @ image centre = -9.67 m`. A **negative distance**, reported to the operator in
millimetres as the number that would justify a press.

Rewritten: saturation dropped, the wall taken from an annulus around the DETECTED CONTROL (which
is by definition mounted on the wall), and the fit sanity-checked against physics -- 0.3-5 m, plane
residual under 5 cm -- returning None rather than a number it cannot stand behind.

### And then the honest answer: this camera cannot make that measurement at all

With the rewrite it returns None at every approach pose. That is correct. At the approach pose the
button's own pixels read **0.294 m** and the plane residual is **181 mm**: the arm crosses the mast
camera's sightline to the plate long before the wall. The camera is 1.47 m up looking down; the arm
comes in from below. There is no wall left to fit.

No code fixes that. `--hold` was added so the arm stays extended and the gap is measured BY HAND,
once, as a constant. Chasing a software fix here would have been chasing a geometry that does not
exist.

### BUG 3 — a flag that did the opposite of its name

`--retreat-only` was added to argparse with help text and **never wired up**. Running it would have
fallen through to a full APPROACH. Caught before use, then implemented properly against a home pose
persisted by `--hold`; it also called an undefined `connect()`, which the dry-run path returned
before reaching. A declared-but-unimplemented flag on a machine that moves is worse than no flag,
because the name is what the operator trusts at the wall.

### Config corrections

  * `config/safety.yaml` `arm_monitor.backend` was `scene_state` -- the SIM backend, reading a
    `/scene/state` topic that does not exist here. The `arm_stowed` gate is fail-closed, so it was
    stuck False and the `nav` and `servo` mux sources were silently dead. Teleop still worked
    because it carries `allows_arm_override`, which is exactly why this hid. Now `xarm_sdk`.
  * arm IP `192.168.1.185` -> `192.168.1.221`, verified reachable over `enx00e04c674c60`.

### Still open

Calibration is still the near-only solve from 2026-08-21: `n_pairs` 10, no far batch. Worst case at
the plate is 35 mm. It did not bite today because this ADA plate is ~170 mm across, so 35 mm is
comfortably inside it -- but that is the target being forgiving, not the calibration being
validated. A smaller control will not forgive it.

## 2026-08-26 — the FSM's World, on hardware; and the lidar cannot see the doors

`RosWorld` implements `utp/pipeline/interfaces.py`'s `World` protocol against the real robot, so
the SAME FSM, reasoner and grounder that run the sim campaign can run on the rover.
`isinstance(RosWorld(), World)` is True. Verified live end to end:

    rgb=(720, 1280, 3)   pose=(1.58, 0.18, 178 deg)
    blocked=True  kind='door'  desc='closed glass double doors labeled Da Vinci Room'

It read the room name off the door.

### The blockage is PERCEIVED, and the classifier is restricted to describing

In simulation `current_blockage()` comes from ground truth: the scene knows a door is there. There
is no ground truth in a corridor, so `bringup/ask_blockage.py` asks the VLM.

**The prompt is deliberately barred from proposing an action.** If it returned "press the button
beside the door", the reasoning would have happened in a prompt we wrote, and `reasoning_correct`
would measure our prompt rather than the reasoner. It reports what is in the way; the reasoner
still chooses the action. For the same reason it does not ask "is this a door?" -- naming the
expected answer inside the question is how you get it back. On the real frame it volunteered the
FIRE pull station as well as the ADA plate, unprompted.

Fails closed everywhere: endpoint down, unparseable JSON, missing field -> `kind=""`, still
blocked. Never coerced to "door", because a wrong kind sends the reasoner hunting a control that
does not exist and the trial then records a REASONING failure that was really a PERCEPTION one.
10 tests on the parser alone.

### THE LIDAR IS NOT THE BLOCKAGE DETECTOR. It was written that way and that was wrong.

Measured with the robot parked in front of closed glass doors:

| | |
|---|---|
| camera | "closed glass double doors labeled Da Vinci Room" |
| lidar, straight ahead | nearest return **6.98 m** |
| `corridor_blocked()` | fired on **5/73** scans — noise, not a detection |

A 2D lidar sees **through** glass, and the doors in this building are glass. The claim that "the
corridor veto IS the blockage detector, same signal Nav2 would give without a map" is true for
opaque obstacles and false for the only door type that matters here.

Left alone this does not fail loudly. It drives into a glass door at full speed while reporting
`reached`, the FSM never reasons, never grounds, never acts, and the experiment silently becomes
"drove into a window". This is gate **S1**, and it is the second time on this project that a
sensor's *silence* has been mistaken for *absence*.

**Fixed by inverting the authority:** the CAMERA is checked before the wheels turn; the lidar
corridor veto stays on as a backstop for opaque things the camera misses. Two sensors, two
failure modes, neither trusted alone.

### Deliberately not implemented

The three `gt_*` methods return empty. They are ground truth for benchmark scoring and there is
no answer key for a real corridor -- so `reasoning_correct` and `grounding_iou` are **not
scoreable on hardware**, and pretending otherwise would be the worst kind of number. Hardware
trials are scored on what actually happened (did the door open); the answer-key metrics stay with
the sim campaign.

### Still open

The stale `/home/minghanwei/...` path in `utp/pipeline/reasoning/capabilities.py` will throw on
this machine and has to be resolved before the FSM itself runs. Also: the HPC key pasted into a
chat transcript on 2026-08-21 has still not been rotated.

## 2026-08-27 — the workflow, tested in Isaac Sim (and three bugs it caught)

Hardware is unplugged. The whole benchmark workflow (drive -> notice blocked -> press -> continue)
now runs in Isaac Sim against the sim repo's trial server, using the SAME executor, the SAME route
files, the SAME safety mux and the SAME grounder. Only the arm action differs (sim IK vs xArm SDK).

### Conditional routes: the workflow the project actually needs

`safety/route_plan.py` gained a `check` step. A route can now branch ONCE, on perception:

    benchmark:
      - goto: door
      - check: blockage
        if_blocked: press_and_pass    # spliced in, then the route continues
      - goto: outside

The VLM chooses BETWEEN TWO PRE-WRITTEN PLANS -- it never invents motion. Branch contents are
validated before anything moves (a typo inside `if_blocked` would otherwise surface with the robot
already parked at a closed door). Branches may not nest: a robot re-deciding inside a decision is
unreviewable before the run. A VLM that cannot be reached FAILS CLOSED and the robot holds.

This is what makes the recording run and the autonomous run the same route: you record with the
doors open, the robot meets them closed, and the check is what absorbs the difference.

### THE BUGS THE SIM CAUGHT (all three would have bitten on hardware)

1. **The corridor veto could deadlock the robot.** `plan_step` zeroed ALL motion when the lidar saw
   an obstruction -- including turning in place. A robot parked facing a closed door could never
   turn around, because the veto keyed on the very rays it was trying to turn away from. Measured:
   parked 0.17 m past a waypoint, `[turn_to_bearing]` -> `[blocked]` -> stop, forever.
   FIX: the veto now gates FORWARD motion only. In-place turns are always permitted -- the
   footprint does not advance, so there is nothing to veto. Two tests replaced with four.
   This is a strong candidate for the 2026-08-26/27 "wheels rotate but the robot goes nowhere".

2. **The controller chattered at the waypoint.** At ~pos_tol the bearing to a point 15 cm away
   swings tens of degrees on millimetre drift. Measured ~20 turn/drive/settle cycles at the goal
   edge before arriving. FIX: arrival hysteresis (1.6x pos_tol once settling), mirroring the turn
   hysteresis already there for the same reason.

3. **The blockage check could not read the sim depth stream.** grab_frame assumed 16UC1 millimetres
   on the RealSense's aligned topic; Isaac publishes 32FC1 METRES on a different topic. FIX:
   both encodings handled, topic overridable via UTP_DEPTH_TOPIC. One code path, two worlds.

### Open, and the most interesting thread

`/safety/arm_stowed` FLAPS in sim: 30 True / 91 False over 121 messages. Root cause is measured and
NOT a sim quirk in its general form: `stale_after_s: 0.5` is sized against a NOMINAL 5 Hz
/scene/state, but headless RTF drops the real rate to **0.55 Hz** (1.8 s between messages), so the
gate collapses to False between messages and the mux blocks with `arm_not_stowed`.

The lesson generalises to hardware: **the arm_stowed staleness window must be sized against the
MEASURED evidence rate, not the nominal one.** On hardware the xarm_sdk backend polls at 20 Hz --
but we already know that SDK session dies when other tools connect (see 2026-08-26). When it does,
the gate goes stale, the mux blocks, and the base silently refuses to move while every other
indicator looks healthy. That is exactly the "not moving :(" symptom, and nothing was watching it.
NEXT: make stale_after_s per-backend, and have health.py assert the arm_stowed duty cycle.

Also caught: I ran `safety_sim.sh` twice and created a SECOND twist_mux, i.e. two publishers on
/cmd_vel -- the same class of bug that cost 2026-08-26 on hardware, this time self-inflicted.
Killed by explicit PID after verifying /proc/PID/cmdline.

### Bring-up notes (the sim was NOT runnable from a fresh checkout)

- `ranger_xarm6_full.usd` is a gitignored BUILD ARTIFACT and was absent -> the server found no
  base_link/lidar/camera and exited. Rebuilt via `sim/build_robot_usd.py` (laptop copy of the sim
  repo's builder) from the committed configuration/ USDs, with sensor meshes streamed from
  NVIDIA's public CDN instead of the old workstation's local asset pack.
- Materials resolve via `ISAAC_ASSETS_ROOT`; pointed at the same CDN (0 MDL errors after).
- Isaac's ROS2 bridge needs system Jazzy sourced BEFORE launch or it fails with
  "ROS2 Bridge startup failed" and falls back to its internal copy.
- **Render products only fill under the replicator orchestrator on this build.** Without
  `rep.orchestrator.run()` after `timeline.play()`, every camera frame is the cleared buffer:
  uniform gray 228, depth all-inf -- and the VLM was being handed a blank image while every
  topic looked healthy at 12 Hz. The sim repo's own robot/verify_render.py documents the same
  quirk. Patched in `sim/trial_server_patched.py` (a COPY; the sim repo is untouched).
  After the fix: rgb std 45.2, depth 0.40-2.77 m.

### New CAD (Ranger_mini_Xarm6_custom_box+Copy.stp) -- lidar height is WRONG in config

Parsed 56 products. Converted CAD (mm) -> ROS base_link (m) with the mapping validated on
2026-08-26 (CAD +X -> ROS -X, CAD +Y -> ROS +Z, CAD +Z -> ROS +Y):

| component | ROS x | ROS y | ROS z |
|---|---|---|---|
| RPLIDAR A1M8 kit | +0.318 | -0.013 | **+0.034** |
| D435f_Solid | -0.320 | +0.000 | +1.061 |
| XI1305 (xArm6 base) | -0.016 | -0.024 | +0.374 |
| AC Control Box | -0.250 | +0.285 | +0.282 |
| Ouster OS0 | -0.375 | +0.000 | +1.146 |

x and y CONFIRM `config/lidar.yaml` exactly (+0.318, -0.013). **z does not: CAD says +0.034, the
config says +0.379 -- the lidar is configured 34 cm too high.** Not yet changed: the 2026-08-25
entry recorded z as a DESIGN estimate with a +-0.07 caveat and the scans were sane, so this needs
one tape-measure check before editing. The camera and the Ouster in the new box are not in the
ROS config at all.

## 2026-08-29 — first autonomous runs at the FAU atrium doors, and every reason they stopped

Hardware day. The route is `start -> doors -> button -> press -> doors -> final`. By the end the
navigation legs closed every time, the base positioned itself into arm reach of the plate on its
own, the arm reached the wall, and the press missed by 10 cm. Every stop had a measured cause.
Listed in the order found, with the fix and the evidence, because the operator's standing
question all day was the right one: *why is the path planning failing* — and it never was.

### Silent-discard, the failure class of the day

Six separate faults shared one signature: the system worked as designed and said so somewhere
nobody was listening. In order of discovery:

| fault | where it was reported | who was listening |
|---|---|---|
| safety mux discarding every command (`arm_not_stowed`) | `/safety/status` | the teleop web page only |
| no publisher for `/safety/arm_stowed` on hardware | nothing | nobody — `arm_monitor_node` appeared in docs, in no launcher |
| chassis in RC mode discarding CAN motion (`control_mode=RC`) | CAN frame 0x211 | nobody in ROS; odom and the mux look healthy |
| waypoints from a dead odom frame | `odom_epoch` (wall clock, never read) | nobody |
| no `/scan_filtered` outside a mapping session → corridor veto fails OPEN | nothing | nobody — every autonomous run ever made had no obstacle check |
| `ROS_DOMAIN_ID` unset → node on an empty graph, "is ranger_bringup running?" | the wrong error | operator, misled |

Each now refuses loudly with the remedy: `mux_watch.py` (2 s abort naming the gate), `safety.sh`
(the launcher hardware never had), `chassis_mode.py` (reads 0x211; `EXCEPTION` outranks the
SWB advice — disconnecting the RC trips a lost-link failsafe and makes things worse),
`waypoint_frame.py` (DDS GID of the `/odom` publisher as the session id; a chassis power-cycle is
still invisible to it, documented), `lidar.sh` now starts the filter, `_ros_env.require_domain`.

### Measurements

* **Twist characterisation, first time run.** Linear scale 0.94, angular **0.59** then **0.80**
  on repeat — inconsistent, i.e. a fixed 4WS re-steer startup cost, not a gain. Signs correct.
  Lidar scan-match on a 27° spin: odom/lidar = **1.02**. Odometry is honest; the chassis
  under-rotates. No correction applied — a constant would over-rotate long turns.
* **Lidar height settled without a tape.** Near returns (<0.35 m) only astern, none forward →
  scan plane is above the 0.345 m deck. `z=+0.379` stands; the newer CAD part was mis-identified.
* **Door livelock.** 90 s pinned at 2.69 m, heading ±4°, avoidance reporting a way round every
  cycle. Two causes: the avoid bearing re-derived each tick (a mode change the firmware answers
  by re-steering all four wheels) and no livelock detection. Fixed: steer-while-driving, latched
  command (re-issue only on >8° change), 25 s / 10 cm no-progress watchdog that raises `STUCK`.
* **Odometry vs waypoints.** From one recorded `button` pose, three runs put the robot
  1.58 / 1.72 / 1.66 m from the plate at +30…36°. Repeatable → the offset lives in the recorded
  coordinate; `reach_control` (the sim's `_approach_press_pose`, ported) closed it to
  **0.68 m / 0.0°**. Lidar anchor (`scan_anchor.py`, `waypoints.py anchor/relocalize`) written and
  unit-tested for the run-to-run drift; not yet exercised on the robot.

### The pipeline on the robot

`run_trial.py` runs the pipeline's own FSM/reasoner/grounder/verifier against `RosWorld` — the
runner's `make_world()` never knew it. Five FSM trials at the doors, all abstaining; the reasoner
was right each time and each abstention pointed at the harness:

1. Asked from 2 m out, square to the doors → "I need to move closer". Approach was after the
   question; `fsm.py:421` has it before. Fixed.
2. Closed to 0.55 m (the *press* standoff) → plate half out of frame. Survey standoff is a
   different number (1.40 m). `widen_view` implemented; approach now achieves the standoff in
   either direction.
3. At +80° with the plate **dead centre**: "I cannot see any button". Same image, asked cold:
   `press_button`, "a silver ADA push-button plate visible". The difference was one prompt line —
   *"you reported no control you could operate in any of them"* — held at temperature 0.
   `SteeredReasoner` (subclass, pipeline untouched) neutralises it and lets the model say
   `params.look = left|right|closer|back`; `RosWorld.strafe_view` executes one bounded motion.
   Verified live: the VLM steered `closer`, `left`, `right`, `left`.
4. Steering oscillated around the plate with a 60° step; blind bearings ±45° (the sim's) never
   reach a control ~80–100° off the door normal in this building. Widened to ±80° first.
5. The grounder at 1.38 m ranks the two wall signs above a ~50 px plate; at ~1.0 m the plate
   wins at 0.489 (81×88 px). `act()` closes in and re-grounds when the first detection is weak.

Operator decision, late afternoon: record the button pose too. `recorded_press` route.

### The press

* First arm motion: base at 1.23 m from a 0.88 m arm, `ControllerError 21`, and `approach_target`
  returned 0 → route logged `complete (4/4)`. Fixed: fault exits non-zero; `reach_envelope.py`
  refuses out-of-envelope targets; positioning is target-relative on odom, never on the lidar
  (its 0.90 m box, measured from a sensor 0.318 m forward, halts the base 1.21 m out for ever).
* Grounder returned the **fire alarm** as "the accessible door push button", highest confidence
  of the session; two rephrasings returned the same box. `press_veto.py` asks the same detector
  what a fire alarm looks like and refuses on overlap — caught it twice more later (97 %, 4 of 4
  agreeing). One false positive (a "lever" query matching the round plate) fixed by weighing
  evidence: ≥2 queries on target, or the top-scoring one.
* At the press pose the plate sits **behind the stowed arm** in the mast camera; re-grounding
  there returns the alarm. Reprojecting the 1.66 m detection through odometry sent the arm
  **10 cm left, 5 cm low** of a 12 cm plate — the sim's "base yawed between observing and
  pressing" warning at small scale. With the arm in READY the camera sees the plate plainly
  (0.413, SAFE, direct lift agrees with the miss). Fix: `press_run` READY → LOOK → GROUND →
  REACH; the reprojected point demoted to a >20 cm disagreement cross-check.

### Not done

Full run end to end. Lidar anchor on the robot. Elevator: never run on any system.
