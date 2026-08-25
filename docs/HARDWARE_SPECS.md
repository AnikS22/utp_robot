# Hardware specs, device IDs, pinned versions

Everything measured or verified on the workstation 2026-08-18 unless marked otherwise. Values
marked **UNMEASURED** are design intent copied from the sim and must be measured on the physical
robot before they are trusted.

## Compute — Dell Pro Max 16 Plus (rover laptop)

Mobile workstation, Intel Core Ultra class + discrete NVIDIA RTX Pro (Blackwell generation).
Confirm on the actual unit before provisioning:

```bash
lsb_release -d; uname -r
nvidia-smi --query-gpu=name,memory.total,driver_version,compute_cap --format=csv
lspci | grep -i ethernet        # real RJ45, or USB-C only?
lsusb -t
```

Three consequences, all of which change what you install:

**Kernel.** This silicon is newer than Ubuntu 24.04 GA (kernel 6.8). Install from a **24.04.3+ ISO
(HWE kernel 6.14)** or move to HWE immediately. Symptoms of getting this wrong are missing WiFi,
dead trackpad, or thermal problems — not an obvious "unsupported" message.

**CUDA compute capability — this one will bite you.** The workstation's frozen perception stack is
`torch==2.5.1+cu121`, built for the RTX 6000 Ada (sm_89). **Blackwell (sm_120) is not supported by
cu121 builds.** Do not copy `env/requirements-perception-cuda.txt` verbatim. Install a CUDA 12.8+
torch (`--index-url https://download.pytorch.org/whl/cu128` or newer) and verify with:

```python
import torch; print(torch.cuda.get_device_capability(), torch.rand(3, device="cuda"))
```

A silent CPU fallback here turns a 0.2 s grounding call into ~20 s and will look like a network
problem.

**Ethernet.** The xArm control box needs **wired** Ethernet on its own subnet, while WiFi carries
internet for the VLM API. If the laptop has no RJ45, dedicate a USB-C→Ethernet adapter to the arm.
Do not share it with a dock.

**Power.** The laptop battery is an independent supply, which is good — a chassis power event does
not kill compute. Expect roughly 1–2 h under sustained GPU load; that defines session length.
Charging from the 48 V rail needs a DC-DC converter sized to this machine's adapter.

**USB.** Put the lidar and the CAN adapter on **separate direct ports**, not a shared hub. On the
workstation a re-plug moved the lidar `ttyUSB0 → ttyUSB1` and knocked the CAN adapter off its hub.

## RPLIDAR A1M8 — VERIFIED WORKING

| Field | Value |
|---|---|
| USB ID | `10c4:ea60` Silicon Labs CP2102 UART bridge (kernel `cp210x`) |
| Stable path | `/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0` |
| Baud | **115200** (A1M8. The A3/S2 are 256000 — wrong baud gives a SILENT no-data start) |
| Model / firmware / hardware | `0x18` / `1.29` / `7` |
| Serial | `6FB2ED93C0EA98C9C2E29EF59D68406E` |
| Range | 0.15 – 12.0 m |
| Beams | 360 @ 1.000° |
| Scan rate | **7.14 Hz measured** (driver is configured for 10; the A1M8 motor is unregulated) |
| `frame_id` | `lidar_link` |
| Mount `base_link→lidar_link` | `[0.25, 0.0, 0.08]` m, zero rotation — **UNMEASURED** |

**Requires our patch.** Firmware 1.29 predates the scan-mode negotiation protocol:
`getAllSupportedScanModes()` returns `0x80008004` (NOT_SUPPORT) and the express path returns
`0x80008000` (INVALID_DATA). `patches/rplidar_ros-legacy-scan.patch` adds `legacy_scan:=true`,
which issues the legacy SCAN command. 13 lines. Applied automatically by `setup_workspace.sh`.

**Known quirk, root cause not established.** Hand-probing across process restarts succeeds only
~70–90% of the time; a bad process reads zero bytes for its entire life while in-session it is
20/20. It lives in the CP2102 reopen path, not the sensor. `bringup/probe_rplidar.py` and
`bringup/lidar.sh` both retry by reopening. Do not re-diagnose this as a hardware fault.

**Never use `/dev/ttyUSBn` directly.** Observed reordering live.

## AgileX Ranger Mini 3.0 — NOT YET BROUGHT UP

| Field | Value |
|---|---|
| USB-CAN adapter | `1d50:606f` Geschwister Schneider / candlelight (kernel `gs_usb`, in-tree) |
| Interface | `can0` (a **netdev**, not a `/dev/tty*`) |
| Bitrate | **500000** |
| Dimensions | 0.720 × 0.500 × 0.345 m, wheelbase 0.494, track 0.364, wheel radius 0.11 |
| Drive | 4WS — spin / traverse (crab) / Ackermann. **Not holonomic.** |

```bash
sudo ip link set can0 up type can bitrate 500000
candump can0        # heartbeat frames appear with the chassis powered
```

**Fault 0x80 is the E-stop.** If the driver runs but nothing moves, check the RC mode switch and
both E-stops before debugging software. This looks exactly like a software bug and is not one.

**A pressed E-stop reads `EXCEPTION` (0x02), not `ESTOP` (0x01).** Measured 2026-08-21: with the
chassis E-stop out, `0x211` reports `vehicle_state=NORMAL`; pressed, it reports `EXCEPTION` with
`error_code=0x0000`. The `VEHICLE_STATE_ESTOP = 0x01` value in `agilex_types.h:33` is **never
observed on this chassis**. Anything gating on `ESTOP` will refuse on correctly-stopped hardware —
`bringup/stale_cmd_test.py` did exactly that until it was fixed.

Distinguish the two meanings of `EXCEPTION` by the error code: E-stop is `EXCEPTION` **with
`error=0x0000`**; a genuine fault is `EXCEPTION` with a non-zero error. There is no separate state
for "stopped by a human" versus "stopped by a fault", so the error code is the only discriminator.

Note also that the **Ranger's** E-stop is the one that moves `vehicle_state`. The xArm has its own,
entirely independent E-stop on its control box; it is on Ethernet and contributes nothing to
`0x211`. Pressing the arm's E-stop does not stop the base.

**Twist truncation (GAP 1) — verified in driver source.** `TwistCmdCallback` auto-selects a motion
mode from `/cmd_vel` and **drops components**:

| Condition | Mode | Dropped |
|---|---|---|
| `linear.y != 0` | PARALLEL | `angular.z` |
| small turn radius | SPINNING | `linear.x` |
| otherwise | DUAL_ACKERMAN | `linear.y` |

The CAN primitive is `SetMotionCommand(linear, steer, angular)`; there is no simultaneous
strafe+yaw and no ROS service to override the mode. Consequences: **use Regulated Pure Pursuit, not
MPPI with `motion_model: Omni`**, and any approach servo must alternate rotate-then-translate
rather than blending.

**`/cmd_vel` stale-command behaviour — half answered from source, half still UNVERIFIED.**
There are two independent failure modes and they need separate answers:

| | Question | Status |
|---|---|---|
| driver | Does `ranger_base` keep **transmitting** the last twist on CAN after its `/cmd_vel` publisher dies? | **No**, from source. `ranger_messenger.cpp:391` calls `SetMotionCommand` directly inside the subscription callback, and `agilex_base.hpp:92` emits exactly one `0x111` frame per call. There is no repeat timer anywhere in `ugv_sdk`. Confirm on the bus with `bringup/stale_cmd_test.py driver` (E-stop engaged, zero risk). |
| firmware | If `0x111` stops **arriving**, does the chassis keep executing the last one? | **YES, for ~1.26 s.** Measured 2026-08-21 on hardware: commanded 0.15 m/s, SIGKILLed the publisher, and `0x221` kept reporting ~0.147 m/s for **1.263 s** before reaching zero. Roughly **18 cm of uncommanded travel**, scaling linearly with speed. |

**Design consequence, and it is the important one.** The chassis watchdog is a *backstop, not a
brake*. It bounds a lost commander — the base will not run forever — but 1.26 s is far too slow to
be the safety mechanism. Stopping must come from software: `safety/teleop_guard.py` acts at 0.35 s,
which beats firmware by ~0.9 s, and that margin is the whole reason the hold lease was worth
building properly instead of just lengthening the heartbeat.

Both gates together also settle the 2026-08-20 runaway: it ran far longer than 1.26 s, so it cannot
have been a latch of any kind. It was continuous commanding from a browser holding a stale key
belief — consistent with the `driver` result, and fixed by the hold lease.

The distinction matters: a PASS on `driver` and a FAIL on `firmware` is still a runaway, and the
driver is blameless. It also reframes the 2026-08-20 teleop runaway — the driver was faithfully
relaying live commands the whole time; the browser was the thing generating them from a stale
held-key belief. That is fixed by the hold lease in `safety/teleop_guard.py`, and it means the
runaway is **not** evidence about either row of this table.

## uFactory xArm6 — BLOCKED

| Field | Value |
|---|---|
| Link | Ethernet to the control box (not CAN, not USB) |
| Default subnet | `192.168.1.x` — read the actual IP off the control box screen |
| Driver | **`xArm-Python-SDK` in our own rclpy node**, not `xarm_ros2`/MoveIt2 |
| Contract | `/arm_reach/goal` (`PointStamped` in `base_link`) → `/arm_reach/result` (`Bool`) |
| Reach | 0.764 m + 0.12 m stylus end effector |
| Arm base height (design) | 0.345 m — **but a riser is fitted, UNMEASURED** |

**SDK Cartesian units are millimetres**; our whole stack is metres. Set `is_radian=True`
explicitly rather than trusting defaults.

**Why not MoveIt2:** the contract is one topic pair, ~150 lines against the SDK. MoveIt2 buys
collision-aware planning we do not need for a straight reach-to-plate and costs a URDF, planning
scene, controller config and a new class of failure modes. `xarm_ros2` *does* have a jazzy branch
(updated 2026-05-21) — this is a choice, not a constraint.

**The riser is load-bearing, not incidental.** ADA elevator hall call buttons are at 1.067 m
(42 in) centreline; car buttons up to 1.22 m. The sim reach envelope is 0.70–1.05 m at a flush
0.345 m base — i.e. at the flush mount the robot **cannot** press a real hall call. The riser is
what makes the elevator tier physically possible. Measure it and put it in `base_link→link_base`,
or every press misses vertically by exactly the riser height.

## Intel RealSense D455 — NOT YET SET UP

| Field | Value |
|---|---|
| Mount | Top mast, **rear-centre**, behind the arm |
| Pose (design) | `[-0.25, 0.0, 1.15]` m, pitch −10° — **UNMEASURED** |
| RGB | 1280×720, ~90°×65° |
| Depth | 1280×720, 0.4–6.0 m usable |
| Frame | `mast_cam_optical` |
| Topics | `/mast_cam/color/image_raw`, `/mast_cam/depth/image_rect_raw`, `/mast_cam/color/camera_info` |

Enable **depth-to-colour alignment**. Misalignment presents as "grounding is right but the 3D point
is wrong", which is easily misdiagnosed as hand-eye error.

**Expect depth dropout on ADA push plates.** Brushed metal is near worst-case for stereo, so
`point3d` is most likely invalid *precisely on the target we care about*. Build the fallback chain
before the first press: patch median → plane fit → known height from the mission spec, each logged.

## Pinned software versions

| Component | Pin | Why |
|---|---|---|
| Ubuntu | 24.04.3+ (HWE kernel) | ROS 2 Jazzy target; GA kernel too old for this laptop |
| ROS 2 | Jazzy | |
| `rplidar_ros` | `24cc9b6dea97e045bda1408eaa867ce730fd3fc3` (branch `ros2`) + our patch | |
| `ranger_ros2` | `b6ea21a275ca5e7168130cc6470e61474681d679` (branch **humble**) | **The `jazzy` branch has no Mini V3 support** — its `RangerSubType` enum stops at `kRangerMiniV2` and it ships no `ranger_mini_v3.launch.py`. Only humble has `kRangerMiniV3`. Do not "fix" this by switching branches. |
| `ugv_sdk` | `f2704eacdc90357078cd93ec60aae08bb4baab35` (branch `main`) | |
| asio | `asio-1-28-0`, vendored headers | `ugv_sdk` needs standalone asio; vendoring avoids needing `sudo apt install libasio-dev` |

## Perception and reasoning

| | |
|---|---|
| Grounder (primary) | `IDEA-Research/grounding-dino-base` via HF `transformers` |
| Grounder (2nd arm) | `google/owlv2-base-patch16-ensemble` |
| Reasoner | OpenAI-compatible client; endpoint `https://chat.hpc.fau.edu/api/v1` (FAU OwlChat) |
| Env vars | `OPENAI_BASE_URL`, `OPENAI_API_KEY`, `UTP_VLM_MODEL` in `.env` |

**Two deployment risks, both must be handled before going to the test site:**

1. `config/detectors.yaml` pins `gdino` to `cuda:0`, `owlv2` to `cuda:1`, `clip` to `cuda:2` — it
   was written for a 3-GPU workstation. **The laptop has one GPU.** Override every backend to
   `cuda:0` or the second arm fails to load.
2. The VLM endpoint is a **university HPC service** and may require the campus network or a VPN.
   The test site is a building, possibly with guest WiFi or none. **Verify reachability from the
   actual test location before relying on it**, and pre-download the HF models so nothing depends
   on venue bandwidth.

## ROS domain

`ROS_DOMAIN_ID=9` is reserved for the hardware stack (`bringup/env.sh`). On the laptop this is
mostly moot — nothing else is running — but keep it so laptop and workstation behave identically
and so the workstation never collides with the sim campaign, which walks domains upward through
137–225 and treats 42/43 as poisoned.
