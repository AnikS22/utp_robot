# Rover laptop provisioning runbook — Dell Pro Max 16 Plus

Follow top to bottom. Every stage ends in a **CHECK** that must pass before moving on; a stage that
"probably worked" is a stage that will be re-debugged later at ten times the cost. Record outcomes
in `EXPERIMENT_LOG.md`.

Read `AGENT_BRIEF.md` first. Device IDs and pinned versions are in `HARDWARE_SPECS.md`.

---

## Stage 0 — OS

Install **Ubuntu 24.04.3 or newer** (HWE kernel 6.14). Not the original 24.04 GA: its 6.8 kernel
predates this laptop's silicon and typically shows up as missing WiFi or a dead trackpad rather
than an honest error.

Resolve **Secure Boot** now, not later: it blocks the NVIDIA DKMS module. Either enrol a MOK during
driver install or disable Secure Boot in BIOS. The lidar (`cp210x`) and CAN (`gs_usb`) drivers are
in-tree and unaffected.

```
CHECK:  lsb_release -d          -> 24.04.x
        uname -r                -> 6.11 or newer
```

## Stage 1 — NVIDIA driver and CUDA

Install the recommended driver, reboot, then:

```
CHECK:  nvidia-smi --query-gpu=name,memory.total,driver_version,compute_cap --format=csv
```

**Record the compute capability in the log.** Blackwell is `12.0` (sm_120), and it determines which
torch build you need in Stage 5. If `nvidia-smi` fails, stop — nothing downstream is worth doing
until the GPU works.

## Stage 2 — ROS 2 Jazzy and tools

Add the ROS 2 apt repository, then:

```bash
sudo apt install -y \
  ros-jazzy-desktop ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox \
  ros-jazzy-realsense2-camera ros-jazzy-tf2-tools \
  python3-colcon-common-extensions python3-rosdep python3-vcstool \
  python3-yaml python3-venv git can-utils
sudo rosdep init && rosdep update
```

```
CHECK:  source /opt/ros/jazzy/setup.bash && ros2 doctor --report | head
```

## Stage 3 — Device permissions and boot-time setup

```bash
sudo usermod -aG dialout $USER      # lidar serial; LOG OUT AND BACK IN for this to take effect
```

udev rule for a stable lidar name — `/etc/udev/rules.d/99-utp-robot.rules`:

```
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="rplidar"
```

`sudo udevadm control --reload-rules && sudo udevadm trigger`

systemd unit to raise CAN at boot — `/etc/systemd/system/can0.service`:

```ini
[Unit]
Description=Bring up can0 for the Ranger Mini 3.0
After=network.target
[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/sbin/ip link set can0 up type can bitrate 500000
ExecStop=/sbin/ip link set can0 down
[Install]
WantedBy=multi-user.target
```

`sudo systemctl enable --now can0.service`

```
CHECK:  id -nG | grep dialout
        ls -l /dev/rplidar
        ip -br link show can0        -> UP  (adapter plugged in)
```

`can0` will fail to come up if the adapter is unplugged — that is expected, not a fault.

## Stage 4 — This repo and the drivers

```bash
git clone <utp_robot remote> ~/utp_robot && cd ~/utp_robot
bash bringup/setup_workspace.sh
```

Clones `rplidar_ros`, `ranger_ros2` (**humble** branch — see `HARDWARE_SPECS.md` for why), `ugv_sdk`
and asio at pinned commits, applies our patches, and builds. No sudo. Idempotent. ~30 s.

The script scrubs conda from `PATH` itself. **This matters:** colcon runs
`package_xml_2_cmake.py` with whatever `python3` it finds first, and conda's has no `catkin_pkg`,
which makes *every* package fail at `ament_package()` with an opaque "returned error code 1" that
mentions neither conda nor python.

```
CHECK:  source bringup/env.sh
        ros2 pkg list | grep -E "rplidar|ranger"     -> 4 packages
        ls ros2_ws/install/ranger_bringup/share/ranger_bringup/launch/  -> includes ranger_mini_v3.launch.py
```

## Stage 5 — The pipeline

```bash
git clone https://github.com/AnikS22/unlocking-the-path.git ~/unlocking-the-path
cd ~/unlocking-the-path
python3.12 -m venv env/.venv && . env/.venv/bin/activate
pip install -e ".[perception,dev]"
```

Then replace the torch that pulled in with a build matching your GPU:

```bash
pip install --index-url https://download.pytorch.org/whl/cu128 torch torchvision
```

**Do not copy `env/requirements-perception-cuda.txt` verbatim.** It pins `torch==2.5.1+cu121` for
the workstation's RTX 6000 Ada (sm_89); a cu121 build does not support Blackwell (sm_120) and will
silently fall back to CPU, turning a 0.2 s grounding call into ~20 s that looks like a network
problem.

```
CHECK:  python -c "import torch; print(torch.cuda.is_available(), torch.cuda.get_device_capability()); print(torch.rand(3,device='cuda'))"
```

**Single-GPU override.** `config/detectors.yaml` was written for a 3-GPU workstation and pins
`gdino: cuda:0`, `owlv2: cuda:1`, `clip: cuda:2`. Set every backend to `cuda:0` or the second arm
fails to load.

**Pre-download the models** while you still have good bandwidth — do not depend on the test site's
WiFi:

```bash
HF_HUB_DISABLE_IMPLICIT_TOKEN=1 python -c "
from transformers import AutoProcessor, AutoModelForZeroShotObjectDetection as M
for m in ['IDEA-Research/grounding-dino-base','google/owlv2-base-patch16-ensemble']:
    AutoProcessor.from_pretrained(m); M.from_pretrained(m); print('cached', m)"
```

**Credentials.** Create `.env` in the pipeline repo with `OPENAI_BASE_URL`, `OPENAI_API_KEY`,
`UTP_VLM_MODEL`. The endpoint is FAU's OwlChat HPC service and **may require the campus network or
a VPN** — verify it is reachable *from the actual test site*, not just from the lab. If it is not,
that is a blocking finding and needs a decision (VPN, phone hotspot, or a different endpoint)
rather than a discovery on the day.

```
CHECK:  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest -q      # sim test suite, no GPU needed
        curl -s -o /dev/null -w '%{http_code}\n' "$OPENAI_BASE_URL/models" -H "Authorization: Bearer $OPENAI_API_KEY"
```

### What of the pipeline is actually used on the laptop

| Used | Not used |
|---|---|
| `utp/` — the pipeline package | `isaac_worker/` — needs Isaac, not installed here |
| `config/` — robot, sensors, methods, detectors, scenes | `office_building.usd` and the USD assets |
| `nav2_bringup/` — params and launch, **adapted for real** | `runs/` — workstation results |

Nav2 needs two changes for hardware, both easy to forget:
- **Regulated Pure Pursuit, not MPPI with `motion_model: Omni`.** The Ranger cannot execute
  simultaneous strafe+yaw (see GAP 1 in `HARDWARE_SPECS.md`), and MPPI-Omni's whole advantage is
  exactly that capability.
- **`use_sim_time:=false` everywhere.** The sim configs default it true; the failure mode is nodes
  waiting silently forever for a clock that never ticks.

## Stage 6 — Lidar

```bash
cd ~/utp_robot && bash bringup/lidar.sh
```

In another terminal:

```bash
source ~/utp_robot/bringup/env.sh
python3 bringup/check_scan_geometry.py --tf
```

```
CHECK:  /scan publishing ~7 Hz, 360 beams, frame lidar_link
        base_link -> lidar_link transform resolves
        an object 1 m in FRONT reads ~0 deg; an object on the LEFT reads ~ +90 deg
```

That last line is a **physical** check and cannot be skipped. A mirrored scan looks perfectly
healthy in the message fields and builds a map that navigates catastrophically. If left reads −90°,
fix it with the driver's `inverted`/`flip_x_axis` parameter, never by negating angles downstream.

## Stage 7 — Base

```bash
sudo ip link set can0 up type can bitrate 500000
candump can0                       # heartbeat frames with the chassis powered
ros2 launch ranger_bringup ranger_mini_v3.launch.py
```

If the driver runs but nothing moves: check the **RC mode switch and both E-stops** before touching
software. That is fault `0x80` and it presents exactly like a software bug.

**The most important measurement of the entire bring-up** — does the driver stop when its publisher
dies? Publish a small twist, kill the publisher, watch the wheels:

```
CHECK:  ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.1}}' -r 10
        ... then Ctrl-C and observe.
```

If the base keeps moving, **stop all driving** and write the watchdog first. A base that holds its
last command after its controller dies is a runaway.

## Stage 8 — Arm

Set a static IP on the arm's interface in the control box's subnet, then:

```bash
python3 -c "
from xarm.wrapper import XArmAPI
a = XArmAPI('192.168.1.xxx'); print(a.get_version()); print(a.get_servo_angle())"
```

Do this with a **hardware E-stop within reach** and reduced speed limits, before any Cartesian
motion. Remember the SDK's Cartesian API is **millimetres** and our stack is metres.

## Stage 9 — Safety stack

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/ -q     # 23 tests, logic only
python3 safety/twist_mux_node.py
python3 safety/arm_monitor_node.py --backend xarm_sdk
```

Then rehearse each failure with a human on the RC and hands clear:

```
CHECK:  arm extended  -> base refuses to move
        arm monitor killed -> base refuses to move (fail-closed)
        commanding publisher killed mid-motion -> base stops
        /safety/estop -> immediate stop, and it stays latched through a topic flap
        teleop preempts nav
```

**The tests only cover the decision logic; the nodes had never run against a moving robot as of
2026-08-18.** This gate is what makes that real.

## Stage 10 — Calibration

`docs/CALIBRATION.md`, in the dependency order given there. Then map the site with `slam_toolbox`,
tune AMCL, and only then attempt a mission.

---

## Quick reference

```bash
source ~/utp_robot/bringup/env.sh     # ROS + workspace + ROS_DOMAIN_ID=9 + conda scrubbed
bash bringup/setup_workspace.sh       # rebuild drivers (idempotent)
bash bringup/lidar.sh                 # /scan + base_link->lidar_link
python3 bringup/probe_rplidar.py      # raw serial probe, no ROS needed
python3 bringup/preflight.py -v       # collision + stale-port check
python3 bringup/check_scan_geometry.py --tf
ros2 topic echo /safety/status
ros2 service call /safety/clear_estop std_srvs/srv/Trigger
```
