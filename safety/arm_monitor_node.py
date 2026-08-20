"""ROS 2 node: publishes /safety/arm_stowed, the gate the base interlock depends on.

    python3 real_world/safety/arm_monitor_node.py [--config real_world/config/safety.yaml]
                                                  [--backend scene_state|xarm_sdk]

Two backends, ONE output topic, so ``twist_mux_node.py`` is byte-identical in sim and on hardware:

  scene_state : sim. Reads /scene/state JSON and treats ``arm_state == "idle"`` as stowed. This is
                a belief, not a measurement — acceptable only because the simulated arm cannot
                actually be somewhere its FSM does not think it is.
  xarm_sdk    : hardware. Reads joint angles from XArmAPI and compares them against the configured
                stow pose. This is a MEASUREMENT, and the distinction is the whole point: an FSM
                reporting "idle" after the process that owns it crashed is precisely the situation
                the interlock exists to catch. On real hardware we never trust a belief.

Both backends publish False when their evidence goes stale. The publisher going silent is also
safe, because the mux's own gate timeout collapses a missing gate to False. Two independent
fail-closed mechanisms, deliberately.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

sys.path.insert(0, str(Path(__file__).resolve().parent))
from twist_mux_node import DEFAULT_CONFIG, load_config   # noqa: E402


class ArmMonitorNode(Node):
    def __init__(self, cfg: dict, backend: str | None = None) -> None:
        super().__init__("utp_safety_arm_monitor")
        mon = cfg.get("arm_monitor", {})
        self.backend = backend or mon.get("backend", "scene_state")
        self.stale_after_s = float(mon.get("stale_after_s", 0.5))

        topic = cfg.get("gates", {}).get("arm_stowed", "/safety/arm_stowed")
        self.pub = self.create_publisher(Bool, topic, 10)

        self._stowed = False          # start blocked; nothing has proven otherwise yet
        self._last_evidence: float | None = None

        if self.backend == "scene_state":
            self.create_subscription(
                String, mon.get("scene_state_topic", "/scene/state"), self._on_scene_state, 10)
        elif self.backend == "xarm_sdk":
            self._init_xarm(mon.get("xarm", {}))
        else:
            raise ValueError(f"unknown arm_monitor backend '{self.backend}'")

        rate = float(mon.get("publish_rate_hz", 20.0))
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(f"arm monitor up: backend={self.backend} -> {topic} @ {rate:g} Hz")

    # ---- sim backend -----------------------------------------------------------------------
    def _on_scene_state(self, msg: String) -> None:
        try:
            st = json.loads(msg.data)
        except (ValueError, TypeError):
            return    # malformed -> no evidence -> goes stale -> False
        self._stowed = (st.get("arm_state") == "idle")
        self._last_evidence = self._now()

    # ---- hardware backend ------------------------------------------------------------------
    def _init_xarm(self, xc: dict) -> None:
        from xarm.wrapper import XArmAPI   # imported lazily: absent on the sim workstation

        self._stow = [float(a) for a in xc.get("stow_pose_deg", [0, -45, -45, 0, 90, 0])]
        self._tol = float(xc.get("joint_tolerance_deg", 5.0))
        ip = xc.get("ip")
        self.get_logger().info(f"connecting to xArm at {ip}")
        self._arm = XArmAPI(ip, is_radian=False)
        self.create_timer(0.05, self._poll_xarm)

    def _poll_xarm(self) -> None:
        code, angles = self._arm.get_servo_angle(is_radian=False)
        if code != 0 or angles is None:
            return    # read failed -> no evidence -> stale -> False
        self._stowed = all(
            abs(a - s) <= self._tol for a, s in zip(angles[:len(self._stow)], self._stow))
        self._last_evidence = self._now()

    # ---- common ----------------------------------------------------------------------------
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _tick(self) -> None:
        fresh = (self._last_evidence is not None
                 and (self._now() - self._last_evidence) <= self.stale_after_s)
        self.pub.publish(Bool(data=bool(fresh and self._stowed)))


def main(argv: list[str] | None = None) -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    ap.add_argument("--backend", choices=["scene_state", "xarm_sdk"], default=None)
    args, ros_args = ap.parse_known_args(argv)

    rclpy.init(args=ros_args)
    node = ArmMonitorNode(load_config(args.config), args.backend)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.pub.publish(Bool(data=False))   # part on the safe value
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
