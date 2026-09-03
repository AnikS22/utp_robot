"""ROS 2 node: the ONLY publisher of /cmd_vel. Thin shell around SafetyArbiter.

    ros2 run  — not packaged yet; run directly:
        python3 real_world/safety/twist_mux_node.py [--config real_world/config/safety.yaml]

All decision logic lives in ``arbiter.py`` (pure, unit-tested). This file does message plumbing
only, so the dangerous part stays testable without a ROS install.

WHY A MUX AT ALL
----------------
In the sim stack, Nav2 and ``utp/pipeline/isaac_world.py`` both publish straight to /cmd_vel. That
means there is no single point where motion can be vetoed. Here, every producer publishes to its
own topic and this node arbitrates, which is what makes the arm interlock, the deadman, the
E-stop, and the slew limiter possible at all.

This node has NO Isaac dependency and NO dependency on the utp package. It is the one piece of
real-robot code that can be written and fully validated before any driver is installed, and the
identical file runs against hardware.

OPERATING NOTE
--------------
Software is layer 2. Layer 0 is the chassis and arm-controller hardware E-stops, layer 1 is the
Ranger RC transmitter, which revokes CAN command authority at the driver — below anything this
node can affect. Whoever is standing next to the robot holds the RC. This node does not replace
that; it covers the failures that happen faster than a human reacts.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path
from typing import Any

import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

sys.path.insert(0, str(Path(__file__).resolve().parent))
from arbiter import Limits, SafetyArbiter, SourceSpec, Twist3   # noqa: E402


DEFAULT_CONFIG = Path(__file__).resolve().parents[1] / "config" / "safety.yaml"


def _expand(obj: Any) -> Any:
    """Resolve ``${ENV:NAME:default}`` the same way config/paths.yaml does, so the real xArm IP can
    come from the environment without editing the file on every machine."""
    if isinstance(obj, str):
        if obj.startswith("${ENV:") and obj.endswith("}"):
            inner = obj[6:-1]
            name, _, default = inner.partition(":")
            return os.environ.get(name, default)
        return obj
    if isinstance(obj, dict):
        return {k: _expand(v) for k, v in obj.items()}
    if isinstance(obj, list):
        return [_expand(v) for v in obj]
    return obj


def load_config(path: Path) -> dict:
    with open(path) as f:
        return _expand(yaml.safe_load(f) or {})


class TwistMuxNode(Node):
    def __init__(self, cfg: dict) -> None:
        super().__init__("utp_safety_twist_mux")
        self.cfg = cfg

        lim = cfg.get("limits", {})
        specs = [
            SourceSpec(
                name=s["name"],
                topic=s["topic"],
                priority=int(s["priority"]),
                requires_enable=bool(s.get("requires_enable", True)),
                allows_arm_override=bool(s.get("allows_arm_override", False)),
            )
            for s in cfg["sources"]
        ]
        timeouts = cfg.get("timeouts", {})
        rate = float(cfg.get("rate_hz", 20.0))
        self.arb = SafetyArbiter(
            sources=specs,
            limits=Limits(**{k: float(v) for k, v in lim.items()}),
            override_speed_factor=float(cfg.get("override_speed_factor", 0.25)),
            input_timeout_s=float(timeouts.get("input_s", 0.3)),
            gate_timeout_s=float(timeouts.get("gate_s", 0.2)),
            nominal_dt_s=1.0 / rate,
            # Defaults TRUE, so omitting it from safety.yaml keeps the interlock.
            require_arm_stowed=bool(cfg.get("require_arm_stowed", True)),
        )
        if not bool(cfg.get("require_arm_stowed", True)):
            self.get_logger().warn(
                "require_arm_stowed: false -- the base WILL move with the arm extended. "
                "There is no force sensor on this arm; the e-stop is the only protection.")

        # --- command sources ---
        for spec in specs:
            self.create_subscription(
                Twist, spec.topic,
                lambda msg, name=spec.name: self._on_cmd(name, msg),
                10,
            )

        # --- gates ---
        for gate, topic in cfg.get("gates", {}).items():
            self.create_subscription(
                Bool, topic,
                lambda msg, g=gate: self.arb.set_gate(g, msg.data, self._now()),
                qos_profile_sensor_data,
            )

        self.pub_cmd = self.create_publisher(Twist, cfg.get("output_topic", "/cmd_vel"), 10)
        self.pub_status = self.create_publisher(String, cfg.get("status_topic", "/safety/status"), 10)

        # Clearing a latched E-stop is a service, not a topic, on purpose: re-arming the robot
        # should require a deliberate act that cannot happen because a Bool flapped.
        self.create_service(Trigger, "/safety/clear_estop", self._on_clear_estop)

        self.create_timer(1.0 / rate, self._tick)

        self._last_block: str | None = "startup"
        self.get_logger().info(
            f"safety mux up: {[s.topic for s in specs]} -> {cfg.get('output_topic')} "
            f"@ {rate:g} Hz; all gates fail-closed"
        )

    # ---- plumbing --------------------------------------------------------------------------
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_cmd(self, name: str, msg: Twist) -> None:
        self.arb.submit(name, Twist3(msg.linear.x, msg.linear.y, msg.angular.z), self._now())

    def _on_clear_estop(self, _req, resp):
        self.arb.clear_estop()
        self.get_logger().warn("E-stop latch CLEARED by service call")
        resp.success = True
        resp.message = "estop latch cleared"
        return resp

    def _tick(self) -> None:
        d = self.arb.step(self._now())

        out = Twist()
        out.linear.x, out.linear.y, out.angular.z = d.twist.vx, d.twist.vy, d.twist.wz
        self.pub_cmd.publish(out)   # EVERY tick, zeros included — see arbiter.py

        self.pub_status.publish(String(data=json.dumps({
            "source": d.source, "blocked_by": d.blocked_by, "estop_latched": d.estop_latched,
            "override_active": d.override_active, "gates": d.gates, "source_ages": d.source_ages,
            "twist": {"vx": round(d.twist.vx, 4), "vy": round(d.twist.vy, 4),
                      "wz": round(d.twist.wz, 4)},
        })))

        # Log only on transitions — at 20 Hz a steady-state block would drown the console, and a
        # console nobody reads is a safety system nobody notices failing.
        if d.blocked_by != self._last_block:
            if d.blocked_by:
                self.get_logger().warn(f"base motion BLOCKED: {d.blocked_by} gates={d.gates}")
            else:
                self.get_logger().info(f"base motion permitted, source={d.source}")
            self._last_block = d.blocked_by


def main(argv: list[str] | None = None) -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    args, ros_args = ap.parse_known_args(argv)

    cfg = load_config(args.config)
    if not cfg.get("enabled", True):
        print(f"[safety] disabled in {args.config} — refusing to start", file=sys.stderr)
        sys.exit(1)

    rclpy.init(args=ros_args)
    node = TwistMuxNode(cfg)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Parting zero, and it matters more than "best-effort" suggests. Measured 2026-08-21:
        # there is NO driver-side watchdog -- ranger_base transmits one 0x111 per callback and
        # simply stops when its publisher dies -- and the CHASSIS watchdog takes 1.26 s, about
        # 18 cm at 0.15 m/s. An explicit zero stops the base at once because it is a command,
        # not a timeout. So this line is the difference between stopping now and coasting 18 cm.
        #
        # It is still only a best effort in the sense that SIGKILL skips it entirely. That case
        # is exactly the 1.26 s coast, and nothing in software can shorten it.
        try:
            node.pub_cmd.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
