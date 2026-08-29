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
        # Staleness must be sized against the rate the evidence ACTUALLY arrives at, and the two
        # backends differ by more than an order of magnitude: the xArm SDK is polled at 20 Hz,
        # while /scene/state is nominally 5 Hz but measured 0.55 Hz on this laptop headless
        # (2026-08-27) -- real time factor, not a fault. One shared 0.5 s window therefore held
        # the gate open on hardware and slammed it shut between every message in sim, which the
        # mux reported, correctly, as arm_not_stowed.
        #
        # It is deliberately NOT adaptive. Widening a safety window because evidence got slow is
        # how an interlock quietly stops interlocking; the window is declared, and a rate that
        # cannot support it is reported as the fault it is.
        self.stale_after_s = float(
            (mon.get(self.backend) or {}).get("stale_after_s", mon.get("stale_after_s", 0.5)))
        self._interval_warned = False

        topic = cfg.get("gates", {}).get("arm_stowed", "/safety/arm_stowed")
        self.pub = self.create_publisher(Bool, topic, 10)

        self._stowed = False          # start blocked; nothing has proven otherwise yet
        self._last_evidence: float | None = None

        if self.backend == "scene_state":
            self.create_subscription(
                String, mon.get("scene_state_topic", "/scene/state"), self._on_scene_state, 10)
        elif self.backend == "xarm_sdk":
            self._init_xarm(mon.get("xarm", {}))
        elif self.backend == "absent":
            self._init_absent(mon.get("xarm", {}))
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
        self._note_evidence()

    # ---- arm-not-fitted backend ------------------------------------------------------------
    def _init_absent(self, xc: dict) -> None:
        """Operator declares the arm is not on the robot. VERIFIED, not taken on trust.

        WHY THIS EXISTS AND WHY IT IS NOT A BYPASS. For a navigation-only trial the arm interlock
        guards nothing: there is no arm to sweep 0.88 m through space the costmap believes is
        empty. Refusing to drive would be safety theatre. But the honest way to express "no arm"
        is a DECLARATION THAT CAN BE FALSIFIED, not a flag that forces the gate true -- because
        the dangerous case is an operator who believes the arm is off the robot while it is
        actually fitted and extended.

        So the declaration is checked against the one piece of reality we can observe: if the
        controller answers on the network, the arm IS present, the declaration is false, and this
        node REFUSES TO START rather than publishing a gate value that is a lie. Selecting this
        backend on a robot with a live arm gets you an error, not permission.

        What it still cannot see: an arm that is fitted but powered down. That is why the
        declaration is explicit, loud, repeated in the log for the whole run, and surfaced by
        health.py -- so it appears in the record of any trial run this way.
        """
        import socket as _s
        ip = xc.get("ip")
        for port in (502, 30000, 30003):    # Modbus + the xArm SDK control/report ports
            try:
                with _s.create_connection((ip, port), timeout=1.0):
                    raise RuntimeError(
                        f"backend 'absent' declares no arm is fitted, but something IS answering "
                        f"at {ip}:{port}. The declaration is false. Use --backend xarm_sdk, or "
                        f"physically disconnect the arm if it really is not part of this trial.")
            except (OSError, _s.timeout):
                continue
        self._absent_ip = ip
        self.get_logger().warn(
            f"ARM DECLARED ABSENT -- nothing answers at {ip}. Publishing arm_stowed=True on an "
            f"OPERATOR DECLARATION, not a measurement. Valid only while no arm is fitted. This "
            f"is recorded for every trial run this way.")
        self.create_timer(1.0, self._poll_absent)
        self._absent_warns = 0

    def _poll_absent(self) -> None:
        self._stowed = True
        self._note_evidence()
        # Repeat in the log every 30 s. A declaration that scrolls away once is not a record.
        self._absent_warns += 1
        if self._absent_warns % 30 == 0:
            self.get_logger().warn("arm_stowed=True by DECLARATION (backend=absent), not measured")

    # ---- hardware backend ------------------------------------------------------------------
    def _init_xarm(self, xc: dict) -> None:
        from xarm.wrapper import XArmAPI   # imported lazily: absent on the sim workstation

        self._stow = [float(a) for a in xc.get("stow_pose_deg", [0, -45, -45, 0, 90, 0])]
        self._tol = float(xc.get("joint_tolerance_deg", 5.0))
        ip = xc.get("ip")
        self.get_logger().info(f"connecting to xArm at {ip}")
        self._ip = ip
        self._arm = XArmAPI(ip, is_radian=False)
        self._last_reconnect = 0.0
        self.create_timer(0.05, self._poll_xarm)

    def _reconnect(self) -> None:
        """Re-open the SDK session, at most once a second.

        WHY THIS IS NEEDED. Every other arm tool -- stow_arm.py, approach_target.py, xArm Studio --
        opens its own connection, and the controller drops this one when they do. Measured
        2026-08-26: after a stow the arm was verifiably AT the stow pose and this node was still
        publishing False, because its session had died and it had no way back. The node stayed
        alive, the topic stayed alive, and the only symptom was the base refusing to move.

        That is the failure this whole interlock is meant to catch, arriving from the wrong side:
        a gate stuck CLOSED is not dangerous, but it is indistinguishable from a navigation fault
        and it will burn an afternoon. Fail-closed is still correct -- the fix is to restore the
        evidence, never to assume stowed."""
        now = self._now()
        if now - self._last_reconnect < 1.0:
            return
        self._last_reconnect = now
        try:
            from xarm.wrapper import XArmAPI
            try:
                self._arm.disconnect()
            except Exception:
                pass
            self._arm = XArmAPI(self._ip, is_radian=False)
            self._arm.connect()
            if self._arm.connected:
                self.get_logger().info("xArm session re-established")
        except Exception as e:
            self.get_logger().warning(f"xArm reconnect failed: {type(e).__name__}", once=False)

    def _poll_xarm(self) -> None:
        if not getattr(self._arm, "connected", False):
            self._reconnect()
            return    # no session -> no evidence -> stale -> False
        try:
            code, angles = self._arm.get_servo_angle(is_radian=False)
        except Exception:
            self._reconnect()
            return
        if code != 0 or angles is None:
            self._reconnect()
            return    # read failed -> no evidence -> stale -> False
        self._stowed = all(
            abs(a - s) <= self._tol for a, s in zip(angles[:len(self._stow)], self._stow))
        self._note_evidence()

    # ---- common ----------------------------------------------------------------------------
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _note_evidence(self) -> None:
        """Record fresh evidence, and complain ONCE if it is arriving too slowly to hold the gate.

        A gate that collapses between messages is indistinguishable from an arm that is not
        stowed, and it is the mux -- three processes away -- that ends up reporting it. Saying it
        here, where the rate is known, turns a mystified afternoon into one log line."""
        now = self._now()
        if self._last_evidence is not None and not self._interval_warned:
            gap = now - self._last_evidence
            if gap > self.stale_after_s:
                self._interval_warned = True
                self.get_logger().error(
                    f"{self.backend} evidence arrived {gap:.2f}s apart but stale_after_s is "
                    f"{self.stale_after_s:.2f}s -- the arm_stowed gate collapses BETWEEN messages "
                    f"and the base will be blocked with 'arm_not_stowed'. Raise "
                    f"arm_monitor.{self.backend}.stale_after_s above the real message interval, "
                    f"or fix the rate.")
        self._last_evidence = now

    def _tick(self) -> None:
        fresh = (self._last_evidence is not None
                 and (self._now() - self._last_evidence) <= self.stale_after_s)
        self.pub.publish(Bool(data=bool(fresh and self._stowed)))


def main(argv: list[str] | None = None) -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    ap.add_argument("--backend", choices=["scene_state", "xarm_sdk", "absent"], default=None)
    args, ros_args = ap.parse_known_args(argv)

    # Take the signals OURSELVES. rclpy's default handler shuts the context down before the
    # `finally` below runs, so the parting False was published into a dead context and silently
    # swallowed by the except, and then rclpy.shutdown() raised
    # "rcl_shutdown already called". Measured 2026-08-29 on a plain SIGTERM.
    #
    # It mattered because that parting False is what CLOSES the gate the instant the monitor
    # stops, rather than leaving the mux to notice 0.2 s later on staleness. Two independent
    # fail-closed mechanisms is the design; one of them was not running.
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    stop = {"v": False}
    import signal as _sig
    _sig.signal(_sig.SIGINT, lambda *_: stop.__setitem__("v", True))
    _sig.signal(_sig.SIGTERM, lambda *_: stop.__setitem__("v", True))

    node = ArmMonitorNode(load_config(args.config), args.backend)
    try:
        while rclpy.ok() and not stop["v"]:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.pub.publish(Bool(data=False))   # part on the safe value, context still alive
        except Exception as e:
            node.get_logger().error(f"could not publish the parting arm_stowed=False: {e}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
