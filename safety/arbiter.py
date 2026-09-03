"""Pure decision logic for the base-motion safety arbiter — no rclpy, no Isaac, no I/O.

Everything that decides *whether the base is allowed to move* lives here as plain Python so it
can be exercised headlessly in ``tests/test_safety_arbiter.py``. The ROS node
(``twist_mux_node.py``) is a thin shell that pumps messages in and publishes the decision out.
Same split as ``utp/control/ranger_4ws.py`` (pure controller) and ``teleop_office.py`` (pure
helpers): the part that can hurt someone is the part that gets unit-tested.

WHY THIS EXISTS
---------------
Before this module, nothing in the stack gated base motion on arm state. ``act()`` happens to be
written sequentially — approach, send ``/arm_reach/goal``, block on the result, retreat — so the
base never moved with the arm extended. That safety was *emergent from control flow*, and control
flow is exactly what stops being trustworthy on hardware: a Nav2 recovery behaviour, a stale
twist, a pipeline crash mid-press, or a Ctrl-C at the wrong moment all produce base motion without
going through ``act()`` at all.

Driving with the arm extended is bad for four independent reasons, and each one shapes a rule below:

  * The Nav2 footprint is the chassis (``[[0.36, 0.25] ... [-0.36, 0.25]]``). Reach is 0.764 m plus a
    0.12 m stylus, so an extended arm sits ~0.88 m outside the polygon the planner collision-checks
    against. The planner will route the base through space the arm occupies.
  * The Ranger is 4WS and can spin about its centre. At 1.2 rad/s the tool tip sweeps ~1 m/s through
    an arc the robot believes is empty. -> ``max_wz`` is clamped hardest.
  * Tipping. High-CoM tip was already a real failure in sim; the riser raised the CG further.
    -> slew limiting on acceleration.
  * During a press the stylus is *touching* a plate. Base motion then side-loads the wrist joints.

FAIL-CLOSED IS THE WHOLE DESIGN
-------------------------------
Every gate treats *silence* as "not permitted", never as "carry on". A gate that has gone stale is
indistinguishable from a gate whose publisher has crashed, and the crashed case is precisely what
we are protecting against. The single most important line in this file is the ``_fresh()`` default.

Correspondingly ``step()`` is meant to be called on a fixed timer and its result published
unconditionally, including zeros. An arbiter that stops publishing when it decides to block is
indistinguishable downstream from an arbiter that died.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional


# --------------------------------------------------------------------------------------------------
# Value types
# --------------------------------------------------------------------------------------------------
@dataclass(frozen=True)
class Twist3:
    """A body twist. Matches geometry_msgs/Twist's used components (Ranger is planar)."""
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0

    def is_zero(self, eps: float = 1e-9) -> bool:
        return abs(self.vx) < eps and abs(self.vy) < eps and abs(self.wz) < eps


ZERO = Twist3()


@dataclass(frozen=True)
class Limits:
    """Speed ceilings and slew ceilings.

    ``max_decel_*`` is deliberately allowed to exceed ``max_accel_*``: ramping *up* gently protects
    against tipping, but refusing to stop promptly is never the safer trade.
    """
    max_vx: float = 0.6
    max_vy: float = 0.4
    max_wz: float = 0.8
    max_accel_lin: float = 0.5      # m/s^2, magnitude increasing
    max_decel_lin: float = 1.5      # m/s^2, magnitude decreasing
    max_accel_ang: float = 1.0      # rad/s^2, magnitude increasing
    max_decel_ang: float = 3.0      # rad/s^2, magnitude decreasing

    def scaled(self, factor: float) -> "Limits":
        """Speed ceilings scaled; slew ceilings untouched (they are about dynamics, not authority)."""
        return Limits(
            max_vx=self.max_vx * factor,
            max_vy=self.max_vy * factor,
            max_wz=self.max_wz * factor,
            max_accel_lin=self.max_accel_lin,
            max_decel_lin=self.max_decel_lin,
            max_accel_ang=self.max_accel_ang,
            max_decel_ang=self.max_decel_ang,
        )


@dataclass(frozen=True)
class SourceSpec:
    """One command publisher competing for the base.

    ``requires_enable`` marks a source as autonomous: it only gets through while the deadman is
    held. Teleop sets this False — a human already has their hand on the control, and requiring a
    second live topic to drive manually would break the one path you need when everything else has
    failed.

    ``allows_arm_override`` marks the source a human may use to move the base with the arm still
    extended, and only while ``override`` is asserted. Exactly one source (teleop) should set this:
    it is the recovery path for a stuck or faulted arm, not a normal operating mode.
    """
    name: str
    topic: str
    priority: int
    requires_enable: bool = True
    allows_arm_override: bool = False


@dataclass
class Decision:
    """What the arbiter decided this tick, and why. Published verbatim to /safety/status."""
    twist: Twist3 = ZERO
    source: Optional[str] = None          # source that won arbitration (None = nothing fresh)
    blocked_by: Optional[str] = None      # first gate that zeroed the output, if any
    estop_latched: bool = False
    override_active: bool = False
    gates: dict = field(default_factory=dict)   # gate name -> effective bool (post-staleness)
    source_ages: dict = field(default_factory=dict)

    @property
    def moving(self) -> bool:
        return not self.twist.is_zero()


# --------------------------------------------------------------------------------------------------
# Helpers
# --------------------------------------------------------------------------------------------------
def _clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else (hi if v > hi else v)


def _slew(current: float, target: float, dt: float, accel: float, decel: float) -> float:
    """Move ``current`` toward ``target``, rate-limited.

    Uses the accel cap when |target| > |current| or the sign flips (a sign flip passes through
    zero, so treating it as acceleration is the conservative reading), and the decel cap when the
    command is shrinking toward zero.

    ``dt <= 0`` holds the current value rather than jumping to the target: no time has elapsed, so
    no change is permitted. Letting it pass the target through is how a restarted mux would slam
    to full speed on its first tick.
    """
    if dt <= 0.0:
        return current
    growing = abs(target) > abs(current) or (current != 0.0 and target * current < 0.0)
    rate = accel if growing else decel
    max_step = rate * dt
    delta = target - current
    if delta > max_step:
        return current + max_step
    if delta < -max_step:
        return current - max_step
    return target


# --------------------------------------------------------------------------------------------------
# The arbiter
# --------------------------------------------------------------------------------------------------
class SafetyArbiter:
    """Priority mux + interlocks + slew limiter.

    Time is passed in explicitly (``now``, seconds, monotonic) rather than read from a clock, so
    tests are deterministic and the node can feed it either wall time or ROS time.

    Typical wiring::

        arb = SafetyArbiter(sources=[teleop, servo, nav], limits=...)
        # on each incoming Twist:      arb.submit("nav", Twist3(...), now)
        # on each incoming gate Bool:  arb.set_gate("arm_stowed", True, now)
        # on a fixed timer:            decision = arb.step(now); publish(decision.twist)
    """

    #: Gates that must be True for autonomous motion. Order matters only for which reason is
    #: reported first when several are unsatisfied; it is chosen to name the most alarming cause.
    GATES = ("estop", "arm_stowed", "enable", "override")

    def __init__(
        self,
        sources: list[SourceSpec],
        limits: Limits = Limits(),
        override_speed_factor: float = 0.25,
        input_timeout_s: float = 0.3,
        gate_timeout_s: float = 0.2,
        nominal_dt_s: float = 0.05,
        require_arm_stowed: bool = True,
    ) -> None:
        if not sources:
            raise ValueError("SafetyArbiter needs at least one SourceSpec")
        names = [s.name for s in sources]
        if len(set(names)) != len(names):
            raise ValueError(f"duplicate source names: {names}")
        # Highest priority first, so selection is a linear scan.
        self.sources = sorted(sources, key=lambda s: -s.priority)
        self.limits = limits
        # POLICY, NOT A DEFAULT TO TWEAK LIGHTLY. When False, an extended arm no longer blocks
        # base motion for any source. Set by the operator 2026-09-03 for the elevator task: the
        # robot must press a call button and then move into the car before the doors close, and
        # waiting for a full fold between the press and the drive does not fit in that window.
        #
        # WHAT IT COSTS. The interlock exists because the arm reaches ~0.88 m from link_base and
        # the chassis coasts ~1.26 s / ~18 cm after commands stop (measured 2026-08-21). With this
        # False, a leg driven with the arm out can put the tool into a door frame or a wall, and
        # nothing in software will stop it -- there is no force sensor on this arm
        # (get_ft_sensor_data answers zeros, collision_sensitivity is 0). The e-stop is the only
        # protection, which is exactly how the operator runs this robot.
        #
        # estop is UNAFFECTED and still hard-blocks. This flag touches only arm_stowed.
        self.require_arm_stowed = bool(require_arm_stowed)
        self.override_limits = limits.scaled(override_speed_factor)
        self.input_timeout_s = input_timeout_s
        self.gate_timeout_s = gate_timeout_s
        # dt assumed for the very first tick, when there is no previous timestamp to difference
        # against. Set it to the node's publish period so slew limiting is in force from tick one.
        self.nominal_dt_s = nominal_dt_s

        self._cmds: dict[str, tuple[Twist3, float]] = {}     # name -> (twist, stamp)
        self._gates: dict[str, tuple[bool, float]] = {}      # name -> (value, stamp)
        self._estop_latched = False
        self._out = Twist3()          # last published twist, for slew continuity
        self._last_step: Optional[float] = None

    # ---- inputs ----------------------------------------------------------------------------
    def submit(self, name: str, twist: Twist3, now: float) -> None:
        """Record a command from ``name``. Unknown names are ignored, not raised: a stray publisher
        must never be able to crash the one node whose job is to keep publishing."""
        if any(s.name == name for s in self.sources):
            self._cmds[name] = (twist, now)

    def set_gate(self, name: str, value: bool, now: float) -> None:
        """Record a gate observation. ``estop`` latches on True and is cleared only by
        :meth:`clear_estop` — an E-stop that un-latches because a topic flapped is not an E-stop."""
        if name not in self.GATES:
            return
        self._gates[name] = (bool(value), now)
        if name == "estop" and value:
            self._estop_latched = True

    def clear_estop(self) -> None:
        """Explicit human re-arm. Deliberately not reachable from a Bool topic."""
        self._estop_latched = False
        self._gates.pop("estop", None)

    # ---- gate evaluation -------------------------------------------------------------------
    def _fresh(self, entry: Optional[tuple], now: float, timeout: float, default: bool) -> bool:
        """THE fail-closed primitive. Never-seen and stale both collapse to ``default``, and every
        permissive gate passes default=False."""
        if entry is None:
            return default
        value, stamp = entry
        if (now - stamp) > timeout:
            return default
        return bool(value)

    def gate(self, name: str, now: float) -> bool:
        return self._fresh(self._gates.get(name), now, self.gate_timeout_s, default=False)

    # ---- the tick --------------------------------------------------------------------------
    def step(self, now: float) -> Decision:
        """Arbitrate, interlock, limit. Call on a fixed timer and publish the result every time,
        zeros included."""
        dt = self.nominal_dt_s if self._last_step is None else max(0.0, now - self._last_step)
        self._last_step = now

        arm_stowed = self.gate("arm_stowed", now)
        enable = self.gate("enable", now)
        override = self.gate("override", now)
        gates = {"estop_latched": self._estop_latched, "arm_stowed": arm_stowed,
                 "enable": enable, "override": override}
        ages = {name: round(now - stamp, 3) for name, (_, stamp) in self._cmds.items()}

        def blocked(reason: str, hard: bool) -> Decision:
            # A hard block skips the slew limiter: an E-stop must not politely ramp down.
            self._out = ZERO if hard else self._decelerate(dt)
            return Decision(twist=self._out, source=None, blocked_by=reason,
                            estop_latched=self._estop_latched, override_active=override,
                            gates=gates, source_ages=ages)

        if self._estop_latched:
            return blocked("estop", hard=True)

        # Highest-priority source with a command that has not gone stale.
        chosen: Optional[SourceSpec] = None
        for spec in self.sources:
            entry = self._cmds.get(spec.name)
            if entry is not None and (now - entry[1]) <= self.input_timeout_s:
                chosen = spec
                break
        if chosen is None:
            # Not a fault — nobody is asking for motion. Ramp down rather than jerk to zero.
            return blocked("no_source", hard=False)

        if chosen.requires_enable and not enable:
            return blocked("deadman", hard=False)

        limits = self.limits
        if not arm_stowed and self.require_arm_stowed:
            if not (chosen.allows_arm_override and override):
                # THE interlock. Fail-closed: a stale arm monitor lands here too.
                return blocked("arm_not_stowed", hard=False)
            # Human-supervised recovery with the arm out — permitted, but crawling.
            limits = self.override_limits

        target = self._cmds[chosen.name][0]
        target = Twist3(
            vx=_clamp(target.vx, -limits.max_vx, limits.max_vx),
            vy=_clamp(target.vy, -limits.max_vy, limits.max_vy),
            wz=_clamp(target.wz, -limits.max_wz, limits.max_wz),
        )
        self._out = Twist3(
            vx=_slew(self._out.vx, target.vx, dt, limits.max_accel_lin, limits.max_decel_lin),
            vy=_slew(self._out.vy, target.vy, dt, limits.max_accel_lin, limits.max_decel_lin),
            wz=_slew(self._out.wz, target.wz, dt, limits.max_accel_ang, limits.max_decel_ang),
        )
        return Decision(twist=self._out, source=chosen.name, blocked_by=None,
                        estop_latched=False, override_active=(limits is self.override_limits),
                        gates=gates, source_ages=ages)

    def _decelerate(self, dt: float) -> Twist3:
        """Ramp the current output to zero under the decel caps (soft block)."""
        return Twist3(
            vx=_slew(self._out.vx, 0.0, dt, self.limits.max_accel_lin, self.limits.max_decel_lin),
            vy=_slew(self._out.vy, 0.0, dt, self.limits.max_accel_lin, self.limits.max_decel_lin),
            wz=_slew(self._out.wz, 0.0, dt, self.limits.max_accel_ang, self.limits.max_decel_ang),
        )
