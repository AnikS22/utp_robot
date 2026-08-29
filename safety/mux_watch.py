"""Did the safety mux actually pass our commands through?

Pure logic, no rclpy, so it is unit-testable -- same split as ``arbiter.py`` / ``twist_mux_node.py``.

WHY THIS EXISTS. ``route_run.py`` published twists on /cmd_vel_teleop for a full 180-second leg
timeout without ever asking whether the mux forwarded a single one of them, then reported "leg
timed out". That reads like a navigation failure. It is not: the mux had been discarding every
command since the first tick because a fail-closed gate was False.

The human teleop page has subscribed to /safety/status since it was written, so an operator
driving by hand SEES `arm_not_stowed` in a red pill. The autonomous runner was the only thing
driving blind, which is why the same interlock felt like an intermittent robot fault. Every
mechanism was working exactly as designed and saying so on a topic nobody was listening to.

Fail-closed is still right. What was missing is that a closed gate must be LOUD.
"""
from __future__ import annotations

from dataclasses import dataclass

# What to actually do about each block reason. The mux names the gate; this names the fix.
HINTS = {
    "arm_not_stowed": ("the arm-stowed interlock is closed. Either the arm is genuinely not at "
                       "the stow pose (run bringup/stow_arm.py), or the arm monitor has no fresh "
                       "evidence -- check `ros2 topic hz /safety/arm_stowed` and that the value "
                       "is actually True, not merely present."),
    "deadman":        ("the /safety/enable deadman is not held. Autonomous sources (nav, servo) "
                       "require it; nothing publishes it unless an operator gamepad or a "
                       "supervisor is running."),
    "estop":          ("the E-stop is LATCHED. It clears only via the /safety/clear_estop "
                       "service -- releasing the physical button is not enough."),
    "no_source":      ("the mux is not receiving our commands at all. The twists are being "
                       "published but not arriving: check the topic name, and check "
                       "ROS_DOMAIN_ID matches (9 = hardware, 42 = sim)."),
}


@dataclass
class Verdict:
    ok: bool
    reason: str = ""

    def __bool__(self) -> bool:
        return self.ok


class MuxWatch:
    """Watches /safety/status against what we are asking the base to do.

    ``block_grace_s`` exists because a brief block is normal and not a fault: gates settle at
    startup, and the mux reports ``no_source`` for one input timeout after we stop commanding.
    Only a block that PERSISTS while we are still asking for motion is a real one.
    """

    def __init__(self, started_at: float, *, block_grace_s: float = 2.0,
                 startup_grace_s: float = 5.0, stale_s: float = 1.0) -> None:
        self.block_grace_s = float(block_grace_s)
        self.startup_grace_s = float(startup_grace_s)
        self.stale_s = float(stale_s)
        self._started = float(started_at)
        self._last_status: float | None = None
        self._blocked_by: str | None = None
        self._blocked_since: float | None = None
        self._asking_since: float | None = None

    # ---- inputs ----------------------------------------------------------------------------
    def note_status(self, blocked_by: str | None, now: float) -> None:
        self._last_status = now
        if blocked_by:
            if self._blocked_by != blocked_by:
                self._blocked_since = now       # a NEW reason restarts the clock
            self._blocked_by = blocked_by
        else:
            self._blocked_by = None
            self._blocked_since = None

    def note_command(self, moving: bool, now: float) -> None:
        """``moving`` is whether the twist we just published is non-zero. Blocking while we are
        commanding zero is not a fault -- it is what the mux is supposed to say."""
        if not moving:
            self._asking_since = None
        elif self._asking_since is None:
            self._asking_since = now

    # ---- output ----------------------------------------------------------------------------
    def verdict(self, now: float) -> Verdict:
        if self._last_status is None:
            if now - self._started >= self.startup_grace_s:
                return Verdict(False,
                               "no /safety/status in %.0fs -- the safety mux is not running, so "
                               "nothing is forwarding our commands to /cmd_vel. Start it "
                               "(safety/ stack) before driving." % self.startup_grace_s)
            return Verdict(True)

        if now - self._last_status > self.stale_s:
            return Verdict(False, "the safety mux stopped publishing /safety/status "
                                  "(%.1fs ago) -- it has died mid-leg." % (now - self._last_status))

        if self._blocked_by and self._asking_since is not None and self._blocked_since is not None:
            held = now - max(self._blocked_since, self._asking_since)
            if held >= self.block_grace_s:
                hint = HINTS.get(self._blocked_by, "")
                msg = ("safety mux has blocked every command for %.1fs: %s"
                       % (held, self._blocked_by))
                return Verdict(False, msg + ((" -- " + hint) if hint else ""))
        return Verdict(True)

    @property
    def blocked_by(self) -> str | None:
        return self._blocked_by

    @property
    def seen_status(self) -> bool:
        return self._last_status is not None
