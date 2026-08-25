"""Pure decision logic for the keyboard teleop guards — no rclpy, no HTTP, no browser.

Same split as ``arbiter.py``: the ROS node is a shell that pumps state in and publishes the
decision out; everything that decides *whether a teleop command is allowed to move the base* lives
here as plain Python so it can be exercised headlessly.

WHY THIS FILE EXISTS
--------------------
On 2026-08-20 the base kept driving after the operator's hands left the keyboard and only the
hardware E-stop stopped it. The heartbeat watchdog did not fire, and could not have: the page was
alive and healthy, posting a stale belief that a key was still held after a ``keyup`` was dropped
while the JS thread was saturated rendering the camera.

The lesson is narrow and worth stating exactly: **a heartbeat proves the sender is alive, not that
a human is still asking for motion.** Those are different claims and need different evidence.

TWO INDEPENDENT GUARDS
----------------------
``heartbeat``  — is the commanding page still there at all?      (covers: tab closed, browser died,
                 network stalled)
``hold lease`` — is a key still PHYSICALLY down?                 (covers: dropped keyup, frozen
                 event loop, a page lying by omission)

The hold lease uses key AUTOREPEAT as its evidence. While a key is physically down the browser
emits keydown events every few tens of milliseconds; when it is released they stop immediately.
Unlike ``keyup``, a dropped autorepeat is self-correcting — another arrives milliseconds later.

Both fail closed, and ``stopped`` (the latched SPACE stop) beats everything except the absence of a
heartbeat, which beats it only because it produces the same answer.
"""
from __future__ import annotations

from dataclasses import dataclass

WATCHDOG_S = 0.35
HOLD_LEASE_S = 1.0
EPS = 1e-9


@dataclass(frozen=True)
class Command:
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0

    def is_zero(self) -> bool:
        return abs(self.vx) < EPS and abs(self.vy) < EPS and abs(self.wz) < EPS


ZERO = Command()


@dataclass(frozen=True)
class Verdict:
    command: Command
    override: bool
    reason: str          # "ok" | "no_heartbeat" | "stop_latched" | "hold_lease_expired"

    @property
    def moving(self) -> bool:
        return not self.command.is_zero()


def decide(cmd: Command, *, override: bool, stopped: bool, heartbeat_age_s: float,
           key_age_s: float, watchdog_s: float = WATCHDOG_S,
           hold_lease_s: float = HOLD_LEASE_S) -> Verdict:
    """Resolve a browser-reported command into what may actually be published.

    heartbeat_age_s : seconds since the last POST from the page. Pass ``inf`` for "never heard".
    key_age_s       : seconds since the page's last keydown (autorepeat counts). ``inf`` == no
                      key evidence at all, which must never authorise motion.
    """
    if not (heartbeat_age_s <= watchdog_s):
        # `not (x <= t)` rather than `x > t` so NaN lands here too — an unparsable age is no
        # evidence, and no evidence means no motion.
        # override is dropped as well: a page that is gone cannot be asserting human supervision,
        # which is the entire meaning of that gate.
        return Verdict(ZERO, False, "no_heartbeat")

    if stopped:
        return Verdict(ZERO, override, "stop_latched")

    if not cmd.is_zero() and not (key_age_s <= hold_lease_s):
        return Verdict(ZERO, override, "hold_lease_expired")

    return Verdict(cmd, override, "ok")
