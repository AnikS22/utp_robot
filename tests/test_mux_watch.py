"""The autonomous runner must notice when the safety mux is discarding its commands."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from safety.mux_watch import HINTS, MuxWatch


def test_silence_from_the_mux_is_a_failure_not_a_wait():
    """No /safety/status at all means no mux, which means /cmd_vel has no publisher."""
    w = MuxWatch(0.0, startup_grace_s=5.0)
    assert w.verdict(4.9).ok                     # still inside the grace window
    v = w.verdict(5.1)
    assert not v.ok and "not running" in v.reason


def test_persistent_block_while_asking_for_motion_aborts():
    w = MuxWatch(0.0, block_grace_s=2.0)
    for t in (0.0, 1.0, 2.0, 3.0):
        w.note_status("arm_not_stowed", t)
        w.note_command(True, t)
    v = w.verdict(3.0)
    assert not v.ok
    assert "arm_not_stowed" in v.reason
    assert HINTS["arm_not_stowed"].split()[0] in v.reason   # the remedy travels with the reason


def test_block_while_commanding_zero_is_not_a_fault():
    """The mux says no_source for one input timeout after we stop. That is correct, not broken."""
    w = MuxWatch(0.0, block_grace_s=2.0)
    for t in (0.0, 1.0, 2.0, 3.0, 4.0):
        w.note_status("no_source", t)
        w.note_command(False, t)
    assert w.verdict(4.0).ok


def test_brief_block_at_startup_is_tolerated():
    """Gates settle. Only a block that outlasts the grace window is real."""
    w = MuxWatch(0.0, block_grace_s=2.0)
    w.note_status("arm_not_stowed", 0.0)
    w.note_command(True, 0.0)
    assert w.verdict(1.0).ok
    for t in (1.5, 2.0, 2.5, 3.0):    # gate came good; mux keeps publishing at rate
        w.note_status(None, t)
        w.note_command(True, t)
    assert w.verdict(3.0).ok


def test_a_new_block_reason_restarts_the_clock():
    w = MuxWatch(0.0, block_grace_s=2.0)
    w.note_status("no_source", 0.0)
    w.note_command(True, 0.0)
    for t in (1.5, 2.0, 2.6):
        w.note_status("arm_not_stowed", t)
        w.note_command(True, t)
    assert w.verdict(2.6).ok                     # 1.1 s into the NEW reason, not 2.6
    for t in (3.0, 3.6):
        w.note_status("arm_not_stowed", t)
        w.note_command(True, t)
    assert not w.verdict(3.6).ok


def test_mux_dying_mid_leg_is_caught():
    w = MuxWatch(0.0, stale_s=1.0)
    w.note_status(None, 0.0)
    w.note_command(True, 0.0)
    assert w.verdict(0.5).ok
    v = w.verdict(2.0)
    assert not v.ok and "stopped publishing" in v.reason


def test_clock_starts_when_we_start_asking_not_when_the_block_started():
    """Blocked long before we commanded anything: the grace window is ours, not the mux's."""
    w = MuxWatch(0.0, block_grace_s=2.0)
    for t in (0.0, 1.0, 2.0, 3.0, 4.0):
        w.note_status("arm_not_stowed", t)
        w.note_command(False, t)
    for t in (4.0, 4.5, 5.0):
        w.note_status("arm_not_stowed", t)
        w.note_command(True, t)
    assert w.verdict(5.0).ok                     # only 1 s of us actually asking
    for t in (5.5, 6.1):
        w.note_status("arm_not_stowed", t)
        w.note_command(True, t)
    assert not w.verdict(6.1).ok


def test_a_blocking_pause_is_not_a_dead_mux():
    """The --confirm prompt blocks the thread, so no status is processed while it waits.

    Without resume() the run aborted with "the safety mux stopped publishing" after four
    seconds at a prompt, while the mux was alive and logging "base motion permitted".
    """
    w = MuxWatch(0.0, stale_s=1.0)
    w.note_status(None, 0.0)
    w.note_command(True, 0.0)
    assert not w.verdict(4.0).ok          # looks dead: nobody spun for 4 s
    w.resume(4.0)                          # operator pressed Enter; we spin again
    assert w.verdict(4.0).ok
    w.note_status(None, 4.1)
    assert w.verdict(4.2).ok


def test_resume_still_catches_a_genuinely_dead_mux():
    """Resuming must not blind the check -- silence after a resume is still a failure."""
    w = MuxWatch(0.0, stale_s=1.0, startup_grace_s=5.0)
    w.resume(100.0)
    assert w.verdict(104.0).ok             # inside the fresh grace window
    v = w.verdict(105.1)
    assert not v.ok and "not running" in v.reason
