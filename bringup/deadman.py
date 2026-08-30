#!/usr/bin/env python3
"""Hold-to-enable: publishes /safety/enable while a human is holding a button.

    python3 bringup/deadman.py            # then open the printed URL and HOLD

WHY THIS HAS TO EXIST. config/safety.yaml marks the autonomous sources -- `nav` (Nav2) and
`servo` (the pipeline's approach/retreat) -- `requires_enable: true`, gated on /safety/enable.
NOTHING IN THIS REPO HAS EVER PUBLISHED THAT TOPIC. So the mux has been correctly discarding
every autonomous command since the day it was written, and Nav2 would plan a path, run MPPI,
publish to /cmd_vel_nav, and go precisely nowhere -- with odom flowing, the costmaps populated
and every node reporting healthy. That is the silent-discard failure class this project has now
hit seven times, and it is why this is a real deadman rather than a Bool latched to true.

WHY IT IS SHAPED LIKE THIS, AND NOT LIKE THE PAGE THAT RAN THE ROBOT AWAY. On 2026-08-20 the
teleop page kept the base moving after the operator let go: the page was alive and posting a
stale belief that a key was held, because a keyup was dropped while the JS thread was saturated.
safety/teleop_guard.py records the lesson exactly -- **a heartbeat proves the sender is alive,
not that a human is still asking for motion.**

So this uses the same two independent guards, and the same evidence shape:

  heartbeat   the page is still there at all           (tab closed, browser died, network stalled)
  hold lease  the button is still PHYSICALLY down      (dropped mouseup, frozen event loop)

The lease is POSITIVE and REPEATING: while held, the page re-asserts every 100 ms, and enable is
published only while the most recent assertion is younger than HOLD_LEASE_S. A dropped release
event is therefore self-correcting -- the assertions simply stop and the lease expires. A dropped
mouseup cannot strand it on, which is the exact failure that got the robot away last time.

Releasing also POSTs an explicit /release, but nothing depends on that arriving.
"""
from __future__ import annotations

import argparse
import json
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "bringup"))

from _ros_env import require_ros      # noqa: E402
require_ros()

import rclpy                          # noqa: E402
from rclpy.node import Node           # noqa: E402
from std_msgs.msg import Bool         # noqa: E402

from safety.teleop_guard import HOLD_LEASE_S, WATCHDOG_S   # noqa: E402

ENABLE_TOPIC = "/safety/enable"
RATE_HZ = 20.0


class Hold:
    def __init__(self) -> None:
        self._k = threading.Lock()
        self.last_assert = 0.0       # monotonic time of the most recent "still held"
        self.last_seen = 0.0         # most recent contact of any kind (heartbeat)
        self.released = True

    def assert_held(self) -> None:
        with self._k:
            self.last_assert = time.monotonic()
            self.last_seen = self.last_assert
            self.released = False

    def release(self) -> None:
        with self._k:
            self.released = True
            self.last_assert = 0.0
            self.last_seen = time.monotonic()

    def beat(self) -> None:
        with self._k:
            self.last_seen = time.monotonic()

    def permitted(self) -> tuple[bool, str]:
        """Fail closed on every unknown. `not (x <= t)` so NaN takes the refusing branch."""
        with self._k:
            released, la, ls = self.released, self.last_assert, self.last_seen
        now = time.monotonic()
        if not (now - ls <= WATCHDOG_S):
            return False, "no heartbeat -- page gone"
        if released:
            return False, "released"
        if not (now - la <= HOLD_LEASE_S):
            return False, "hold lease expired"
        return True, "held"


class DeadmanNode(Node):
    def __init__(self, hold: Hold) -> None:
        super().__init__("utp_deadman")
        self.hold = hold
        self.pub = self.create_publisher(Bool, ENABLE_TOPIC, 10)
        self.last = None
        self.create_timer(1.0 / RATE_HZ, self._tick)

    def _tick(self) -> None:
        ok, why = self.hold.permitted()
        self.pub.publish(Bool(data=bool(ok)))
        if ok != self.last:
            self.get_logger().info(f"{ENABLE_TOPIC} -> {ok}  ({why})")
            self.last = ok

    def drop(self) -> None:
        for _ in range(3):
            self.pub.publish(Bool(data=False))


PAGE = """<!doctype html><meta charset=utf-8><title>deadman</title>
<style>
 body{margin:0;background:#12151a;color:#e6e9ef;font:14px ui-monospace,Menlo,monospace;
   display:flex;flex-direction:column;align-items:center;justify-content:center;height:100vh}
 #b{width:300px;height:300px;border-radius:50%;border:3px solid #30363d;background:#1b2029;
   color:#8b94a3;font:600 20px ui-monospace,Menlo,monospace;cursor:pointer;user-select:none;
   -webkit-user-select:none;touch-action:none}
 #b.on{background:#1f6f3f;border-color:#2ea043;color:#fff}
 .n{margin-top:18px;color:#8b94a3;max-width:460px;text-align:center;line-height:1.5}
 b{color:#e6e9ef}
</style>
<button id=b>HOLD TO ENABLE</button>
<div class=n>Publishes <b>/safety/enable</b> only while this is held.<br>
Nav2 and the pipeline's servo motions need it; teleop does not.<br>
Let go, close the tab, or lose the page and it drops within
<b>a second</b>.</div>
<script>
let held=false, t=null;
const b=document.getElementById('b');
async function assert_(){ try{ await fetch('/hold',{method:'POST'}); }catch(e){} }
async function rel(){ held=false; b.classList.remove('on'); b.textContent='HOLD TO ENABLE';
  if(t){clearInterval(t);t=null;} try{ await fetch('/release',{method:'POST'}); }catch(e){} }
function grab(ev){ ev.preventDefault(); if(held) return; held=true;
  b.classList.add('on'); b.textContent='ENABLED';
  assert_(); t=setInterval(assert_,100); }
b.addEventListener('mousedown',grab); b.addEventListener('touchstart',grab,{passive:false});
for(const e of ['mouseup','mouseleave','touchend','touchcancel','blur'])
  b.addEventListener(e,rel);
window.addEventListener('blur',rel);
document.addEventListener('visibilitychange',()=>{ if(document.hidden) rel(); });
// Space works too, and autorepeat is the same positive evidence a held mouse button gives.
document.addEventListener('keydown',e=>{ if(e.code==='Space'){ e.preventDefault(); grab(e); }});
document.addEventListener('keyup',e=>{ if(e.code==='Space') rel(); });
setInterval(()=>{ if(!held) fetch('/beat',{method:'POST'}).catch(()=>{}); }, 200);
</script>
"""


def make_handler(hold: Hold):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _ok(self, body=b'{"ok":true}'):
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path in ("/", "/index.html"):
                b = PAGE.encode()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(b)))
                self.end_headers()
                self.wfile.write(b)
                return
            if self.path == "/state":
                ok, why = hold.permitted()
                self._ok(json.dumps({"enabled": ok, "why": why}).encode())
                return
            self.send_response(404); self.end_headers()

        def do_POST(self):
            if self.path == "/hold":
                hold.assert_held(); self._ok(); return
            if self.path == "/release":
                hold.release(); self._ok(); return
            if self.path == "/beat":
                hold.beat(); self._ok(); return
            self.send_response(404); self.end_headers()
    return H


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8089)
    a = ap.parse_args()

    rclpy.init()
    hold = Hold()
    node = DeadmanNode(hold)
    srv = ThreadingHTTPServer((a.host, a.port), make_handler(hold))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    print(f"\n  deadman  ==> http://{a.host}:{a.port}")
    print(f"  publishes {ENABLE_TOPIC} ONLY while the button is held "
          f"(lease {HOLD_LEASE_S}s, heartbeat {WATCHDOG_S}s)\n")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.drop()
        srv.shutdown()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
