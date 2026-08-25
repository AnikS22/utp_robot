#!/usr/bin/env python3
"""Ranger Mini 3.0 dashboard: WASD teleop + live camera, lidar and chassis telemetry.

    python3 bringup/teleop_keyboard.py            # then open http://127.0.0.1:8420

Publishes /cmd_vel_teleop, never /cmd_vel. Everything else on the page is READ-ONLY telemetry.

The camera and lidar panels are here because "the base moved" and "the base moved somewhere
sensible" are different claims, and the second one needs eyes on the sensors at the moment of
driving. Telemetry polling is deliberately on a SEPARATE timer from the command heartbeat, so a
slow camera frame can never delay or extend the teleop watchdog.

WHY A BROWSER AND NOT A TERMINAL
--------------------------------
A terminal cannot tell you that a key was RELEASED. Every curses/termios teleop therefore latches:
you tap W, a velocity is set, and it persists until you tap something else. On a 700x500 mm base
that is a robot that keeps driving after you take your hand off the keyboard, and the usual "hold
the key down" illusion is really just autorepeat -- which stops the instant the OS decides to
stop repeating. A browser gives real keydown/keyup, so "moving" and "a human is holding a key"
can be the same fact instead of two things that drift apart.

WHY IT PUBLISHES /cmd_vel_teleop
--------------------------------
config/safety.yaml makes the twist mux the ONLY publisher of /cmd_vel. Publishing there directly
from here would silently take the E-stop, the speed ceilings, the slew limiter and the arm
interlock out of the loop -- every protection this repo has for base motion. So this node is just
another source, at teleop priority, and the mux decides. It follows that YOU MUST RUN THE MUX or
nothing moves; the status line says so rather than leaving you guessing.

Because teleop's arm gate is fail-closed and nothing is publishing /safety/arm_stowed, the mux
will block teleop until /safety/override is asserted -- and then limits to override_speed_factor
(0.25). That is the documented "human-supervised recovery with the arm out, crawling" path, and it
is the correct mode for first keyboard driving. The override toggle in the UI is deliberately
OFF at start and deliberately not remembered: asserting it is meant to be a conscious act.

THREE INDEPENDENT STOPS, because one is not enough
--------------------------------------------------
  1. keyup            -- release the key, the twist goes to zero.
  2. blur / hidden    -- alt-tab away while holding W and the browser NEVER delivers the keyup.
                         That is the classic stuck-key runaway. Losing focus zeroes everything.
  3. heartbeat        -- the page posts its state ~12 Hz. If this node hears nothing for
                         WATCHDOG_S it publishes zero regardless of what it last knew. Covers the
                         tab being closed, the browser crashing, and the laptop's wifi stalling.
Plus SPACE as an explicit latched stop that must be cleared deliberately.

MOTION MODES -- GAP 1 made visible instead of surprising
--------------------------------------------------------
The Ranger driver auto-selects a motion mode from the twist and DROPS components (HARDWARE_SPECS.md):
linear.y != 0 -> PARALLEL, angular.z dropped; small turn radius -> SPINNING, linear.x dropped;
otherwise DUAL_ACKERMAN, linear.y dropped. A teleop that blends strafe and yaw therefore commands
one thing and gets another, with nothing reporting the difference. So the operator picks the mode
here and the UI only ever emits components that survive it.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import threading
import time
from pathlib import Path
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Bool, String

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "safety"))
from teleop_guard import HOLD_LEASE_S, WATCHDOG_S, Command, decide   # noqa: E402

try:
    import cv2
except ImportError:                     # telemetry degrades, teleop must not
    cv2 = None

# ranger_msgs ships CONTROL_MODE_RC=0 / CONTROL_MODE_CAN=1, which DISAGREES with the protocol the
# driver actually speaks. ugv_sdk/include/ugv_sdk/details/interface/agilex_types.h is authoritative
# and ranger_messenger.cpp passes the raw SDK value straight through, so decode with the SDK enums.
# Reading the ROS message's own constants tells you the base is in RC mode when it is in CAN mode.
CONTROL_MODE = {0: "STANDBY", 1: "CAN", 2: "UART", 3: "RC"}
VEHICLE_STATE = {0: "NORMAL", 1: "ESTOP", 2: "EXCEPTION"}
# RangerInterface::MotionMode (ranger_interface.hpp) -- note kPark=3, NOT side-slip as ranger_msgs says
MOTION_MODE = {0: "DualAckerman", 1: "Parallel", 2: "Spinning", 3: "Park", 4: "SideSlip"}

PUBLISH_HZ = 20.0        # matches config/safety.yaml rate_hz
# WATCHDOG_S and HOLD_LEASE_S come from safety/teleop_guard.py, which owns the decision logic.

# WHY HOLD_LEASE_S EXISTS -- read before weakening it
# --------------------------------------------------
# 2026-08-20: the base kept driving after the operator's hands left the keyboard, and only the
# hardware E-stop stopped it. WATCHDOG_S did not help, and could not have: it detects the PAGE
# DYING. Here the page was alive and healthy, faithfully posting a stale belief that W was still
# held, because a `keyup` event had been dropped while the JS thread was saturated rendering the
# camera. A heartbeat proves the sender is alive. It proves nothing about whether a human is still
# pressing anything.
#
# The fix uses KEY AUTOREPEAT as independent evidence. While a key is physically down the browser
# keeps emitting keydown events (~every 30-50 ms after the initial delay); when the key is
# physically released they stop IMMEDIATELY -- and unlike `keyup`, a dropped autorepeat is
# self-correcting, because another one follows milliseconds later. The page reports the age of its
# last keydown and THIS NODE zeroes any non-zero command whose age exceeds the lease.
#
# Enforced here, in the node, and not in the page, precisely because the page is the component
# that already proved untrustworthy. If autorepeat is disabled on the operator's machine the
# command stutters to zero once a second -- annoying, obvious, and safe. That is the correct
# direction to fail.


class State:
    """Shared between the HTTP thread and the ROS timer. Guarded by a lock because the browser
    writes it and the publisher reads it, and a torn read here is a wrong velocity."""

    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.vx = self.vy = self.wz = 0.0
        self.override = False
        self.stopped = False
        self.stamp = 0.0          # 0.0 == never heard from the page; watchdog treats it as stale
        # Telemetry is written by ROS callbacks and read by the HTTP thread. Kept under the SAME
        # lock but on a separate cadence: a stalled camera must never delay the command watchdog.
        self.key_age_s = 1e9      # age of the page's last keydown; huge == no key evidence at all
        self.jpeg: bytes | None = None
        self.telem: dict = {}

    def set(self, vx: float, vy: float, wz: float, override: bool, stopped: bool,
            key_age_s: float) -> None:
        with self.lock:
            self.vx, self.vy, self.wz = vx, vy, wz
            self.override, self.stopped = override, stopped
            self.key_age_s = key_age_s
            self.stamp = time.monotonic()

    def read(self) -> tuple[float, float, float, bool, bool, str]:
        """Current command after both guards. Returns (vx, vy, wz, override, fresh, reason).

        A thin adapter: every rule lives in safety/teleop_guard.decide(), which is pure and unit
        tested. Same split as arbiter.py / twist_mux_node.py -- the part that can hurt someone is
        the part that gets exercised headlessly.
        """
        with self.lock:
            cmd = Command(self.vx, self.vy, self.wz)
            override, stopped, key_age = self.override, self.stopped, self.key_age_s
            hb_age = float("inf") if self.stamp <= 0.0 else (time.monotonic() - self.stamp)
        v = decide(cmd, override=override, stopped=stopped,
                   heartbeat_age_s=hb_age, key_age_s=key_age)
        return (v.command.vx, v.command.vy, v.command.wz, v.override,
                v.reason != "no_heartbeat", v.reason)


class TeleopNode(Node):
    def __init__(self, state: State, cmd_topic: str, override_topic: str) -> None:
        super().__init__("utp_teleop_keyboard")
        self.state = state
        self.pub = self.create_publisher(Twist, cmd_topic, 10)
        self.pub_override = self.create_publisher(Bool, override_topic, 10)
        self.create_timer(1.0 / PUBLISH_HZ, self._tick)
        self._last_log = 0.0

        self.get_logger().info(f"teleop -> {cmd_topic} @ {PUBLISH_HZ:g} Hz (zeros included)")
        self.get_logger().info(f"override -> {override_topic}")

        # ---- read-only telemetry -------------------------------------------------------------
        self._t = {}
        self.create_subscription(LaserScan, "/scan", self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Image, "/mast_cam/color/image_raw", self._on_img, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(String, "/safety/status", self._on_safety, 10)
        try:
            from ranger_msgs.msg import SystemState
            self.create_subscription(SystemState, "/system_state", self._on_sys, 10)
        except ImportError:
            self.get_logger().warn("ranger_msgs not on the path; chassis panel will stay empty")
        self._img_every = 0

    # ---- telemetry callbacks ----------------------------------------------------------------
    def _on_scan(self, m: LaserScan) -> None:
        r = [None if (not math.isfinite(v) or v <= 0.0) else round(float(v), 3) for v in m.ranges]
        self._t["scan"] = {"ranges": r, "amin": float(m.angle_min), "ainc": float(m.angle_increment),
                           "rmax": float(m.range_max), "n": len(r),
                           "valid": sum(1 for v in r if v is not None)}
        self._push()

    def _on_img(self, m: Image) -> None:
        if cv2 is None:
            return
        self._img_every = (self._img_every + 1) % 3      # ~10 fps out of 30, plenty for a dashboard
        if self._img_every:
            return
        try:
            a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)
            if m.encoding == "rgb8":
                a = cv2.cvtColor(a, cv2.COLOR_RGB2BGR)
            a = cv2.resize(a, (640, int(640 * m.height / m.width)))
            ok, buf = cv2.imencode(".jpg", a, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
            if ok:
                with self.state.lock:
                    self.state.jpeg = buf.tobytes()
        except (ValueError, cv2.error):
            pass          # a bad frame must never take the teleop node down

    def _on_odom(self, m: Odometry) -> None:
        p, tw = m.pose.pose.position, m.twist.twist
        q = m.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        self._t["odom"] = {"x": round(p.x, 3), "y": round(p.y, 3), "yaw": round(math.degrees(yaw), 1),
                           "vx": round(tw.linear.x, 3), "vy": round(tw.linear.y, 3),
                           "wz": round(tw.angular.z, 3)}
        self._push()

    def _on_safety(self, m: String) -> None:
        try:
            self._t["safety"] = json.loads(m.data)
        except ValueError:
            pass
        self._push()

    def _on_sys(self, m) -> None:
        # Decoded with the SDK enums, NOT ranger_msgs' constants -- see the note at the top.
        self._t["chassis"] = {
            "vehicle_state": VEHICLE_STATE.get(m.vehicle_state, f"?{m.vehicle_state}"),
            "control_mode": CONTROL_MODE.get(m.control_mode, f"?{m.control_mode}"),
            "motion_mode": MOTION_MODE.get(m.motion_mode, f"?{m.motion_mode}"),
            "battery_v": round(float(m.battery_voltage), 1),
            "error_code": int(m.error_code),
        }
        self._push()

    def _push(self) -> None:
        with self.state.lock:
            self.state.telem = dict(self._t)

    def _tick(self) -> None:
        vx, vy, wz, override, _fresh, reason = self.state.read()
        if reason == "hold_lease_expired" and time.monotonic() - self._last_log > 1.0:
            self._last_log = time.monotonic()
            self.get_logger().warn(
                "hold lease expired: the page reported a held key but no keydown arrived within "
                f"{HOLD_LEASE_S}s -- commanding zero")
        # Publish EVERY tick including zeros, for the same reason the mux does: a teleop that goes
        # quiet when it stops is indistinguishable downstream from a teleop that died.
        t = Twist()
        t.linear.x, t.linear.y, t.angular.z = vx, vy, wz
        self.pub.publish(t)
        self.pub_override.publish(Bool(data=bool(override)))

    def stop_and_park(self) -> None:
        """Publish a burst of zeros on the way out. One message can be lost; a burst is not."""
        for _ in range(5):
            self.pub.publish(Twist())
            self.pub_override.publish(Bool(data=False))
            time.sleep(0.02)


PAGE = r"""<!doctype html><html><head><meta charset=utf-8>
<title>Ranger Mini 3 — dashboard</title>
<style>
 :root{--bg:#0e1014;--fg:#e8eaee;--dim:#8b93a1;--ok:#3ddc84;--warn:#ffb020;--bad:#ff5c5c;
       --card:#171a21;--line:#262b35}
 *{box-sizing:border-box} body{margin:0;background:var(--bg);color:var(--fg);padding:16px;
   font:13px/1.5 ui-monospace,SFMono-Regular,Menlo,monospace}
 h1{font-size:14px;margin:0;letter-spacing:.05em}
 .sub{color:var(--dim);font-size:11px;margin:2px 0 14px}
 .grid{display:grid;grid-template-columns:290px 1fr 1fr;gap:14px;align-items:start}
 @media(max-width:1100px){.grid{grid-template-columns:1fr}}
 .card{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:12px}
 .card h2{font-size:11px;color:var(--dim);margin:0 0 10px;letter-spacing:.09em;text-transform:uppercase}
 .keys{display:grid;grid-template-columns:repeat(3,56px);gap:7px;justify-content:center;margin:6px 0 12px}
 .k{height:50px;display:flex;align-items:center;justify-content:center;border:1px solid var(--line);
    border-radius:8px;background:#11141a;color:var(--dim);font-size:16px;font-weight:700;user-select:none}
 .k.on{background:var(--ok);color:#06240f;border-color:var(--ok)}
 .k.sp{grid-column:1/4;height:34px;font-size:11px}
 .k.sp.on{background:var(--bad);color:#fff;border-color:var(--bad)}
 .row{display:flex;justify-content:space-between;gap:10px;padding:3px 0;border-bottom:1px solid var(--line)}
 .row:last-child{border:0}
 label{display:block;margin:9px 0 3px;color:var(--dim);font-size:11px}
 select,input[type=range]{width:100%}
 select{background:#11141a;color:var(--fg);border:1px solid var(--line);border-radius:6px;padding:6px}
 .ov{margin-top:11px;padding:9px;border:1px solid var(--warn);border-radius:8px;background:#241d0d}
 .ov.on{border-color:var(--bad);background:#2a1214}
 .ov label{color:var(--warn);display:flex;gap:7px;margin:0;cursor:pointer;font-size:11px}
 .ov.on label{color:var(--bad)}
 .pill{display:inline-block;padding:1px 7px;border-radius:99px;font-size:10px;font-weight:700}
 .p-ok{background:#0f2e1c;color:var(--ok)} .p-bad{background:#2e1414;color:var(--bad)}
 .p-warn{background:#2e2411;color:var(--warn)} .p-dim{background:#20242c;color:var(--dim)}
 img,canvas{width:100%;border-radius:8px;display:block;background:#0a0c10}
 .note{color:var(--dim);font-size:10px;margin-top:9px;line-height:1.6}
 b{font-weight:700}
</style></head><body>
<h1>RANGER MINI 3.0 — DASHBOARD</h1>
<div class=sub>teleop publishes <b>/cmd_vel_teleop</b> · the mux owns /cmd_vel · everything else is read-only</div>
<div class=grid>

 <div>
  <div class=card>
   <h2>Teleop</h2>
   <div class=keys>
    <div></div><div class=k id=kw>W</div><div></div>
    <div class=k id=ka>A</div><div class=k id=ks>S</div><div class=k id=kd>D</div>
    <div class="k sp" id=ksp>SPACE — STOP</div>
   </div>
   <label>motion mode</label>
   <select id=mode>
    <option value=ackermann>Ackermann — W/S drive, A/D yaw</option>
    <option value=spin>Spin — A/D rotate in place</option>
    <option value=strafe>Strafe — A/D crab sideways</option>
   </select>
   <label>speed <span id=spd>30</span>%</label>
   <input type=range id=speed min=5 max=100 value=30>
   <div class=ov id=ovbox>
    <label><input type=checkbox id=ov><span><b>Assert /safety/override.</b>
     Required while nothing publishes /safety/arm_stowed. Speed drops to 25%.</span></label>
   </div>
   <div class=row style="margin-top:10px"><span>commanded</span><b id=cmd>0.00, 0.00, 0.00</b></div>
   <div class=row><span>link</span><b id=slink><span class="pill p-warn">…</span></b></div>
  </div>

  <div class=card style="margin-top:14px">
   <h2>Chassis</h2>
   <div class=row><span>vehicle</span><b id=cv><span class="pill p-dim">—</span></b></div>
   <div class=row><span>control mode</span><b id=cc><span class="pill p-dim">—</span></b></div>
   <div class=row><span>motion mode</span><b id=cm>—</b></div>
   <div class=row><span>battery</span><b id=cb>—</b></div>
   <div class=row><span>error</span><b id=ce>—</b></div>
   <div class=note>Decoded with the ugv_sdk enums — ranger_msgs' own constants disagree with the
    protocol (RC is 3, not 0; motion_mode 3 is Park, not SideSlip).</div>
  </div>

  <div class=card style="margin-top:14px">
   <h2>Safety mux</h2>
   <div class=row><span>source</span><b id=ss>—</b></div>
   <div class=row><span>blocked by</span><b id=sb>—</b></div>
   <div class=row><span>arm stowed</span><b id=sa>—</b></div>
   <div class=row><span>e-stop</span><b id=se>—</b></div>
  </div>
 </div>

 <div class=card>
  <h2>Mast camera — /mast_cam/color/image_raw</h2>
  <img id=cam alt="waiting for frames">
  <div class=row style="margin-top:9px"><span>odom x,y,yaw</span><b id=ox>—</b></div>
  <div class=row><span>measured v</span><b id=ov2>—</b></div>
 </div>

 <div class=card>
  <h2>Lidar — /scan</h2>
  <canvas id=lidar width=520 height=520></canvas>
  <div class=row style="margin-top:9px"><span>returns</span><b id=lr>—</b></div>
  <div class=note>Up is <b>+x</b> (forward), left is <b>+y</b>, per REP-103. Rings at 1/3/5 m.
   The orange spoke is the robot's forward axis — if an object in front of the robot does not
   appear near the spoke, the scan is rotated or mirrored and the map will be wrong.</div>
 </div>
</div>
<script>
const held={w:0,a:0,s:0,d:0}; let stopped=false, mode='ackermann', speed=0.30, override=false;
// Age of the most recent keydown, INCLUDING autorepeat. The node refuses any non-zero command
// whose age exceeds its hold lease. Autorepeat stops the instant a key is physically released,
// so this is liveness evidence that does not depend on `keyup` being delivered at all.
let lastKeyMs=0;
const $=id=>document.getElementById(id);
const MAXVX=0.6, MAXVY=0.4, MAXWZ=0.8;
function clearAll(){for(const k in held)held[k]=0;}
function twist(){
  if(stopped) return [0,0,0];
  const f=(held.w?1:0)-(held.s?1:0), r=(held.a?1:0)-(held.d?1:0);
  if(mode==='spin')   return [0,0,r*MAXWZ*speed];
  if(mode==='strafe') return [f*MAXVX*speed, r*MAXVY*speed, 0];
  return [f*MAXVX*speed, 0, r*MAXWZ*speed];
}
function paint(){
  for(const k in held) $('k'+k).classList.toggle('on', !!held[k] && !stopped);
  $('ksp').classList.toggle('on', stopped);
  const [x,y,w]=twist();
  $('cmd').textContent=`${x.toFixed(2)}, ${y.toFixed(2)}, ${w.toFixed(2)}`;
  $('ovbox').classList.toggle('on',override);
}
async function post(){
  const [vx,vy,wz]=twist();
  try{
    await fetch('/state',{method:'POST',headers:{'Content-Type':'application/json'},
      body:JSON.stringify({vx,vy,wz,override,stopped,
        key_age_ms: lastKeyMs? (performance.now()-lastKeyMs) : 1e12})});
    $('slink').innerHTML='<span class="pill p-ok">ok</span>';
  }catch(e){ $('slink').innerHTML='<span class="pill p-bad">LOST</span>'; }
}
addEventListener('keydown',e=>{
  const k=e.key.toLowerCase();
  if(k===' '){e.preventDefault(); stopped=!stopped; if(stopped)clearAll(); lastKeyMs=performance.now(); paint(); post(); return;}
  if(k in held){
    e.preventDefault();
    lastKeyMs=performance.now();          // renewed by autorepeat too -- deliberately not filtered
    if(!held[k]){held[k]=1; paint(); post();}
  }
});
addEventListener('keyup',e=>{const k=e.key.toLowerCase(); if(k in held){held[k]=0; paint(); post();}});
function panic(){clearAll(); lastKeyMs=0; paint(); post();}   // focus loss never delivers keyup
addEventListener('blur',panic);
addEventListener('pagehide',panic);
addEventListener('visibilitychange',()=>{if(document.hidden)panic();});
$('mode').onchange=e=>{mode=e.target.value; clearAll(); paint(); post();};
$('speed').oninput=e=>{speed=e.target.value/100; $('spd').textContent=e.target.value; paint(); post();};
$('ov').onchange=e=>{override=e.target.checked; paint(); post();};
setInterval(post,80);   // command heartbeat — the node zeroes if this stops arriving

// ---- telemetry: a SEPARATE, slower loop. Must never gate the heartbeat above. ----
const pill=(t,c)=>`<span class="pill ${c}">${t}</span>`;
function drawScan(s){
  const cv=$('lidar'), g=cv.getContext('2d'), P=cv.width, c=P/2, MAX=6.0, sc=(P/2-14)/MAX;
  g.fillStyle='#0a0c10'; g.fillRect(0,0,P,P);
  g.strokeStyle='#242a34';
  for(const r of [1,3,5]){g.beginPath();g.arc(c,c,r*sc,0,7);g.stroke();}
  g.strokeStyle='#ff9a3c'; g.beginPath(); g.moveTo(c,c); g.lineTo(c,12); g.stroke();
  if(!s) return;
  g.fillStyle='#3ddc84';
  for(let i=0;i<s.ranges.length;i++){
    const v=s.ranges[i]; if(v===null||v>MAX) continue;
    const a=s.amin+i*s.ainc;
    g.fillRect(c-v*Math.sin(a)*sc-1.5, c-v*Math.cos(a)*sc-1.5, 3, 3);
  }
  g.fillStyle='#e8eaee'; g.fillRect(c-2,c-2,4,4);
}
async function telem(){
  try{
    const t=await (await fetch('/telemetry.json',{cache:'no-store'})).json();
    if(t.chassis){const ch=t.chassis;
      $('cv').innerHTML=pill(ch.vehicle_state, ch.vehicle_state==='NORMAL'?'p-ok':'p-bad');
      $('cc').innerHTML=pill(ch.control_mode, ch.control_mode==='CAN'?'p-ok':'p-warn');
      $('cm').textContent=ch.motion_mode;
      $('cb').textContent=ch.battery_v.toFixed(1)+' V';
      $('ce').innerHTML=ch.error_code?pill('0x'+ch.error_code.toString(16),'p-bad'):pill('none','p-ok');}
    if(t.safety){const sf=t.safety, gt=sf.gates||{};
      $('ss').textContent=sf.source||'—';
      $('sb').innerHTML=sf.blocked_by?pill(sf.blocked_by,'p-warn'):pill('permitted','p-ok');
      $('sa').innerHTML=pill(gt.arm_stowed?'true':'false', gt.arm_stowed?'p-ok':'p-bad');
      $('se').innerHTML=pill(sf.estop_latched?'LATCHED':'clear', sf.estop_latched?'p-bad':'p-ok');}
    if(t.odom){const o=t.odom;
      $('ox').textContent=`${o.x.toFixed(2)}, ${o.y.toFixed(2)}, ${o.yaw.toFixed(1)}°`;
      $('ov2').textContent=`vx ${o.vx.toFixed(3)}  vy ${o.vy.toFixed(3)}  wz ${o.wz.toFixed(3)}`;}
    if(t.scan){drawScan(t.scan); $('lr').textContent=`${t.scan.valid}/${t.scan.n} valid`;}
  }catch(e){}
}
// Render rates are deliberately LOW and pause when the tab is hidden. On 2026-08-20 an aggressive
// camera refresh (120 ms) saturated this single JS thread, delayed key events, and contributed to
// a runaway. Telemetry is a convenience; the key handlers are a safety mechanism. When they
// compete, the safety mechanism wins.
setInterval(()=>{ if(!document.hidden) telem(); }, 500);
let camBusy=false;
function camTick(){
  if(document.hidden||camBusy) return;
  camBusy=true;
  const im=new Image();
  im.onload=()=>{ $('cam').src=im.src; camBusy=false; };   // never queue faster than it decodes
  im.onerror=()=>{ camBusy=false; };
  im.src='/camera.jpg?'+Date.now();
}
setInterval(camTick,350);
paint(); telem();
</script></body></html>"""


def make_handler(state: State):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):      # keep the ROS log readable
            pass

        def _bin(self, body: bytes, ctype: str):
            self.send_response(200)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            root = self.path.split("?")[0]
            if root == "/camera.jpg":
                with state.lock:
                    j = state.jpeg
                if j is None:
                    self.send_error(503, "no frame yet")
                    return
                try:
                    self._bin(j, "image/jpeg")
                except (BrokenPipeError, ConnectionResetError):
                    pass      # the browser closed mid-frame; routine, not an error
                return
            if root == "/telemetry.json":
                with state.lock:
                    t = dict(state.telem)
                try:
                    self._bin(json.dumps(t).encode(), "application/json")
                except (BrokenPipeError, ConnectionResetError):
                    pass
                return
            if root not in ("/", "/index.html"):
                self.send_error(404)
                return
            body = PAGE.encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_POST(self):
            if self.path != "/state":
                self.send_error(404)
                return
            try:
                n = int(self.headers.get("Content-Length", 0))
                d = json.loads(self.rfile.read(n) or b"{}")
                state.set(float(d.get("vx", 0.0)), float(d.get("vy", 0.0)), float(d.get("wz", 0.0)),
                          bool(d.get("override", False)), bool(d.get("stopped", False)),
                          # absent/garbage key_age is treated as NO evidence, i.e. fail closed
                          float(d.get("key_age_ms", 1e12)) / 1000.0)
            except (ValueError, TypeError):
                # A malformed post must not update the command. Saying nothing means the watchdog
                # takes over, which is the safe direction.
                self.send_error(400)
                return
            self.send_response(204)
            self.end_headers()
    return H


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=8420)
    ap.add_argument("--host", default="127.0.0.1", help="127.0.0.1 by default: this drives a robot")
    ap.add_argument("--cmd-topic", default="/cmd_vel_teleop")
    ap.add_argument("--override-topic", default="/safety/override")
    args, ros_args = ap.parse_known_args()

    state = State()
    rclpy.init(args=ros_args)
    node = TeleopNode(state, args.cmd_topic, args.override_topic)

    srv = ThreadingHTTPServer((args.host, args.port), make_handler(state))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    node.get_logger().info(f"UI  ->  http://{args.host}:{args.port}")
    node.get_logger().info("hold W/A/S/D to move; SPACE latches a stop; releasing a key stops")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        srv.shutdown()
        try:
            node.stop_and_park()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
