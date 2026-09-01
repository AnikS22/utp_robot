#!/usr/bin/env python3
"""A map in the browser: click a waypoint, press GO, watch the robot drive there.

    python3 bringup/drive_ui.py            # http://localhost:8together / see the printed URL
    python3 bringup/drive_ui.py --port 8090

WHAT IT DRIVES WITH. Not a new controller. It calls safety/waypoint_drive.plan_step -- the same
pure function route_run.py uses, with the same 260 tests behind it, including the w_min stall
floor added 2026-08-30. A second implementation of "drive to a point" is a second thing to get
wrong.

WHERE IT PUBLISHES. /cmd_vel_teleop, never /cmd_vel. config/safety.yaml makes the twist mux the
ONLY publisher of /cmd_vel, so estop, arm_stowed and enable all still gate everything this page
asks for. A UI that could bypass the mux would be a UI that could bypass the E-stop interlock.

THE TELEOP RUNAWAY, AND WHY THIS IS SHAPED DIFFERENTLY. On 2026-08-20 the keyboard teleop page
ran the base away with the operator's hands off the keys: the page stayed alive posting a stale
belief that a key was held, and a heartbeat proved only that the SENDER was alive, never that a
human still wanted motion. This page cannot repeat that, because the command is not "keep going"
-- it is "go to THIS point", which is bounded and self-terminating: it stops on arrival, on
timeout, on STOP, and on the browser going quiet. There is no held state to go stale.

NOTHING MOVES ON LOAD. Selecting a waypoint is not a command; GO is, and it is a separate click.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "bringup"))

from _ros_env import require_ros          # noqa: E402
require_ros()

import rclpy                              # noqa: E402
import yaml                               # noqa: E402
from geometry_msgs.msg import Twist       # noqa: E402
from rclpy.node import Node               # noqa: E402
from rclpy.qos import qos_profile_sensor_data   # noqa: E402
from sensor_msgs.msg import LaserScan     # noqa: E402
from std_msgs.msg import String           # noqa: E402

from pose_source import PoseSource, current_map_name, mola_session_id  # noqa: E402
from safety.map_frame import frame_of                                  # noqa: E402
from safety.waypoint_drive import Limits, corridor_blocked, plan_step, to_goal  # noqa: E402

CMD_TOPIC = "/cmd_vel_teleop"
SAFETY_STATUS_TOPIC = "/safety/status"
RATE_HZ = 20.0
# If the browser stops asking for state, it is gone (closed, crashed, laptop asleep) and there is
# nobody watching the robot. Stop. Generous enough to survive a slow poll, short enough that an
# unattended robot does not keep driving.
UI_WATCHDOG_S = 2.0
LEG_TIMEOUT_S = 180.0
# A clicked goal further than this is a misclick on a zoomed-out map, not an intention. The
# controller would happily set off across the building at it.
MAX_CLICK_GOAL_M = 15.0
STORE = Path(os.environ.get("UTP_WAYPOINTS", "")) if os.environ.get("UTP_WAYPOINTS") \
    else REPO / "maps" / "waypoints.yaml"


def load_waypoints() -> dict:
    if not STORE.exists():
        return {}
    return yaml.safe_load(STORE.read_text()) or {}


class State:
    """Shared between the HTTP threads and the ROS thread. Every field under one lock."""

    def __init__(self) -> None:
        self._k = threading.Lock()
        self.goal = None            # label of what we are driving to, or None
        self.goal_xy = None         # (x, y) when the goal was CLICKED rather than named
        self.go = False             # armed by GO, cleared by STOP/arrival/timeout
        self.last_poll = 0.0
        self.started = 0.0
        self.status = "idle"
        self.detail = ""
        self.snapshot = {}

    def arm(self, name: str) -> None:
        with self._k:
            self.goal = name
            self.goal_xy = None
            self.go = True
            self.started = time.monotonic()
            self.status = "driving"
            self.detail = f"driving to {name}"

    def arm_xy(self, x: float, y: float) -> None:
        """Drive to a point clicked on the map, with no stored waypoint behind it."""
        with self._k:
            self.goal = f"({x:.2f}, {y:.2f})"
            self.goal_xy = (float(x), float(y))
            self.go = True
            self.started = time.monotonic()
            self.status = "driving"
            self.detail = f"driving to {self.goal}"

    def halt(self, why: str) -> None:
        with self._k:
            self.go = False
            self.status = "stopped" if why == "stop" else why
            self.detail = {"stop": "STOP pressed",
                           "arrived": f"arrived at {self.goal}",
                           "timeout": f"gave up on {self.goal} after {LEG_TIMEOUT_S:.0f}s",
                           "watchdog": "browser went quiet -- stopped",
                           }.get(why, why)

    def poll(self) -> None:
        with self._k:
            self.last_poll = time.monotonic()

    def read(self):
        with self._k:
            return (self.goal, self.go, self.last_poll, self.started, self.status, self.detail,
                    self.goal_xy)

    def publish_snapshot(self, d: dict) -> None:
        with self._k:
            self.snapshot = d

    def get_snapshot(self) -> dict:
        with self._k:
            return dict(self.snapshot)


class DriveNode(Node):
    def __init__(self, state: State, frame: str) -> None:
        super().__init__("utp_drive_ui")
        self.state = state
        self.pose = None
        self.stamp = 0.0
        self.scan = None
        self.mux_blocked = None
        self.mux_seen = False
        self.src = PoseSource(self, frame)
        ok, self.frame_desc = self.src.resolve()
        self.frame_ok = ok
        self.pub = self.create_publisher(Twist, CMD_TOPIC, 10)
        self.create_subscription(LaserScan, "/scan_filtered", self._scan, qos_profile_sensor_data)
        self.create_subscription(String, SAFETY_STATUS_TOPIC, self._safety, 10)
        self.lim = Limits()
        self.prev_state = ""
        self.create_timer(1.0 / RATE_HZ, self._tick)

    def _scan(self, m) -> None:
        self.scan = m

    def _safety(self, m) -> None:
        try:
            st = json.loads(m.data)
        except (ValueError, TypeError):
            return
        self.mux_seen = True
        self.mux_blocked = st.get("blocked_by")

    def set_limits(self, v_max: float, w_max: float) -> None:
        self.base_lim = Limits(v_max=float(v_max), w_max=float(w_max))
        self.lim = self.base_lim

    def set_cap(self, cap: float) -> None:
        """Slider cap. Scales the LINEAR ceiling only -- deliberately not the angular one.

        w_max cannot be scaled down freely: waypoint_drive floors an in-place turn UP to w_min
        (0.20 rad/s, the slowest rotation this 4WS chassis actually executes) and then clamps
        DOWN to w_max. Scaling w_max toward zero would put the ceiling under the floor, and every
        turn would be issued below the rate the chassis responds to -- the robot would sit
        commanding a rotation that never happens. Slower is the intent; not turning is not.
        """
        base = getattr(self, "base_lim", Limits())
        cap = max(0.04, min(1.0, float(cap)))
        self.lim = Limits(v_max=max(base.v_min, base.v_max * cap), w_max=base.w_max)

    def _blocked(self) -> bool:
        if self.scan is None:
            return False
        return corridor_blocked(self.scan.ranges, self.scan.angle_min, self.scan.angle_increment)

    def _tick(self) -> None:
        goal, go, last_poll, started, _, _, goal_xy = self.state.read()
        now = time.monotonic()
        wps = load_waypoints()

        # --- always publish the view, moving or not -------------------------------------------
        snap = {
            "frame": self.src.frame or "?",
            "frame_desc": self.frame_desc,
            "map_name": current_map_name(self),
            "pose": list(self.pose) if self.pose else None,
            "fresh": self.src.fresh(),
            "mux": self.mux_blocked if self.mux_seen else "no /safety/status",
            "blocked": self._blocked(),
            "waypoints": {k: {"x": v["x"], "y": v["y"], "yaw": v["yaw"],
                              "frame": frame_of(v)} for k, v in wps.items()},
            "scan": self._scan_xy(),
            "goal": goal,
            "goal_xy": list(goal_xy) if goal_xy else None,
            "driving": bool(go),
        }
        self.state.publish_snapshot(snap)

        if not go:
            return

        # --- every reason to stop, checked before every command --------------------------------
        if now - last_poll > UI_WATCHDOG_S:
            self.state.halt("watchdog"); self._stop(); return
        if now - started > LEG_TIMEOUT_S:
            self.state.halt("timeout"); self._stop(); return
        if goal_xy is None and goal not in wps:
            self.state.halt(f"waypoint '{goal}' is gone"); self._stop(); return
        if self.pose is None or not self.src.fresh():
            self.state.halt("pose went stale -- stopped"); self._stop(); return

        gx, gy = goal_xy if goal_xy is not None else (wps[goal]["x"], wps[goal]["y"])
        dist, bearing = to_goal(self.pose[0], self.pose[1], self.pose[2], gx, gy)
        step = plan_step(dist, bearing, None, self._blocked(), self.lim, self.prev_state)
        self.prev_state = step.state
        if step.state == "arrived":
            self.state.halt("arrived"); self._stop(); return

        t = Twist()
        t.linear.x = float(step.twist.vx)
        t.angular.z = float(step.twist.wz)
        self.pub.publish(t)

    def _scan_xy(self):
        """Scan as x,y in the robot frame, decimated -- this is drawn, not reasoned about."""
        m = self.scan
        if m is None:
            return []
        out = []
        for i in range(0, len(m.ranges), 2):
            r = m.ranges[i]
            if r != r or r in (float("inf"), float("-inf")) or r <= 0.05 or r > 20.0:
                continue
            a = m.angle_min + i * m.angle_increment
            out.append([round(r * math.cos(a), 3), round(r * math.sin(a), 3)])
        return out

    def _stop(self) -> None:
        for _ in range(3):
            self.pub.publish(Twist())

    def stop_and_park(self) -> None:
        self._stop()


PAGE = """<!doctype html><meta charset=utf-8><title>utp drive</title>
<style>
 :root{--bg:#12151a;--fg:#e6e9ef;--dim:#8b94a3;--ok:#3fb950;--warn:#d29922;--bad:#f85149;--acc:#58a6ff}
 *{box-sizing:border-box} body{margin:0;background:var(--bg);color:var(--fg);
   font:14px/1.45 ui-monospace,SFMono-Regular,Menlo,monospace}
 header{padding:10px 14px;border-bottom:1px solid #262c36}
 h1{margin:0;font-size:15px;font-weight:600}
 .sub{color:var(--dim);font-size:12px;margin-top:3px}
 .wrap{display:flex;gap:14px;padding:14px;flex-wrap:wrap}
 canvas{background:#0b0e12;border:1px solid #262c36;border-radius:6px;touch-action:none}
 .side{min-width:280px;flex:1}
 .row{display:flex;justify-content:space-between;padding:3px 0;border-bottom:1px solid #1b2029}
 .k{color:var(--dim)} .ok{color:var(--ok)} .warn{color:var(--warn)} .bad{color:var(--bad)}
 button{font:inherit;padding:9px 14px;border-radius:6px;border:1px solid #30363d;
   background:#1b2029;color:var(--fg);cursor:pointer}
 button:hover{border-color:var(--acc)}
 #go{background:#1f6f3f;border-color:#2ea043;font-weight:600}
 #go:disabled{background:#21262d;border-color:#30363d;color:#57606a;cursor:not-allowed}
 #stop{background:#8e1519;border-color:var(--bad);font-weight:700;flex:1;padding:16px}
 .wps{max-height:230px;overflow:auto;margin:8px 0}
 .wp{padding:7px 9px;border:1px solid #262c36;border-radius:5px;margin-bottom:5px;cursor:pointer}
 .wp:hover{border-color:var(--acc)} .wp.sel{border-color:var(--acc);background:#132133}
 .wp .m{color:var(--dim);font-size:12px}
 .badge{font-size:11px;padding:1px 6px;border-radius:9px;border:1px solid #30363d;color:var(--dim)}
 .speed{margin:10px 0;padding:9px;border:1px solid #262c36;border-radius:6px}
 input[type=range]{width:100%}
</style>
<header>
  <h1>utp drive &mdash; click a waypoint or anywhere on the map, then GO</h1>
  <div class=sub>publishes <b>/cmd_vel_teleop</b> &middot; the safety mux owns /cmd_vel, so
    e-stop, arm_stowed and enable still gate everything here</div>
</header>
<div class=wrap>
  <canvas id=c width=620 height=620></canvas>
  <div class=side>
    <div class=row><span class=k>pose frame</span><span id=frame>-</span></div>
    <div class=row><span class=k>map</span><span id=mapname>-</span></div>
    <div class=row><span class=k>pose</span><span id=pose>-</span></div>
    <div class=row><span class=k>safety mux</span><span id=mux>-</span></div>
    <div class=row><span class=k>path ahead</span><span id=blocked>-</span></div>
    <div class=row><span class=k>status</span><span id=status>-</span></div>

    <div class=speed>
      <div class=row style=border:none>
        <span class=k>speed cap</span><span id=spdval>-</span>
      </div>
      <input type=range id=spd min=4 max=100 value=100>
      <div class=sub id=spdnote></div>
    </div>

    <div class=wps id=wps></div>
    <div style=display:flex;gap:8px;margin-bottom:8px>
      <button id=go disabled>GO</button>
      <span class=sub id=sel>nothing selected</span>
    </div>
    <div style=display:flex><button id=stop>STOP</button></div>
  </div>
</div>
<script>
let S={}, sel=null, clickGoal=null, cap=1.0, VMAX=null;
const $=i=>document.getElementById(i);
const c=$('c'), g=c.getContext('2d');

function fit(){
  const pts=[]; if(S.pose) pts.push([S.pose[0],S.pose[1]]);
  for(const k in (S.waypoints||{})) pts.push([S.waypoints[k].x, S.waypoints[k].y]);
  if(!pts.length) return {cx:0,cy:0,s:40};
  let xs=pts.map(p=>p[0]), ys=pts.map(p=>p[1]);
  let x0=Math.min(...xs), x1=Math.max(...xs), y0=Math.min(...ys), y1=Math.max(...ys);
  const pad=3.0, w=(x1-x0)+2*pad, h=(y1-y0)+2*pad;
  const s=Math.min(c.width/Math.max(w,4), c.height/Math.max(h,4));
  return {cx:(x0+x1)/2, cy:(y0+y1)/2, s:s};
}
// world -> screen. ROS x is forward/right-handed with y LEFT, so y flips for a screen that
// grows downward; without this the map is mirrored and every turn looks backwards.
function T(x,y,f){ return [c.width/2+(x-f.cx)*f.s, c.height/2-(y-f.cy)*f.s]; }
function Tinv(sx,sy,f){ return [f.cx+(sx-c.width/2)/f.s, f.cy-(sy-c.height/2)/f.s]; }

function draw(){
  g.clearRect(0,0,c.width,c.height);
  const f=fit();
  g.strokeStyle='#1b2029'; g.lineWidth=1;
  for(let i=-50;i<=50;i++){ let [gx]=T(i,0,f), [,gy]=T(0,i,f);
    g.beginPath(); g.moveTo(gx,0); g.lineTo(gx,c.height); g.stroke();
    g.beginPath(); g.moveTo(0,gy); g.lineTo(c.width,gy); g.stroke(); }
  if(S.pose){
    const [px,py,pth]=S.pose;
    g.fillStyle='#2d3d52';
    for(const p of (S.scan||[])){
      const wx=px+p[0]*Math.cos(pth)-p[1]*Math.sin(pth);
      const wy=py+p[0]*Math.sin(pth)+p[1]*Math.cos(pth);
      const [sx,sy]=T(wx,wy,f); g.fillRect(sx-1,sy-1,2,2);
    }
  }
  for(const k in (S.waypoints||{})){
    const w=S.waypoints[k], [sx,sy]=T(w.x,w.y,f);
    const isSel=(k===sel), isGoal=(k===S.goal&&S.driving);
    g.beginPath(); g.arc(sx,sy,isSel?9:6,0,7);
    const bad=(w.frame!==S.frame);
    g.fillStyle=isGoal?'#3fb950':(isSel?'#58a6ff':(bad?'#2a2f38':'#39414d')); g.fill();
    g.fillStyle='#8b94a3'; g.font='11px monospace'; g.fillText(k,sx+11,sy+4);
  }
  const cg = clickGoal || (S.goal_xy ? {x:S.goal_xy[0], y:S.goal_xy[1]} : null);
  if(cg){
    const [sx,sy]=T(cg.x,cg.y,f);
    g.strokeStyle=S.driving?'#3fb950':'#58a6ff'; g.lineWidth=2;
    g.beginPath(); g.arc(sx,sy,10,0,7); g.stroke();
    g.beginPath(); g.moveTo(sx-15,sy); g.lineTo(sx+15,sy);
    g.moveTo(sx,sy-15); g.lineTo(sx,sy+15); g.stroke();
    if(S.pose){ const [rx,ry]=T(S.pose[0],S.pose[1],f);
      g.setLineDash([4,4]); g.strokeStyle='#30465e';
      g.beginPath(); g.moveTo(rx,ry); g.lineTo(sx,sy); g.stroke(); g.setLineDash([]); }
  }
  if(S.pose){
    const [px,py,pth]=S.pose, [sx,sy]=T(px,py,f);
    g.save(); g.translate(sx,sy); g.rotate(-pth);
    g.fillStyle=S.driving?'#3fb950':'#e6e9ef';
    g.beginPath(); g.moveTo(13,0); g.lineTo(-8,7); g.lineTo(-8,-7); g.closePath(); g.fill();
    g.restore();
  }
}

function setTxt(id,t,cls){ const e=$(id); e.textContent=t; e.className=cls||''; }

async function tick(){
  try{
    const r=await fetch('/state',{cache:'no-store'}); S=await r.json();
  }catch(e){ setTxt('status','lost the server','bad'); return; }
  if(VMAX===null && S.v_max_cli){ VMAX=S.v_max_cli; }
  setTxt('frame', S.frame==='map'?'map (SLAM)':'odom (wheel)', S.frame==='map'?'ok':'warn');
  setTxt('mapname', S.map_name||'none loaded', S.map_name?'ok':'warn');
  setTxt('pose', S.pose?`${S.pose[0].toFixed(2)}, ${S.pose[1].toFixed(2)}  ${(S.pose[2]*57.3).toFixed(0)}°`:'-',
         S.fresh?'':'bad');
  setTxt('mux', S.mux===null?'permitting':String(S.mux), (S.mux===null)?'ok':'bad');
  setTxt('blocked', S.blocked?'BLOCKED':'clear', S.blocked?'bad':'ok');
  setTxt('status', S.detail||S.status||'idle', S.driving?'ok':'');
  const box=$('wps');
  if(box.dataset.sig!==JSON.stringify(Object.keys(S.waypoints||{}))){
    box.dataset.sig=JSON.stringify(Object.keys(S.waypoints||{}));
    box.innerHTML='';
    for(const k of Object.keys(S.waypoints||{}).sort()){
      const w=S.waypoints[k], d=document.createElement('div');
      d.className='wp'; d.dataset.k=k;
      const bad=(w.frame!==S.frame);
      d.innerHTML=`<b>${k}</b> <span class="badge ${bad?'bad':''}">${w.frame}</span>`+
                  `<div class=m>x ${w.x.toFixed(2)}  y ${w.y.toFixed(2)}`+
                  (bad?` &mdash; <span class=bad>wrong frame, cannot drive</span>`:``)+`</div>`;
      if(bad){ d.style.opacity=.45; }
      d.onclick=()=>{ sel=k;
        $('go').disabled=bad; $('sel').textContent=bad
          ? `${k} is ${w.frame}-frame; robot is in ${S.frame}. Re-record with --frame ${S.frame}.`
          : 'selected: '+k;
        [...box.children].forEach(e=>e.classList.toggle('sel', e.dataset.k===k)); draw(); };
      box.appendChild(d);
    }
  }
  draw();
}
$('go').onclick=async()=>{
  let r;
  if(clickGoal){ r=await fetch('/goto_xy',{method:'POST',
        body:JSON.stringify({x:clickGoal.x,y:clickGoal.y,cap:cap})}); }
  else if(sel){ r=await fetch('/goto',{method:'POST',
        body:JSON.stringify({name:sel,cap:cap})}); }
  else return;
  if(!r.ok){ const j=await r.json().catch(()=>({})); setTxt('status', j.why||'refused','bad'); } };
$('stop').onclick=async()=>{ clickGoal=null; await fetch('/stop',{method:'POST'}); };
$('spd').oninput=e=>{ cap=e.target.value/100;
  $('spdval').textContent=(VMAX?(VMAX*cap).toFixed(2)+' m/s':(cap*100).toFixed(0)+'%');
  $('spdnote').textContent='applies to the next GO';
  fetch('/cap',{method:'POST',body:JSON.stringify({cap:cap})}); };
c.onclick=ev=>{
  const f=fit(), r=c.getBoundingClientRect();
  const mx=ev.clientX-r.left, my=ev.clientY-r.top;
  let best=null,bd=1e9;
  for(const k in (S.waypoints||{})){ const w=S.waypoints[k], [sx,sy]=T(w.x,w.y,f);
    const d=Math.hypot(sx-mx,sy-my); if(d<bd){bd=d;best=k;} }
  if(best && bd<22){
    const w=S.waypoints[best], bad=(w.frame!==S.frame);
    sel=best; clickGoal=null; $('go').disabled=bad;
    $('sel').textContent=bad?`${best} is ${w.frame}-frame; robot is in ${S.frame}.`
                            :'selected: '+best;
    [...$('wps').children].forEach(e=>e.classList.toggle('sel', e.dataset.k===best));
  } else {
    // Empty space: drive to the point itself. No stored waypoint, no frame question -- it was
    // clicked on a map drawn in the frame the robot is already localized in.
    const [wx,wy]=Tinv(mx,my,f);
    clickGoal={x:wx,y:wy}; sel=null; $('go').disabled=false;
    const d=S.pose?Math.hypot(wx-S.pose[0],wy-S.pose[1]):0;
    $('sel').textContent=`point ${wx.toFixed(2)}, ${wy.toFixed(2)}  (${d.toFixed(1)} m away)`;
    [...$('wps').children].forEach(e=>e.classList.remove('sel'));
  }
  draw();
};
document.addEventListener('keydown',e=>{ if(e.code==='Space'){ e.preventDefault(); $('stop').click(); }});
setInterval(tick, 200); tick();
</script>
"""


def make_handler(state: State, node: DriveNode, vmax: float):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _send(self, body: bytes, ctype: str, code: int = 200):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path.startswith("/state"):
                state.poll()
                snap = state.get_snapshot()
                goal, go, _, _, status, detail, goal_xy = state.read()
                snap.update({"status": status, "detail": detail, "goal": goal,
                             "driving": bool(go), "v_max_cli": vmax})
                self._send(json.dumps(snap).encode(), "application/json")
                return
            if self.path in ("/", "/index.html"):
                self._send(PAGE.encode(), "text/html; charset=utf-8")
                return
            self._send(b"not found", "text/plain", 404)

        def do_POST(self):
            n = int(self.headers.get("Content-Length") or 0)
            raw = self.rfile.read(n) if n else b"{}"
            try:
                body = json.loads(raw or b"{}")
            except ValueError:
                body = {}
            if self.path == "/goto":
                name = str(body.get("name") or "")
                cap = float(body.get("cap") or 1.0)
                wps = load_waypoints()
                if name not in wps:
                    self._send(json.dumps({"ok": False,
                                           "why": f"no waypoint named '{name}'"}).encode(),
                               "application/json", 400)
                    return
                # FRAME MISMATCH IS A WRONG-PLACE BUG, NOT A UNIT BUG. An odom-frame coordinate
                # read while localized in a map frame is not "slightly off" -- it is a number
                # from a different origin entirely, and driving to it sends the robot metres
                # away with total confidence. The pose looks healthy the whole time, which is
                # why this is refused here rather than left to look like a planner fault.
                wp_frame = frame_of(wps[name])
                cur = node.src.frame or "?"
                if wp_frame != cur:
                    self._send(json.dumps({
                        "ok": False,
                        "why": (f"'{name}' is a {wp_frame}-frame waypoint but the robot is "
                                f"localized in the {cur} frame. Those are different origins; "
                                f"driving there would go somewhere arbitrary. "
                                f"Re-record it with --frame {cur}.")}).encode(),
                        "application/json", 409)
                    return
                node.set_cap(cap)
                state.arm(name)
                self._send(b'{"ok":true}', "application/json")
                return
            if self.path == "/goto_xy":
                # A clicked goal is driven the same way a stored one is: plan_step aims at it and
                # the corridor veto stops on anything in the way. There is NO PATH PLANNING here
                # -- this drives at the point, it does not route around obstacles. A click with a
                # wall in between will turn, advance, and stop blocked. That is the honest
                # behaviour of a go-to-point controller and it is why Nav2 exists.
                try:
                    gx, gy = float(body["x"]), float(body["y"])
                except (KeyError, TypeError, ValueError):
                    self._send(b'{"ok":false,"why":"bad point"}', "application/json", 400)
                    return
                pose = node.pose
                if pose is None:
                    self._send(b'{"ok":false,"why":"no pose"}', "application/json", 409)
                    return
                d = math.hypot(gx - pose[0], gy - pose[1])
                if d > MAX_CLICK_GOAL_M:
                    self._send(json.dumps({"ok": False, "why":
                               f"that point is {d:.1f} m away (limit {MAX_CLICK_GOAL_M:.0f} m). "
                               f"Almost always a misclick on a zoomed-out map."}).encode(),
                               "application/json", 409)
                    return
                node.set_cap(float(body.get("cap") or 1.0))
                state.arm_xy(gx, gy)
                self._send(b'{"ok":true}', "application/json")
                return
            if self.path == "/cap":
                node.set_cap(float(body.get("cap") or 1.0))
                self._send(b'{"ok":true}', "application/json")
                return
            if self.path == "/stop":
                state.halt("stop")
                node.stop_and_park()
                self._send(b'{"ok":true}', "application/json")
                return
            self._send(b"not found", "text/plain", 404)
    return H


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8088)
    ap.add_argument("--frame", choices=["auto", "map", "odom"], default="auto")
    ap.add_argument("--max-speed", type=float, default=0.12,
                    help="linear ceiling, m/s. Default 0.12 -- half of waypoint_drive's 0.25, "
                         "because a UI is used with the robot in front of you.")
    ap.add_argument("--max-turn", type=float, default=0.40,
                    help="angular ceiling, rad/s. Default 0.40.")
    a = ap.parse_args()

    base = Limits()
    # THE INTERACTION THAT WOULD SILENTLY BREAK TURNING. waypoint_drive floors an in-place turn
    # UP to w_min (0.20 rad/s) because the 4WS chassis will not rotate below roughly that -- the
    # command is absorbed re-steering the wheels. _turn_rate then clamps DOWN to w_max. So a
    # w_max under w_min inverts the whole thing: every turn is issued below the stall floor and
    # the robot sits there commanding a rotation that never happens. Refuse it out loud.
    if a.max_turn < base.w_min:
        print(f"--max-turn {a.max_turn} is below the chassis stall floor w_min={base.w_min} "
              f"rad/s.\n  Every turn would be commanded below the rate this chassis will "
              f"actually execute, and the robot would sit still while believing it was "
              f"turning.\n  Use --max-turn {base.w_min} or higher.", file=sys.stderr)
        return 1
    if a.max_speed < base.v_min:
        print(f"--max-speed {a.max_speed} is below v_min={base.v_min} m/s, where the chassis "
              f"stalls rather than creeps.", file=sys.stderr)
        return 1

    rclpy.init()
    state = State()
    node = DriveNode(state, a.frame)
    node.set_limits(a.max_speed, a.max_turn)
    print(f"\n  {node.frame_desc}")
    print(f"  speed ceiling {a.max_speed:.2f} m/s, turn ceiling {a.max_turn:.2f} rad/s")
    print(f"\n  ==> http://{a.host}:{a.port}\n")
    print("  Nothing moves until you select a waypoint and press GO. Space bar = STOP.\n")

    srv = ThreadingHTTPServer((a.host, a.port), make_handler(state, node, a.max_speed))
    t = threading.Thread(target=srv.serve_forever, daemon=True)
    t.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_and_park()
        srv.shutdown()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
