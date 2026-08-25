#!/usr/bin/env python3
"""Local xArm safety-boundary dashboard. Opening it never enables motion."""
from __future__ import annotations

import argparse, json, threading, time, sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2, numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "bringup"))
from arm_workspace import (DEFAULT_CONFIG, LABELS, apply_limits, arm_error, connect, contains,
                           effective_boundary, load, pose, write_bound)
from check_marker import detect


class Dashboard(Node):
    def __init__(self, arm, config):
        super().__init__("utp_arm_ui")
        self.arm, self.config, self.lock = arm, config, threading.RLock()
        self.jpeg = None; self.marker = {"seen": False}; self.frame_n = 0
        self.create_subscription(Image, "/mast_cam/color/image_raw", self.image, qos_profile_sensor_data)

    def image(self, m):
        try:
            a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, :m.width*3].reshape(m.height,m.width,3)
            bgr = cv2.cvtColor(a, cv2.COLOR_RGB2BGR) if m.encoding == "rgb8" else a.copy()
            self.frame_n += 1
            if self.frame_n % 5 == 0:
                corners, ids, name = detect(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY), cv2.aruco)
                if ids is not None:
                    cv2.aruco.drawDetectedMarkers(bgr, corners, ids)
                    self.marker = {"seen": True, "ids": ids.ravel().tolist(), "dictionary": name}
                else: self.marker = {"seen": False}
            out = cv2.resize(bgr, (720, int(720*m.height/m.width)))
            ok, buf = cv2.imencode(".jpg", out, [cv2.IMWRITE_JPEG_QUALITY, 72])
            if ok:
                with self.lock: self.jpeg = buf.tobytes()
        except Exception: pass

    def status(self):
        with self.lock:
            p = pose(self.arm); code, joints = self.arm.get_servo_angle(is_radian=False)
            rc, reduced = self.arm.get_reduced_states(is_radian=False)
            cfg = load(self.config)
            return {"tcp": p, "joints": list(joints[:6]) if code == 0 else None,
                    "error": int(self.arm.error_code or 0), "error_text": arm_error(self.arm),
                    "state": self.arm.state, "mode": self.arm.mode, "bounds": cfg["bounds_mm"],
                    "reduced": reduced if rc == 0 else None, "marker": self.marker}

    def action(self, d):
        with self.lock:
            cfg = load(self.config); action = d.get("action")
            if action == "record":
                label = d.get("label")
                if label not in LABELS: raise ValueError("invalid boundary label")
                write_bound(self.config, cfg, label, pose(self.arm))
                return f"Recorded {label} read-only"
            boundary = effective_boundary(cfg)
            if action == "apply":
                if not d.get("confirmed"): raise ValueError("confirmation required")
                if self.arm.error_code: raise RuntimeError(f"controller error {arm_error(self.arm)}")
                apply_limits(self.arm, cfg, boundary)
                return "Controller fence applied and read back"
            if action == "jog":
                if not d.get("confirmed"): raise ValueError("motion enable required")
                if self.arm.error_code: raise RuntimeError(f"controller error {arm_error(self.arm)}")
                axis, sign = d.get("axis"), int(d.get("sign", 0))
                if axis not in "xyz" or sign not in (-1,1): raise ValueError("invalid jog")
                # Reapply and verify on every motion request, so a reboot cannot silently remove it.
                apply_limits(self.arm, cfg, boundary)
                cur = pose(self.arm); target = cur[:]; target["xyz".index(axis)] += 5*sign
                if not contains(boundary, target[:3]): raise ValueError("BLOCKED by software envelope")
                self.arm.motion_enable(enable=True); self.arm.set_mode(0); self.arm.set_state(0)
                ret = self.arm.set_position(x=target[0],y=target[1],z=target[2],roll=target[3],
                    pitch=target[4],yaw=target[5],speed=20,mvacc=50,is_radian=False,wait=True,timeout=10)
                self.arm.set_state(4)
                if ret != 0 or self.arm.error_code:
                    raise RuntimeError(f"stopped: SDK={ret}, controller={arm_error(self.arm)}")
                return "Moved one 5 mm step; controller returned STOPPED"
            raise ValueError("unknown action")


PAGE = r'''<!doctype html><meta name=viewport content="width=device-width"><title>xArm Safety</title>
<style>body{font:16px system-ui;background:#111;color:#eee;margin:20px;max-width:1100px}h1{margin:.2em 0}.warn{background:#5b160d;padding:14px;border:2px solid #ff765f}.ok{color:#74e58d}.bad{color:#ff766d}button{font-size:17px;padding:10px;margin:4px;background:#303640;color:white;border:1px solid #697282;border-radius:5px}button.motion{background:#8a321f}.grid{display:grid;grid-template-columns:1fr 1fr;gap:18px}img{width:100%;background:#222;min-height:260px}pre{background:#20242b;padding:12px;white-space:pre-wrap}.bounds{display:grid;grid-template-columns:repeat(3,1fr)}label{display:block;padding:10px;background:#292d35;margin:8px 0}@media(max-width:800px){.grid{grid-template-columns:1fr}}</style>
<h1>xArm Safety Envelope</h1><div class=warn><b>TCP fence is not a whole-arm fence.</b> It cannot guarantee the elbow or gripper body stays above the laptop screen. Keep the E-stop held and screen clear while teaching. Opening this page never enables motion.</div>
<div class=grid><section><h2>Camera / AprilTag</h2><img id=cam src=/camera.jpg><div id=marker>waiting...</div></section><section><h2>Robot</h2><pre id=status>loading...</pre><label><input type=checkbox id=enable> I am holding the arm E-stop and authorize exactly one 5 mm motion</label><div><button class=motion onclick="jog('x',1)">X+</button><button class=motion onclick="jog('x',-1)">X−</button><button class=motion onclick="jog('y',1)">Y+</button><button class=motion onclick="jog('y',-1)">Y−</button><button class=motion onclick="jog('z',1)">Z+</button><button class=motion onclick="jog('z',-1)">Z−</button></div></section></div>
<h2>Record current TCP as safe extreme</h2><div class=bounds id=bounds></div><p>Use manufacturer teach/manual mode to position the arm. Recording is read-only. A 20 mm inward margin is applied later.</p>
<button onclick=applyFence()>Apply and verify controller fence</button><pre id=msg></pre>
<script>
const labels=['x_min','x_max','y_min','y_max','z_min','z_max'];
for(const x of labels) bounds.innerHTML+=`<button onclick="act({action:'record',label:'${x}'})">Record ${x}</button>`;
async function refresh(){try{let s=await(await fetch('/status')).json();status.textContent=`TCP mm: ${s.tcp.slice(0,3).map(x=>x.toFixed(1)).join(', ')}\nRPY deg: ${s.tcp.slice(3).map(x=>x.toFixed(1)).join(', ')}\nJoints deg: ${s.joints?s.joints.map(x=>x.toFixed(1)).join(', '):'unavailable'}\nState ${s.state}, mode ${s.mode}\nError ${s.error_text}\nBounds: ${JSON.stringify(s.bounds,null,2)}\nController: ${JSON.stringify(s.reduced)}`;marker.textContent=s.marker.seen?`MARKER SEEN: ${s.marker.dictionary}, ids ${s.marker.ids}`:'No AprilTag/Aruco detected';cam.src='/camera.jpg?t='+Date.now()}catch(e){status.textContent=e}};
async function act(x){let r=await fetch('/action',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(x)});let j=await r.json();msg.textContent=(r.ok?'OK: ':'STOP: ')+(j.message||j.error);enable.checked=false;refresh()}
function jog(axis,sign){act({action:'jog',axis,sign,confirmed:enable.checked})}
function applyFence(){act({action:'apply',confirmed:confirm('Apply these measured limits in the xArm controller?')})}
setInterval(refresh,1000);refresh();
</script>'''


def handler(dash):
    class H(BaseHTTPRequestHandler):
        def log_message(self,*a): pass
        def sendb(self, code, kind, body):
            self.send_response(code); self.send_header("Content-Type",kind); self.send_header("Cache-Control","no-store"); self.end_headers(); self.wfile.write(body)
        def do_GET(self):
            if self.path.startswith("/camera.jpg"):
                with dash.lock: b=dash.jpeg
                return self.sendb(200,"image/jpeg",b or b"")
            if self.path=="/status":
                try: return self.sendb(200,"application/json",json.dumps(dash.status()).encode())
                except Exception as e: return self.sendb(500,"application/json",json.dumps({"error":str(e)}).encode())
            return self.sendb(200,"text/html; charset=utf-8",PAGE.encode())
        def do_POST(self):
            try:
                d=json.loads(self.rfile.read(int(self.headers.get("Content-Length",0))) or b"{}"); m=dash.action(d)
                self.sendb(200,"application/json",json.dumps({"message":m}).encode())
            except Exception as e: self.sendb(400,"application/json",json.dumps({"error":str(e)}).encode())
    return H


def main():
    ap=argparse.ArgumentParser(); ap.add_argument("--port",type=int,default=8430); ap.add_argument("--host",default="127.0.0.1"); a=ap.parse_args()
    arm=connect("192.168.1.221"); rclpy.init(); dash=Dashboard(arm,DEFAULT_CONFIG)
    srv=ThreadingHTTPServer((a.host,a.port),handler(dash)); threading.Thread(target=srv.serve_forever,daemon=True).start()
    print(f"ARM UI: http://{a.host}:{a.port}")
    try:rclpy.spin(dash)
    except KeyboardInterrupt:pass
    finally:
        try:arm.set_state(4)
        except Exception:pass
        srv.shutdown();arm.disconnect();dash.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
