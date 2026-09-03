#!/usr/bin/env python3
"""Record a trial for the paper figures, without disturbing the thing being recorded.

    python3 bringup/run_recorder.py --scene elevator --method ours
    python3 bringup/run_recorder.py --scene elevator --dir runs/my_run    # explicit
    # stop with Ctrl-C, or SIGTERM from the route that started it

Writes runs/<utc>_<method>_<scene>/ :

    meta.json      what was run: scene, method, git sha, map, config hashes, host
    poses.jsonl    {stamp, map:{x,y,yaw}} at --pose-hz          <- the green line
    frames/*.jpg   ego-centric, throttled, downscaled           <- the keyframe column
    events.jsonl   created here, APPENDED BY THE ROUTE (see bringup/run_event.sh)

WHY IT IS BUILT THIS CAREFULLY. The detector is the thing we are recording, it runs on
cuda:0, and it has already been starved into CUDA OOM once -- by RViz holding 14.3 GB of
GPU with the full 131k-point cloud displayed. A recorder that costs the detector its
frame budget does not just slow the run down, it changes the result being published. So:

  * NO GPU. Nothing here opens a CUDA context. JPEG encoding is cv2 on the CPU.
  * NO /ouster/points. /scan already runs 4.6-6.4 Hz against the sensor's 10 because of
    DDS transport loss; another subscriber on the ~1 MB cloud makes that worse. The map
    background for the figure comes from maps/<name>.pgm, which is already on disk.
  * IMAGES ARE THROTTLED AND DOWNSCALED. 2 Hz, 640 px wide, JPEG q85 -- about 40 kB a
    frame, ~5 MB a minute. grab_frame.py's full-resolution PNG plus a depth .npy is for
    grounding; a film strip does not need it. The subscriber is BEST_EFFORT with depth 1,
    so a slow writer drops frames rather than queueing them and adding latency upstream.
  * POSES ARE FREE. A TF lookup at 10 Hz is a few hundred bytes a second, so the
    trajectory is logged continuously and never sampled.
  * SEPARATE PROCESS. Start it with `nice -n 10` so it cannot take CPU from the
    detector, and it writes to its own directory so a crash here cannot corrupt a
    capture the trial depends on.

Deliberately NOT a ROS service or an action. The route is bash; it appends to
events.jsonl with echo (bringup/run_event.sh). One less thing that can fail to connect.
"""
from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent


def _decode_rgb(msg) -> np.ndarray:
    """Image -> HxWx3 uint8 RGB. Same no-cv_bridge approach as bringup/grab_frame.py:48."""
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    if msg.encoding not in ("rgb8", "bgr8"):
        raise ValueError(f"unexpected colour encoding {msg.encoding!r}")
    a = buf.reshape(msg.height, msg.step)[:, : msg.width * 3].reshape(msg.height, msg.width, 3)
    return a[:, :, ::-1] if msg.encoding == "bgr8" else a


def _git_sha() -> str:
    try:
        return subprocess.run(["git", "-C", str(REPO), "rev-parse", "HEAD"],
                              capture_output=True, text=True, timeout=5).stdout.strip()
    except Exception:
        return ""


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scene", default="elevator", help="scene name, goes in the path and meta")
    ap.add_argument("--method", default="ours", help="method name; matches trials.jsonl method_name")
    ap.add_argument("--dir", default=None, help="explicit output dir (default runs/<utc>_<method>_<scene>)")
    ap.add_argument("--ns", default="mast_cam", help="camera namespace")
    ap.add_argument("--fps", type=float, default=2.0, help="frames per second to SAVE while capturing")
    ap.add_argument("--frames", choices=("on-event", "continuous"), default="on-event",
                    help="on-event (default): subscribe only in a window around each event. "
                         "continuous: hold the subscription for the whole run")
    ap.add_argument("--window", type=float, default=2.5,
                    help="seconds to keep capturing after an event, in on-event mode")
    ap.add_argument("--width", type=int, default=640, help="downscale frames to this width (0 = full)")
    ap.add_argument("--quality", type=int, default=85, help="JPEG quality")
    ap.add_argument("--pose-hz", type=float, default=10.0, help="trajectory sample rate")
    ap.add_argument("--map", default=None, help="map name for meta (default: from maps/.loaded_map)")
    a = ap.parse_args()

    import cv2
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import Image
    import tf2_ros

    stamp_name = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    out = Path(a.dir) if a.dir else REPO / "runs" / f"{stamp_name}_{a.method}_{a.scene}"
    (out / "frames").mkdir(parents=True, exist_ok=True)
    (out / "decisions").mkdir(parents=True, exist_ok=True)

    loaded_map = a.map
    if loaded_map is None:
        try:
            loaded_map = (REPO / "maps" / ".loaded_map").read_text().split()[0]
        except Exception:
            loaded_map = ""

    meta = {
        "scene": a.scene, "method": a.method, "map": loaded_map,
        "started_utc": datetime.now(timezone.utc).isoformat(),
        "git_sha": _git_sha(),
        "host": os.uname().nodename,
        "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", ""),
        "camera_ns": a.ns, "frame_fps": a.fps, "frame_width": a.width,
        "note": "poses.jsonl is the trajectory; events.jsonl is appended by the route, not by this process",
    }
    (out / "meta.json").write_text(json.dumps(meta, indent=2) + "\n")
    # Create it so the route can append without testing for existence.
    (out / "events.jsonl").touch()

    rclpy.init()
    node = Node("utp_run_recorder")
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, node)

    # BEST_EFFORT depth 1: if we are slow, DROP frames. A queue here would hold messages
    # and add latency to a topic the grounder also reads.
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                     history=HistoryPolicy.KEEP_LAST)

    state = {"last_frame": 0.0, "n_frames": 0, "n_poses": 0, "latest": None}

    def on_img(msg):
        now = time.time()
        if now - state["last_frame"] < 1.0 / max(a.fps, 0.01):
            return                      # throttle BEFORE decoding -- decoding is the cost
        state["last_frame"] = now
        try:
            rgb = _decode_rgb(msg)
        except Exception:
            return
        if a.width and rgb.shape[1] > a.width:
            h = int(round(rgb.shape[0] * a.width / rgb.shape[1]))
            rgb = cv2.resize(rgb, (a.width, h), interpolation=cv2.INTER_AREA)
        name = out / "frames" / f"{now:.3f}.jpg"
        cv2.imwrite(str(name), rgb[:, :, ::-1], [int(cv2.IMWRITE_JPEG_QUALITY), a.quality])
        state["n_frames"] += 1

    # SUBSCRIBE ONLY WHEN WE NEED A FRAME.
    #
    # An always-on subscriber is NOT free, and claiming otherwise was wrong. BEST_EFFORT
    # with depth 1 bounds THIS reader's queue; it does not stop the publisher and the DDS
    # middleware from serialising and transporting every sample to an extra process. The
    # D435 runs 1280x720 rgb8 at 30 Hz (config/camera.yaml:31-35) -- about 83 MB/s before
    # overhead -- and that work happens for every frame even though the callback throws
    # most of them away. The subscriber that matters here is the GROUNDER's, in
    # grab_frame.py, and adding transport load beside it is exactly the kind of
    # "shouldn't matter" that has already cost this project a night.
    #
    # The figure needs frames AT THE MARKERS, not a continuous stream. So the reader only
    # exists inside a short window after an event; the rest of the run there is no second
    # reader at all. Poses stay continuous -- TF is small and it is the trajectory.
    img_sub = {"h": None}

    def _sub_on():
        if img_sub["h"] is None:
            img_sub["h"] = node.create_subscription(
                Image, f"/{a.ns}/color/image_raw", on_img, qos)

    def _sub_off():
        if img_sub["h"] is not None:
            node.destroy_subscription(img_sub["h"])
            img_sub["h"] = None

    if a.frames == "continuous":
        _sub_on()
    else:
        # Watch events.jsonl -- the route appends to it (bringup/run_event.sh). A new line
        # means something happened worth a picture. Polling a file size is far cheaper than
        # holding a 30 Hz image reader open, and it needs no connection to the route.
        ev_state = {"size": 0, "until": 0.0}

        def poll_events():
            try:
                sz = (out / "events.jsonl").stat().st_size
            except OSError:
                return
            if sz > ev_state["size"]:
                ev_state["size"] = sz
                ev_state["until"] = time.time() + a.window
                _sub_on()
            elif img_sub["h"] is not None and time.time() > ev_state["until"]:
                _sub_off()

        node.create_timer(0.2, poll_events)

    posef = (out / "poses.jsonl").open("a", buffering=1)

    def on_pose():
        try:
            t = buf.lookup_transform("map", "base_link", rclpy.time.Time())
        except Exception:
            return
        q, v = t.transform.rotation, t.transform.translation
        yaw = 2.0 * np.arctan2(q.z, q.w)
        posef.write(json.dumps({"stamp": time.time(),
                                "map": {"x": v.x, "y": v.y, "yaw": float(yaw)}}) + "\n")
        state["n_poses"] += 1

    node.create_timer(1.0 / max(a.pose_hz, 0.1), on_pose)

    stop = {"now": False}

    def _sig(_s, _f):
        stop["now"] = True
    signal.signal(signal.SIGINT, _sig)
    signal.signal(signal.SIGTERM, _sig)

    print(f"[recorder] {out}", flush=True)
    print(f"[recorder] frames: {a.frames} ({a.fps} Hz @ {a.width or 'full'} px"
          + (f", {a.window}s window)" if a.frames == "on-event" else ")")
          + f", poses {a.pose_hz} Hz, no GPU, no /ouster/points", flush=True)
    if a.frames == "on-event":
        print("[recorder] the image reader exists ONLY around events, so the grounder does not "
              "share the topic with a second full-rate subscriber", flush=True)
    try:
        while rclpy.ok() and not stop["now"]:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        posef.close()
        meta["ended_utc"] = datetime.now(timezone.utc).isoformat()
        meta["n_frames"] = state["n_frames"]
        meta["n_poses"] = state["n_poses"]
        (out / "meta.json").write_text(json.dumps(meta, indent=2) + "\n")
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
        print(f"[recorder] {state['n_frames']} frames, {state['n_poses']} poses -> {out}", flush=True)
        if state["n_poses"] == 0:
            print("[recorder] WARNING: no poses. Was map->base_link available? "
                  "The trajectory figure needs this and it cannot be recovered later.",
                  file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
