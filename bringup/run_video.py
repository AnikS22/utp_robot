#!/usr/bin/env python3
"""Turn a recorded run's camera stream into a video file, with the route's own events burnt in.

    python3 bringup/run_video.py runs/<run>                       # the colour stream
    python3 bringup/run_video.py runs/<run> --topic /scan_nav     # (not an image topic: refused)
    python3 bringup/run_video.py runs/<run> --fps 30 --no-overlay

WHY THIS EXISTS. run_recorder.py already keeps everything -- a full rosbag of every topic, plus
on-event JPEGs at 2 fps -- but 21 GB of mcap is not something anyone can watch, and 49 stills are
not a video. The thing a person actually wants out of a trial is the camera stream with the
decisions marked on it: this is where it localized, this is where it refused the reach, this is
where the gripper met the plate.

THE EVENTS ARE THE POINT. events.jsonl carries the route's own account of what it decided and
when -- localized, leg_start, press_contact, doors_open -- and those stamps share a clock with the
image messages, so each frame can be labelled with the stage it belongs to. A video without them
is footage; with them it is evidence you can scrub.

No ffmpeg on this machine, so the writer is OpenCV's, which bundles its own encoder.
"""
from __future__ import annotations
import argparse, bisect, json, sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO)); sys.path.insert(0, str(REPO / "bringup"))
from _ros_env import require_ros  # noqa: E402
require_ros()

import cv2  # noqa: E402
import numpy as np  # noqa: E402
import rosbag2_py  # noqa: E402
from rclpy.serialization import deserialize_message  # noqa: E402
from sensor_msgs.msg import Image  # noqa: E402


def load_events(run: Path):
    """[(seconds, kind, detail)] sorted by time, or [] if the route logged none."""
    f = run / "events.jsonl"
    if not f.exists():
        return []
    out = []
    for line in f.read_text(errors="replace").splitlines():
        if not line.strip():
            continue
        try:
            e = json.loads(line)
        except json.JSONDecodeError:
            continue
        if "stamp" in e:
            out.append((float(e["stamp"]), e.get("kind", ""), e.get("detail", "")))
    return sorted(out)


def to_bgr(msg: Image) -> np.ndarray | None:
    """Image -> BGR, without cv_bridge: the encodings this rig records are few and known."""
    h, w, enc = msg.height, msg.width, msg.encoding
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    if enc in ("rgb8", "bgr8"):
        if buf.size < h * w * 3:
            return None
        img = buf[: h * w * 3].reshape(h, w, 3)
        return img[:, :, ::-1].copy() if enc == "rgb8" else img.copy()
    if enc in ("16UC1", "mono16"):
        d = np.frombuffer(msg.data, dtype=np.uint16)[: h * w].reshape(h, w).astype(float)
        # Depth is metres-ish; show it as a ramp so the video is legible rather than black.
        finite = d[(d > 0) & np.isfinite(d)]
        hi = float(np.percentile(finite, 95)) if finite.size else 1.0
        u8 = np.clip(d / max(hi, 1e-6) * 255.0, 0, 255).astype(np.uint8)
        return cv2.applyColorMap(u8, cv2.COLORMAP_TURBO)
    if enc == "mono8":
        return cv2.cvtColor(buf[: h * w].reshape(h, w), cv2.COLOR_GRAY2BGR)
    return None


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run", type=Path)
    ap.add_argument("--topic", default="/mast_cam/color/image_raw")
    ap.add_argument("--out", type=Path, default=None)
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--every", type=int, default=1, help="keep 1 frame in N (thins a 30 fps stream)")
    ap.add_argument("--no-overlay", action="store_true")
    a = ap.parse_args()

    bag = a.run / "rosbag"
    if not bag.exists():
        print(f"no rosbag in {a.run}", file=sys.stderr)
        return 2
    out = a.out or (a.run / "artifacts" / f"{a.topic.strip('/').replace('/', '_')}.mp4")
    out.parent.mkdir(parents=True, exist_ok=True)

    events = load_events(a.run)
    ev_t = [e[0] for e in events]

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap"),
                rosbag2_py.ConverterOptions("", ""))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[a.topic]))

    writer = None
    n_read = n_written = 0
    t0 = None
    while reader.has_next():
        topic, data, stamp_ns = reader.read_next()
        n_read += 1
        if (n_read - 1) % a.every:
            continue
        msg = deserialize_message(data, Image)
        frame = to_bgr(msg)
        if frame is None:
            continue
        t = stamp_ns / 1e9
        if t0 is None:
            t0 = t
        if writer is None:
            h, w = frame.shape[:2]
            writer = cv2.VideoWriter(str(out), cv2.VideoWriter_fourcc(*"mp4v"),
                                     a.fps / max(a.every, 1) if a.fps > 0 else 30.0, (w, h))
            if not writer.isOpened():
                print(f"could not open {out} for writing", file=sys.stderr)
                return 1
        if not a.no_overlay:
            # The stage this frame belongs to: the last event at or before it.
            i = bisect.bisect_right(ev_t, t) - 1
            label = f"{events[i][1]} {events[i][2]}"[:70] if i >= 0 else ""
            cv2.rectangle(frame, (0, 0), (frame.shape[1], 34), (0, 0, 0), -1)
            cv2.putText(frame, f"t+{t - t0:6.1f}s   {label}", (10, 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 1, cv2.LINE_AA)
        writer.write(frame)
        n_written += 1
        if n_written % 200 == 0:
            print(f"  {n_written} frames ...", flush=True)

    if writer is None:
        print(f"no frames on {a.topic}", file=sys.stderr)
        return 1
    writer.release()
    mb = out.stat().st_size / 1e6
    print(f"{out}  ({n_written} frames from {n_read} messages, {mb:.0f} MB, "
          f"{n_written / max(a.fps / max(a.every,1), 1e-6):.0f} s)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
