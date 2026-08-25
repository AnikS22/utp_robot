#!/usr/bin/env python3
"""Record spatially indexed camera frames during a mapping drive.

Saves a frame after meaningful motion rather than recording 30 FPS raw video. Each JSONL row
contains odom pose and the current map->odom correction, so imagery can later be laid out against
the occupancy grid without pretending a forward camera is a metric overhead sensor.
"""
from __future__ import annotations

import argparse, json, math, time
from pathlib import Path

import cv2, numpy as np, rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage


def yaw(q):
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Recorder(Node):
    def __init__(self, out: Path, distance: float, angle: float):
        super().__init__("utp_map_image_recorder")
        self.out=out; self.frames=out/"frames"; self.frames.mkdir(parents=True,exist_ok=True)
        self.log=(out/"poses.jsonl").open("a",buffering=1)
        self.distance=distance; self.angle=math.radians(angle); self.img=None; self.depth=None
        self.info=None; self.odom=None
        self.map_odom=None; self.last=None; self.n=0
        self.create_subscription(Image,"/mast_cam/color/image_raw",self.on_img,qos_profile_sensor_data)
        self.create_subscription(Image,"/mast_cam/aligned_depth_to_color/image_raw",self.on_depth,qos_profile_sensor_data)
        self.create_subscription(CameraInfo,"/mast_cam/aligned_depth_to_color/camera_info",self.on_info,qos_profile_sensor_data)
        self.create_subscription(Odometry,"/odom",self.on_odom,10)
        self.create_subscription(TFMessage,"/tf",self.on_tf,10)
        self.create_timer(.2,self.tick)

    def on_img(self,m): self.img=m
    def on_depth(self,m): self.depth=m
    def on_info(self,m): self.info=m
    def on_odom(self,m): self.odom=m
    def on_tf(self,m):
        for t in m.transforms:
            if t.header.frame_id=="map" and t.child_frame_id=="odom":
                self.map_odom=(t.transform.translation.x,t.transform.translation.y,yaw(t.transform.rotation))

    def decode(self,m):
        a=np.frombuffer(m.data,np.uint8).reshape(m.height,m.step)[:,:m.width*3].reshape(m.height,m.width,3)
        return cv2.cvtColor(a,cv2.COLOR_RGB2BGR) if m.encoding=="rgb8" else a.copy()

    def decode_depth(self,m):
        if m.encoding not in ("16UC1", "mono16"):
            raise ValueError(f"unsupported aligned depth encoding {m.encoding}")
        a=np.frombuffer(m.data,np.uint16).reshape(m.height,m.step//2)[:,:m.width]
        return a.byteswap() if bool(m.is_bigendian) != (not np.little_endian) else a.copy()

    def tick(self):
        # Never label odom coordinates as map coordinates during SLAM startup. Wait until the
        # current session has actually published map->odom; yesterday's default identity produced
        # plausible-looking but wrong image geotags.
        if any(x is None for x in (self.img,self.depth,self.info,self.odom,self.map_odom)):return
        image_ns=self.img.header.stamp.sec*1_000_000_000+self.img.header.stamp.nanosec
        depth_ns=self.depth.header.stamp.sec*1_000_000_000+self.depth.header.stamp.nanosec
        if abs(image_ns-depth_ns)>100_000_000:return
        p=self.odom.pose.pose; cur=(p.position.x,p.position.y,yaw(p.orientation))
        if self.last is not None:
            if math.hypot(cur[0]-self.last[0],cur[1]-self.last[1])<self.distance and abs(wrap(cur[2]-self.last[2]))<self.angle:return
        self.n+=1; name=f"frame_{self.n:05d}.jpg"; depth_name=f"frame_{self.n:05d}_depth.png"
        frame=self.decode(self.img); depth=self.decode_depth(self.depth)
        if not cv2.imwrite(str(self.frames/name),frame,[cv2.IMWRITE_JPEG_QUALITY,90]):return
        if not cv2.imwrite(str(self.frames/depth_name),depth):return
        mx,my,mt=self.map_odom; c,s=math.cos(mt),math.sin(mt)
        map_pose=(mx+c*cur[0]-s*cur[1],my+s*cur[0]+c*cur[1],wrap(mt+cur[2]))
        row={"frame":name,"depth":depth_name,"stamp":time.time(),
             "image_stamp_ns":image_ns,"depth_stamp_ns":depth_ns,"depth_units_m":0.001,
             "intrinsics":{"width":self.info.width,"height":self.info.height,"k":list(self.info.k)},
             "odom":{"x":cur[0],"y":cur[1],"yaw":cur[2]},
             "map_to_odom":{"x":mx,"y":my,"yaw":mt},
             "map_pose":{"x":map_pose[0],"y":map_pose[1],"yaw":map_pose[2]}}
        self.log.write(json.dumps(row)+"\n"); self.last=cur
        self.get_logger().info(f"saved {name} at map ({map_pose[0]:.2f},{map_pose[1]:.2f})")

def main():
    ap=argparse.ArgumentParser();ap.add_argument("--name",default="bottom_floor_visual_01")
    ap.add_argument("--distance-m",type=float,default=.75);ap.add_argument("--angle-deg",type=float,default=20)
    a=ap.parse_args();out=Path(__file__).resolve().parent.parent/"maps"/a.name
    rclpy.init();n=Recorder(out,a.distance_m,a.angle_deg)
    print(f"recording geotagged images -> {out}")
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:
        n.log.close();n.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=="__main__":main()
