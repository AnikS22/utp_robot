#!/usr/bin/env python3
"""Read-only simultaneous ROS rate, map, safety and TF probe for startup."""
import importlib, json, sys, time
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (qos_profile_sensor_data, QoSProfile, ReliabilityPolicy,
                           HistoryPolicy, DurabilityPolicy)
except Exception as exc:                       # a probe that cannot run must say so, not read 0 Hz
    print(f"err|rclpy unavailable: {exc}")
    raise SystemExit(0)

settle, window, tf_budget = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
topics, tfs, gate_topic = [], [], None
for spec in sys.argv[4:]:
    part = spec.split(":")
    if part[0] == "topic":  topics.append((part[1], part[2], part[3], part[4]))
    elif part[0] == "tf":   tfs.append((part[1], part[2]))
    elif part[0] == "gates": gate_topic = part[1]

RELIABLE = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
                      history=HistoryPolicy.KEEP_LAST)
# /map is LATCHED and published only when it changes, so it must be counted as "ever arrived",
# never as a rate: a transient-local message lands once, during discovery.
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)

def qos_for(kind):
    if kind == "reliable": return RELIABLE
    if kind == "latched":  return LATCHED
    return qos_profile_sensor_data          # BEST_EFFORT: compatible with any publisher

rclpy.init()
node = Node("utp_bringup_probe")
count, seen = {}, {}
for (topic, mod, cls, kind) in topics:
    try:
        msg = getattr(importlib.import_module(mod), cls)
    except Exception as exc:
        print(f"err|{topic}: {exc}")
        continue
    count[topic] = 0
    seen[topic] = 0
    def make(name):
        def cb(_m):
            count[name] += 1
            seen[name] += 1
        return cb
    node.create_subscription(msg, topic, make(topic), qos_for(kind))

gates, status = {}, {"n": 0}
if gate_topic:
    from std_msgs.msg import String
    def on_status(m):
        try:
            st = json.loads(m.data)
        except Exception:
            return
        status["n"] += 1
        for k, v in (st.get("gates") or {}).items():
            gates[k] = gates.get(k, 0) + (1 if v else 0)
    node.create_subscription(String, gate_topic, on_status, qos_profile_sensor_data)

buf = None
if tfs:
    try:
        from tf2_ros import Buffer, TransformListener
        buf = Buffer()
        TransformListener(buf, node, spin_thread=False)
    except Exception as exc:
        print(f"err|tf2_ros unavailable: {exc}")

# PHASE 1 -- discovery. Spin without believing anything counted here.
t0 = time.monotonic()
while time.monotonic() - t0 < settle:
    rclpy.spin_once(node, timeout_sec=0.02)
# PHASE 2 -- reset, then measure. `seen` is deliberately NOT reset: it carries the latched
# messages that can only ever have arrived during discovery.
for k in count:
    count[k] = 0
gates.clear()
status["n"] = 0
t1 = time.monotonic()
while time.monotonic() - t1 < window:
    rclpy.spin_once(node, timeout_sec=0.02)
el = max(time.monotonic() - t1, 1e-6)

for topic in count:
    print(f"hz:{topic}|{count[topic]/el:.2f}")
    print(f"seen:{topic}|{seen[topic]}")
if gate_topic:
    print(f"gaten|{status['n']}")
    for k, v in gates.items():
        print(f"gate:{k}|{(100.0 * v / status['n']) if status['n'] else 0:.0f}")

# TF, asked so that it can come back "no". can_transform with a ZERO timeout measures this node's
# own subscription setup, not availability. Poll all missing edges against one shared
# deadline so absent downstream stages cannot multiply the startup delay.
if buf is not None:
    from rclpy.time import Time
    from startup_wait import wait_for_transforms
    results = wait_for_transforms(
        tfs, lambda a, b: buf.can_transform(a, b, Time()),
        lambda: rclpy.spin_once(node, timeout_sec=0.05), tf_budget)
    for (a, b), good in results.items():
        print(f"tf:{a}>{b}|{'ok' if good else 'MISSING'}")

node.destroy_node()
try:
    rclpy.shutdown()
except Exception:
    pass
