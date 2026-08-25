"""Lesson 4a: prove the bridge works -- list topics and measure their rates.

    C:\\isaacsim\\python.bat ros2_client\\topic_monitor.py

This is the tool to reach for first when something looks wrong. It answers
three questions in order:

  1. Is anything advertised at all?     -> bridge / domain problem
  2. Is it advertised but silent?       -> simulation is not *playing*
  3. Is it publishing but too slowly?   -> render rate or frameSkipCount

Nothing here is Isaac-specific: it is an ordinary ROS 2 node, which is exactly
the point -- from ROS 2's side, the simulator is just another robot.
"""

from __future__ import annotations

import argparse
import time
from collections import defaultdict

from _bootstrap import init_ros2


def parse_args():
    p = argparse.ArgumentParser(description="Monitor topics published by Isaac Sim")
    p.add_argument("--duration", type=float, default=10.0, help="seconds to sample")
    return p.parse_args()


def main() -> int:
    args = parse_args()
    rclpy, node, cfg = init_ros2("warehouse_amr_monitor")

    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import CameraInfo, Image, JointState, LaserScan

    counts = defaultdict(int)
    last_msg = {}

    def counter(name):
        def cb(msg):
            counts[name] += 1
            last_msg[name] = msg
        return cb

    subs = [
        (cfg.topic("clock"), Clock),
        (cfg.topic("odom"), Odometry),
        (cfg.topic("scan"), LaserScan),
        (cfg.topic("joint_states"), JointState),
        (cfg.topic("rgb"), Image),
        (cfg.topic("depth"), Image),
        (cfg.topic("camera_info"), CameraInfo),
    ]
    for topic, msg_type in subs:
        node.create_subscription(msg_type, topic, counter(topic), 10)

    print("Discovering (2s) ...")
    deadline = time.time() + 2.0
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)

    advertised = dict(node.get_topic_names_and_types())
    print("\n--- topics visible on domain %d ---" % cfg.ros_domain_id)
    for name in sorted(advertised):
        print("  %-30s %s" % (name, ",".join(advertised[name])))
    if len(advertised) <= 3:
        print("\n  Only built-in topics are visible. Is 02_run_sim_ros2.py running,")
        print("  and does its ROS_DOMAIN_ID in .env match this one?")

    print("\nSampling for %.0fs ..." % args.duration)
    start = time.time()
    while time.time() - start < args.duration:
        rclpy.spin_once(node, timeout_sec=0.05)
    elapsed = time.time() - start

    print("\n--- measured rates ---")
    print("  %-30s %8s  %s" % ("topic", "Hz", "detail"))
    for topic, _ in subs:
        hz = counts[topic] / elapsed
        print("  %-30s %8.1f  %s" % (topic, hz, _detail(topic, last_msg.get(topic), cfg)))

    if counts[cfg.topic("clock")] == 0:
        print("\n  /clock is silent -> the simulation is not playing.")
        print("  ROS 2 publishers in Isaac Sim only run while the timeline is playing.")

    node.destroy_node()
    rclpy.shutdown()
    return 0


def _detail(topic: str, msg, cfg) -> str:
    if msg is None:
        return "-"
    if topic == cfg.topic("scan"):
        # A no-return beam is not always inf: the SICK profile reports -1, and
        # other scanners use 0. Trusting range_min/range_max is the portable
        # way to tell a real measurement from a "nothing there" marker.
        hits = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        detail = "closest %.2f m" % min(hits) if hits else "no returns"
        return "%d beams, %d hits, %s" % (len(msg.ranges), len(hits), detail)
    if topic == cfg.topic("odom"):
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        return "pos (%.2f, %.2f)  v=%.2f m/s" % (p.x, p.y, v.x)
    if topic in (cfg.topic("rgb"), cfg.topic("depth")):
        return "%dx%d %s" % (msg.width, msg.height, msg.encoding)
    if topic == cfg.topic("camera_info"):
        return "%dx%d  fx=%.1f" % (msg.width, msg.height, msg.k[0])
    if topic == cfg.topic("joint_states"):
        if not msg.velocity:
            return "%d joints, no velocity field" % len(msg.name)
        return ", ".join("%s=%.2f rad/s" % (n, v) for n, v in zip(msg.name, msg.velocity))
    if topic == cfg.topic("clock"):
        return "sim t=%.2fs" % (msg.clock.sec + msg.clock.nanosec * 1e-9)
    return "-"


if __name__ == "__main__":
    raise SystemExit(main())
