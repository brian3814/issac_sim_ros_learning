"""Lesson 4c: a closed sense-think-act loop over ROS 2.

    C:\\isaacsim\\python.bat ros2_client\\patrol.py

The robot patrols the aisle and stops for whatever is in front of it:

    /scan  (lidar)  --.
                       >--  this node  --> /cmd_vel  --> Isaac Sim --> PhysX
    /odom  (pose)   --'

That is the entire architecture of the project in one picture, and the reason
the simulator is worth the trouble: this node cannot tell it is talking to a
simulation. Point it at a real AMR publishing the same topics and it drives
that instead, unchanged.

State machine::

    DRIVE  --obstacle within stop distance--> BACKUP
    DRIVE  --leg length travelled----------> TURN
    BACKUP --cleared--------------------->   TURN
    TURN   --90 degrees turned------------>  DRIVE
"""

from __future__ import annotations

import argparse
import math
import time

from _bootstrap import init_ros2

STOP_DISTANCE = 0.9        # m -- start avoiding
CLEAR_DISTANCE = 1.4       # m -- considered clear again
LEG_LENGTH = 4.0           # m -- straight run before turning
FRONT_ARC_DEG = 40.0       # only beams within +/- this angle count as "ahead"


def yaw_from_quaternion(q) -> float:
    """Z-axis rotation from a ROS quaternion (the only component we need)."""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def angle_diff(a: float, b: float) -> float:
    """Shortest signed difference a-b, wrapped to [-pi, pi]."""
    return (a - b + math.pi) % (2 * math.pi) - math.pi


class Patrol:
    def __init__(self, node, cfg, pub):
        self.node = node
        self.cfg = cfg
        self.pub = pub
        self.state = "DRIVE"
        self.front_range = float("inf")
        self.pose = (0.0, 0.0, 0.0)   # (x, y, yaw)
        self.have_pose = False
        self.leg_start = (0.0, 0.0, 0.0)
        self.turn_target = 0.0
        self.legs_done = 0

    # ---- sensing ----------------------------------------------------------
    def on_scan(self, msg):
        """Reduce the scan to one number: how far away is the nearest thing ahead.

        A LaserScan is an array of ranges starting at ``angle_min`` and
        stepping by ``angle_increment``; only the beams inside a narrow arc in
        front of the robot matter for a stop decision.
        """
        arc = math.radians(FRONT_ARC_DEG)
        closest = float("inf")
        for i, r in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            if -arc <= angle <= arc:
                # Filter the readings a real scanner also produces: zeros for
                # no-return, infinities beyond range, and anything out of spec.
                if msg.range_min < r < msg.range_max and not math.isinf(r):
                    closest = min(closest, r)
        self.front_range = closest

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.pose = (p.x, p.y, yaw_from_quaternion(msg.pose.pose.orientation))

    # ---- acting -----------------------------------------------------------
    def publish(self, linear: float, angular: float):
        from geometry_msgs.msg import Twist

        t = Twist()
        t.linear.x = float(linear)
        t.angular.z = float(angular)
        self.pub.publish(t)

    def distance_from(self, origin) -> float:
        if self.pose is None or origin is None:
            return 0.0
        return math.hypot(self.pose[0] - origin[0], self.pose[1] - origin[1])

    # ---- thinking ---------------------------------------------------------
    def tick(self):
        if self.pose is None:
            self.publish(0.0, 0.0)          # no odometry yet: do not move blind
            return "waiting for /odom"

        if self.leg_start is None:
            self.leg_start = self.pose

        if self.state == "DRIVE":
            if self.front_range < STOP_DISTANCE:
                self.state = "BACKUP"
            elif self.distance_from(self.leg_start) >= LEG_LENGTH:
                self._begin_turn()
            else:
                # Ease off as an obstacle approaches instead of driving flat
                # out into a stop -- gentler on the wheels and easier to watch.
                margin = min(1.0, max(0.0, (self.front_range - STOP_DISTANCE) / 2.0))
                self.publish(self.cfg.max_linear_speed * (0.35 + 0.65 * margin), 0.0)

        elif self.state == "BACKUP":
            if self.front_range > CLEAR_DISTANCE:
                self._begin_turn()
            else:
                self.publish(-0.25, 0.0)

        elif self.state == "TURN":
            if self.turn_target is None:
                self._begin_turn()
            err = angle_diff(float(self.turn_target), self.pose[2])
            if abs(err) < math.radians(6.0):
                self.state = "DRIVE"
                self.leg_start = self.pose
                self.legs_done += 1
            else:
                # Proportional control, clamped: turn fast when far off,
                # slow down as it converges so it does not oscillate.
                w = max(-self.cfg.max_angular_speed,
                        min(self.cfg.max_angular_speed, 1.6 * err))
                self.publish(0.0, w)

        return "%-7s front=%5.2fm  pose=(%6.2f,%6.2f) yaw=%6.1f  legs=%d" % (
            self.state,
            self.front_range if self.front_range != float("inf") else -1.0,
            self.pose[0], self.pose[1], math.degrees(self.pose[2]), self.legs_done,
        )

    def _begin_turn(self):
        if self.pose is None:
            return
        self.state = "TURN"
        self.turn_target = angle_diff(self.pose[2] + math.pi / 2.0, 0.0)


def main() -> int:
    ap = argparse.ArgumentParser(description="Autonomous aisle patrol with lidar stopping")
    ap.add_argument("--duration", type=float, default=120.0, help="seconds to patrol")
    args = ap.parse_args()

    rclpy, node, cfg = init_ros2("warehouse_amr_patrol")
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan

    pub = node.create_publisher(Twist, cfg.topic("cmd_vel"), 10)
    patrol = Patrol(node, cfg, pub)
    node.create_subscription(LaserScan, cfg.topic("scan"), patrol.on_scan, 10)
    node.create_subscription(Odometry, cfg.topic("odom"), patrol.on_odom, 10)

    print("patrolling for %.0fs -- Ctrl+C to stop\n" % args.duration)
    start = time.time()
    try:
        while time.time() - start < args.duration:
            rclpy.spin_once(node, timeout_sec=0.05)
            status = patrol.tick()
            print("\r  " + status + "   ", end="", flush=True)
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        patrol.publish(0.0, 0.0)
        time.sleep(0.2)
        node.destroy_node()
        rclpy.shutdown()
        print("\nstopped.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
