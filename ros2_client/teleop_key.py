"""Lesson 4b: drive the robot by hand over /cmd_vel.

    C:\\isaacsim\\python.bat ros2_client\\teleop_key.py

    W / S   forward / back        A / D   turn left / right
    SPACE   stop                  Q       quit

The entire "robot interface" is one publisher of ``geometry_msgs/Twist``. The
simulator's OmniGraph turns that into wheel velocities; a real AMR's firmware
would do the same thing. Neither side knows or cares which is on the other end,
which is the whole reason sim-to-real transfer works at the ROS 2 layer.
"""

from __future__ import annotations

import sys
import time

from _bootstrap import init_ros2

try:
    import msvcrt  # Windows: read single keypresses without Enter
except ImportError:  # pragma: no cover - non-Windows fallback
    msvcrt = None

STEP_LINEAR = 0.15
STEP_ANGULAR = 0.3


def read_key():
    if msvcrt is None:
        return sys.stdin.read(1)
    if msvcrt.kbhit():
        return msvcrt.getch().decode("utf-8", errors="ignore").lower()
    return None


def main() -> int:
    rclpy, node, cfg = init_ros2("warehouse_amr_teleop")
    from geometry_msgs.msg import Twist

    pub = node.create_publisher(Twist, cfg.topic("cmd_vel"), 10)
    twist = Twist()

    print("  W / S   forward / back        A / D   turn left / right")
    print("  SPACE   stop                  Q       quit")
    print("\npublishing to %s  (max %.1f m/s, %.1f rad/s)\n"
          % (cfg.topic("cmd_vel"), cfg.max_linear_speed, cfg.max_angular_speed))

    running = True
    try:
        while running:
            key = read_key()
            if key == "w":
                twist.linear.x = min(twist.linear.x + STEP_LINEAR, cfg.max_linear_speed)
            elif key == "s":
                twist.linear.x = max(twist.linear.x - STEP_LINEAR, -cfg.max_linear_speed)
            elif key == "a":
                twist.angular.z = min(twist.angular.z + STEP_ANGULAR, cfg.max_angular_speed)
            elif key == "d":
                twist.angular.z = max(twist.angular.z - STEP_ANGULAR, -cfg.max_angular_speed)
            elif key == " ":
                twist.linear.x = 0.0
                twist.angular.z = 0.0
            elif key == "q":
                running = False

            # Republish continuously rather than only on keypress: a controller
            # that stops hearing commands should be able to time out and stop,
            # and every real robot's safety layer expects a steady stream.
            pub.publish(twist)
            print("\r  v = %+5.2f m/s   w = %+5.2f rad/s   " % (twist.linear.x, twist.angular.z),
                  end="", flush=True)
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)
        time.sleep(0.2)
        node.destroy_node()
        rclpy.shutdown()
        print("\nstopped.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
