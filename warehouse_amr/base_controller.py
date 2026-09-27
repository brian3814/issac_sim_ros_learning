"""The robot's base controller: /cmd_vel in, wheel velocity targets out.

With ``run\\sim.cmd`` this replaces the command half of the OmniGraph bridge
(``ROS2SubscribeTwist -> DifferentialController -> ArticulationController``).
It does the same job -- the differential-drive maths is two lines -- but adds
the one thing the graph cannot express cleanly: a command timeout. See
``cmd_watchdog.py`` for why that matters.

It is the Python counterpart of the base controller on a real AMR (for
example ros2_control's ``diff_drive_controller``, whose ``cmd_vel_timeout``
defaults to 0.5 s), and it is deliberately the *only* thing that writes wheel
targets: two writers would simply overwrite each other every frame.

Call order, all from the Kit process after ``play()``::

    controller = BaseController(cfg, robot, clock=lambda: sim.current_time)
    controller.start()
    while running:
        controller.step()          # read /cmd_vel, apply watchdog, write targets
        sim.step(render=True)
    controller.shutdown()
"""

from __future__ import annotations

import sys

from .cmd_watchdog import CmdVelWatchdog
from .scene.robot import LEFT_WHEEL_JOINT, RIGHT_WHEEL_JOINT


def import_rclpy(cfg):
    """Import Isaac Sim's bundled rclpy inside the Kit process and init it once.

    ``app.launch()`` already put the bridge's native libraries on PATH; this
    only adds the Python package directory. Safe to call more than once.
    """
    if str(cfg.ros2_rclpy_dir) not in sys.path:
        sys.path.insert(0, str(cfg.ros2_rclpy_dir))
    import rclpy

    if not rclpy.ok():
        rclpy.init(domain_id=cfg.ros_domain_id)
    return rclpy


class BaseController:
    def __init__(self, cfg, robot, clock):
        """``clock`` returns the current *simulation* time in seconds.

        Simulation time, not wall time: the robot only moves when the
        simulation advances, so a frame that takes two seconds to render
        (shader compilation, a slow GPU) is one short step for the robot and
        must not be mistaken for two seconds of silence.
        """
        self.cfg = cfg
        self.robot = robot
        self.clock = clock
        self.watchdog = CmdVelWatchdog(
            timeout=cfg.cmd_vel_timeout,
            max_linear=cfg.max_linear_speed,
            max_angular=cfg.max_angular_speed,
        )
        self.rclpy = None
        self.node = None
        self._executor = None
        self._art = None
        self._targets = None
        self._left = self._right = None

    def start(self) -> None:
        """Create the subscriber and the articulation handle. Call after ``play()``."""
        import numpy as np
        from isaacsim.core.prims import Articulation

        rclpy = self.rclpy = import_rclpy(self.cfg)
        from geometry_msgs.msg import Twist
        from rclpy.executors import SingleThreadedExecutor

        self.node = rclpy.create_node("warehouse_amr_base_controller")
        # Depth 1: only the newest command matters. A deeper queue lets old
        # commands pile up whenever the loop runs slower than the publisher,
        # and the robot would then act on increasingly stale ones.
        self.node.create_subscription(Twist, self.cfg.topic("cmd_vel"), self._on_twist, 1)
        # A private executor, so spinning this node never services anyone
        # else's callbacks (the self-test runs its own node in this process).
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self.node)

        # Only valid once physics is running, hence start() rather than __init__.
        self._art = Articulation(prim_paths_expr=self.robot.root, name="amr_base_controller")
        self._art.initialize()
        names = list(self._art.joint_names)
        missing = [j for j in (LEFT_WHEEL_JOINT, RIGHT_WHEEL_JOINT) if j not in names]
        if missing:
            raise RuntimeError("wheel joints %s not found in articulation joints %s" % (missing, names))
        # Look the wheels up by name rather than assuming an order: the
        # articulation's joint order is PhysX's choice, not ours.
        self._left = names.index(LEFT_WHEEL_JOINT)
        self._right = names.index(RIGHT_WHEEL_JOINT)
        self._targets = np.zeros((1, len(names)), dtype=np.float32)
        print("[base] listening on %s  (cmd_vel timeout %.2fs)"
              % (self.cfg.topic("cmd_vel"), self.cfg.cmd_vel_timeout))

    def _on_twist(self, msg) -> None:
        self.watchdog.on_command(msg.linear.x, msg.angular.z, self.clock())

    def step(self) -> "tuple[float, float]":
        """Service /cmd_vel once and write wheel targets. Call once per sim step."""
        # One spin handles at most one callback; with a depth-1 queue that is
        # always the newest message.
        self._executor.spin_once(timeout_sec=0.0)
        v, w = self.watchdog.output(self.clock())
        half = self.cfg.wheel_base / 2.0
        self._targets[0, self._left] = (v - w * half) / self.cfg.wheel_radius
        self._targets[0, self._right] = (v + w * half) / self.cfg.wheel_radius
        # Written every step, zeros included: PhysX keeps the previous target
        # until something replaces it, so "not writing" would not stop anything.
        self._art.set_joint_velocity_targets(self._targets)
        return v, w

    def shutdown(self) -> None:
        if self._executor is not None:
            self._executor.shutdown()
        if self.node is not None:
            self.node.destroy_node()
        if self.rclpy is not None and self.rclpy.ok():
            self.rclpy.shutdown()
