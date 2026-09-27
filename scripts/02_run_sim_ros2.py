"""Lesson 3-4: run the warehouse simulation with the ROS 2 bridge live.

    C:\\isaacsim\\python.bat scripts\\02_run_sim_ros2.py

Leave it running, then in a second terminal drive the robot:

    C:\\isaacsim\\python.bat ros2_client\\topic_monitor.py
    C:\\isaacsim\\python.bat ros2_client\\patrol.py

Useful flags:
    --duration 30      stop after N seconds (0 = run until closed)
    --headless         no window, for CI or a slow GPU
    --self-test        check every topic is advertised, then drive the robot
                       and confirm odometry and /scan respond. Writes
                       _output/self_test_report.txt; exits non-zero on failure.

/cmd_vel is handled by warehouse_amr/base_controller.py, not by the graph: it
stops the robot when commands stop arriving (CMD_VEL_TIMEOUT in .env).
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

# Kit writes its own log straight to the console. Without line buffering our
# prints get held in a block buffer and interleave unhelpfully -- or vanish
# entirely if the process dies.
try:
    sys.stdout.reconfigure(line_buffering=True)
    sys.stderr.reconfigure(line_buffering=True)
except AttributeError:  # pragma: no cover
    pass

from warehouse_amr import app                      # noqa: E402
from warehouse_amr.config import describe, load_config  # noqa: E402


def parse_args():
    p = argparse.ArgumentParser(description="Warehouse AMR + ROS 2 bridge")
    p.add_argument("--duration", type=float, default=0.0,
                   help="seconds to run; 0 means until the window is closed")
    p.add_argument("--headless", action="store_true", help="force headless")
    p.add_argument("--self-test", action="store_true",
                   help="verify topics are advertised, print a report, and exit")
    return p.parse_args()


def main():
    args = parse_args()
    cfg = load_config()
    print(describe(cfg))

    # Kit boots here; every omni/pxr/isaacsim import must come after this line.
    headless = True if (args.headless or args.self_test) else None
    simulation_app = app.launch(cfg, headless=headless)

    app.enable_required_extensions(with_ros2=True)
    simulation_app.update()
    bridge_ok = app.verify_ros2_bridge()

    import omni.usd  # noqa: E402
    from isaacsim.core.api import SimulationContext  # noqa: E402

    from warehouse_amr.base_controller import BaseController  # noqa: E402
    from warehouse_amr.ros_graph import build_ros2_graph, expected_topics  # noqa: E402
    from warehouse_amr.scene.robot import build_amr  # noqa: E402
    from warehouse_amr.scene.warehouse import build_warehouse  # noqa: E402
    from warehouse_amr.sensors import setup_sensors  # noqa: E402

    omni.usd.get_context().new_stage()
    stage = omni.usd.get_context().get_stage()

    print("[run] building warehouse ...")
    build_warehouse(stage, cfg)

    print("[run] building robot ...")
    robot = build_amr(stage, cfg)
    simulation_app.update()

    print("[run] creating sensors ...")
    sensors = setup_sensors(cfg, robot)
    simulation_app.update()

    print("[run] wiring ROS 2 graph ...")
    # The graph publishes state; the wheels belong to BaseController, which
    # adds the command timeout the graph's /cmd_vel chain does not have.
    build_ros2_graph(cfg, robot, sensors, drive_from_graph=False)
    simulation_app.update()

    # physics_dt must agree with the PhysicsScene's timeStepsPerSecond, which
    # warehouse.py set from the same config value.
    simulation_context = SimulationContext(
        physics_dt=1.0 / cfg.physics_hz,
        rendering_dt=1.0 / cfg.render_hz,
        stage_units_in_meters=1.0,
    )
    simulation_context.initialize_physics()
    simulation_app.update()

    # ROS 2 publishers only exist while the timeline is playing -- a stopped
    # simulation advertises nothing at all.
    simulation_context.play()
    print("[run] playing. topics:")
    for t in expected_topics(cfg):
        print("        " + t)

    # Simulation time, not wall time -- see BaseController.__init__.
    controller = BaseController(cfg, robot, clock=lambda: simulation_context.current_time)
    controller.start()

    def step():
        """One simulation step with the base controller in the loop."""
        controller.step()
        simulation_context.step(render=True)

    if args.self_test:
        return _self_test(simulation_app, simulation_context, cfg, bridge_ok, controller, step)

    start = time.time()
    try:
        while simulation_app.is_running():
            step()
            if args.duration and (time.time() - start) >= args.duration:
                break
    except KeyboardInterrupt:
        print("\n[run] interrupted")

    controller.shutdown()
    simulation_context.stop()
    app.shutdown(simulation_app, 0)   # does not return


def _self_test(simulation_app, simulation_context, cfg, bridge_ok: bool, controller, step):
    """Spin the sim, then ask ROS 2 what it can actually see."""
    # BaseController.start() already imported and initialised the bundled
    # rclpy in this process; the test node shares that context.
    rclpy = controller.rclpy

    from warehouse_amr.ros_graph import expected_topics

    warmup_frames = 240
    print("[self-test] stepping %d frames to warm up renderers ..." % warmup_frames)
    for _ in range(warmup_frames):
        step()

    node = rclpy.create_node("warehouse_amr_self_test")

    # Discovery is not instantaneous; keep stepping the sim while we wait so
    # the publishers stay alive and keep announcing themselves.
    seen = {}
    deadline = time.time() + 15.0
    wanted = set(expected_topics(cfg))
    while time.time() < deadline:
        for _ in range(10):
            step()
        rclpy.spin_once(node, timeout_sec=0.05)
        for name, types in node.get_topic_names_and_types():
            seen[name] = types
        if wanted.issubset(seen.keys()):
            break

    missing = sorted(wanted - set(seen.keys()))
    report = ["---- phase 1: topic discovery ----"]
    for name in sorted(seen):
        report.append("   %-28s %s" % (name, ",".join(seen[name])))
    report.append("")
    report.append("bridge enabled : %s" % bridge_ok)
    report.append("expected       : %d" % len(wanted))
    report.append("missing        : %s" % (", ".join(missing) if missing else "none"))

    # ---- phase 2: does the loop actually close? --------------------------
    # Advertised topics prove the graph was built. They do not prove a Twist
    # reaches PhysX or that the lidar sees anything, so drive the robot and
    # watch odometry and /scan respond.
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan

    state = {"odom": None, "scan": None}
    node.create_subscription(Odometry, cfg.topic("odom"), lambda m: state.update(odom=m), 10)
    node.create_subscription(LaserScan, cfg.topic("scan"), lambda m: state.update(scan=m), 10)
    pub = node.create_publisher(Twist, cfg.topic("cmd_vel"), 10)

    drive_ok, drive_lines = _drive_test(rclpy, node, cfg, simulation_context, step, state, pub)
    report.append("")
    report.append("---- phase 2: closed-loop drive ----")
    report.extend(drive_lines)

    passed = (not missing) and bridge_ok and drive_ok
    report.append("")
    report.append("RESULT         : %s" % ("PASS" if passed else "FAIL"))

    node.destroy_node()
    controller.shutdown()

    text = "\n".join(report)
    print("\n[self-test] " + text.replace("\n", "\n[self-test] "))
    # Also on disk, so the outcome survives a crash during shutdown.
    (cfg.output_dir / "self_test_report.txt").write_text(text, encoding="utf-8")

    simulation_context.stop()
    app.shutdown(simulation_app, 0 if passed else 1)   # does not return


def _drive_test(rclpy, node, cfg, simulation_context, step, state, pub):
    """Publish /cmd_vel, then check odometry and lidar respond.

    This is the test that matters. It exercises the full round trip:
    Twist -> DDS -> BaseController (watchdog, differential drive) ->
    articulation targets -> PhysX -> IsaacComputeOdometry -> Odometry -> DDS.
    """
    from geometry_msgs.msg import Twist

    # Let subscriptions match publishers before measuring anything.
    for _ in range(60):
        step()
        rclpy.spin_once(node, timeout_sec=0.01)

    start_odom = state["odom"]
    start_x = start_odom.pose.pose.position.x if start_odom else None

    # Measure elapsed *simulation* time rather than counting steps. One
    # step(render=True) call advances physics however many times it needs to
    # keep physics_dt while hitting rendering_dt -- two, at 60 Hz physics and
    # 30 Hz rendering -- so step count is not a reliable clock.
    speed = 0.5
    t_start = simulation_context.current_time

    cmd = Twist()
    cmd.linear.x = speed
    for _ in range(360):
        pub.publish(cmd)
        step()
        rclpy.spin_once(node, timeout_sec=0.005)

    t_end = simulation_context.current_time

    cmd.linear.x = 0.0
    for _ in range(30):
        pub.publish(cmd)
        step()
        rclpy.spin_once(node, timeout_sec=0.005)

    end_odom = state["odom"]
    scan = state["scan"]
    lines = []
    ok = True

    if start_odom is None or end_odom is None:
        lines.append("   odom          : NO MESSAGES RECEIVED")
        ok = False
    else:
        end_x = end_odom.pose.pose.position.x
        end_y = end_odom.pose.pose.position.y
        travelled = end_x - (start_x or 0.0)
        sim_seconds = t_end - t_start
        expected = speed * sim_seconds
        ratio = (travelled / expected) if expected > 0 else 0.0
        lines.append("   commanded     : %.2f m/s for %.2f s of simulated time" % (speed, sim_seconds))
        lines.append("   expected      : %.3f m" % expected)
        lines.append("   travelled     : %.3f m  (%.0f%% of expected)" % (travelled, ratio * 100))
        lines.append("   lateral drift : %.3f m" % end_y)
        if travelled < 0.5:
            lines.append("   -> FAIL: robot did not move; cmd_vel is not reaching PhysX")
            ok = False
        elif abs(end_y) > 1.0:
            lines.append("   -> FAIL: drifted %.2f m sideways on a straight command" % end_y)
            ok = False
        elif not (0.80 <= ratio <= 1.20):
            # Outside this band the wheels are slipping, the wheel radius in
            # .env disagrees with the robot that was built, or something is
            # dragging on the floor.
            lines.append("   -> FAIL: distance is %.0f%% of commanded -- check WHEEL_RADIUS"
                         % (ratio * 100))
            ok = False
        else:
            lines.append("   -> OK: commanded motion produced matching odometry")

    if scan is None:
        lines.append("   scan          : NO MESSAGES RECEIVED")
        ok = False
    else:
        finite = [r for r in scan.ranges
                  if scan.range_min < r < scan.range_max and r == r and r != float("inf")]
        lines.append("   scan          : %d beams, %d finite, range [%.2f, %.2f]"
                     % (len(scan.ranges), len(finite), scan.range_min, scan.range_max))
        lines.append("   scan frame_id : %s" % scan.header.frame_id)
        if not finite:
            lines.append("   -> FAIL: lidar returned no hits; it sees nothing at all")
            ok = False
        else:
            lines.append("   -> OK: closest return %.2f m" % min(finite))

    return ok, lines


if __name__ == "__main__":
    sys.exit(main())
