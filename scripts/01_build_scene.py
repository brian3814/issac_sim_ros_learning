"""Lesson 1-2: build the warehouse and the robot, no ROS 2 involved.

    C:\\isaacsim\\python.bat scripts\\01_build_scene.py --save

Start here. It answers "what is a robot, on a USD stage?" before any
networking is in the picture, and it drops the robot with a fixed wheel
command so you can watch the physics actually work.

    --save        write the assembled stage to _output/warehouse_amr.usd
    --seconds N   how long to simulate (default 8)
    --spin        drive the wheels open-loop so the robot moves
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

try:
    sys.stdout.reconfigure(line_buffering=True)
except AttributeError:  # pragma: no cover
    pass

from warehouse_amr import app                          # noqa: E402
from warehouse_amr.config import describe, load_config  # noqa: E402


def parse_args():
    p = argparse.ArgumentParser(description="Build the AMR warehouse scene")
    p.add_argument("--save", action="store_true", help="save the stage to _output/")
    p.add_argument("--seconds", type=float, default=8.0, help="seconds to simulate")
    p.add_argument("--spin", action="store_true", help="drive the wheels open-loop")
    p.add_argument("--headless", action="store_true")
    return p.parse_args()


def main():
    args = parse_args()
    cfg = load_config()
    print(describe(cfg))

    simulation_app = app.launch(cfg, headless=True if args.headless else None)
    # No ROS 2 in this lesson -- the bridge is deliberately left off so it is
    # obvious that the scene and physics stand on their own.
    app.enable_required_extensions(with_ros2=False)
    simulation_app.update()

    import omni.usd  # noqa: E402
    from isaacsim.core.api import SimulationContext  # noqa: E402

    from warehouse_amr.scene.robot import build_amr  # noqa: E402
    from warehouse_amr.scene.warehouse import build_warehouse  # noqa: E402

    omni.usd.get_context().new_stage()
    stage = omni.usd.get_context().get_stage()

    build_warehouse(stage, cfg)
    robot = build_amr(stage, cfg)
    simulation_app.update()

    print("\n[scene] robot prim tree:")
    for path in (robot.root, robot.base_link, robot.chassis, robot.left_wheel,
                 robot.right_wheel, robot.left_joint, robot.right_joint, robot.camera):
        prim = stage.GetPrimAtPath(path)
        print("   %-42s %s" % (path, prim.GetTypeName() if prim.IsValid() else "MISSING"))

    if args.save:
        out = cfg.output_dir / "warehouse_amr.usd"
        stage.Export(str(out))
        print("\n[scene] saved -> %s" % out)
        print("[scene] open it with: C:\\isaacsim\\isaac-sim.bat  (File > Open)")

    sim = SimulationContext(
        physics_dt=1.0 / cfg.physics_hz,
        rendering_dt=1.0 / cfg.render_hz,
        stage_units_in_meters=1.0,
    )
    sim.initialize_physics()
    sim.play()

    # The articulation view only resolves once physics has started, which is
    # why this lives below play() rather than beside the scene build.
    from isaacsim.core.prims import Articulation

    articulation = Articulation(prim_paths_expr=robot.root, name="amr")
    articulation.initialize()
    print("\n[scene] joints found: %s" % list(articulation.joint_names))

    import numpy as np  # noqa: E402

    print("\n[scene] simulating %.1fs ..." % args.seconds)
    print("   %6s %24s %14s" % ("sim t", "base_link (x, y, z)", "wheels rad/s"))

    # Drive the loop off simulated time, not a step count. One step(render=True)
    # advances physics as many times as it needs to hit rendering_dt -- twice,
    # at 60 Hz physics and 30 Hz rendering -- so counting steps would run for
    # double the requested duration.
    t0 = sim.current_time
    next_report = 0.0
    while sim.current_time - t0 < args.seconds:
        if args.spin:
            # Straight ahead at ~0.5 m/s: both wheels at v / r rad/s.
            w = 0.5 / cfg.wheel_radius
            articulation.set_joint_velocity_targets(np.array([[w, w]], dtype=np.float32))
        sim.step(render=True)
        elapsed = sim.current_time - t0
        if elapsed >= next_report:
            _report(articulation, elapsed)
            next_report += 2.0

    _report(articulation, sim.current_time - t0)
    sim.stop()
    app.shutdown(simulation_app, 0)   # does not return


def _report(articulation, t: float) -> None:
    """Report the robot's pose from PhysX.

    Deliberately *not* read off the USD stage. Isaac Sim writes simulation
    results into Fabric for speed, and the USD attributes can lag behind or
    never update at all -- reading them mid-run gives numbers that look
    plausible and are wrong. The tensor API below is the authority.
    """
    pos, _ = articulation.get_world_poses()
    vel = articulation.get_joint_velocities()[0]
    print("   %5.1fs  (%7.2f, %7.2f, %6.3f)   [%5.2f %5.2f]"
          % (t, pos[0][0], pos[0][1], pos[0][2], vel[0], vel[1]))


if __name__ == "__main__":
    # main() ends in app.shutdown(), which exits the process itself; this is
    # only reached if that ever changes.
    raise SystemExit(main())
