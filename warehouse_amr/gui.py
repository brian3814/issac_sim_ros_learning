"""Build this project inside a *already running* Isaac Sim (the GUI app).

The standalone scripts in ``scripts/`` start Kit themselves via
``SimulationApp``. In the GUI, Kit is already running, and creating a second
``SimulationApp`` is not allowed -- so the GUI needs its own entry point.

Two other things differ from the standalone flow:

* **You cannot call ``simulation_app.update()``.** The GUI owns the update
  loop. Steps that need a frame to settle (an extension finishing startup, a
  new stage being ready, an RTX sensor being created) must ``await`` the app's
  next update instead, which is why the real work below is a coroutine.
* **You do not create a ``SimulationContext``.** The GUI's own timeline drives
  physics, and the PhysicsScene authored by ``build_warehouse`` already pins
  the solver rate. Press ▶ to start; the ROS 2 publishers appear then and
  disappear on ■.

Usage from Window ▸ Script Editor -- see ``docs/04_gui.md`` for the snippet.
"""

from __future__ import annotations

import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))


async def build_async(with_ros2: bool = True, new_stage: bool = True, verbose: bool = True):
    """Assemble the warehouse, robot, sensors and ROS 2 graph in the running app.

    Returns ``(cfg, robot, sensors)``; ``sensors`` is None when
    ``with_ros2`` is False.

    ``new_stage=True`` replaces whatever is currently open, which is almost
    always what you want here but will discard unsaved work.
    """
    import omni.kit.app
    import omni.usd

    from warehouse_amr.app import enable_required_extensions
    from warehouse_amr.config import apply_ros2_process_env, describe, load_config

    app = omni.kit.app.get_app()

    def log(msg):
        if verbose:
            print("[gui] " + msg)

    cfg = load_config()
    if verbose:
        print(describe(cfg))

    if with_ros2:
        # Harmless if isaac-sim.bat already ran setup_ros_env.bat -- it writes
        # the same values. Doing it here means the bridge also works if Kit was
        # started some other way, and it pins the distro to .env either way.
        apply_ros2_process_env(cfg)

    log("enabling extensions ...")
    enable_required_extensions(with_ros2=with_ros2)
    await app.next_update_async()

    if new_stage:
        log("creating a new stage ...")
        await omni.usd.get_context().new_stage_async()
        await app.next_update_async()

    stage = omni.usd.get_context().get_stage()

    from warehouse_amr.scene.robot import build_amr
    from warehouse_amr.scene.warehouse import build_warehouse

    log("building warehouse ...")
    build_warehouse(stage, cfg)
    await app.next_update_async()

    log("building robot ...")
    robot = build_amr(stage, cfg)
    await app.next_update_async()

    sensors = None
    if with_ros2:
        from warehouse_amr.ros_graph import GRAPH_PATH, build_ros2_graph, expected_topics
        from warehouse_amr.sensors import setup_sensors

        log("creating sensors ...")
        sensors = setup_sensors(cfg, robot)
        await app.next_update_async()

        log("wiring ROS 2 graph ...")
        build_ros2_graph(cfg, robot, sensors)
        await app.next_update_async()

        log("graph is at " + GRAPH_PATH + " -- open Window > Graph Editors >")
        log("  Action Graph and select it to see the nodes.")
        log("press PLAY to start publishing:")
        for t in expected_topics(cfg):
            log("    " + t)
    else:
        log("done -- ROS 2 was skipped (with_ros2=False)")

    return cfg, robot, sensors


def build(with_ros2: bool = True, new_stage: bool = True, verbose: bool = True):
    """Schedule :func:`build_async` on Kit's event loop and return the task.

    This is the one to call from the Script Editor, which cannot ``await``.
    The build runs across the next handful of frames; watch the console for
    ``[gui]`` lines.
    """
    import asyncio

    return asyncio.ensure_future(
        build_async(with_ros2=with_ros2, new_stage=new_stage, verbose=verbose)
    )
