"""Isaac Sim application bootstrap.

Ordering rules that this module exists to enforce, because getting them wrong
produces confusing failures rather than clear errors:

1. ``SimulationApp(...)`` must be constructed **before** any ``omni.*``,
   ``pxr.*`` or ``isaacsim.*`` import. It is what boots the Kit runtime that
   provides those modules in the first place.
2. The ROS 2 library paths must be applied **before** the bridge extension is
   enabled, or the extension loads without its DDS backend and every publisher
   silently does nothing.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Optional

PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))


def launch(cfg, headless: Optional[bool] = None, extra_config: Optional[dict] = None):
    """Start Kit and return the ``SimulationApp``.

    Call this before importing anything from ``omni`` or ``pxr``.
    """
    from warehouse_amr.config import apply_ros2_process_env

    # Step 2 above -- done first because it is pure os.environ bookkeeping and
    # must be in place before Kit loads any extension.
    apply_ros2_process_env(cfg)

    from isaacsim import SimulationApp  # noqa: E402  (import must follow env setup)

    launch_config = {
        "headless": cfg.headless if headless is None else headless,
        "width": 1280,
        "height": 720,
    }
    if extra_config:
        launch_config.update(extra_config)
    return SimulationApp(launch_config)


def enable_required_extensions(with_ros2: bool = True) -> None:
    """Turn on the extensions this project depends on.

    Isaac Sim ships hundreds of extensions and enables a subset by default;
    the ROS 2 bridge and the RTX sensor stack are not always among them, so we
    ask explicitly rather than hoping.
    """
    from isaacsim.core.utils.extensions import enable_extension

    for ext in ("isaacsim.sensors.rtx", "omni.replicator.core"):
        enable_extension(ext)
    if with_ros2:
        enable_extension("isaacsim.ros2.bridge")


def shutdown(simulation_app, code: int = 0):
    """Close Kit and make sure the process actually exits.

    ``SimulationApp.close()`` occasionally hangs on Windows after everything
    useful is done -- the script's work is finished and its output written, but
    the process lingers holding several GB, which reads as "it froze". Since
    nothing remains to flush by this point, force the exit rather than leave an
    orphan behind.

    Anything that must reach disk (reports, Replicator's
    ``wait_until_complete()``) has to happen *before* this call.
    """
    import os

    try:
        simulation_app.close()
    except Exception as exc:  # pragma: no cover
        print("[app] close() raised: %s" % exc)
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(code)


def verify_ros2_bridge() -> bool:
    """Confirm the bridge extension actually came up.

    A disabled bridge is the single most common reason for "Isaac is running
    but ``ros2 topic list`` shows nothing", and it is worth failing loudly.
    """
    import omni.kit.app

    manager = omni.kit.app.get_app().get_extension_manager()
    enabled = manager.is_extension_enabled("isaacsim.ros2.bridge")
    if not enabled:
        print("[app] WARNING: isaacsim.ros2.bridge is NOT enabled -- no topics will appear.")
    return enabled
