"""Make Isaac Sim's bundled ROS 2 importable in a plain Python process.

Isaac Sim ships a complete minimal ROS 2 (rclpy, the common message packages,
tf2, and even the ``ros2`` CLI) under

    <ISAACSIM_ROOT>/exts/isaacsim.ros2.bridge/<distro>/

so this project needs **no ROS 2 installation at all** -- and, just as
importantly, installs nothing on your machine. Both sides of the wire use the
exact same libraries, which removes a whole class of version-mismatch bugs.

All of the setup below is process-local: ``sys.path`` and ``os.environ`` are
per-process in Python, so running this never touches your shell or your system.
"""

from __future__ import annotations

import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from warehouse_amr.config import apply_ros2_process_env, describe, load_config  # noqa: E402


def init_ros2(node_name: str, quiet: bool = False):
    """Configure paths, start rclpy on the configured domain, return a node.

    Returns ``(rclpy_module, node, cfg)``.
    """
    cfg = load_config()
    if not quiet:
        print(describe(cfg))

    apply_ros2_process_env(cfg)

    rclpy_dir = cfg.ros2_rclpy_dir
    if not rclpy_dir.exists():
        raise FileNotFoundError(
            "Bundled rclpy not found at " + str(rclpy_dir) + "\n"
            "Check ISAACSIM_ROOT / ROS_DISTRO in .env"
        )
    if str(rclpy_dir) not in sys.path:
        sys.path.insert(0, str(rclpy_dir))

    import rclpy

    # Passing domain_id explicitly rather than relying on ROS_DOMAIN_ID keeps
    # .env as the single source of truth for which DDS domain we join.
    rclpy.init(domain_id=cfg.ros_domain_id)
    node = rclpy.create_node(node_name)
    if not quiet:
        print("[ros2] node '%s' up on domain %d\n" % (node_name, cfg.ros_domain_id))
    return rclpy, node, cfg
