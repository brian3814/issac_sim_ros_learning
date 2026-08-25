"""The real ``ros2`` command line, using Isaac Sim's bundled CLI.

    C:\\isaacsim\\python.bat ros2_client\\ros2cli.py topic list
    C:\\isaacsim\\python.bat ros2_client\\ros2cli.py topic hz /scan
    C:\\isaacsim\\python.bat ros2_client\\ros2cli.py topic echo /odom --once
    C:\\isaacsim\\python.bat ros2_client\\ros2cli.py node list

Isaac Sim bundles ros2cli along with rclpy, so the standard tooling is
available with nothing installed. The domain comes from ``.env`` like
everything else in this project.

Two caveats worth knowing:

* ``ros2 topic echo`` on an image topic will flood your terminal. Use
  ``--once``, or ``ros2 topic hz`` instead.
* The CLI starts a background daemon for faster discovery. If listings ever
  look stale, ``ros2cli.py daemon stop`` clears it.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from warehouse_amr.config import apply_ros2_process_env, load_config  # noqa: E402


def main() -> int:
    cfg = load_config()
    apply_ros2_process_env(cfg)

    rclpy_dir = str(cfg.ros2_rclpy_dir)
    if rclpy_dir not in sys.path:
        sys.path.insert(0, rclpy_dir)
    # ros2cli discovers its verbs through entry points, which are read from the
    # .egg-info directories sitting beside the packages -- so PYTHONPATH has to
    # carry the same directory into the daemon subprocess it spawns.
    os.environ["PYTHONPATH"] = rclpy_dir + os.pathsep + os.environ.get("PYTHONPATH", "")

    # ros2cli.daemon imports netifaces, which Isaac Sim does not bundle. Supply
    # a stand-in before ros2cli is imported; see _netifaces_shim for why.
    from _netifaces_shim import install as install_netifaces

    install_netifaces()

    if len(sys.argv) < 2:
        print(__doc__)
        return 1

    from ros2cli.cli import main as ros2_main

    return ros2_main(argv=sys.argv[1:]) or 0


if __name__ == "__main__":
    raise SystemExit(main())
