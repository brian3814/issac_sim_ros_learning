"""Project configuration, loaded exclusively from the project-local ``.env``.

Design rule for this project: **the process environment is never consulted for
configuration.** ``os.environ`` is only ever *written* (never read), and only
inside the running process, so nothing here can depend on -- or leak into --
your shell, your user profile, or the machine.

That has a payoff beyond tidiness. Isaac Sim's ROS 2 bridge and the client
scripts both take their DDS domain from this file rather than from a global
``ROS_DOMAIN_ID``, so a stale variable in some terminal can never make the two
sides silently stop seeing each other -- the classic "where did my topics go?"
afternoon.
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
ENV_FILE = PROJECT_ROOT / ".env"
ENV_EXAMPLE = PROJECT_ROOT / ".env.example"


def _parse_env_file(path: Path) -> "dict[str, str]":
    """Minimal KEY=VALUE parser. No interpolation, no shell semantics."""
    values: "dict[str, str]" = {}
    if not path.exists():
        return values
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, _, val = line.partition("=")
        val = val.split(" #", 1)[0].strip()
        if len(val) >= 2 and val[0] == val[-1] and val[0] in "\"'":
            val = val[1:-1]
        values[key.strip()] = val
    return values


class _Env:
    def __init__(self, path: Path):
        self.path = path
        self.values = _parse_env_file(path)

    def str(self, key: str, default: str = "") -> str:
        return self.values.get(key, default)

    def path_(self, key: str, default: str) -> Path:
        p = Path(self.values.get(key, default))
        return p if p.is_absolute() else (PROJECT_ROOT / p).resolve()

    def int(self, key: str, default: int) -> int:
        try:
            return int(self.values.get(key, default))
        except (TypeError, ValueError):
            return default

    def float(self, key: str, default: float) -> float:
        try:
            return float(self.values.get(key, default))
        except (TypeError, ValueError):
            return default

    def bool(self, key: str, default: bool) -> bool:
        raw = self.values.get(key)
        if raw is None:
            return default
        return raw.strip().lower() in ("1", "true", "yes", "on")


@dataclass
class Config:
    # paths
    isaacsim_root: Path
    usd_assets_root: Path
    output_dir: Path
    # ros 2
    ros_distro: str
    ros_domain_id: int
    rmw_implementation: str
    ros_namespace: str
    cmd_vel_timeout: float
    # simulation
    headless: bool
    physics_hz: int
    render_hz: int
    camera_publish_every_n_frames: int
    # robot
    robot_name: str
    wheel_radius: float
    wheel_base: float
    max_linear_speed: float
    max_angular_speed: float
    robot_start_x: float
    robot_start_y: float
    # sensors
    camera_width: int
    camera_height: int
    lidar_config: str
    # scene
    load_full_warehouse: bool
    num_pallets: int
    num_racks: int
    random_seed: int
    # synthetic data generation
    sdg_num_frames: int
    sdg_output_subdir: str
    sdg_resolution: tuple
    sdg_subframes: int

    missing_env_file: bool = field(default=False)

    # ---- derived paths ------------------------------------------------------
    @property
    def ros2_bridge_dir(self) -> Path:
        return self.isaacsim_root / "exts" / "isaacsim.ros2.bridge"

    @property
    def ros2_lib_dir(self) -> Path:
        return self.ros2_bridge_dir / self.ros_distro / "lib"

    @property
    def ros2_rclpy_dir(self) -> Path:
        return self.ros2_bridge_dir / self.ros_distro / "rclpy"

    @property
    def industrial_assets(self) -> Path:
        return self.usd_assets_root / "Industrial_NVD@10012" / "Assets" / "ArchVis" / "Industrial"

    @property
    def sdg_output_dir(self) -> Path:
        return self.output_dir / self.sdg_output_subdir

    def topic(self, name: str) -> str:
        """Apply the configured namespace to a topic name."""
        ns = self.ros_namespace.strip("/")
        base = name.lstrip("/")
        return "/" + ns + "/" + base if ns else "/" + base


def load_config() -> Config:
    env = _Env(ENV_FILE)
    cfg = Config(
        isaacsim_root=Path(env.str("ISAACSIM_ROOT", r"C:\isaacsim")),
        usd_assets_root=Path(env.str("USD_ASSETS_ROOT", r"D:\code\omniverse_learn\usd_assets")),
        output_dir=env.path_("OUTPUT_DIR", "./_output"),
        ros_distro=env.str("ROS_DISTRO", "humble"),
        ros_domain_id=env.int("ROS_DOMAIN_ID", 0),
        rmw_implementation=env.str("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp"),
        ros_namespace=env.str("ROS_NAMESPACE", ""),
        cmd_vel_timeout=env.float("CMD_VEL_TIMEOUT", 0.5),
        headless=env.bool("HEADLESS", False),
        physics_hz=env.int("PHYSICS_HZ", 60),
        render_hz=env.int("RENDER_HZ", 30),
        camera_publish_every_n_frames=env.int("CAMERA_PUBLISH_EVERY_N_FRAMES", 2),
        robot_name=env.str("ROBOT_NAME", "amr"),
        wheel_radius=env.float("WHEEL_RADIUS", 0.10),
        wheel_base=env.float("WHEEL_BASE", 0.44),
        max_linear_speed=env.float("MAX_LINEAR_SPEED", 1.0),
        max_angular_speed=env.float("MAX_ANGULAR_SPEED", 1.5),
        robot_start_x=env.float("ROBOT_START_X", -7.0),
        robot_start_y=env.float("ROBOT_START_Y", 0.0),
        camera_width=env.int("CAMERA_WIDTH", 640),
        camera_height=env.int("CAMERA_HEIGHT", 480),
        lidar_config=env.str("LIDAR_CONFIG", "Example_Rotary_2D"),
        load_full_warehouse=env.bool("LOAD_FULL_WAREHOUSE", False),
        num_pallets=env.int("NUM_PALLETS", 6),
        num_racks=env.int("NUM_RACKS", 4),
        random_seed=env.int("RANDOM_SEED", 42),
        sdg_num_frames=env.int("SDG_NUM_FRAMES", 40),
        sdg_output_subdir=env.str("SDG_OUTPUT_SUBDIR", "sdg_dataset"),
        sdg_resolution=(env.int("SDG_RESOLUTION_WIDTH", 1024), env.int("SDG_RESOLUTION_HEIGHT", 768)),
        sdg_subframes=env.int("SDG_SUBFRAMES", 4),
        missing_env_file=not ENV_FILE.exists(),
    )
    cfg.output_dir.mkdir(parents=True, exist_ok=True)
    return cfg


def apply_ros2_process_env(cfg: Config) -> None:
    """Point *this process* at Isaac Sim's bundled ROS 2 libraries.

    Call before enabling ``isaacsim.ros2.bridge`` (or before ``import rclpy``
    on the client side). Every write below lands in this process's own copy of
    the environment block, which is discarded when the process exits.
    """
    lib_dir = cfg.ros2_lib_dir
    if not lib_dir.exists():
        raise FileNotFoundError(
            "Bundled ROS 2 libraries not found at " + str(lib_dir) + "\n"
            "Check ISAACSIM_ROOT and ROS_DISTRO in " + str(ENV_FILE)
        )
    os.environ["ROS_DISTRO"] = cfg.ros_distro
    os.environ["RMW_IMPLEMENTATION"] = cfg.rmw_implementation
    os.environ["ROS_DOMAIN_ID"] = str(cfg.ros_domain_id)
    os.environ["PATH"] = str(lib_dir) + os.pathsep + os.environ.get("PATH", "")
    if hasattr(os, "add_dll_directory"):
        os.add_dll_directory(str(lib_dir))


def describe(cfg: Config) -> str:
    note = "  (MISSING -> built-in defaults)" if cfg.missing_env_file else ""
    return "\n".join(
        [
            "-" * 70,
            " config source : " + str(ENV_FILE) + note,
            " isaac sim     : " + str(cfg.isaacsim_root),
            " usd assets    : " + str(cfg.usd_assets_root),
            " ros 2         : " + cfg.ros_distro + " (bundled)  domain=" + str(cfg.ros_domain_id)
            + "  rmw=" + cfg.rmw_implementation,
            " namespace     : " + (cfg.ros_namespace or "<none>")
            + "   cmd_vel timeout=" + str(cfg.cmd_vel_timeout) + "s",
            " headless      : " + str(cfg.headless) + "   physics=" + str(cfg.physics_hz)
            + "Hz  render=" + str(cfg.render_hz) + "Hz",
            " lidar profile : " + cfg.lidar_config,
            " output dir    : " + str(cfg.output_dir),
            "-" * 70,
        ]
    )
