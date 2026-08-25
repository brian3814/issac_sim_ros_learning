"""Sensors: an RTX lidar and an RGB-D camera bolted to the robot.

The important idea here is the **render product**. Isaac's RTX sensors are not
special-cased physics queries -- they are cameras. A lidar is a camera with a
strange projection, and a render product is the handle to "one thing being
rendered, at one resolution". Both the ROS 2 publishers and Replicator's
writers attach to render products, which is why the same camera can feed a
live ROS 2 topic and a training dataset without being defined twice.
"""

from __future__ import annotations

from dataclasses import dataclass

import omni.kit.commands  # type: ignore
import omni.replicator.core as rep  # type: ignore
from pxr import Gf  # type: ignore


@dataclass
class SensorHandles:
    lidar_sensor_prim: str      # the OmniLidar itself -- feeds the render product
    lidar_mount_prim: str       # the prim bolted to the robot -- feeds TF
    lidar_render_product: str
    camera_prim: str


def create_lidar(cfg, robot) -> "tuple[str, str, str]":
    """Create an RTX lidar on the robot and give it its own render product.

    ``config`` names a sensor asset that Isaac Sim resolves against its asset
    server (``/Isaac/Sensors/<vendor>/...``), so the first run downloads it.
    Vendor profiles such as ``SICK_TIM781`` model a real scanner's FOV, range
    and beam pattern. Only 2D profiles yield a sensible
    ``sensor_msgs/LaserScan``; ``Example_Rotary_2D`` is the generic fallback.

    Note the two paths this returns. A vendor asset arrives as an Xform with
    the actual OmniLidar nested inside it, while ``Example_Rotary_2D`` *is* the
    OmniLidar. The render product needs the sensor; TF wants the mount point,
    whose prim name gives the frame a stable, meaningful id.
    """
    mount_path = robot.base_link + "/front_lidar"

    result, sensor = omni.kit.commands.execute(
        "IsaacSensorCreateRtxLidar",
        path="/front_lidar",
        parent=robot.base_link,
        config=cfg.lidar_config,
        translation=(0.30, 0.0, 0.34),
        orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),  # w, i, j, k -- aligned with base_link
    )
    if not result or sensor is None:
        raise RuntimeError(
            "Failed to create RTX lidar with config '%s'. Names are case sensitive "
            "(SICK_TIM781, not SICK_tim781); Example_Rotary_2D always works."
            % cfg.lidar_config
        )

    sensor_path = str(sensor.GetPath())
    # RTX sensors render through the same pipeline as cameras, so they need a
    # render product. The 1x1 resolution is deliberate: lidar returns come from
    # the sensor's own buffers, not from image pixels.
    render_product = rep.create.render_product(sensor_path, [1, 1], name="AmrLidar")
    return sensor_path, mount_path, render_product.path


def setup_sensors(cfg, robot) -> SensorHandles:
    sensor_path, mount_path, lidar_rp = create_lidar(cfg, robot)
    print("[sensors] lidar '%s' -> %s" % (cfg.lidar_config, sensor_path))
    print("[sensors] camera %dx%d -> %s" % (cfg.camera_width, cfg.camera_height, robot.camera))
    return SensorHandles(
        lidar_sensor_prim=sensor_path,
        lidar_mount_prim=mount_path,
        lidar_render_product=lidar_rp,
        camera_prim=robot.camera,
    )
