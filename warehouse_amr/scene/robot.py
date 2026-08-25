"""Build a differential-drive AMR as a PhysX articulation, from scratch.

Why build it instead of loading a shipped robot USD? Because the whole point
of the exercise is to see what a robot *is* on a USD stage: a handful of rigid
bodies, two revolute joints with velocity drives, and an articulation root
tying them together. Every Nova Carter or Franka you load later is the same
thing with nicer meshes.

Frame layout (metres, Z-up, ROS REP-103 conventions: +X forward, +Y left):

    /World/amr                  articulation root (floating base)
      base_link                 rigid body -- chassis, casters, sensors
        chassis_body            box collider + visual
        caster_front/rear       frictionless spheres, held just off the floor
        front_camera            UsdGeom.Camera looking down +X
        front_lidar             RTX lidar (added by sensors.py)
      left_wheel                rigid body, capsule collider
      right_wheel               rigid body, capsule collider
      joints/left_wheel_joint   revolute about +Y, velocity drive
      joints/right_wheel_joint  revolute about +Y, velocity drive
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from pxr import Gf, UsdGeom  # type: ignore

from ..usd_utils import (
    add_velocity_drive,
    bind_physics_material,
    create_box,
    create_capsule,
    create_physics_material,
    create_revolute_joint,
    create_sphere,
    look_along,
    make_articulation_root,
    make_collider,
    make_rigid_body,
    set_transform,
)

# Chassis dimensions (metres)
CHASSIS_LENGTH = 0.70
CHASSIS_WIDTH = 0.45
CHASSIS_HEIGHT = 0.18
CHASSIS_MASS = 15.0
WHEEL_WIDTH = 0.05
WHEEL_MASS = 0.8
CASTER_RADIUS = 0.045
# Casters hover just clear of the floor so the drive wheels take the load.
CASTER_LIFT = 0.004

# Velocity-drive gains. stiffness stays 0 (see add_velocity_drive); damping is
# what fights the tracking error, max_force caps the torque PhysX may apply.
# ~1.5 N.m per wheel is enough to accelerate this robot, so 1e3 is generous.
WHEEL_DRIVE_DAMPING = 1.0e4
WHEEL_DRIVE_MAX_FORCE = 1.0e3

LEFT_WHEEL_JOINT = "left_wheel_joint"
RIGHT_WHEEL_JOINT = "right_wheel_joint"


@dataclass
class RobotPaths:
    """Prim paths the ROS 2 graph needs to reference later."""

    root: str
    base_link: str
    chassis: str
    left_wheel: str
    right_wheel: str
    left_joint: str
    right_joint: str
    camera: str
    lidar_mount: str


def build_amr(stage, cfg, root_path: Optional[str] = None) -> RobotPaths:
    """Create the AMR articulation and return the paths of interest."""
    root_path = root_path or "/World/" + cfg.robot_name
    r = cfg.wheel_radius
    half_track = cfg.wheel_base / 2.0

    # ---- articulation root -------------------------------------------------
    # A plain Xform carries the articulation root API. Because it is not itself
    # a rigid body, PhysX builds a floating-base articulation: nothing anchors
    # the robot to the world, it simply rests on its wheels.
    root = stage.DefinePrim(root_path, "Xform")
    # Start 2 cm up so the wheels are not already interpenetrating the floor on
    # the very first physics step; the robot settles within a few frames.
    set_transform(root, translate=(cfg.robot_start_x, cfg.robot_start_y, 0.02))
    make_articulation_root(root)

    # ---- physics materials -------------------------------------------------
    mat_scope = root_path + "/PhysicsMaterials"
    stage.DefinePrim(mat_scope, "Scope")
    rubber = create_physics_material(stage, mat_scope + "/Rubber", 1.0, 0.9, 0.0)
    # A caster is modelled as a frictionless ball rather than another
    # articulated link: the robot slides on it exactly like a real castor
    # wheel, with two fewer joints for the solver to worry about.
    slick = create_physics_material(stage, mat_scope + "/Frictionless", 0.0, 0.0, 0.0)

    # ---- base_link ---------------------------------------------------------
    base_path = root_path + "/base_link"
    base = stage.DefinePrim(base_path, "Xform")
    set_transform(base, translate=(0, 0, 0))
    make_rigid_body(base, mass=CHASSIS_MASS)

    chassis_path = base_path + "/chassis_body"
    chassis = create_box(
        stage,
        chassis_path,
        size=(CHASSIS_LENGTH, CHASSIS_WIDTH, CHASSIS_HEIGHT),
        translate=(0.0, 0.0, r + CHASSIS_HEIGHT / 2.0),
        color=(0.15, 0.45, 0.75),
    )
    make_collider(chassis)

    for name, x in (("caster_front", 0.26), ("caster_rear", -0.26)):
        caster = create_sphere(
            stage,
            base_path + "/" + name,
            # Sitting CASTER_LIFT above the ground plane, the casters normally
            # touch nothing: the drive wheels carry the full weight, which is
            # what gives them the traction to actually push the robot. The
            # casters are there to catch a pitch, not to share the load.
            radius=CASTER_RADIUS,
            translate=(x, 0.0, CASTER_RADIUS + CASTER_LIFT),
            color=(0.05, 0.05, 0.05),
        )
        make_collider(caster)
        bind_physics_material(caster, slick)

    # ---- wheels ------------------------------------------------------------
    wheels = {}
    for name, y in (("left_wheel", half_track), ("right_wheel", -half_track)):
        wheel_path = root_path + "/" + name
        wheel = create_capsule(
            stage,
            wheel_path,
            radius=r,
            height=WHEEL_WIDTH,
            axis="Y",            # spin axis is the robot's +Y, so no extra rotation
            translate=(0.0, y, r),
            color=(0.08, 0.08, 0.08),
        )
        make_rigid_body(wheel, mass=WHEEL_MASS)
        make_collider(wheel)
        bind_physics_material(wheel, rubber)
        wheels[name] = wheel

    # ---- joints ------------------------------------------------------------
    joints_scope = root_path + "/joints"
    stage.DefinePrim(joints_scope, "Scope")

    left_joint_path = joints_scope + "/" + LEFT_WHEEL_JOINT
    right_joint_path = joints_scope + "/" + RIGHT_WHEEL_JOINT

    left_joint = create_revolute_joint(
        stage, left_joint_path, base, wheels["left_wheel"],
        axis="Y", local_pos0=(0.0, half_track, r), local_pos1=(0.0, 0.0, 0.0),
    )
    right_joint = create_revolute_joint(
        stage, right_joint_path, base, wheels["right_wheel"],
        axis="Y", local_pos0=(0.0, -half_track, r), local_pos1=(0.0, 0.0, 0.0),
    )
    add_velocity_drive(left_joint.GetPrim(), WHEEL_DRIVE_DAMPING, WHEEL_DRIVE_MAX_FORCE)
    add_velocity_drive(right_joint.GetPrim(), WHEEL_DRIVE_DAMPING, WHEEL_DRIVE_MAX_FORCE)

    # ---- sensor mounts -----------------------------------------------------
    camera_path = base_path + "/front_camera"
    camera = UsdGeom.Camera.Define(stage, camera_path)
    cam_prim = camera.GetPrim()
    set_transform(
        cam_prim,
        translate=(CHASSIS_LENGTH / 2.0 + 0.02, 0.0, r + CHASSIS_HEIGHT + 0.12),
        orient=look_along((1.0, 0.0, 0.0), up=(0.0, 0.0, 1.0)),
    )
    # A plausible fixed-focal industrial camera: ~65 deg horizontal FOV.
    camera.CreateFocalLengthAttr(18.0)
    camera.CreateHorizontalApertureAttr(23.0)
    camera.CreateVerticalApertureAttr(17.25)
    camera.CreateProjectionAttr("perspective")
    camera.CreateClippingRangeAttr(Gf.Vec2f(0.05, 100.0))

    # The lidar itself is created by sensors.py via an Isaac command; here we
    # only reserve where it will hang.
    lidar_mount = base_path + "/front_lidar"

    return RobotPaths(
        root=root_path,
        base_link=base_path,
        chassis=chassis_path,
        left_wheel=root_path + "/left_wheel",
        right_wheel=root_path + "/right_wheel",
        left_joint=left_joint_path,
        right_joint=right_joint_path,
        camera=camera_path,
        lidar_mount=lidar_mount,
    )
