"""The warehouse the robot drives around in.

Two flavours, chosen by ``LOAD_FULL_WAREHOUSE`` in ``.env``:

* **procedural** (default) -- a floor and four walls authored in code. Loads in
  about a second, is guaranteed to have collision, and keeps the lesson about
  ROS 2 rather than about asset loading.
* **full building** -- references ``Warehouse01.usd`` from the Industrial
  content pack. Photoreal, ~128 MB, noticeably slower to open.

Either way the *props* -- pallets and racks -- come from the real NVIDIA
content pack, because those are what we will be labelling for the synthetic
dataset later, and training on programmer-art boxes would defeat the point.
"""

from __future__ import annotations

import random
from dataclasses import dataclass, field
from pathlib import Path
from typing import List

from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics  # type: ignore

from ..usd_utils import (
    add_colliders_recursive,
    asset_scale_to_meters,
    create_box,
    create_physics_material,
    bind_physics_material,
    make_collider,
    make_rigid_body,
    reference_asset,
    set_transform,
)

# Aisle the robot patrols, in metres.
AISLE_HALF_LENGTH = 10.0
AISLE_HALF_WIDTH = 6.0
WALL_HEIGHT = 4.0
WALL_THICKNESS = 0.2

PALLET_LABEL = "pallet"
RACK_LABEL = "rack"


@dataclass
class SceneHandles:
    """Prim paths downstream stages (SDG, sensors) need to find again."""

    world: str = "/World"
    environment: str = "/World/Environment"
    props: str = "/World/Props"
    pallets: List[str] = field(default_factory=list)
    racks: List[str] = field(default_factory=list)


def _add_lighting(stage: Usd.Stage) -> None:
    """A dome plus a couple of area lights.

    Replicator will randomise these later; without at least one light the RGB
    camera publishes a black image, which is a confusing first result.
    """
    dome = UsdLux.DomeLight.Define(stage, "/World/Environment/DomeLight")
    dome.CreateIntensityAttr(900.0)
    dome.CreateColorAttr(Gf.Vec3f(0.95, 0.96, 1.0))

    for i, x in enumerate((-5.0, 5.0)):
        light = UsdLux.RectLight.Define(stage, "/World/Environment/CeilingLight_%d" % i)
        light.CreateIntensityAttr(9000.0)
        light.CreateWidthAttr(6.0)
        light.CreateHeightAttr(3.0)
        light.CreateColorAttr(Gf.Vec3f(1.0, 0.97, 0.9))
        set_transform(light.GetPrim(), translate=(x, 0.0, WALL_HEIGHT - 0.3))


def _add_physics_scene(stage: Usd.Stage, cfg) -> None:
    """Create the PhysicsScene and pin the solver to the configured rate.

    ``timeStepsPerSecond`` here is the authority on physics rate; the
    SimulationContext's ``physics_dt`` should agree with it or you get two
    different opinions about how fast time moves.
    """
    scene_path = "/World/PhysicsScene"
    scene = UsdPhysics.Scene.Define(stage, scene_path)
    scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr().Set(9.81)

    physx_scene = PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath(scene_path))
    physx_scene.CreateEnableCCDAttr().Set(True)
    physx_scene.CreateTimeStepsPerSecondAttr().Set(cfg.physics_hz)
    physx_scene.CreateSolverTypeAttr().Set("TGS")


def _add_ground_and_walls(stage: Usd.Stage) -> None:
    env = "/World/Environment"
    stage.DefinePrim(env, "Xform")

    concrete = create_physics_material(stage, env + "/ConcreteMaterial", 0.9, 0.8, 0.0)

    floor = create_box(
        stage,
        env + "/Floor",
        size=(AISLE_HALF_LENGTH * 2 + 4, AISLE_HALF_WIDTH * 2 + 4, 0.2),
        translate=(0.0, 0.0, -0.1),
        color=(0.42, 0.42, 0.44),
    )
    make_collider(floor)
    bind_physics_material(floor, concrete)
    # Static geometry: a collider with no RigidBodyAPI never moves and costs
    # the solver nothing.

    walls = [
        ("WallNorth", (0.0, AISLE_HALF_WIDTH, WALL_HEIGHT / 2), (AISLE_HALF_LENGTH * 2, WALL_THICKNESS, WALL_HEIGHT)),
        ("WallSouth", (0.0, -AISLE_HALF_WIDTH, WALL_HEIGHT / 2), (AISLE_HALF_LENGTH * 2, WALL_THICKNESS, WALL_HEIGHT)),
        ("WallEast", (AISLE_HALF_LENGTH, 0.0, WALL_HEIGHT / 2), (WALL_THICKNESS, AISLE_HALF_WIDTH * 2, WALL_HEIGHT)),
        ("WallWest", (-AISLE_HALF_LENGTH, 0.0, WALL_HEIGHT / 2), (WALL_THICKNESS, AISLE_HALF_WIDTH * 2, WALL_HEIGHT)),
    ]
    for name, pos, size in walls:
        wall = create_box(stage, env + "/" + name, size=size, translate=pos, color=(0.62, 0.62, 0.6))
        make_collider(wall)
        bind_physics_material(wall, concrete)


def _load_full_warehouse(stage: Usd.Stage, cfg) -> bool:
    asset = cfg.industrial_assets / "Buildings" / "Warehouse" / "Warehouse01.usd"
    if not asset.exists():
        print("[scene] full warehouse not found at %s -- falling back to procedural room" % asset)
        return False
    scale = asset_scale_to_meters(asset)
    prim = reference_asset(stage, "/World/Environment/Warehouse", asset, uniform_scale=scale)
    n = add_colliders_recursive(prim, approximation="none")
    print("[scene] referenced %s (unit scale %.4f, %d colliders)" % (asset.name, scale, n))
    return True


def _place_props(stage: Usd.Stage, cfg, handles: SceneHandles) -> None:
    """Scatter pallets and racks from the Industrial content pack.

    Pallets are dynamic rigid bodies: the robot can nudge them, and Replicator
    can drop them into new poses. Racks stay static -- they are scenery.
    """
    rng = random.Random(cfg.random_seed)
    stage.DefinePrim(handles.props, "Xform")

    pallet_dir = cfg.industrial_assets / "Pallets"
    rack_dir = cfg.industrial_assets / "Racks"

    pallet_assets = sorted(pallet_dir.glob("Pallet_*.usd")) if pallet_dir.exists() else []
    rack_assets = sorted(rack_dir.glob("RackLarge_A*.usd")) if rack_dir.exists() else []

    if not pallet_assets:
        print("[scene] no pallet assets under %s -- using stand-in boxes" % pallet_dir)

    # ---- racks along both sides of the aisle --------------------------------
    if rack_assets:
        xs = _spread(-AISLE_HALF_LENGTH + 3.0, AISLE_HALF_LENGTH - 3.0, cfg.num_racks)
        for i, x in enumerate(xs):
            side = 1 if i % 2 == 0 else -1
            asset = rack_assets[i % len(rack_assets)]
            scale = asset_scale_to_meters(asset)
            path = handles.props + "/Rack_%02d" % i
            prim = reference_asset(
                stage, path, asset,
                translate=(x, side * (AISLE_HALF_WIDTH - 1.6), 0.0),
                orient=Gf.Quatf(Gf.Rotation(Gf.Vec3d(0, 0, 1), 90.0 if side > 0 else -90.0).GetQuat()),
                uniform_scale=scale,
            )
            add_colliders_recursive(prim, approximation="none")
            _label(prim, RACK_LABEL)
            handles.racks.append(path)

    # ---- pallets scattered down the aisle -----------------------------------
    for i in range(cfg.num_pallets):
        path = handles.props + "/Pallet_%02d" % i
        x = rng.uniform(-AISLE_HALF_LENGTH + 2.5, AISLE_HALF_LENGTH - 2.5)
        y = rng.uniform(-AISLE_HALF_WIDTH + 2.5, AISLE_HALF_WIDTH - 2.5)
        yaw = rng.uniform(0.0, 360.0)
        orient = Gf.Quatf(Gf.Rotation(Gf.Vec3d(0, 0, 1), yaw).GetQuat())

        if pallet_assets:
            asset = pallet_assets[i % len(pallet_assets)]
            scale = asset_scale_to_meters(asset)
            prim = reference_asset(stage, path, asset, translate=(x, y, 0.06),
                                   orient=orient, uniform_scale=scale)
            add_colliders_recursive(prim, approximation="convexHull")
        else:
            prim = create_box(stage, path, size=(1.2, 0.8, 0.14),
                              translate=(x, y, 0.07), color=(0.55, 0.4, 0.22))
            make_collider(prim)

        make_rigid_body(prim, mass=25.0)
        _label(prim, PALLET_LABEL)
        handles.pallets.append(path)


def _spread(lo: float, hi: float, n: int) -> List[float]:
    if n <= 1:
        return [(lo + hi) / 2.0]
    step = (hi - lo) / (n - 1)
    return [lo + step * i for i in range(n)]


def _label(prim: Usd.Prim, label: str) -> None:
    """Attach a semantic label for Replicator / segmentation output.

    Isaac Sim 5.x replaced the old ``SemanticsAPI`` (``add_update_semantics``)
    with ``add_labels``, which writes ``UsdSemantics.LabelsAPI``. The old
    helper still exists but emits a deprecation warning.
    """
    try:
        from isaacsim.core.utils.semantics import add_labels

        add_labels(prim, labels=[label], instance_name="class")
    except Exception as exc:  # pragma: no cover - only if the API moves again
        print("[scene] could not label %s: %s" % (prim.GetPath(), exc))


def build_warehouse(stage: Usd.Stage, cfg) -> SceneHandles:
    """Assemble the whole environment. Returns paths for later stages."""
    handles = SceneHandles()

    world = stage.DefinePrim(handles.world, "Xform")
    stage.SetDefaultPrim(world)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    _add_physics_scene(stage, cfg)
    stage.DefinePrim(handles.environment, "Xform")

    used_full = _load_full_warehouse(stage, cfg) if cfg.load_full_warehouse else False
    if not used_full:
        _add_ground_and_walls(stage)

    _add_lighting(stage)
    _place_props(stage, cfg, handles)

    print("[scene] warehouse ready: %d racks, %d pallets" % (len(handles.racks), len(handles.pallets)))
    return handles
