"""Small USD/PhysX helpers shared by the scene builders.

Nothing here is Isaac-specific magic -- it is plain OpenUSD plus the
``UsdPhysics``/``PhysxSchema`` schemas. Reading this file is a decent crash
course in how a robot is actually described on a USD stage.
"""

from __future__ import annotations

from typing import Optional

from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade  # type: ignore


# --------------------------------------------------------------------------
# transforms
# --------------------------------------------------------------------------
def set_transform(prim, translate=None, orient=None, scale=None) -> None:
    """Set a clean translate/orient/scale op stack on ``prim``.

    We deliberately reset the xformOpOrder instead of appending, so calling
    this twice on the same prim is idempotent rather than cumulative.
    """
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    if translate is not None:
        xf.AddTranslateOp().Set(Gf.Vec3d(*translate))
    if orient is not None:
        xf.AddOrientOp().Set(orient)
    if scale is not None:
        xf.AddScaleOp().Set(Gf.Vec3f(*scale))


def look_along(forward, up=(0.0, 0.0, 1.0)) -> Gf.Quatf:
    """Orientation for a USD *camera* (or sensor) that should face ``forward``.

    USD cameras look down their local -Z with +Y up, while robots think in
    "+X is forward, +Z is up". This converts between the two so mounting a
    camera reads as ``look_along((1, 0, 0))`` instead of a magic Euler triple.
    """
    f = Gf.Vec3d(*forward).GetNormalized()
    u = Gf.Vec3d(*up).GetNormalized()
    z = -f                                       # camera -Z points forward
    x = Gf.Cross(u, z).GetNormalized()           # camera +X points right
    y = Gf.Cross(z, x).GetNormalized()           # camera +Y points up
    # USD uses row-vector convention: rows are the images of the basis vectors.
    m = Gf.Matrix3d(x[0], x[1], x[2], y[0], y[1], y[2], z[0], z[1], z[2])
    return Gf.Quatf(Gf.Quatd(m.ExtractRotation().GetQuat()))


# --------------------------------------------------------------------------
# physics materials
# --------------------------------------------------------------------------
def create_physics_material(
    stage,
    path: str,
    static_friction: float,
    dynamic_friction: float,
    restitution: float = 0.0,
):
    mat = UsdShade.Material.Define(stage, path)
    UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    api = UsdPhysics.MaterialAPI(mat.GetPrim())
    api.CreateStaticFrictionAttr().Set(static_friction)
    api.CreateDynamicFrictionAttr().Set(dynamic_friction)
    api.CreateRestitutionAttr().Set(restitution)
    return mat


def bind_physics_material(prim, material) -> None:
    UsdShade.MaterialBindingAPI.Apply(prim)
    UsdShade.MaterialBindingAPI(prim).Bind(
        material,
        bindingStrength=UsdShade.Tokens.weakerThanDescendants,
        materialPurpose="physics",
    )


# --------------------------------------------------------------------------
# rigid bodies and colliders
# --------------------------------------------------------------------------
def make_rigid_body(prim, mass: Optional[float] = None, kinematic: bool = False) -> None:
    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.RigidBodyAPI(prim).CreateKinematicEnabledAttr().Set(kinematic)
    if mass is not None:
        UsdPhysics.MassAPI.Apply(prim)
        UsdPhysics.MassAPI(prim).CreateMassAttr().Set(mass)


def make_collider(prim, approximation: Optional[str] = None) -> None:
    """Turn a gprim into a collider.

    ``approximation`` only applies to *mesh* geometry (``convexHull``,
    ``convexDecomposition``, ``boundingCube``, ...). Analytic prims -- Cube,
    Sphere, Cylinder -- already have exact PhysX shapes and ignore it.
    """
    UsdPhysics.CollisionAPI.Apply(prim)
    if approximation and prim.IsA(UsdGeom.Mesh):
        UsdPhysics.MeshCollisionAPI.Apply(prim)
        UsdPhysics.MeshCollisionAPI(prim).CreateApproximationAttr().Set(approximation)


def add_colliders_recursive(prim, approximation: str = "convexHull") -> int:
    """Apply colliders to every mesh under ``prim``. Returns how many were added.

    Referenced art assets (pallets, racks) ship as pure visual geometry -- no
    physics at all -- so the robot's lidar would see them but drive straight
    through. This is the step that makes them solid.
    """
    count = 0
    for p in Usd.PrimRange(prim):
        if p.IsA(UsdGeom.Mesh):
            make_collider(p, approximation)
            count += 1
    return count


# --------------------------------------------------------------------------
# geometry primitives
# --------------------------------------------------------------------------
def create_box(stage, path, size, translate=(0, 0, 0), color=(0.4, 0.4, 0.45)):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)  # unit cube; the scale op gives it real dimensions
    cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    cube.CreateExtentAttr([Gf.Vec3f(-0.5, -0.5, -0.5), Gf.Vec3f(0.5, 0.5, 0.5)])
    set_transform(cube.GetPrim(), translate=translate, scale=size)
    return cube.GetPrim()


def create_cylinder(stage, path, radius, height, axis="Z", translate=(0, 0, 0),
                    color=(0.1, 0.1, 0.1)):
    cyl = UsdGeom.Cylinder.Define(stage, path)
    cyl.CreateRadiusAttr(radius)
    cyl.CreateHeightAttr(height)
    cyl.CreateAxisAttr(axis)
    cyl.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    half = max(radius, height / 2.0)
    cyl.CreateExtentAttr([Gf.Vec3f(-half, -half, -half), Gf.Vec3f(half, half, half)])
    set_transform(cyl.GetPrim(), translate=translate)
    return cyl.GetPrim()


def create_capsule(stage, path, radius, height, axis="Z", translate=(0, 0, 0),
                   color=(0.08, 0.08, 0.08)):
    """A capsule -- the shape to reach for when you want a wheel.

    ``UsdGeom.Cylinder`` looks like the obvious choice, but PhysX does not
    produce a working collider from one here: a cylinder-wheeled robot sinks
    until its chassis rests on the floor, and then spins its wheels forever
    without moving. A capsule is a cylinder with rounded ends, collides
    natively, and rolls correctly. The rounding is 0.05 m at this scale and
    invisible in practice.
    """
    cap = UsdGeom.Capsule.Define(stage, path)
    cap.CreateRadiusAttr(radius)
    cap.CreateHeightAttr(height)
    cap.CreateAxisAttr(axis)
    cap.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    half = radius + height / 2.0
    cap.CreateExtentAttr([Gf.Vec3f(-half, -half, -half), Gf.Vec3f(half, half, half)])
    set_transform(cap.GetPrim(), translate=translate)
    return cap.GetPrim()


def create_sphere(stage, path, radius, translate=(0, 0, 0), color=(0.2, 0.2, 0.2)):
    sph = UsdGeom.Sphere.Define(stage, path)
    sph.CreateRadiusAttr(radius)
    sph.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    sph.CreateExtentAttr([Gf.Vec3f(-radius, -radius, -radius), Gf.Vec3f(radius, radius, radius)])
    set_transform(sph.GetPrim(), translate=translate)
    return sph.GetPrim()


# --------------------------------------------------------------------------
# joints
# --------------------------------------------------------------------------
def create_revolute_joint(
    stage,
    path: str,
    body0,
    body1,
    axis: str,
    local_pos0=(0, 0, 0),
    local_pos1=(0, 0, 0),
):
    joint = UsdPhysics.RevoluteJoint.Define(stage, path)
    joint.CreateBody0Rel().SetTargets([body0.GetPath()])
    joint.CreateBody1Rel().SetTargets([body1.GetPath()])
    joint.CreateAxisAttr(axis)
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*local_pos0))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(*local_pos1))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))
    # No limit attributes are authored on purpose. In UsdPhysics the limits
    # default to -inf/+inf, which is exactly what a wheel wants: continuous
    # rotation. Writing any finite pair here (including a "reversed" one such
    # as lower=1, upper=-1) gives PhysX a real, and in that case impossible,
    # range -- the joint then locks solid and the robot sits there with its
    # velocity targets perfectly set and nothing moving.
    return joint


def add_velocity_drive(joint_prim, damping: float, max_force: float) -> None:
    """Configure an angular drive for *velocity* control.

    stiffness = 0 with damping > 0 is what makes a USD drive a velocity
    servo rather than a position servo. Isaac's Articulation Controller then
    writes target velocities (rad/s) into it every physics step.
    """
    drive = UsdPhysics.DriveAPI.Apply(joint_prim, "angular")
    drive.CreateTypeAttr().Set("force")
    drive.CreateStiffnessAttr().Set(0.0)
    drive.CreateDampingAttr().Set(damping)
    drive.CreateMaxForceAttr().Set(max_force)
    drive.CreateTargetVelocityAttr().Set(0.0)


def make_articulation_root(prim, self_collisions: bool = False,
                           solver_position_iterations: int = 32,
                           solver_velocity_iterations: int = 1) -> None:
    """Mark ``prim`` as the root of a PhysX articulation.

    Applied to a plain Xform (not a rigid body) this yields a *floating-base*
    articulation, which is what a mobile robot is -- nothing pins it to the
    world, it stays up because its wheels touch the floor.
    """
    UsdPhysics.ArticulationRootAPI.Apply(prim)
    PhysxSchema.PhysxArticulationAPI.Apply(prim)
    api = PhysxSchema.PhysxArticulationAPI(prim)
    api.CreateEnabledSelfCollisionsAttr().Set(self_collisions)
    api.CreateSolverPositionIterationCountAttr().Set(solver_position_iterations)
    api.CreateSolverVelocityIterationCountAttr().Set(solver_velocity_iterations)


# --------------------------------------------------------------------------
# references to external art assets
# --------------------------------------------------------------------------
def reference_asset(stage, prim_path: str, asset_path: str,
                    translate=(0, 0, 0), orient=None,
                    uniform_scale: float = 1.0):
    """Reference an external USD file and place it on the stage.

    The referenced layer may well have been authored in centimetres while our
    stage is in metres. ``uniform_scale`` is where that correction goes; see
    ``asset_scale_to_meters`` for computing it.
    """
    prim = stage.DefinePrim(prim_path, "Xform")
    prim.GetReferences().AddReference(str(asset_path))
    set_transform(
        prim,
        translate=translate,
        orient=orient if orient is not None else Gf.Quatf(1.0),
        scale=(uniform_scale, uniform_scale, uniform_scale),
    )
    return prim


def asset_scale_to_meters(asset_path: str) -> float:
    """Factor that converts a referenced layer's units into stage metres.

    A USD reference does *not* rescale itself to the referencing stage, so a
    pallet authored in centimetres arrives 100x too large. Opening the layer
    just to read ``metersPerUnit`` is cheap and removes the guesswork.
    """
    try:
        layer = Sdf.Layer.FindOrOpen(str(asset_path))
        if layer is None:
            return 1.0
        mpu = layer.pseudoRoot.GetInfo("metersPerUnit") if layer.pseudoRoot.HasInfo("metersPerUnit") else None
        if mpu is None:
            sub = Usd.Stage.Open(str(asset_path), load=Usd.Stage.LoadNone)
            mpu = UsdGeom.GetStageMetersPerUnit(sub) if sub else 1.0
        return float(mpu) if mpu else 1.0
    except Exception:
        return 1.0
