# Architecture

How the pieces fit, and why they are arranged this way.

## 1. Four layers

```
┌─────────────────────────────────────────────────────────────────┐
│ ROS 2 clients        patrol.py, teleop_key.py, topic_monitor.py │
│                      ordinary rclpy nodes; know nothing of Isaac│
└───────────────────────────────┬─────────────────────────────────┘
                                │ DDS (FastDDS, domain from .env)
┌───────────────────────────────┴─────────────────────────────────┐
│ ROS 2 bridge         isaacsim.ros2.bridge OmniGraph nodes       │
│                      publishers / subscribers as graph nodes    │
└───────────────────────────────┬─────────────────────────────────┘
                                │ OmniGraph attributes
┌───────────────────────────────┴─────────────────────────────────┐
│ Simulation           PhysX (articulations, joints, contacts)    │
│                      RTX renderer (camera, lidar render products)│
└───────────────────────────────┬─────────────────────────────────┘
                                │ reads / writes
┌───────────────────────────────┴─────────────────────────────────┐
│ Scene                OpenUSD stage: geometry, physics schemas,  │
│                      semantic labels, materials                 │
└─────────────────────────────────────────────────────────────────┘
```

Each layer is replaceable. That is the actual value proposition: swap the
bottom two for a real robot and the top layer does not change.

## 2. USD is the database

Everything in the simulator is a prim on a USD stage. A robot is not a special
object type — it is rigid bodies with `UsdPhysics.RigidBodyAPI`, joints with
`UsdPhysics.RevoluteJoint`, and a marker (`UsdPhysics.ArticulationRootAPI`)
saying "treat this subtree as one articulated system".

`warehouse_amr/scene/robot.py` builds ours in about 150 lines. Worth reading
before you load anyone else's robot, because a Nova Carter or a Franka is the
same thing with better meshes.

Two USD facts that cause most beginner grief:

**Units.** A stage has `metersPerUnit`. Referencing a layer does *not* rescale
it, so a pallet authored in centimetres arrives 100× too big.
`asset_scale_to_meters()` in `usd_utils.py` reads the referenced layer's own
metadata and compensates.

**References carry no physics.** Art assets — pallets, racks — are visual
geometry only. `add_colliders_recursive()` is what makes them solid; without
it the lidar sees them but the robot drives straight through.

## 3. OmniGraph is the bridge

There is no message-pump loop. `ros_graph.py` builds a graph under
`/World/ROS2Graph`, and PhysX evaluates it every frame. Nodes fall into three
groups:

- **Sources** — `OnPlaybackTick` (fires each frame while playing),
  `IsaacReadSimulationTime`, `ROS2Context`.
- **Compute** — `DifferentialController` (v, ω → wheel speeds),
  `IsaacComputeOdometry`, `BreakVector3`.
- **I/O** — the `ROS2Publish*` / `ROS2Subscribe*` nodes and the sensor helpers.

Two wiring details that are easy to get wrong:

**Execution vs data connections.** An `execIn`/`execOut` pair says *when* a
node runs. A value connection says *what* it reads, and it also forces
evaluation order — `ArticulationController` pulls `velocityCommand` from
`DifferentialController`, so the controller is guaranteed to have computed
before the articulation is written, even though both hang off the same tick.

**Twist is 3-vectors, the controller wants scalars.** A `geometry_msgs/Twist`
carries `linear` and `angular` as `vectord[3]`. `BreakVector3` taps `linear.x`
and `angular.z`. That is the only reason those two extra nodes exist.

**Publishers exist only while playing.** Isaac Sim creates the DDS entities
when the timeline starts. A stopped simulation advertises nothing — this is
the number one cause of "`ros2 topic list` is empty".

## 4. Sensors are cameras

An RTX lidar is a camera with an unusual projection. Both it and the RGB-D
camera produce a **render product**: a handle to one thing being rendered at
one resolution.

```
UsdGeom.Camera  ──► IsaacCreateRenderProduct ──► ROS2CameraHelper   ──► /rgb, /depth
                                             └─► ROS2CameraInfoHelper ─► /camera_info

OmniLidar (RTX) ──► rep.create.render_product ──► ROS2RtxLidarHelper ──► /scan
```

The camera's render product is created *inside* the graph
(`IsaacCreateRenderProduct`) because the node can take the camera prim
directly. The lidar's is created in Python (`sensors.py`) because an RTX
sensor must exist and have a render product before the helper can wrap it.

The same render-product abstraction is what lets Replicator attach writers to
a camera for dataset generation — one camera, two consumers.

**Lidar profiles are USD assets.** In Isaac Sim 5.x, `config="SICK_TIM781"`
resolves to `/Isaac/Sensors/SICK/TIM781/SICK_TIM781.usd` on NVIDIA's asset
server and is downloaded on first use. Names are case sensitive, and vendor
assets arrive as an `Xform` with the real `OmniLidar` nested inside — which is
why `sensors.py` tracks the sensor path and the mount path separately.

## 5. Frames

```
odom ──(ROS2PublishRawTransformTree, from IsaacComputeOdometry)──► base_link
                                                                     │
   (ROS2PublishTransformTree, parentPrim=base_link) ─────────────────┤
                                                                     ├─► left_wheel
                                                                     ├─► right_wheel
                                                                     ├─► front_camera
                                                                     └─► front_lidar
```

Frame ids come from prim names, so the prim name and the `frameId` on the
corresponding publisher must agree or the data lands in a frame nothing else
references.

> **Known nuance.** ROS convention (REP-103) expects camera *optical* frames
> with +Z forward and +Y down, while the USD camera frame here is +X forward,
> +Z up. If you plug this into a perception stack that assumes optical frames,
> publish a static transform between `front_camera` and a
> `front_camera_optical` frame rather than rotating the camera prim — moving
> the prim would also rotate the rendered image.

## 6. Two pipelines, two processes

| | ROS 2 bridge (`02_run_sim_ros2.py`) | Replicator SDG (`03_generate_dataset.py`) |
|---|---|---|
| Driven by | physics clock, real time | `rep.orchestrator.step()` |
| Bottleneck | physics + render latency | render throughput |
| Cares about | staying real-time | image quality, label correctness |
| Runs | GUI or headless | headless |

They deliberately do not share a process. Data generation wants to render each
frame several times until it converges (`rt_subframes`) and does not care
about wall-clock; a robot control loop cares about nothing else. This is also
how these jobs are split in production.

## 7. Reading simulation state

Once the timeline is playing, **do not read poses off the USD stage.** Isaac
Sim writes physics results into Fabric (USDRT) for speed, and the USD
attributes may lag or never update. `UsdGeom.Xformable.ComputeLocalToWorld
Transform()` will happily return a number — it just will not be the number
PhysX is using.

The authoritative reads are the tensor APIs:

```python
from isaacsim.core.prims import Articulation, RigidPrim

art = Articulation(prim_paths_expr="/World/amr", name="amr")
art.initialize()                      # only valid after play()
positions, orientations = art.get_world_poses()
joint_velocities = art.get_joint_velocities()
```

Inside the graph, `IsaacComputeOdometry` reads the same source, which is why
`/odom` and the tensor API agree while a stage read may not.

## 8. The command watchdog

Two things in the simulator remember the last command: `ROS2SubscribeTwist`
keeps its last message, and a PhysX joint drive keeps its last target until
something overwrites it. If the node publishing `/cmd_vel` dies without
sending a zero, the robot drives on forever. A real AMR's base controller
prevents this with a command timeout (ros2_control's `diff_drive_controller`
calls it `cmd_vel_timeout`, default 0.5 s), so the simulator has one too.

With `run\sim.cmd`, the graph is built with `drive_from_graph=False` and the
wheels belong to `warehouse_amr/base_controller.py` instead:

```
/cmd_vel ──► rclpy subscriber (depth 1) ──► CmdVelWatchdog ──► v, ω ──► wheel targets ──► PhysX
                                              │
                          no valid message for CMD_VEL_TIMEOUT → (0, 0)
```

The rules it follows, and why:

- **It sits on the receiving side.** A sender can stream at a steady rate and
  publish a zero on exit (both clients do), but only the receiver can notice
  that the stream has stopped.
- **It times the last valid message received**, not changes in value — an
  unchanged command repeated 20 times a second is still a heartbeat.
- **Silence produces an active zero**, written every step, because nothing
  else will overwrite the last target.
- **It measures simulation time.** The robot only moves when the simulation
  advances, so a slow frame is not mistaken for silence. A clock that jumps
  backwards (Stop → Play) drops the stored command instead of reviving it.
- **The queue is one deep.** `spin_once()` services one callback per call; a
  deeper queue lets stale commands pile up whenever the loop runs slower
  than the publisher.

It is the only writer of wheel targets. Two writers — the graph's
`ArticulationController` and this — would overwrite each other every frame.

The GUI path (`gui.build()`) still drives from the graph, since there is no
Python loop to run the controller in, and so has no timeout.

A software timeout is a functional safeguard, not a safety function. A real
robot adds a communication timeout in its motor drives and a certified layer
(emergency stop, safety-rated scanners, a safety PLC) underneath.

## 9. Configuration and isolation

`config.py` reads `.env` and **never reads `os.environ` for configuration**.
It writes `os.environ` — process-locally — only to point Kit at the bundled
ROS 2 libraries.

The DDS domain is the case that pays for itself. `ROS2Context` is created with
`useDomainIDEnvVar = False`, and clients pass `domain_id=` straight to
`rclpy.init()`. Both sides therefore take the domain from the same file, and
no stray shell variable can put them on different domains — a failure that
otherwise presents as "everything started fine but there are no topics".
