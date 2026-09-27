# Warehouse AMR — ROS 2 × NVIDIA Isaac Sim 5.1

A hands-on learning project. You build a warehouse, build a robot, drive it
over ROS 2, and generate a labelled training dataset from the same scene —
the four things every industrial Isaac Sim project is made of.

The 101 use case is deliberately the boring, ubiquitous one: **an autonomous
mobile robot (AMR) patrolling a warehouse aisle, looking for pallets.** It is
the "hello world" of warehouse robotics, and it exercises every subsystem you
would use on a real project.

**Nothing is installed on your machine.** No ROS 2, no pip packages, no
environment variables. Isaac Sim ships its own ROS 2, and this project uses it
from inside the process. See [Isolation](#isolation) for how and why.

---

## The mental model

The single most useful thing to understand: **Isaac Sim does not "run ROS 2
code".** It runs a dataflow graph — OmniGraph — that PhysX ticks every frame,
and some of the nodes in that graph happen to be DDS publishers and
subscribers. There is no Python loop pumping messages.

```
        YOUR ROS 2 CODE                    ISAAC SIM
   ┌──────────────────────┐        ┌───────────────────────────┐
   │  patrol.py           │        │  OmniGraph (ticks @60 Hz) │
   │                      │        │                           │
   │  sub /scan  ◄────────┼──DDS───┼──◄ ROS2RtxLidarHelper      │
   │  sub /odom  ◄────────┼──DDS───┼──◄ ROS2PublishOdometry     │
   │      │               │        │        ▲                  │
   │      ▼ decide        │        │  IsaacComputeOdometry      │
   │      │               │        │        ▲                  │
   │  pub /cmd_vel ───────┼──DDS───┼──► ROS2SubscribeTwist      │
   └──────────────────────┘        │        │                  │
                                   │        ▼                  │
                                   │  DifferentialController   │
                                   │        ▼                  │
                                   │  ArticulationController   │
                                   │        ▼                  │
                                   │      PhysX               │
                                   └───────────────────────────┘
```

The client on the left is an ordinary ROS 2 node. It cannot tell it is talking
to a simulator, which is exactly the point — swap in a real AMR publishing the
same topics and the code runs unchanged.

Full detail in [`docs/01_architecture.md`](docs/01_architecture.md).

---

## Setup

You need Isaac Sim 5.1 installed and an NVIDIA RTX GPU. That is all.

```powershell
cd D:\code\omniverse_learn\ros2_integration
Copy-Item .env.example .env
notepad .env        # check ISAACSIM_ROOT and USD_ASSETS_ROOT
```

`.env` is the single source of truth for every setting — paths, DDS domain,
robot geometry, sensor choice, dataset size. Nothing reads your shell.

> **Use the `.cmd` launchers.** Windows blocks `.ps1` files by default
> (execution policy `Restricted`), and changing that is a machine-wide setting
> this project has no business requiring. Batch files are exempt, so
> `run\sim.cmd` works everywhere. Identical `.ps1` versions are there if you
> have already relaxed the policy; see
> [troubleshooting](docs/03_troubleshooting.md) for the one-off bypass.

> **First run downloads assets.** The RTX lidar profile (`SICK_TIM781`) is
> fetched from NVIDIA's asset server the first time, and Isaac Sim compiles
> ray-tracing shaders. Expect the first launch to take a few minutes; later
> ones are much faster.

### Verify everything works

```cmd
run\sim.cmd --self-test
```

This builds the scene headless, brings up the bridge, checks every topic is
advertised, then **drives the robot and confirms the odometry responds** — a
real end-to-end test, not just "did it start". It writes
`_output/self_test_report.txt` and exits non-zero on failure.

---

## Two ways to start Isaac Sim

The difference is simply *who launches Kit* — the project, or you. Both end up
with the same scene, the same bridge and the same topics, and the client
scripts in terminal 2 are identical either way.

### A — let the project launch it

```cmd
run\sim.cmd
```

This starts Isaac Sim, builds the warehouse, wires the bridge and presses play
for you. It opens the **full editor window** — `--headless` is opt-in, not the
default. Wait for `[run] playing.` and the topic list, then leave it running.

Use this for everyday work. It is one command and it cannot get out of step
with the code.

### B — start Isaac Sim yourself, then build into it

Use this when you want the editor open first, when you are already poking at a
stage, or when you want to rebuild the scene repeatedly without restarting Kit.

**1. Launch Isaac Sim normally.**

```cmd
C:\isaacsim\isaac-sim.bat
```

`isaac-sim.bat` runs `setup_ros_env.bat` for you — inside its own `setlocal`,
so it does not touch your shell. That is why the bridge works in the GUI with
no extra setup on your part.

**2. Open Window ▸ Script Editor and run:**

```python
import sys
sys.path.insert(0, r"D:\code\omniverse_learn\ros2_integration")   # <- your path

from warehouse_amr import gui
gui.build()
```

**3. Press ▶ Play.** The nine topics appear when the timeline starts.

`gui.build()` returns immediately and does the work across the next several
frames — watch the **console** for `[gui]` lines, not the Script Editor output
pane. Options:

```python
gui.build(with_ros2=False)    # scene + robot only, no bridge
gui.build(new_stage=False)    # add to the current stage instead of replacing it
```

> `new_stage=True` is the default and **discards whatever is open**. Save first
> if you care about it.

**Why this needs its own entry point.** The scripts under `scripts/` call
`SimulationApp(...)`, which *starts* Kit. Inside the GUI, Kit is already
running and a second `SimulationApp` is not allowed — so
[`warehouse_amr/gui.py`](warehouse_amr/gui.py) does the same assembly without
one. It also skips `SimulationContext`: the GUI's own timeline drives physics,
and the PhysicsScene written by `build_warehouse` already pins the solver rate
from `PHYSICS_HZ`. And because the GUI owns the update loop, anything that
needs a frame to settle — an extension finishing startup, a fresh stage, an RTX
sensor being created — has to `await` the next update rather than call
`simulation_app.update()`. That is why `build()` is asynchronous.

### C — just open the saved stage

```cmd
run\scene.cmd --save          :: writes _output/warehouse_amr.usd
C:\isaacsim\isaac-sim.bat     :: then File ▸ Open
```

Scene and robot only, with no code running — good for inspecting how the robot
is put together. Lesson 1 deliberately leaves ROS 2 out, so pressing ▶ leaves
the robot sitting still. That is the correct result, not a fault: nothing is
commanding the wheels.

### Once it is up

Whichever path you took, the editor is where the abstractions become concrete:

- **Stage ▸ `/World/amr`** — expand it, then click `joints/left_wheel_joint`
  and read the **Drive** section. Those are the attributes
  `add_velocity_drive()` writes in code.
- **Window ▸ Graph Editors ▸ Action Graph** — select `/World/ROS2Graph` and the
  bridge appears as boxes and wires. Click a node and its inputs
  (`topicName`, `frameId`, `queueSize`) are editable **live** — the fastest way
  to experiment before moving a change into `ros_graph.py`.
- **Script Editor, while it runs** — inspect the live simulation:

  ```python
  from isaacsim.core.prims import Articulation
  art = Articulation(prim_paths_expr="/World/amr", name="probe"); art.initialize()
  print(art.get_world_poses()[0], art.get_joint_velocities())
  ```

  Read poses this way rather than off the USD stage — Isaac Sim writes physics
  results into Fabric, so stage transforms can return plausible but wrong
  numbers during playback.

Full detail, including GUI-specific gotchas, in
[`docs/04_gui.md`](docs/04_gui.md).

---

## The lessons

### Lesson 1–2 — USD, physics, and what a robot actually is

```cmd
run\scene.cmd --spin --save
```

Builds the warehouse and assembles a differential-drive robot from scratch:
rigid bodies, revolute joints, velocity drives, an articulation root. No ROS 2
anywhere. `--spin` drives the wheels open-loop so you can watch physics work;
`--save` writes `_output/warehouse_amr.usd`, which you can open in the Isaac
Sim GUI and poke at.

Read [`warehouse_amr/scene/robot.py`](warehouse_amr/scene/robot.py) alongside
it — it is about 150 lines and it is the whole story.

### Lesson 3–4 — the ROS 2 bridge

Terminal 1:

```cmd
run\sim.cmd
```

Terminal 2 — pick one:

```cmd
run\client.cmd monitor      # what is published, and at what rate
run\client.cmd teleop       # drive it yourself: W A S D, space, Q
run\client.cmd patrol       # autonomous patrol that stops for obstacles
run\client.cmd ros2 topic list
run\client.cmd ros2 topic hz /scan
```

`patrol` is the one to read: it closes the loop from `/scan` to `/cmd_vel` and
is a complete, if small, robot behaviour.

The graph itself lives in
[`warehouse_amr/ros_graph.py`](warehouse_amr/ros_graph.py) — and it is worth
seeing it as boxes and wires under `/World/ROS2Graph` in the editor while it
runs. See [Two ways to start Isaac Sim](#two-ways-to-start-isaac-sim).

### Lesson 5 — synthetic data generation

```cmd
run\dataset.cmd --frames 40
```

Renders a labelled pallet-detection dataset into `_output/sdg_dataset`:
RGB, tight 2D boxes, segmentation masks, depth, and camera intrinsics — with
lighting, pallet poses and camera angle randomised every frame. The labels are
derived from the scene graph, so they are exact and free.

---

## Topics

| Topic | Type | Direction | Source in the graph |
|---|---|---|---|
| `/clock` | `rosgraph_msgs/Clock` | out | `ROS2PublishClock` |
| `/cmd_vel` | `geometry_msgs/Twist` | **in** | `ROS2SubscribeTwist` |
| `/odom` | `nav_msgs/Odometry` | out | `ROS2PublishOdometry` |
| `/tf` | `tf2_msgs/TFMessage` | out | `ROS2PublishRawTransformTree` + `ROS2PublishTransformTree` |
| `/joint_states` | `sensor_msgs/JointState` | out | `ROS2PublishJointState` |
| `/scan` | `sensor_msgs/LaserScan` | out | `ROS2RtxLidarHelper` |
| `/rgb` | `sensor_msgs/Image` | out | `ROS2CameraHelper` |
| `/depth` | `sensor_msgs/Image` | out | `ROS2CameraHelper` |
| `/camera_info` | `sensor_msgs/CameraInfo` | out | `ROS2CameraInfoHelper` |

TF tree: `odom` → `base_link` → {`left_wheel`, `right_wheel`, `front_camera`,
`front_lidar`}.

Set `ROS_NAMESPACE` in `.env` to prefix all of them.

---

## Isolation

The project takes "don't touch my machine" seriously:

| | How |
|---|---|
| **No ROS 2 install** | Isaac Sim bundles rclpy, the message packages, tf2 and the `ros2` CLI under `exts/isaacsim.ros2.bridge/<distro>/`. The scripts add that to their own `sys.path`. |
| **No global env vars** | `config.py` *writes* `os.environ` but never *reads* it for configuration. `os.environ` is a per-process copy, so changes die with the process. |
| **No shell mutation** | The `.cmd` and `.ps1` launchers set no variables at all. They locate `python.bat` and hand off. |
| **No pip installs** | Nothing is installed into Isaac Sim's Python or anywhere else. The one missing dependency of the bundled `ros2` CLI — `netifaces` — is supplied as a ~90-line stand-in registered in `sys.modules` at runtime (`ros2_client/_netifaces_shim.py`). |
| **DDS domain from `.env`** | `ROS2Context` is configured with `useDomainIDEnvVar = False`, and the clients pass `domain_id` to `rclpy.init()` directly. A stale `ROS_DOMAIN_ID` in some terminal cannot desync the two sides. |

The only thing written outside the project is Isaac Sim's own shader cache and
its downloaded sensor assets, both under your existing Isaac Sim install.

---

## Troubleshooting

See [`docs/03_troubleshooting.md`](docs/03_troubleshooting.md) for the full
list. The three that will actually bite you:

- **`ros2 topic list` shows nothing.** The simulation must be *playing* —
  Isaac Sim's ROS 2 publishers do not exist while the timeline is stopped.
  Also check `ROS_DOMAIN_ID` matches in `.env`.
- **Out of memory / `vkCreateRayTracingPipelinesKHR failed` on startup.** Ray
  tracing shader compilation is host-RAM hungry. Close other applications; if
  WSL is running, it may be holding several GB.
- **The robot does not move.** Run `run\sim.cmd --self-test` — phase 2 tells
  you whether commands reach PhysX, and `docs/03_troubleshooting.md` explains
  the joint-limit trap that causes this.

---

## Where to go next

The project stops at a working AMR on purpose. Natural extensions, roughly in
order of effort:

- **Nav2.** You already publish `/scan`, `/odom`, `/tf` and consume
  `/cmd_vel` — the exact interface Nav2 expects. Add a map and a Nav2 stack
  (in WSL2 or another machine on the same DDS domain) and the robot navigates.
- **Train the detector.** `_output/sdg_dataset` is a labelled detection
  dataset. Convert the `.npy` boxes to COCO or YOLO format and train.
- **Close the perception loop.** Run the trained detector on `/rgb`, publish
  detections, and have `patrol.py` drive to a pallet instead of past it.
- **Swap in a real robot USD.** Load Nova Carter from the Isaac asset server
  and repoint `ros_graph.py` at its prims. The graph barely changes — which is
  the point of having built one by hand first.
- **Multi-robot.** Set `ROS_NAMESPACE` and instantiate the scene twice.

## Layout

```
.env / .env.example        all configuration
warehouse_amr/
  config.py                .env loader + process-local ROS 2 setup
  app.py                   Kit bootstrap (ordering rules live here)
  usd_utils.py             USD + PhysX helpers
  scene/warehouse.py       environment, props, lighting, semantics
  scene/robot.py           the AMR articulation
  sensors.py               RTX lidar + camera render products
  ros_graph.py             the ROS 2 bridge OmniGraph
  sdg.py                   Replicator randomisers + writer
  gui.py                   build it all inside a running Isaac Sim editor
scripts/                   entry points (run with Isaac Sim's python)
ros2_client/               plain ROS 2 nodes (no Isaac Sim)
run/                       .cmd and .ps1 launchers
docs/                      markdown docs + an illustrated index.html
```

## Docs

- **[`docs/index.html`](docs/index.html) — the illustrated version.** Open it in a browser:
  architecture, control-flow, sensor and TF diagrams, plus the measured results. Single
  self-contained file, no dependencies, light and dark themes.
- **[`docs/code_flow.zh-TW.html`](docs/code_flow.zh-TW.html) — 繁體中文運作原理圖解**：執行範圍（哪些在 Omniverse 裡跑）、
  啟動順序、OmniGraph 接線、WASD 訊號流（含互動示範）、digital twin 開發指南。
- [`docs/01_architecture.md`](docs/01_architecture.md) — how the layers fit, and the ideas worth understanding before editing
- [`docs/02_running.md`](docs/02_running.md) — every command, every `.env` knob
- [`docs/03_troubleshooting.md`](docs/03_troubleshooting.md) — the failures this project actually hit, with measurements
- [`docs/04_gui.md`](docs/04_gui.md) — using it inside the Isaac Sim editor
