# Using the project inside the Isaac Sim GUI

Everything here also works in the full Isaac Sim editor, where you can see the
stage tree, inspect the physics properties this project sets in code, and open
the ROS 2 bridge graph as actual boxes and wires. There are three ways in,
increasing in usefulness.

---

## 1. The simplest: just do not pass `--headless`

```cmd
run\sim.cmd
```

`HEADLESS=false` is the default in `.env`, so this already opens the Isaac Sim
window with the scene built, the bridge wired, and the timeline playing. The
`--headless` flag exists for automated runs and slow GPUs.

While it runs you can:

- expand `/World/amr` in the **Stage** panel and click through the prims
- select `/World/amr/joints/left_wheel_joint` and read its **Drive** section in
  the property panel — the same attributes `add_velocity_drive()` sets
- open **Window ▸ Graph Editors ▸ Action Graph**, select `/World/ROS2Graph`,
  and watch the bridge nodes evaluate
- drive it from a second terminal with `run\client.cmd teleop`

This is a normal Kit window, so ▶ / ■ work as usual. Stopping the timeline
destroys the ROS 2 publishers; starting it recreates them.

---

## 2. Open the saved stage

```cmd
run\scene.cmd --save          # writes _output/warehouse_amr.usd
C:\isaacsim\isaac-sim.bat       # then File > Open
```

Good for poking at the robot's construction with no code running. Note this
file contains the **scene and robot only** — `01_build_scene.py` deliberately
leaves ROS 2 out of the lesson, so there is no bridge graph in it. Press ▶ and
the robot just sits there, which is the correct result: nothing is commanding
the wheels.

---

## 3. Build it from the Script Editor (the interesting one)

Launch the editor normally:

```powershell
C:\isaacsim\isaac-sim.bat
```

`isaac-sim.bat` runs `setup_ros_env.bat` for you (inside its own `setlocal`, so
it does not touch your shell), which is why the bridge works in the GUI with no
extra setup.

Open **Window ▸ Script Editor** and run:

```python
import sys
sys.path.insert(0, r"D:\code\omniverse_learn\ros2_integration")   # <- your path

from warehouse_amr import gui
gui.build()
```

Then press **▶ Play**.

`gui.build()` returns immediately and does the work across the next several
frames — watch the console for `[gui]` lines. That is not a style choice: in
the GUI, Kit owns the update loop, so anything that needs a frame to settle
(an extension finishing startup, a fresh stage, an RTX sensor being created)
has to `await` the next update rather than call `simulation_app.update()`.

Options:

```python
gui.build(with_ros2=False)    # scene + robot only, no bridge
gui.build(new_stage=False)    # add to the current stage instead of replacing it
```

> `new_stage=True` (the default) discards whatever is open. Save first if you
> care about it.

### Why there is a separate entry point at all

The scripts under `scripts/` call `SimulationApp(...)`, which *starts* Kit.
Inside the GUI, Kit is already running and a second `SimulationApp` is not
allowed — so `warehouse_amr/gui.py` does the same assembly without it, and
without creating a `SimulationContext` (the GUI's own timeline drives physics,
and the PhysicsScene written by `build_warehouse` already pins the solver rate
from `PHYSICS_HZ`).

### What to look at once it is built

| Where | What it shows |
|---|---|
| Stage ▸ `/World/ROS2Graph` | every bridge node as a prim |
| Window ▸ Graph Editors ▸ Action Graph | the same graph as a node editor — select the graph prim to load it |
| Stage ▸ `/World/amr/base_link` | Physics section: the rigid body and mass this project applies |
| Stage ▸ `/World/amr/left_wheel` | the capsule collider that replaced the cylinder (see troubleshooting) |
| Window ▸ Visual Scripting ▸ Generate Graph Settings | node evaluation timing, if a publisher seems slow |

Select a node in the Action Graph editor and its inputs appear in the property
panel — `topicName`, `frameId`, `queueSize`, and the rest. Editing them there
changes the live graph, which is a fast way to experiment before moving the
change into `ros_graph.py`.

---

## Talking to it from outside

Identical in all three cases — the clients do not care how the simulator was
started:

```cmd
run\client.cmd monitor
run\client.cmd teleop
run\client.cmd patrol
run\client.cmd ros2 topic hz /scan
```

The one requirement is that the timeline is **playing**. A paused or stopped
GUI advertises no topics at all.

---

## GUI-specific gotchas

**Nothing happens after `gui.build()`.** It is asynchronous. Check the console
for `[gui]` lines; an exception in the coroutine is reported there rather than
in the Script Editor output pane.

**`No module named 'warehouse_amr'`.** The `sys.path.insert` line needs your
actual project path, and it must run before the import.

**The robot falls through the floor or explodes on ▶.** You almost certainly
have `new_stage=False` over a stage that already has a ground plane or a second
PhysicsScene. Rebuild with `new_stage=True`.

**Topics vanish when you press ■.** Working as intended — see
`docs/03_troubleshooting.md`.

**The GUI is much slower than headless.** The viewport is rendering on top of
the sensors. Raise `CAMERA_PUBLISH_EVERY_N_FRAMES` in `.env`, or shrink the
camera, or use the headless path for anything throughput-bound like dataset
generation.
