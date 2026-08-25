# Running the project

Every command below is run from the project root.

## Once, before anything else

```powershell
Copy-Item .env.example .env
```

Open `.env` and check two paths:

- `ISAACSIM_ROOT` — the folder containing `isaac-sim.bat` (default `C:\isaacsim`)
- `USD_ASSETS_ROOT` — where your `Industrial_NVD@...` content packs live

Everything else has a working default.

## Verify the install

```cmd
run\sim.cmd --self-test
```

Two phases, both automatic:

1. **Topic discovery** — brings up the scene and bridge headless and confirms
   all nine topics are advertised.
2. **Closed-loop drive** — publishes `/cmd_vel`, then checks the odometry
   moved by the commanded distance and that `/scan` returns real hits.

Writes `_output/self_test_report.txt`; exits non-zero on failure. A healthy
report ends with `RESULT : PASS`.

The first run is slow — Isaac Sim compiles ray-tracing shaders and downloads
the lidar profile. Later runs start in well under a minute.

---

## Lesson 1–2: scene and physics

```cmd
run\scene.cmd                          # watch it settle
run\scene.cmd --spin                   # drive the wheels open-loop
run\scene.cmd --spin --save            # also write _output/warehouse_amr.usd
run\scene.cmd --headless --seconds 4   # no window
```

Prints the robot's prim tree with each prim's type, then reports `base_link`'s
world position every two seconds so you can see it move.

The saved `.usd` opens in the Isaac Sim GUI (`C:\isaacsim\isaac-sim.bat`,
then File ▸ Open). Select `/World/amr` and look at the Physics section of the
property panel — every API this project applied in code is visible there.

---

## Lesson 3–4: ROS 2

**Terminal 1** — the simulator, left running:

```cmd
run\sim.cmd
```

**Terminal 2** — a client:

```cmd
run\client.cmd monitor              # topic list + measured rates
run\client.cmd monitor --duration 20
run\client.cmd teleop               # W/S forward/back, A/D turn, space stop, Q quit
run\client.cmd patrol               # autonomous, stops for obstacles
run\client.cmd patrol --duration 60
```

The bundled `ros2` CLI, no install needed:

```cmd
run\client.cmd ros2 topic list
run\client.cmd ros2 topic hz /scan
run\client.cmd ros2 topic echo /odom --once
run\client.cmd ros2 node list
run\client.cmd ros2 topic info /cmd_vel --verbose
```

Avoid `ros2 topic echo /rgb` — it will dump a 640×480 image as text. Use
`--once`, or `ros2 topic hz`.

### Suggested order

1. `monitor` first — confirm data is flowing and see the rates.
2. `teleop` — drive into a rack and watch `/scan` react in `monitor`.
3. `patrol` — read the source, then watch the state machine in the terminal.

`patrol` prints a line per tick:

```
  DRIVE   front= 2.64m  pose=( 12.22, -1.56) yaw=  -3.8  legs=10
```

`pose` comes from `/odom`, whose origin is **wherever the robot was when the
simulation started** — not the world origin. With the default
`ROBOT_START_X=-7`, an odom x of 12.22 means world x ≈ 5.2. That is standard
ROS behaviour (REP-105: the `odom` frame is fixed at start-up), and it is why
a real stack adds a `map` frame on top rather than treating odom as global.

### Useful flags

```cmd
run\sim.cmd --headless          # no window, more GPU for sensors
run\sim.cmd --duration 60       # stop automatically after a minute
```

---

## Lesson 5: synthetic data

```cmd
run\dataset.cmd --frames 40
run\dataset.cmd --frames 200          # a dataset worth training on
run\dataset.cmd --frames 10 --gui     # watch the randomisation
```

Output lands flat in `_output/sdg_dataset`, one set of files per frame:

```
rgb_0000.png                              the training image
bounding_box_2d_tight_0000.npy            boxes, one structured row per instance
bounding_box_2d_tight_labels_0000.json    semanticId -> class name
bounding_box_2d_tight_prim_paths_0000.json which prim each box came from
semantic_segmentation_0000.png            per-pixel masks
semantic_segmentation_labels_0000.json
distance_to_image_plane_0000.npy          depth in metres
camera_params_0000.json                   intrinsics + pose
```

Each box row has fields
`(semanticId, x_min, y_min, x_max, y_max, occlusionRatio)`:

```python
import json, numpy as np
boxes  = np.load("_output/sdg_dataset/bounding_box_2d_tight_0000.npy")
labels = json.load(open("_output/sdg_dataset/bounding_box_2d_tight_labels_0000.json"))
for b in boxes:
    print(labels[str(b["semanticId"])]["class"], b["x_min"], b["y_min"], b["x_max"], b["y_max"])
```

Two things that look like bugs but are not: `occlusionRatio` is `-1` for an
object clipped by the image border, and depth is `inf` wherever a ray hit
nothing.

The script prints a file-count summary per annotator at the end. Zero boxes
usually means the pallets were not labelled — check `USD_ASSETS_ROOT`.

Turn `SDG_SUBFRAMES` up in `.env` if images look blurry or textures appear
unloaded; turn it down for speed.

---

## In the Isaac Sim editor

`run\sim.cmd` without `--headless` is already the GUI. To build the scene
from Isaac Sim's own Script Editor instead, see [`04_gui.md`](04_gui.md).

---

## Which launcher, and running without one

`run\` holds two equivalent sets:

| | Works when | Notes |
|---|---|---|
| `run\*.cmd` | always | Batch files are exempt from PowerShell's execution policy. **Start here.** |
| `run\*.ps1` | execution policy allows scripts | Same arguments. Windows defaults to `Restricted`, which blocks these — see [troubleshooting](03_troubleshooting.md). |

Neither sets an environment variable anywhere; both just locate `python.bat`
and hand off. They are convenience only — this is all they do:

```
C:\isaacsim\python.bat -u scripts\02_run_sim_ros2.py --self-test
C:\isaacsim\python.bat -u ros2_client\patrol.py
```

Use `-u`; otherwise Python's buffered output interleaves badly with Kit's log.

---

## Changing the robot

Everything below is in `.env` and takes effect on the next run:

| Setting | Effect |
|---|---|
| `WHEEL_RADIUS`, `WHEEL_BASE` | robot geometry **and** the differential controller — they are read from the same values, so they cannot drift apart |
| `MAX_LINEAR_SPEED`, `MAX_ANGULAR_SPEED` | command clamps |
| `LIDAR_CONFIG` | `SICK_TIM781`, `SICK_picoScan150`, `RPLIDAR_S2E`, `Example_Rotary_2D` |
| `CAMERA_WIDTH`, `CAMERA_HEIGHT` | image size |
| `CAMERA_PUBLISH_EVERY_N_FRAMES` | raise it if image publishing is slowing the sim |
| `NUM_PALLETS`, `NUM_RACKS`, `RANDOM_SEED` | scene contents |
| `LOAD_FULL_WAREHOUSE` | `true` swaps the procedural room for the 128 MB photoreal building |
| `ROS_NAMESPACE` | prefixes every topic |
| `ROS_DOMAIN_ID` | DDS domain, used by both the simulator and the clients |

Structural changes — a different chassis, a third wheel, a second camera —
belong in `warehouse_amr/scene/robot.py`, which is short enough to edit
comfortably.
