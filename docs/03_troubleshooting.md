# Troubleshooting

Ordered roughly by how often they bite. Several of these were hit while
building this project, and the notes say what the symptom actually looked like.

---

## `.ps1 cannot be loaded because running scripts is disabled on this system`

```
.\run\scene.ps1 : File ...\run\scene.ps1 cannot be loaded because running
scripts is disabled on this system.
    + FullyQualifiedErrorId : UnauthorizedAccess
```

Windows ships with the PowerShell execution policy set to `Restricted`, which
blocks every `.ps1` file. Nothing is wrong with your setup.

**Use the `.cmd` launchers instead** — batch files are not subject to
execution policy, and they take the same arguments:

```cmd
run\scene.cmd --spin --save
run\sim.cmd --self-test
run\dataset.cmd --frames 40
run\client.cmd monitor
```

If you prefer the PowerShell versions, run them through a one-off bypass,
which changes nothing permanently:

```powershell
powershell -ExecutionPolicy Bypass -File .\run\sim.ps1 --self-test
```

Or, if you want `.ps1` files to work generally, this is the usual setting —
**it is a change to your user profile, so it is your call, not something this
project needs**:

```powershell
Set-ExecutionPolicy -Scope CurrentUser RemoteSigned
```

Check what you currently have with `Get-ExecutionPolicy -List`. A `Process`
scope of `Bypass` only applies to that one shell, which is why a script can
work in one terminal and fail in another.

---

## `ros2 topic list` shows only `/parameter_events` and `/rosout`

**The simulation is not playing.** Isaac Sim creates its DDS publishers when
the timeline starts and destroys them when it stops. A loaded-but-stopped
simulation advertises nothing.

- Standalone scripts (`run\sim.cmd`) call `play()` for you.
- In the GUI, press the ▶ button.

If it *is* playing, check the DDS domain. `.env`'s `ROS_DOMAIN_ID` is used by
both sides; if you edited it, restart both the simulator and the client.

Then check the bridge is actually enabled — `run\sim.cmd` prints a warning if
it is not, and `--self-test` reports `bridge enabled : False`.

---

## The robot stops by itself after half a second

That is the command watchdog: `/cmd_vel` went quiet for longer than
`CMD_VEL_TIMEOUT` (default 0.5 s of simulation time). The console shows
`[watchdog] no fresh /cmd_vel -> STOP`.

A publisher must keep sending, even when the command has not changed — the
bundled clients send 20 per second. A node that publishes once and goes quiet
(some teleop tools only publish on a keypress) will be stopped. Make it
stream, or raise `CMD_VEL_TIMEOUT` if you accept the longer blind distance:
at 1 m/s, every 0.1 s of timeout is another 10 cm travelled without a fresh
command.

---

## The robot does not move, but `/cmd_vel` is being published

Run `run\sim.cmd --self-test`. Phase 2 drives the robot and reports exactly
where the chain breaks. Two causes account for nearly all of it:

### Joint limits lock the wheel

In UsdPhysics, `physics:lowerLimit` / `physics:upperLimit` default to
∓infinity, which is what a continuously rotating wheel needs. Authoring *any*
finite pair constrains the joint. Authoring a reversed pair (lower `1.0`,
upper `-1.0`) — a convention some engines use to mean "unlimited" — gives
PhysX an impossible range and the joint locks solid.

The symptom is maddening because everything upstream looks perfect: the
subscriber receives the Twist, the differential controller emits the right
wheel speeds, the articulation controller accepts them, and the wheels report
a velocity of exactly 0.

**Fix:** do not author limit attributes on a wheel joint at all. See
`create_revolute_joint()` in `usd_utils.py`.

### Cylinder wheels do not collide

`UsdGeom.Cylinder` seems like the natural wheel primitive, but PhysX does not
produce a usable collider from one in this configuration. A measured
comparison on flat ground, all commanded to 0.5 m/s for 6 seconds:

| wheel shape | distance travelled |
|---|---|
| `UsdGeom.Cylinder` | 0.000 m |
| `UsdGeom.Capsule` | 2.942 m |
| `UsdGeom.Sphere` | 2.939 m |

The tell is the robot's resting height: with cylinder wheels the chassis sinks
until *it* touches the floor, exactly one wheel-radius lower than it should
be, and the wheels spin buried in the ground at their commanded velocity.

**Fix:** use `UsdGeom.Capsule` for wheels — geometrically a cylinder with
rounded ends, and natively supported. `create_capsule()` in `usd_utils.py`.

### The casters are carrying the weight

Two casters plus two wheels is a statically indeterminate four-point contact,
and the solver may put most of the normal force on the frictionless casters,
leaving the wheels to spin with no grip. This project lifts the casters 4 mm
clear of the floor (`CASTER_LIFT`) so the drive wheels carry the load and the
casters only catch a pitch.

---

## `vkCreateRayTracingPipelinesKHR failed` / `LLVM ERROR: out of memory`

Ray-tracing shader compilation is **host RAM** hungry, not VRAM hungry. Isaac
Sim logs how much it found at start-up:

```
Max allowing 3 shader compiler sessions based on 6.1 GB of free system memory.
```

If that number is low, the compile can run out of memory and take the process
down during the first rendered frames.

- Close other applications. WSL in particular (`vmmemWSL`) can hold several GB.
- The first run is the expensive one; the shader cache makes later runs cheap.
- `C:\isaacsim\warmup.bat` pre-builds the cache if you would rather do it up
  front.

---

## `Config 'SICK_tim781' not found for OmniLidar`

Lidar profile names are **case sensitive** and resolve to USD assets on
NVIDIA's asset server, not to local JSON files. The correct spelling is
`SICK_TIM781` (or just `TIM781`).

Valid names come from `SUPPORTED_LIDAR_CONFIGS` in
`isaacsim/sensors/rtx/impl/supported_lidar_configs.py`. If the name does not
match, the command still creates a prim, so the failure is a warning rather
than an error and easy to miss.

The first use of a profile downloads it. No internet, or a blocked asset
server, means no lidar — set `LIDAR_CONFIG=Example_Rotary_2D` and check that
`get_assets_root_path()` resolves.

---

## `ros2 topic list` fails with `KeyError: 'list'`

The verb exists in the entry points but could not be *imported*, and ros2cli
reports the missing key rather than the underlying error. The real cause here
is that `ros2cli.daemon` imports `netifaces`, which Isaac Sim does not bundle.

This project ships `ros2_client/_netifaces_shim.py`, a small stand-in
registered in `sys.modules` before ros2cli is imported, so `ros2cli.py`
handles it for you. If you invoke the bundled CLI some other way you will hit
this; import the shim first, or `pip install netifaces` into Isaac Sim's
Python if you do not mind modifying the install.

To see the true error behind any such `KeyError`, import the verb directly:

```powershell
C:\isaacsim\python.bat -c "import sys; sys.path.insert(0, r'C:\isaacsim\exts\isaacsim.ros2.bridge\humble\rclpy'); import ros2topic.verb.list"
```

---

## `/scan` publishes but every range is `inf`

The lidar sees nothing solid. Referenced art assets are visual geometry only —
they have no physics until colliders are added. `add_colliders_recursive()`
handles the pallets and racks; if you reference new assets yourself, do the
same or the robot will drive straight through them.

Also confirm the profile is a **2D** one. A 3D profile still produces a
`LaserScan`, but a single scan line through a 3D pattern is not what you want.

---

## The camera image is black

- No lights in the scene. `build_warehouse()` adds a dome plus ceiling lights;
  a hand-built stage often has none.
- The render product has not warmed up. The first few frames after `play()`
  can be blank — step 20 or 30 frames before judging.

---

## `DLSS increasing input dimensions: Render resolution of (320, 240) is below minimal input resolution of 300`

Harmless. DLSS renders internally at half the target resolution, and 640×480
lands just under its minimum. Raise `CAMERA_WIDTH`/`CAMERA_HEIGHT` above
600×600 if you want it to stop.

---

## Python `print()` output is missing or out of order

Kit writes its log straight to the console while Python's stdout is block
buffered, so prints can interleave badly or vanish if the process dies. Run
with `-u`, which the `run\*.ps1` launchers already do.

---

## Pallets and racks are enormous, or invisibly small

A USD reference does not rescale itself to the referencing stage's
`metersPerUnit`. An asset authored in centimetres arrives 100× too large.
`asset_scale_to_meters()` reads the referenced layer's own metadata and
applies the correction; if you add assets by hand, do the same.

---

## The script finished its work but the process will not exit

`SimulationApp.close()` sometimes hangs on Windows after everything useful is
done — you see the final output and `Simulation App Shutting Down`, then
nothing, with `kit.exe` sitting at 0% CPU holding several GB.

The entry scripts here end with `app.shutdown()`, which calls `close()` and
then forces the process to exit. Anything that must reach disk (the self-test
report, Replicator's `wait_until_complete()`) happens before that call. If you
write your own standalone script, do the same or expect the occasional orphan.

---

## A previous run is still holding the GPU / everything is slow

Isaac Sim runs as `kit.exe`. If a launch is interrupted, an orphan can survive
and sit idle holding several GB. Check and clear it:

```powershell
Get-Process kit -ErrorAction SilentlyContinue |
    Select-Object Id, @{N='GB';E={[math]::Round($_.WorkingSet64/1GB,2)}}, StartTime
Get-Process kit -ErrorAction SilentlyContinue | Stop-Process -Force
```

An idle orphan is easy to spot: near-zero CPU with a large working set. Two
live instances also compete badly — the ROS 2 sim and a dataset run should not
overlap.

---

## Nothing works and you want a clean slate

```powershell
Remove-Item -Recurse -Force _output
Copy-Item .env.example .env -Force
run\sim.cmd --self-test
```

If the shader cache itself is suspect, `C:\isaacsim\clear_caches.bat` clears
it — at the cost of a slow next start-up.
