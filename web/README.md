# Vector simulacrum in the browser

The simulacrum one directory up, running in a web page: the same MuJoCo model,
the same firmware tread control law and the same cliff reaction, with a 3D view
you can drive. MuJoCo runs in the page as WebAssembly. There is no server and
nothing here connects to a robot.

The structure follows the
[MicroDuck simulator](https://huggingface.co/spaces/pollen-robotics/microduck-simulator)
(pollen-robotics, Apache-2.0): a Vite and React shell around an imperative
core, a small store between them. No code or assets were copied from it.

## Published simulator

The [shelf and dock scene](https://p1r4n351-vector-simulacrum.static.hf.space/index.html?env=shelf&cube=free&start=docked) and the default table scene use the same application. Publish the production build from GitHub `main` to the existing Space and record its source commit in the Space card. Optional SDK meshes are for local use.

Local WireOS experiments connect this application to the firmware relay. A full-stack shelf run reported 1.560937 V in WebViz 4.124 seconds after the cube connection event. This checks delivery of the simulated battery reading through the firmware.

## Run it

```bash
./demo.sh
```

That installs the packages, checks the exported model against the parameter
table, fetches the SDK's robot model, runs the tests, builds, and serves the
page on http://127.0.0.1:5183. Needs node 20 or later and python3.
`./demo.sh --build-only` stops after the build; `--no-sdk-model` skips the
model fetch; `--port N` changes the port.

For development: `npm install`, then `npm run dev`.

## What you see

- By default (`vector-free-dock.xml`), the Vector 1.0 robot starts on the
  simulacrum's 600 mm table with a movable Light Cube 100 mm ahead and the
  charger dock behind (`?cube=none` and `?dock=none` omit them). The table has
  real edges and a floor 300 mm below.
- **Cliff reaction on** (the default control level): drive at an edge and the
  robot stops, takes control away, and backs off. Press X to take control back.
- **Cliff reaction off** (the override level): the robot keeps driving and
  falls. On a real Vector that is the mode in which it fell off a table.
- A panel with what the robot reports (wheel speeds from encoder counts, head
  angle, lift height, range ahead, cliff sensors) and what only the simulator
  knows (true position, distance to the nearest edge, motor power).
- Beams for the four cliff sensors and the forward range sensor, and the path
  driven.
- The drive train working: each wheel turns by its own angle from the physics
  model, and cleats travel round each track. See "Wheels and tracks" below.
- What the robot's own camera sees, in a window at the bottom right, with the
  outline of its view drawn in the main scene. See "The robot's camera" below.
- The forward range sensor's cone, as a fan of rays, with what the robot makes
  of the reading. See "The range sensor" below.

| Keys | |
|---|---|
| W A S D or arrows | drive; Shift doubles the speed |
| R F, T G | head up and down, lift up and down: moves toward the end stop while held, stops on release |
| 1 2 3 4 | forward 100 mm, back 100 mm, turn left 90°, turn right 90° (SDK actions) |
| 5 6, 7 8 | head to its top and bottom, lift to its top and bottom (SDK actions) |
| Space, X | stop; take control back |
| O | cliff reaction on or off |
| Backspace | reset |
| C V B Q P | follow camera, sensor beams, physics shapes, robot camera, pause |

A standard gamepad works: left stick drives, right stick tilts the head,
bumpers move the lift, A stops, B takes control, X resets, Y switches the
cliff reaction. On a touch screen a four-way pad appears. Every action also
has a button, including head up, level and down and lift up and down.

Open `?robot=2` for Vector 2.0 on the table, or `?env=shelf` for the Vector 2.0
measured shelf configuration (`vector2/vector-free-shelf.xml`, with charger dock,
free cube, shelf navmap, and body-clock ToF enabled by default; adding
`&cubeSensor=unit3` selects unit 3's measured shelf cube profile and `5.083 ms`
supervisor loop). This loads the unit 3 head-camera calibration (800 × 600),
head IMU mounting and accelerometer calibration, display aspect, battery
capacity and shelf scene. Add `&contact=42` to try the 42 mm tread contacts.
The default robot is Vector 1.0. Either generation accepts an explicit shell
override (`?shell=1` or `?shell=2`) for visual comparisons.

By default a movable Light Cube starts 100 mm ahead of the front axle
(`?cube=none` or `?cube=0` omits it; `?cube=free` keeps it explicit). Drive
forward to push it, use Q for the robot camera, and use Backspace to reset.
The range sensor sees the cube in both visual modes. Running `npm run sdk-model`
with a full SDK asset folder installs the textured cube locally; the fallback
shows the shell and rubber contact boxes.

| Query parameter | Values | Effect |
|---|---|---|
| `robot` | `2` | Select Vector 2.0 table parameters and scenes (`vector2/`; default Vector 1.0) |
| `env` | `shelf` | Load Vector 2.0 shelf scene (`vector-free-shelf.xml`), dock, cube, navmap, and body-clock ToF |
| `cube` | `none`, `0`, `free` | Omit or enable the free Light Cube on table scenes (on by default) |
| `dock` | `none`, `0`, `docked` | Omit the charger dock (`none`/`0`) or start seated on the pins (`docked`) |
| `start` | `docked` | Start seated on the charger dock |
| `contact` | `42` | Use the 42 mm tread contact-span scenes (`vector-contact42*`) |
| `cubeSensor` | `unit3` | Apply unit 3's shelf cube profile + `5.083 ms` supervisor loop (implies `env=shelf` if `env` unset) |
| `map` | `live` | Load `shelf-navmap-live.json` instead of `shelf-navmap.json` in `env=shelf` |
| `rangeClock` | `body`, other | Force body-clock ToF acquisition on/off explicitly (shelf turns it on by default) |
| `chargerContact` | `1` | Assert charger electrical contact at boot |
| `firmware` | `off` | Skip loading optional `cube.plain.bin` / `syscon-flash.bin` firmware binaries |
| `wireos` | `1` | Connect live WireOS firmware relay (`WIREOS_BODY=1` dev server) |
| `look` | `shapes`, `sdk-only` | Show physics collision shapes only, or standalone `vector.obj`/`cube.obj` instead of the blended rig |
| `shell` | `1`, `2` | Override visual shell generation independently of `robot=` |
| `faceplate` | `off` | Hide the head faceplate to inspect the internal LCD carrier and camera mount |
| `fork`, `wheels`, `print` | `printed`, `1` | Overlay fetched/exported printable Vector 2 STLs (`fork=printed` requires `robot=2`) |

In the free-cube scene, the cube acceleration readout shows the magnitude of
each of the three ordered slots in the latest simulated stock Bluetooth packet.
The chip filters specific force before sampling. The app keeps the last chip
frame on each 10 ms tick and publishes after three ticks. In the debug console,
`window.__sim.cubeSensor.packet` contains send time, tap count and signed XYZ
counts for all three slots. The default calibration represents a generic cube.
The [stationary shelf comparison](../evidence/unit3-wheelstand/shelf-roll-stop/browser-rest.png)
shows a median of 1.0000 g in the browser and 0.9703 and 0.9704 g in two
recordings of the same physical cube (828 slots each, same resting pose).

## Programs

The **Program** tab, next to Controls, holds a small editor. Write a robot
program in it, press Run (or Ctrl+Enter, ⌘+Enter on a Mac), and it drives the
simulated robot in the page: no Python, no server. Five examples load from
the menu:

1. **Drive a square** (`square`)
2. **Head, lift and wheels** (`look`)
3. **Drive at the edge** (`edge`)
4. **Explore and self-dock** (`explore-dock`)
5. **NoMaD visual explore & navigate** (`nomad-nav`)

In addition to `drive_straight`, `turn_in_place`, `set_head_angle`,
`set_lift_height`, `set_wheel_motors`, `stop_all_motors`, `request_control`,
and `wait`, `robot` exposes `drive_off_charger`, `explore`,
`dock_with_charger`, `nomad_capture_goal`, `nomad_explore`, and
`nomad_navigate` (listed under "Robot calls" in the tab).

```js
const drove = await robot.drive_straight(120, 80);   // mm, mm/s
await robot.turn_in_place(90);                        // degrees, left is positive
await robot.set_head_angle(30);                       // degrees
await robot.set_lift_height(1.0);                     // 0.0 bottom to 1.0 top, as in the SDK
await robot.set_wheel_motors(40, 80);                 // mm/s, returns at once
await wait(2);                                        // seconds of robot time
print(drove.result, drove.moved_mm, robot.state.x_mm);
```

The calls carry the Vector Python SDK's names and units. Each is asynchronous
and must be awaited before the next. An action resolves to `{ result,
moved_mm, turned_deg, time_s }`. `result` is `completed`, `cancelled_cliff`,
`no_control`, `timeout` or `aborted_off_ground`, as from the robot; robot
outcomes are results, not errors. `robot.state` is the state the robot last
reported, every 60 ms. It holds `x_mm`, `y_mm`, `pose_angle_deg`,
`head_angle_deg`, `lift_height_mm`, `prox_distance_mm`, `cliff_detected`,
`has_control`, `now_s` and the other fields of the SDK's robot state. The
full list is under "Robot calls" in the tab. Output from `print` and any error,
with its line, appear under the editor.

**Robot time waits for the program.** The simulation advances only while the
program awaits an action or `wait`; reading the state or setting the wheel
motors takes none. A program therefore gives the same run every time from the
same start, whatever the frame rate or the speed of the machine: the tests run
the square at 1, 7 and 24 control ticks a frame and get the same final state
to the last bit. The real robot
does not wait for a slow program, so a program that computes for a long time
between calls behaves differently there. A program that stops calling the
robot holds robot time still, and the page says so after 3 seconds.

The program runs in a Web Worker, so **Stop** always ends it, a loop that
never awaits included. Stop cancels the running action. A program that ends,
stops or fails stops the treads, as an SDK program does when it lets go of the
robot; a head or lift move still settling is left to finish. While a program
runs, held keys and the action buttons do nothing; Space and the Stop button
stop it, and Backspace stops it and resets the robot. The editor's text is kept
in the browser between visits.

Programs are JavaScript, not Python. The page loads nothing extra for them;
the worker is 2.6 kB, and the tests run the same code under node with no new
package. Python in the page (Pyodide) is the closer match to the SDK, at a
cost: Pyodide 0.28.3's core files are 12.2 MB (8.6 MB of it WebAssembly, 2.7 MB
of that over the wire compressed), to be fetched from a CDN at run time or
shipped with the page, and the tests would need its 13.9 MB npm package.
Interrupting Python without ending its worker needs a cross-origin-isolated
page, which a plain static server does not give, so each Stop would also mean
loading Python again. The robot end (`src/program/host.js`) takes calls by
name from anything that can send it messages, so a Python worker could be
added beside the JavaScript one.

## Wheels and tracks

On the robot, each side has a motor-driven front wheel, a free-spinning rear
wheel and a track round both. The physics model is built the same way: the two
motors act on the front wheel joints only, and each rear wheel is turned by a
coupling that stands for the track (rear angle = front angle x 1.349, the ratio
of the wheel radii). `npm test` checks both: no actuator is attached to a rear
wheel, and with the treads commanded to 80 and 40 mm/s each rear wheel's speed
is within 2% of 1.349 times its front wheel's.

The page shows this. Each of the four wheels turns about its own axle by the
model's angle for that wheel, so the smaller rear wheels visibly turn faster.
Cleats travel round each track by the front wheel's rotation, along the path
of the robot's own track (its wheel radii at its axles). The panel gives the
front (motor) and rear (idler) wheel speeds in rpm. The spokes and cleats are
drawing only; contact with the table is the wheels', as in the Python.

## How it stays the same robot as the Python

`tools/export_model.py` writes the MJCF and the parameter values from
`../simulacrum` into `public/robot/` (and `public/robot/vector2/`), and
records runs of the Python into `test/fixtures/`. `src/sim/` ports
`drive.py`, `hazards.py`, `prox.py`, `imu_filter.py`, `cube.py`, `autonomy.py`,
`nomad*.py`, and `robot.py` (including the firmware lift integral term).
`npm test` (**127** tests: **122** passing, **5** skipped without optional
fetched SDK/firmware binaries) replays those fixtures through the port.

Initially ported from the simulacrum beside it (as of `96c3bfd`), the port
now also tracks subsequent canonical model additions: the BMI160 digital
filter, free Light Cube shell `solref="0.004 1"` contacts and `CubeSensor` /
listener replay, the firmware lift integral term, charger dock and Vector 2.0
shelf scenes, body-clock ToF acquisition, WireOS firmware execution,
occupancy-grid explore/docking, and Goal-Masked NoMaD navigation.

Measured on 2026-11-18, MuJoCo 4.0.1 on both sides (native for the Python,
WebAssembly here), sensor noise set to zero on both:

| Run | Largest difference in position |
|---|---|
| Driving, turning, head and lift on the table, 900 ticks | 3.1e-8 mm |
| Driving to the edge, stopping and backing off, 1500 ticks | 5.6e-5 mm |
| Spinning at a commanded 300 mm/s for 6 s, then stopping, 1400 ticks | 2.4e-8 mm; battery and tread heat equal to 1e-6 |
| A weak, sagging battery (3.6 V at rest, 0.25 V per unit of power), 700 ticks | within the same tolerances |
| Driving off the table at the override level | equal up to the edge; the tumble after it is chaotic and differs |

Sequences of SDK actions are held to a different standard, because the model
is not smooth: the encoder counts whole edges and the firmware keeps speed in
whole mm/s, so there are points where a difference in the last digit changes
what happens next. In the ten-action sequence the first four actions match
the Python to 4e-9 mm with equal tick counts. At the fifth the two differ by
0.004 mm, and after the sixth, a slow 135° turn, by 1.2 mm and 7 ticks.
Nudging the start by one nanometre in the JavaScript alone produces the same
0.004 mm at the fifth action and 1.5 mm and 11 ticks at the sixth. So the test
holds the port to the Python as closely as the port holds to itself under a
nudge of one or two nanometres: exactly where the sequence is insensitive,
loosely only where it is not.

`npm run mutate` tests **83** mutation arms (**81** injected faults and **2**
clean/neutral controls) and confirms every fault fails its target tests while
both neutral controls stay green (**83/83** as required, **0** survived, **0**
setup errors).

`npm run check` (`python3 tools/export_model.py --check`) verifies all **24**
exported XML, JSON, and fixture files against `../parameters.json` and
`../parameters-vector2.json`. `./demo.sh` runs it first.

Differences from the Python, on purpose:

- The Python's actions (`drive_straight` and the rest) block. Here an action is
  started and advances one control tick per `tick()`, so a frame loop can run
  it. `stepAction()` gives the Python's exact stepping, and is what the tests use.
- Sensor noise comes from a different random generator: the same distribution,
  not the same numbers.
- A wheel command is refused while an action is running.

The panel also shows the battery voltage and each tread's heat state. In the
table the battery rests at the reference voltage and does not sag, and only
the right tread loses drive with heat (a fit to one robot's endurance run), so
the voltage stays at 4.03 V and the heat effect is small in a short drive.
Driving is not limited to 220 mm/s: Shift with a turn commands up to 290 mm/s
on one tread, and the motor sets what is reached.

## The robot's camera

Where the sources disagree, the real robot wins over documents, and over
making the picture look tidy.

| | Value used | From | What documents say |
|---|---|---|---|
| Optics | fx 361.16, fy 356.24 px; optical centre 310.66, 183.83 px; 640 x 360 | One real Vector's factory calibration, as the robot reports it through the SDK (`camera_calibration.json`) | |
| Field of view | 83.1° x 53.6° | Follows from that calibration | Manual: 90° x 50°. Anki's simulator: 94° wide. The simulacrum's MJCF: 50° tall, which makes 79.3° wide |
| Position | 18.84 mm ahead of and 7.96 mm below the neck joint | The firmware header (`head_camera_position_mm` in the table); read each frame from the physics model's camera, so it moves with the head | Same in Anki's simulator |
| Tilt | 4° below the head's forward axis | The engine the robot runs (`engine/fullRobotPose.cpp`, `kDefaultHeadCamRotation`; works out to 4.002°) | Same in Anki's simulator. The MJCF and browser both use the engine mounting angle |

The optical centre is not the middle of the image, so the view reaches 40.7°
left and 42.4° right, 27.3° up and 26.3° down. `npm test` checks the position
and direction at three head angles against the table (within 0.2 mm and 0.1°),
that the calibration reproduces the field of view the robot reported with it,
and that the image corners map to the corners of the view.

Limits. The calibration is from one robot and was read by another household
session on or before 2026-11-13; a re-read on 2026-11-18 failed because no
robot was on the network. The tilt is the engine's figure, not a measurement.
There is no lens distortion (the coefficients are in the robot's logs, not in
the SDK message), no exposure and no noise. The robot's own head is not drawn
in its camera view; its lift is, as far as the 3D model of the lift is right.

## The range sensor

The forward time-of-flight sensor is drawn as the fan of 19 rays that samples
its 25 degree cone, from its place 10 mm ahead of the origin and 16 mm up,
tilted up 6.5 degrees. A ray is drawn to what it reached; one that reached
nothing is short and dim. Rays are green when the robot counts an object as
found and amber when something is there but the signal is too weak to count.

The panel shows the distance the robot reports, its verdict by the robot
engine's own rules (object, clear, lift in the beam), and the signal quality.
The model is the Python's (`../simulacrum/prox.py`, described in the main
README): readings clamped to 30..400 mm, the lift in the beam between 34 and
56 mm of lift height, a signal law fitted to 9929 recorded rows from one real
robot, and the weak, scattered distance the real sensor reports when nothing
is there. So on the open table the readout says "clear" while showing a
distance near 69 mm: that is what the robot does.

`npm test` replays 1800 readings of the Python sensor through the port and
runs the robot against a Python run that faces the block, passes the lift
through the beam and turns away.

## The robot's 3D model

The page draws the robot with the model the Vector Python SDK ships for its 3D
viewer (`anki_vector/opengl/assets/vector.obj`), with the head, arms and fork
posed as that viewer poses them. **That model is not in this repository.** It
is under the Anki Image and 3D Model License, which permits use with the SDK
and the robot and is not an open-source licence. `tools/fetch_sdk_model.py`
copies it from an SDK you have installed, or downloads the SDK wheel from PyPI
with `--download` (the wheel is unpacked, not installed), and checks the files
against the hashes of `wirepod_vector_sdk` 0.8.1. The copy lands in
`public/robot/sdk/`, which git ignores.

Without the model the robot is drawn as the physics model's own boxes and
cylinders. With it, press B to see those shapes over the model.

The model is placed and moved by the robot's geometry, not the SDK viewer's.
The file's front wheels sit 2.95 mm ahead of its own origin, so the whole
model is shifted to put its front axle on the robot's origin. With that shift
the file's lift pivot lands 0.8 mm from the robot's lift shoulder and its head
pivot 1.8 mm from the robot's neck joint; before it they were 3.7 and 2.5 mm
away. The head then turns about the robot's neck joint, the arm about the
robot's lift shoulder by the physics joint's angle, the fork pivots on the
wrist at the end of the robot's arm length (around `fork_pivot` for the stock
fork), the lower linkage tracks the SDK shoulder-to-lower-pivot offset, each
wheel turns about the robot's axle for it, and the 42 cleats run round each
track carrier's belt path. Three viewer-pose measurements still come from the
SDK viewer because the robot's table does not publish them: the default drawn
arm angle, the lower-arm offset from the upper pivot, and the stock fork pivot.
The local demo loads Dauler’s stock-pattern outer wheels and replacement lift
fork from the original Thingiverse archives. Each STL retains its millimetre
scale. The front wheels measure 26.0 mm across and the rear wheels 18.5 mm.
Their outside faces sit on the SDK wheel faces, with the clips pointing inward.
Two collision boxes on the wrist approximate the outer fork plates above the physical crossbar. The opening between the plates is clear. The 48.0 mm wide fork stays level on the wrist. Its centre crossbar is aligned
using the firmware offsets and 8.5 mm bar height. These sizes describe printable
replacement parts. Fit to a specific physical robot has not been measured.

The SDK view supports `?faceplate=off` to inspect the LCD carrier, fasteners and camera bracket. The head cover has ten slits. The four microphone ports sit in the outer notches of the U-shaped gold touch plate. A convex indicator lens rises above the floor of its channel. Their profiles use the owner's three close-up photographs. See the [visual and physics checks](../evidence/stock-shell-physics-20261118.md) for the matched stock views and remaining hardware discrepancies.

When `public/robot/sdk/rig/rig_manifest.json` is present alongside `public/robot/sdk/vector.obj`, the viewer defaults to the blended `Victor_rig_01.obj` hardware shell (with animated cleats, symmetric eyes, WireOS LCD plane, rear badge, and `?faceplate=off` internal carrier). By default the browser opens `vector-free-dock.xml` with the Light Cube (`Victor_cube_rig.obj` blended with our 6 fiducial markers and 4 sculpted LED windows) ahead and the charger dock (`Vic_charger_hp_geo.obj`) behind (`?cube=none` / `?dock=none` omit them; `?dock=docked` or **Seat on charger** starts on the pins; `?look=sdk-only` forces standalone `vector.obj`/`cube.obj`; `?fork=printed` / `?wheels=printed` / `?print=1` overlay printed Vector 2 STLs; see `public/robot/sdk/DDL-ASSET-LICENSE.txt` for the Digital Dream Labs asset terms covering the rig, dock, and Light Cube meshes).

The shell and ear profiles are visual estimates
from the [iFixit teardown photographs](https://www.ifixit.com/Guide/Anki+Vector+Capacitive+Touch+Sensor+Replacement/123280).
The lens uses the camera mount from the parameter table. The robot uses its SDK
wheel or fork wherever a print part is unavailable. To refresh the local print
meshes, run `python3 tools/fetch_print_parts.py`. To populate the blended rig,
charger dock, and Light Cube meshes into `public/robot/sdk/`, run
`python3 tools/fetch_rig_model.py`, `python3 tools/fetch_dock_model.py`, and
`python3 tools/fetch_cube_model.py --download` (which include `DDL-ASSET-LICENSE.txt`).
The printed-part download includes the creator’s attribution and noncommercial
licence alongside each set. For a local Docker build add `--build-arg WITH_SDK_MODEL=1`,
`--build-arg WITH_RIG_MODEL=1`, `--build-arg WITH_DOCK_MODEL=1`,
`--build-arg WITH_CUBE_MODEL=1`, and `--build-arg WITH_PRINT_PARTS=1`.

A built bundle (`dist/`) or a container image that includes the model carries
that licence with it. Read `public/robot/sdk/LICENSE.txt` before publishing one.

## Not done

- The `rsl_rl` state-space locomotion PPO policy (`../simulacrum/task.py`, `../simulacrum/onnx_policy.py`) is not ported to the browser; instead the browser runs Goal-Masked NoMaD visual diffusion (`src/sim/nomad.js`, `src/sim/nomad_cam.js`).
- The `Dockerfile` has not been built: the machine this was written on had no
  container engine running.
- Checked in one browser engine (Chromium), at a desktop size and at 375 x 812.
  Keyboard and on-screen buttons were exercised; a gamepad and a real touch
  screen were not.
- Real-time rate was 1.0 when the page was visible. In a background tab the
  browser slows the frame loop and the simulation slows with it.
- The pose shown is the simulator's true pose, as in the Python. The real
  robot reports an estimate from odometry.
- Programs were run in Chromium only, in a built page served by
  `python3 -m http.server`. An error's line number comes from the engine
  naming the program in its stack trace; an engine that does not gives the
  error without a line. A syntax error has no line in Chromium.
- Programs are JavaScript; Python in the page is not done (see "Programs").

## Files

| | |
|---|---|
| `src/sim/` | The port: `drive.js`, `hazards.js`, `prox.js`, `imu_filter.js`, `robot.js`, `util.js`; `optics.js` (camera geometry); `thumb.js`, `cube_firmware.js`, `syscon_firmware.js`, `wire_body.js` (real firmware execution & body relay); `cube_sensor.js`, `cube_listeners.js`, `shelf_cube_pose.js`, `shelf_cube.json` (Light Cube BMA253 + tap/movement listeners); `autonomy.js` (optical marker localization, `NavMapGrid`, explore & self-dock); and `nomad.js`, `nomad_cam.js` (pure Float32 Goal-Masked NoMaD Transformer + FiLM 1D U-Net + DDIM sampler, `TopologicalGraph`) |
| `public/robot/` | Exported Vector 1.0 (`vector*.xml`, `model.json`) and Vector 2.0 (`vector2/vector*.xml`, `vector2/model.json`, `vector2/shelf-navmap.json`) MuJoCo scenes |
| `public/robot/nomad/` | Pretrained Float32 weights (`nomad_weights.bin`, `nomad.json`) and ONNX graphs (`nomad_encoder.onnx`, `nomad_denoiser.onnx`) exported by `train_nomad.py` |
| `src/game/` | `game.js` (boot, control loop, views), `scene.js` (drawing), `sdkmodel.js` (SDK model), `rigmodel.js` (blended `Victor_rig_01` rig), `dock.js` (charger dock OBJ & decals), `cube.js`, `cube-marks/` (blended `Victor_cube_rig` & fiducials), `face.js`, `wire_face.js`, `headcam.js`, `wire_camera.js` (robot camera & WireOS bridge), `tracks.js` (track path, cleats, spokes), `liftlinks.js`, `printparts.js`, `shell.js`, `topology.js`, `underbody.js`, `input.js` |
| `camera_calibration*.json` | Measured factory camera calibrations (`camera_calibration.json` for Vector 1.0, `camera_calibration_vector2.json` for Vector 2.0) |
| `src/program/` | Programs written in the page: `host.js` (the robot end, calls against the simulator in robot time), `runtime.js` (the robot object, running a program, line numbers), `worker.js`, `runner.js` (the page end), `examples.js`. All but `worker.js` run under node |
| `src/ui/`, `src/store.js` | The panels (`Hud.jsx`, `BootOverlay.jsx`, `MiniMap.jsx`, `ProgramPanel.jsx`) and the store between them and the core |
| `tools/` | Exporters, fetchers & relays: `export_model.py` (`--check` for drift), `export_printed_fork.py`, `cube-landing-fixture.py`; `fetch_sdk_model.py`, `fetch_rig_model.py`, `fetch_dock_model.py`, `fetch_cube_model.py`, `fetch_firmware.py`, `fetch_print_parts.py`; JS replay tools `cube-rest.mjs`, `cube-range-replay.mjs`, `replay-cube-listeners.mjs`; WireOS relay `wire_relay.js`, `wire_camera.js`, `run-wire-robot.sh`, `run-wire-stack.sh` |
| `test/` | Node tests (`29` `*.test.js` files, `127` tests), `mutate.mjs` (`83` arms), and Python parity fixtures |
| `demo.sh`, `Dockerfile` | One command to a running page; a static container on port 8080 (`WITH_FIRMWARE=1` bundles real firmware) |

### Standalone real firmware & autonomous explore / self-docking

Run `python3 tools/fetch_firmware.py --download` to fetch unmodified Dialog DA14580 Light Cube firmware (`cube.dfu` decrypted in memory -> `cube.plain.bin`) and STM32F030 body syscon flash (`sysboot.bin` + `syscon.dfu` -> `syscon-flash.bin`) into `public/robot/firmware/` (gitignored; `?firmware=off` skips loading). In-browser execution runs unmodified `cube.plain.bin` inside an ARMv6-M Thumb VM (`src/sim/thumb.js` + `src/sim/cube_firmware.js`) and packs/verifies CRC-32 `BodyToHead` spine frames (`src/sim/syscon_firmware.js`).

Use the **Explore**, **Dock**, and **Undock** HUD buttons — or `await robot.explore(seconds)`, `await robot.dock_with_charger()`, and `await robot.drive_off_charger()` in the **Program** tab (see the `"Explore and self-dock"` example) — to build a 20 mm `NavMapGrid` (rendered live on the MiniMap), detect charger and cube optical markers through the calibrated pinhole camera, and self-dock onto the charger pins (`4.20 V`). When connected to a live WireOS simulator via `?wireos=1` and `WIREOS_BODY=1`, `wire_relay.js` forwards **Explore / Dock / Undock** intents to `vic-webserver` (`run-wire-stack.sh` also launches `vic-webserver` when present).

### NoMaD visual exploration & navigation

Goal-Masked Diffusion Policies (Sridhar et al., ICRA 2024) run natively in the browser in Float32 JavaScript (`src/sim/nomad.js`, `src/sim/nomad_cam.js`) without PyTorch or external ML runtime dependencies, matching the PyTorch/ONNX training export (`public/robot/nomad/nomad_weights.bin`) to `< 1e-4`. At boot the page loads `./robot/nomad/nomad.json` and `nomad_weights.bin`, renders `64x36` live and goal thumbnails in the right panel, and overlays the `TopologicalGraph` nodes/edges and `4` DDIM candidate waypoint trajectories on the MiniMap.

Use **Capture goal**, **NoMaD explore**, and **NoMaD navigate** in Controls — or `await robot.nomad_capture_goal(label?)`, `await robot.nomad_explore(seconds?)`, and `await robot.nomad_navigate(max_seconds?, goal_id?)` in **Program** (see `"NoMaD visual explore & navigate"`). With `goal_mask = 1`, the Transformer encoder ignores `goal_img` (`< 1e-6` invariance) and expands the topological memory graph safely within cliff and forward-ToF bounds; with `goal_mask = 0`, Dijkstra selects the intermediate topological subgoal and DDIM steers toward the goal view.

To use the measured shelf cube with the free scene, open
`?cube=free&cubeSensor=unit3`. This selects its chip scale, noise and word format
from the Vector 2 parameter table. The robot model is selected separately.
The stationary median is 0.97083 g with the shelf profile and 1.00002 g with
the generic chip. Two physical recordings gave 0.97034 and 0.97038 g.
The timestamps exposed by the simulator are cube send times. The hardware
recordings include Bluetooth delivery and collector timing.
[Stationary distributions](../evidence/unit3-wheelstand/shelf-roll-stop/browser-rest-unit3.png)

The free-cube HUD also shows the upward axis inferred from acceleration and
the movement listener state. The latest packet's `events` lists transitions
in slot order with the packet send time. These are engine listener candidates
before SDK broadcast suppression during carrying or docking.

### Unit 3 printed fork contact

For the local noncommercial Vector 2 demo, run these from the web directory. The exporter uses the same STL registration, convex decomposition and seed as the five physical cube push replays. It retains the wrist mass and joint and replaces the three contact boxes. The exact print installed on unit 3 remains unconfirmed.

```sh
python3 tools/fetch_print_parts.py
uv run --python 3.12 --with numpy --with coacd==1.0.14 python tools/export_printed_fork.py
```

Open `/?robot=2&fork=printed&cube=free&contact=42` in the local simulator. Toggle the collision overlay to inspect the fork opening. Rerun the exporter after changing the base scenes. The locally generated scenes contain geometry derived from Dauler's forks7.stl and carry the source part's usage terms in `public/robot/sdk/print/Anki_Vector_Lifting_Forks-LICENSE.txt`. [Physical trial results](../evidence/unit3-printed-fork-contact/README.md)

### Body-clock range acquisition

Add `rangeClock=body` to the simulator URL to acquire range on the published syscon timer. Add `chargerContact=1` for a replay with charger contact asserted. This input represents the electrical contact signal only. The simulation starts with a fresh range sample. Its 5 ms control step observes subsequent acquisitions on the first step at or after the body deadline. Each SDK state uses the held distance and quality while evaluating lift visibility and pitch at publication. Tread power above 10% selects the six-tick interval even when charger contact is asserted. The stationary off-dock and on-dock intervals are 12 and 200 ticks. Hardware comparison covers the steady docked period. Acquisition-to-report latency and the two faster periods are still uncalibrated. [Recorded clock comparison](../evidence/unit3-cube-range/README.md)

The measured shelf scene uses the body clock by default; any other `rangeClock` value turns it off.

Python replays select the same acquisition clock with `VectorSim(body_range_clock=True)` (the default for `environment="p2_shelf"`). Set `sim.charger_contact` from the recorded contact state. For a policy environment, `TaskConfig(body_range_clock=True)` selects the body clock across episode resets.
