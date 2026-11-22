# Vector simulacrum in the browser

The simulacrum one directory up, running in a web page: the same MuJoCo model,
the same firmware tread control law and the same cliff reaction, with a 3D view
you can drive. MuJoCo runs in the page as WebAssembly. There is no server and
nothing here connects to a robot.

The structure follows the
[MicroDuck simulator](https://huggingface.co/spaces/pollen-robotics/microduck-simulator)
(pollen-robotics, Apache-2.0): a Vite and React shell around an imperative
core, a small store between them. No code or assets were copied from it.

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

- The robot on the simulacrum's 600 mm table, with one block on it. The table
  has real edges and a floor 300 mm below.
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

Add `?look=shapes` to the address to draw the robot as its physics shapes even
when the SDK model has been fetched.

Open `?cube=free` to put a movable light cube 100 mm ahead of the front axle.
Drive forward to push it, use Q for the robot camera, and use Backspace to reset.
The range sensor sees the cube in both visual modes. Running `npm run sdk-model`
with a full SDK asset folder installs the textured cube locally. The fallback
shows the shell and rubber contact boxes. Add `&contact=42` for the experimental
42 mm contact span.

In the free-cube scene, the cube acceleration readout shows the magnitude of
each of the three ordered slots in the latest simulated stock Bluetooth packet.
The chip filters specific force before sampling. The app keeps the last chip
frame on each 10 ms tick and publishes after three ticks. In the debug console,
`window.__sim.cubeSensor.packet` contains send time, tap count and signed XYZ
counts for all three slots. The default calibration represents a generic cube.
The [stationary shelf comparison](../evidence/unit3-wheelstand/shelf-roll-stop/browser-rest.png)
shows a median of 1.0000 g in the browser and 0.9703 and 0.9704 g in two
recordings of the same physical cube (828 slots each, same resting pose).

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
`../simulacrum` into `public/robot/`, and records runs of the Python into
`test/fixtures/`. `src/sim/` is a port of `drive.py`, `hazards.py` and
`robot.py`. `npm test` replays the fixtures through the port.

Ported from the simulacrum beside it (as of the range sensor change that
followed `96c3bfd`), using its base parameter set (the one
it calls `vector_1`; the recorded robot is a Vector 1.0, on its
word), which
includes head and lift acceleration, battery voltage and tread heating in the
drive, and direct drive above the path-following speed limit.

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

`npm run mutate` breaks each tested property once (41 faults) and confirms the
tests fail; the last run caught 41 of 41, and both clean copies passed.

`npm run check` fails when `../parameters.json` has changed and the export has
not been re-run. `./demo.sh` runs it first.

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

The shell and ear profiles are visual estimates
from the [iFixit teardown photographs](https://www.ifixit.com/Guide/Anki+Vector+Capacitive+Touch+Sensor+Replacement/123280).
The lens uses the camera mount from the parameter table. The robot uses its SDK
wheel or fork wherever a print part is unavailable. To refresh the local print
meshes, run `python3 tools/fetch_print_parts.py`. The download includes the
creator’s attribution and noncommercial licence alongside each set. For a local
Docker build add both `--build-arg WITH_SDK_MODEL=1` and `--build-arg WITH_PRINT_PARTS=1`.

A built bundle (`dist/`) or a container image that includes the model carries
that licence with it. Read `public/robot/sdk/LICENSE.txt` before publishing one.

## Not done

- No trained policy runs in the page yet. The simulacrum exports policies as
  ONNX (`../simulacrum/onnx_policy.py`); the observation and action layout of
  `../simulacrum/task.py` is not ported.
- The `Dockerfile` has not been built: the machine this was written on had no
  container engine running.
- Checked in one browser engine (Chromium), at a desktop size and at 375 x 812.
  Keyboard and on-screen buttons were exercised; a gamepad and a real touch
  screen were not.
- Real-time rate was 1.0 when the page was visible. In a background tab the
  browser slows the frame loop and the simulation slows with it.
- The pose shown is the simulator's true pose, as in the Python. The real
  robot reports an estimate from odometry.

## Files

| | |
|---|---|
| `src/sim/` | The port: `drive.js`, `hazards.js`, `prox.js`, `robot.js`, `util.js`; and `optics.js` (camera geometry). No browser APIs; runs under node |
| `src/game/` | `game.js` (boot, control loop, views), `scene.js` (drawing), `sdkmodel.js` (the SDK model), `headcam.js` (the robot's camera), `tracks.js` (track path, cleats, spokes), `input.js` |
| `camera_calibration.json` | One real robot's camera calibration |
| `src/ui/`, `src/store.js` | The panels and the store between them and the core |
| `tools/export_model.py` | Model, parameter values and fixtures from the Python; `--check` for drift |
| `tools/fetch_sdk_model.py` | The SDK's robot model, into `public/robot/sdk/` |
| `test/` | Ten test files (53 tests), `mutate.mjs`, the fixtures |
| `demo.sh`, `Dockerfile` | One command to a running page; a static container on port 8080 |

To use the measured shelf cube with the free scene, open
`?cube=free&cubeSensor=p2y8`. This selects its chip scale, noise and word format
from the Vector 2 parameter table. The robot model is selected separately.
The stationary median is 0.97083 g with the shelf profile and 1.00002 g with
the generic chip. Two physical recordings gave 0.97034 and 0.97038 g.
The timestamps exposed by the simulator are cube send times. The hardware
recordings include Bluetooth delivery and collector timing.
[Stationary distributions](../evidence/unit3-wheelstand/shelf-roll-stop/browser-rest-p2y8.png)
