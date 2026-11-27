---
name: vector-simulacrum
description: A MuJoCo model of the Anki/DDL Vector robot for training policies that must transfer to the real robot. Every physical number comes from one table that says where it came from; an SDK-shaped layer drives the model in the robot's own units, with the firmware's own tread control law, and models its cliff reaction. Replayed against one recording of the real robot and fitted to part of it.
---

# vector-simulacrum

GitHub `main` is the canonical simulator source. The [public browser simulator](https://p1r4n351-vector-simulacrum.static.hf.space/index.html) serves a build of `web/`. Changes to the browser, firmware bridge and physical model belong in this repository.

[Physics overlay gallery](evidence/physics-plots/README.md) · [Held-out encoder/gyro timing test](evidence/hw-rec-20261119/unit2-ccw100.md#encoder-to-gyro-timing-test)

This is the physics model that `skills/vector-sim` (the Go gRPC simulator,
branch `cpcs/vector-cliff-guard`) says it is not. It does not replace that
simulator and is not connected to it.

**The initial fit compared one recording of one real Vector, tuned to part of it**
(one Vector 1.0, 162 s run on one surface, one battery charge from 4.10 V to
3.96 V, wheel speeds up to 100 mm/s). Subsequent recordings and fixtures under
`evidence/` extend verification across both Vector 1.0 (unit 1) and Vector 2.0
(unit 3): endurance, spins, camera tracking, head/lift moves, wheelstands,
dock seating/returns, and free/carried light-cube pickup and mount impacts.

In `parameters.json` (Vector 1.0), of **198** parameters: **10** are measured on
recordings, **19** are fitted to them, **17** come from the manual, **104** from
vendor or household software, **39** are assumed about the robot or cube, and
**9** are settings of the scene, solver, and tip supports that have no real
value to find (plus **18** open measurements in `missing`).
`parameters-vector2.json` overrides **16** entries and inherits **26**
measured/fitted entries for Vector 2.0 (**11** `measured`, **23** `fitted`,
**14** `manual`, **105** `sdk`, **36** `assumed`, **9** `chosen`). On the
Nov 15 segments the fit never saw, wheel-speed and gyro error fell from 0.33 of
each segment's scale (the version pushed before any recording existed) to 0.14.
Mass, tread friction and centre of mass are still assumed: the first recording
could not tell their values apart. The fore-aft centre of mass is the assumed
value the policy task is most sensitive to, by a wide margin ("The assumed
values, ranked" below).

## What is here

| File | What it is |
|---|---|
| `parameters.json` | The single source of every physical number for Vector 1.0, with provenance |
| `parameters-vector2.json` | Vector 2.0 parameter overrides (16 overrides, 26 inherited measured/fitted entries over `parameters.json`) |
| `simulacrum/params.py` | Loads and checks the tables (`vector_1` / `vector_2`); draws randomised robots from a seed |
| `simulacrum/mjcf.py` | Generates the MJCF (robot, table/shelf scene, free/carried cube, charger dock) from the table |
| `simulacrum/body_mesh.py` | The robot's real outer shape from Anki's animation rig, fetched at a pinned hash and cut into convex collision pieces and visual meshes; `VectorSim(body=...)` uses it. See [evidence/body-shape](evidence/body-shape/README.md) |
| `simulacrum/drive.py` | The firmware's tread encoder arithmetic and speed control law |
| `simulacrum/imu_filter.py` | BMI160 two-stage digital low-pass filter stepped at the 1 ms physics step |
| `simulacrum/robot.py` | `VectorSim`: SDK-shaped commands and state, sensor noise, cliff reaction, lift integral term, and body-clock range option |
| `simulacrum/hazards.py` | Cliff, falling and picked-up flags |
| `simulacrum/prox.py` | The forward time-of-flight sensor: its cone, signal quality, and the robot's reporting rules. No MuJoCo |
| `simulacrum/prox_fit.py` | Recomputes that sensor's fitted values from the recordings. No MuJoCo |
| `simulacrum/cube.py` | Light Cube BMA253 accelerometer, FIFO, tap detector, and shipped/mean firmware apps (`cube="free"` / `"lift"`) |
| `simulacrum/dock.py`, `simulacrum/autonomy.py` | Charger dock geometry/seating and optical-marker `NavMapGrid` explore and self-docking |
| `simulacrum/printed_parts.py` | Printable replacement fork, rims and treads loaded by mesh hash (`evidence/printed-parts/`) |
| `simulacrum/recording.py` | Reads the recording of the real robot and measures it. No MuJoCo |
| `simulacrum/imu_fit.py` | Fits the accelerometer's steady offsets, x scale and the pitch of its frame from recordings. No MuJoCo |
| `simulacrum/motor_heat_fit.py`, `simulacrum/spin_fit.py`, `simulacrum/camera_spins.py` | Tread heating fit, contact inset fit against held spins, and camera spin-centre circle fit |
| `simulacrum/wheelstand.py`, `wheelstand_check.py` | The centre of mass from balance angles on the rear wheels and on one tread edge, read by the robot's own accelerometer; the check that the model tips where the formula says (MuJoCo) |
| `simulacrum/replay.py` | Replays the recording's commands through the model and scores the difference |
| `simulacrum/task.py` | The training task: observation, action, reward, episode ends, randomisation |
| `simulacrum/vec_env.py` | Many tasks stepped together, in up to 4 worker processes |
| `simulacrum/training.py` | PPO with `rsl_rl`, ONNX export, the parity recording |
| `simulacrum/onnx_policy.py` | Runs an exported policy with `onnxruntime` only |
| `simulacrum/nomad_cam.py`, `nomad_model.py`, `nomad_onnx.py`, `nomad.py` | Goal-Masked NoMaD visual diffusion policy (`64x36` calibrated `head_cam` renderer, Transformer + FiLM 1D U-Net + DDIM sampler, `TopologicalGraph`, closed-loop explorer/navigator) |
| `cube_firmware/` | Cortex-M0 Thumb mean-per-tick cube app (`tick_avg.s`, `tick_avg.c`), XXTEA DFU tool (`dfu.py`), ARMv6-M interpreter (`thumb.py`), and `mutate_tick.py` |
| `hardware/` | Host-side recording and capture tools (`record_barriered.py`, `record_listen.py`, `hold_cube.py`, `cube_accel.py`, `capture_peers.py`, `save_capture.py`, `board_echo.py`, `play_board_probe.py`). **Nothing here connects to or runs against a robot automatically** |
| `web/` | Browser MuJoCo 4.0.1 WebAssembly + Three.js simulator, Program tab, WireOS relay, NoMaD visual navigation, two-robot righting rescue macro, Node tests, and mutation suite (`web/README.md`) |
| `contrib/` | Contributor recording kit (`contrib/kit/`), privacy scrub, schema validator (`contrib/validator.py`), and `contrib/UNITS.md` |
| `replay_recording.py`, `fit_recording.py`, `fit_spins.py` | Replay scoring, coordinate parameter fit, and spin-inset fit (print values, change no file) |
| `train.py`, `train_nomad.py`, `run_policy.py`, `run_nomad.py`, `evaluate.py`, `smoke.py` | Train and export PPO / NoMaD policies; run without torch; compare baselines |
| `rank_assumed.py` | Moves each assumed or chosen value to its bounds and measures what that does to the training task |
| `build_mjcf.py` | Writes the generated MJCF to a file |
| `demo.py`, `measure.py` | A short run; the model figures quoted below |
| `evidence/hw-rec-20261115/` | The initial recording, the script that made it, SHA-256 in the table's `REC` source |
| `evidence/` | Smoke run learning curves and outputs; fit, replay, measurement, ranking, cube pickup, wheelstand, dock, and fault-check evidence |
| `tests/` | `run_all.py` (36 test modules, 485 declared tests), `test_cube_pickup.py` (10 standalone pickup tests; 37 `test_*.py` files total), `mutate.py` (273 fault arms + 1 neutral control) |
| `run_in_ct.sh` | Copies this directory into a container host and runs a file there, on MuJoCo 4.0.1 |
| `env/`, `pinned_run.py`, `simulacrum/mujoco_pin.py` | The pinned MuJoCo 4.0.1 environment (hashed wheels, setup script) and the check that a run used it |
| `mcp_server.py`, `simulacrum/mcp_service.py` | An MCP server over stdio: the table, the recordings and the recorder's command line as tools. Read-only; no robot, no MuJoCo |

## Use

```bash
./run_in_ct.sh tests/run_all.py --require-mujoco   # all tests, in a GPU host
./run_in_ct.sh tests/mutate.py                      # break each property once
./run_in_ct.sh replay_recording.py --sensitivity    # the model against the real robot
./run_in_ct.sh fit_recording.py                     # refit (4 workers; prints values, changes no file)
./run_in_ct.sh measure.py                           # the model figures in this file
./run_in_ct.sh smoke.py                             # train, export, check without torch, compare
./run_in_ct.sh train_nomad.py --out /root/nomad-out # train Goal-Masked NoMaD and export ONNX + web weights
./run_in_ct.sh run_nomad.py --model-dir /root/nomad-out # run torch-free NoMaD ONNX explore + navigate
./run_in_ct.sh rank_assumed.py --policy /root/vector-runs/smoke3/policy.onnx   # what each assumed value does (about 25 minutes)
python3 -m simulacrum.recording                     # anywhere: what the recording itself says
python3 -m simulacrum.imu_fit                       # anywhere with numpy: the accelerometer's offsets
python3 tests/run_all.py                            # anywhere: the tests that need no MuJoCo
python3 build_mjcf.py --out vector.xml              # the model as a file
```

```python
from simulacrum.robot import VectorSim, CONTROL_DEFAULT, CONTROL_OVERRIDE

sim = VectorSim(control_level=CONTROL_DEFAULT)      # cliff reaction on
sim.drive_straight(200.0, 100.0)                    # mm, mm/s
sim.turn_in_place(90.0)                             # degrees, counter-clockwise positive
sim.set_wheel_motors(60.0, 40.0)                    # mm/s; returns at once
state = sim.advance(1.0)                            # seconds of robot time
state.cliff_detected, state.picked_up, state.falling, state.prox_distance_mm
sim.sim_truth()["fallen"]                           # what the robot itself cannot report

VectorSim.randomised(seed=7)                        # ranged parameters drawn inside their ranges
```

Nothing in this directory connects to or commands a robot. The recording was
made by another session (`evidence/hw-rec-20261115/record_table.py`).

Keep to at most four worker processes and short runs in a GPU host: the node is
a shared hypervisor. One process runs about 9 times real time. Do not start
a second `run_in_ct.sh` while one is running: each run replaces the copy in
the container.

### Which MuJoCo: 4.0.1

Fits, tests and training use MuJoCo 4.0.1, the version the browser build
(`web/`, `@mujoco/mujoco` 4.0.1) runs. The GPU host's own environment has
3.11.0, and until 2026-11-22 the fits and the suite ran there while the
browser and the macOS checks ran 4.0.1. The two are different models.
Measured on this tree (inset 3.1 mm) on the same Linux host, sensor noise off
where it says so:

| | 3.11.0 | 4.0.1 |
|---|---|---|
| Wheel replay, fit / held out | 0.1291 / 0.1284 | 0.1260 / 0.1278 |
| Head and lift replay, fit / held out | 0.0325 / 0.0355 | 0.0325 / 0.0355 |
| Contact points under each wheel at rest | 5 | 2 |
| Resting accelerometer, noise off, spread of 50 messages before the first move | up to 0.074 mg (largest 0.17 mg) | 0.000 mg |
| `web/test/fixtures/robot.json` remade on Linux, against the committed file | up to 28.6 mm apart; `npm test` fails 8 of 63 | within 1.2e-7 mm, except the tumble off the table (23.6 mm); `npm test` passes |
| Unit 1's run E spins, fitted inset (pull request #110) | 3.105 mm | 3.242 mm |

Against the real robot the two tie: the replay scores above differ by
less than one refit moves them. 4.0.1 is the reference because it is the
model the browser shows and the fixtures pin, and because Linux and macOS
agree on it. The tumble off the table is chaotic on every build, so
`test_sim.Cliff.test_reaction_off_the_robot_drives_off_and_falls` checks the
drop itself (off the edge, down to the floor, a free fall the detector
sees) and `tests/test_hazards.py` checks the fall flag's rule on known
traces.

How it is pinned:

- `env/requirements-mujoco-401.txt`: mujoco, mujoco-core, numpy, glfw and
  pyopengl at exact versions with sha256 hashes for Linux x86_64 and macOS
  arm64, CPython 3.11 and 3.12. Installed with `--require-hashes --no-deps`.
- `env/setup_mujoco_401.sh` builds the environment (default
  `/root/vector-mujoco-401`) from that file. With `--borrow` (or
  `VECTOR_TRAINING_SITE`) torch, rsl_rl, tensordict, onnx and onnxruntime
  are read from the GPU host's own environment, read-only:
  `env/borrowed-training.json` pins each one's version and the hash of its
  installed-file list, and the setup refuses to link them if either differs.
  It never deletes or rebuilds an existing environment; one that does not
  pass is reported and left alone.
- `run_in_ct.sh` runs that setup (a check, once built) and then the file,
  through `pinned_run.py`. At the end of every run `simulacrum/mujoco_pin.py`
  reads the libmujoco actually mapped into the process (from
  `/proc/self/maps`, or the dynamic loader on macOS), not only the version
  the bindings report, and the run exits 3 if it is not 4.0.1.
  `tests/run_all.py --require-mujoco` makes the same check itself.
  `CAULDRON_PYTHON` names the container's own Python command; it supplies
  the MUJOCO_* environment both interpreters run with.
- `VECTOR_MUJOCO=container ./run_in_ct.sh ...` runs on the container's own
  3.11.0 instead, with `VECTOR_MUJOCO_EXPECT=3.11.0` so the check expects that.

The GPU host's environment stays on 3.11.0 because mjlab pins it (see "Why
not mjlab's task classes", item 7). Its `mujoco/` directory also holds
`mujoco-core` 4.0.1, pulled in by `mjviser`: today every file the two
disagree on is 3.11.0's and only `libmujoco.so.3.11.0` loads, but a reinstall
of `mujoco-core` would overwrite 3.11.0's bindings. The check above would say so.

The fitted entries that come from replaying the recordings through MuJoCo
were re-checked on 4.0.1 on 2026-11-22, with the same tools on both versions.
None was refitted:

| Entry, range | Table | 3.11.0 refit | 4.0.1 refit | Scores (fit / held out) |
|---|---|---|---|---|
| `command_delay_ms`, 40-120 | 75 | 80 | 80 | `fit_recording.py --wheels-held-noload`, both versions: the table 0.129 / 0.128 (3.11.0) and 0.126 / 0.128 (4.0.1); refitted 0.120 / 0.132 and 0.117 / 0.133. Held out gets worse on both |
| `tread_friction_power_fraction`, 0.1-0.2 | 0.1573 | 0.1730 | 0.1730 | as above |
| `wheel_stall_torque_mNm`, 50-200 | 156.25 | 195.3 | 250 | as above. The only value that leaves its range, on 4.0.1. The recording fixes the motor's strength relative to the load, not the torque itself (the entry's own note), and the refit loses on the held-out segments, so the table keeps 156.25 |
| `wheel_armature_gcm2`, 1000-4000 | 2380 | 2332 | 2380 | as above |
| `tread_noload_speed_mmps`, 280-320 | 305 | - | - | Held, not searched. Run F's first full-power spin each way: model 241.4 / 241.3 mm/s on 3.11.0, 240.8 / 240.8 on 4.0.1, robot 247.1 / 243.3 |
| `servo_command_delay_ms`, 80-130 | 105 | 105 | 105 | `fit_recording.py --servos`: 0.033 / 0.035 on both; run A's 15 moves 0.031 on both |
| `head_speed_max_radps`, 3.5-10 | 10 | 10 | 10 | as above |
| `lift_speed_max_radps`, 6-10 | 10 | 10 | 10 | as above |
| `tread_contact_inset_mm`, 2-4 | 3.1 | 3.105 | 3.242 | Pull request #110 (`evidence/inset-portability/`), which keeps 3.10 mm and checks the live fit against the range on each build |

The tread values move on 3.11.0 as well, so the table is no longer at the
optimum of `fit_recording.py` on this tree whichever version runs it; that
is the model having changed since the fit, not the version.

## The range sensor

The forward time-of-flight sensor follows the real robot in three ways the
single ray it replaced did not.

**It sees a cone.** 25 degrees across (the firmware's own figure), sampled by
a fan of 19 rays: one on the axis, 6 at half the half-angle, 12 at 0.9 of it.
The rays are cast at the scene only. The robot's own lift is a box in this
model and would block the sensor where the real fork does not, so the lift is
handled by the robot's own rule instead (below).

**It reports what the robot's engine reports** (`proxSensorComponent.cpp`,
source `ENGPROX`):

| | Rule | Recordings |
|---|---|---|
| Distance | Clamped to 30..400 mm | Lowest 30, highest 400 in 9929 rows |
| Lift in view | Lift height between 34 and 56 mm, and for 90 ms after it leaves | 111 of the 205 rows in that window read 30 mm, with signal quality about 12 |
| Unobstructed | Signal quality at or below 0.01, or distance at 400 | 4713 of 9929 rows have quality at or below 0.01 |
| Found object | Not unobstructed, lift not in view, pitch within 5 degrees | |

The state message carries `prox_distance_mm`, `prox_signal_quality`,
`prox_unobstructed`, `prox_found_object` and `prox_is_lift_in_fov`.

**Its signal is fitted to the recordings** (`python3 -m simulacrum.prox_fit`).
Signal quality falls with distance as a power law: 0.139 at 100 mm, exponent
-2.71, from the 4776 rows with a usable signal. Quality is the mean over the
fan, so a target that fills half the cone returns half the signal, and the
distance is the signal-weighted mean of the hits. With that law a target
filling the cone drops to the 0.01 threshold at about 264 mm, which agrees
with the manual's "max useful range closer to 300 mm".

With no usable signal the real sensor still reports a distance: median 69 mm,
quartiles 56 and 86, quality about 0.0034. The model reports the same, because
a policy trained here will meet it on the robot. On an open table the fan's
three lowest rays reach the table top between 193 and 281 mm; that is too
little signal to count, so the open table reads as clear, as it does on the
robot.

Limits. One robot, and whatever stood in front of it: the scatter about the
signal law is more than a factor of ten. What produces the weak readings'
distance is not established. The weak distance is drawn from a normal
distribution, which leaves out the real tails (6% of weak rows read 30, 3%
read 400). `task.py` still scales the distance by the sensor's 1200 mm range,
not by the 400 mm the robot reports.

## The MCP server

Any MCP client that starts a stdio server can run it: the command is a
Python 3.9 or later and the argument is the path to `mcp_server.py`. No MCP
library, no MuJoCo, no `anki_vector`, nothing to install. The generic entry,
for clients configured with JSON:

```json
{"mcpServers": {"vector-simulacrum": {"command": "python3", "args": ["<skill>/mcp_server.py"]}}}
```

On Windows the command is usually `py` with `["-3", "<skill>\\mcp_server.py"]`.
With Claude Code:

```bash
claude mcp add --scope user vector-simulacrum -- python3 <skill>/mcp_server.py
```

Messages are one JSON-RPC object per line, UTF-8 with a bare newline on
every platform (the server switches its streams off the platform's text
mode); log lines go to stderr. Tools are added as the work needs them;
`tools/list` shows only what is wired and tested (`tests/test_mcp.py`).

Run on: macOS with Python 3.14 and with the stock Python 3.9, and Linux in
a GPU host. Not run on Windows; the newline and encoding handling is written
for it but has not been seen working there.

| Tool | What it returns |
|---|---|
| `parameters` | Table entries, filtered by provenance and by part of the name, with range and source |
| `recording_summary` | For one recording: rows, the recorder's begin and end rows and any skip or failure, cliff rows, state interval, each spin's turn fraction, sensor scatter in spins, the gyro frame fit, the gyro at rest. A figure the recording cannot support comes back as `not_computable` with the reason |
| `spin_fractions` | Turn rate over the no-slip rate for each spin, by wheel speed, beside the first recording and the table's 0.50 |
| `wheelstand_balance` | `com_x` and `com_z` from a wheelstand recording, with both balance angles, every trial and the intervals; with no path, the angles the table's centre of mass predicts |
| `recorder_plan` | Checks a choice of the recorder's phases and writes out the command for a person to run: as an argv list with the files for stdin and stdout, and as one line for a POSIX shell and one for Windows `cmd.exe`, with the space each phase needs and which ones need watching. **It runs nothing** |

What it will not do: connect to the robot, run the recorder, write a file,
or read anything but a `.jsonl` file inside this directory (64 MB at most).
A recording copied in, for example `run2-dark.jsonl`, can be read by name.
`recorder_plan` refuses an edge run without a ruler distance, `repeat`
anywhere but last, a phase listed twice, and a note with any character that
could break the shell line (quotes, `$`, backticks, backslashes).

## The parameter table

Each entry has a value, a unit, a provenance of exactly one of `measured`,
`fitted`, `manual`, `sdk`, `assumed`, `chosen`, and a source. Assumed, fitted
and chosen values carry a low and high bound. `params.load()` refuses a table that breaks
those rules.

- `measured` (10): read directly off the recording, with no model in between.
  Each must cite the `REC` source, and `simulacrum/recording.py` recomputes
  each one from the files, so the table and the recording cannot drift apart.
- `fitted` (19): a value chosen by a fit to the recording. Eight by
  `fit_recording.py`, so that the model reproduces named segments; two by
  `simulacrum/imu_fit.py`, for the accelerometer alone, with no MuJoCo model
  involved; six by `simulacrum/prox_fit.py` (the range sensor), two by
  `simulacrum/motor_heat_fit.py` (tread heating) and one by
  `simulacrum/spin_fit.py` (where the treads meet the table). A fit is not a
  measurement: it depends on the shape of what was
  fitted and, for the eight, on every assumed value that was in place when
  it ran. Each says what it was fitted to and what was held out, and must
  carry a range.
  The validator refuses a `measured` or `fitted` entry that does not cite the
  recording, and a fitted one without a range.
- `manual` (17): the Vector Technical Reference Manual, by section and line.
- `sdk` (104): published in software. `source_kind` says which:
  `vendor_firmware` (81, the robot's released source), `vendor_animation_rig`
  (12, Anki's animation rig), `vendor_webots_model` (5, Anki's Webots model),
  `household_code` (4, the limits in `vector-slam-maze`) or `vendor_sdk` (2,
  the SDK's own code). Cited by URL, line and SHA-256.
- `assumed` (39): properties of the robot or light cube that no source
  states. Each is a measurement still to be taken.
- `chosen` (9): 4 table sizes, 3 solver/sim settings (`sim_timestep_ms`,
  `contact_time_constant_ms`, `settle_time_ms`) and 2 tip-support modelling
  settings (`tip_support_clearance_mm`, `tip_supports_driven`). They have no
  real physical value to find. The validator refuses a robot parameter filed as
  chosen and a scene or solver setting filed as assumed.

For Vector 2.0, `parameters-vector2.json` overrides 15 entries on top of
`parameters.json` and marks 27 inherited measured/fitted entries as `inherited`
(see "Vector 2.0: a separate set (2026-11-18)" below).

Part datasheets (BMI160, VL53L0X) are a part maker's figures under stated
conditions, not statements about this robot. Nothing from them is filed as
`manual` or `sdk`.

The Go simulator's geometry (cliff sensors at x +28 and -32 mm, y 12 mm)
is measured from the body centre; the firmware's (x +2 and -50 mm, y 14 mm)
from the front axle. This model uses the firmware's.

### Measured on the recording

| Parameter | Value | From |
|---|---|---|
| `state_period_ms` | 60 | Median 59.8 ms between state messages over 2,699 intervals. The firmware's own 30 ms is kept as `state_period_firmware_ms`; the SDK stream is half that rate |
| `gyro_noise_rms_dps` | 0.034 (0.017 to 0.07) | 334 rows asleep on the charger: 0.032, 0.036, 0.032 deg/s on the three axes. The 334 rows hold only 79 different gyro readings, so the value is refreshed less often than the message is sent |
| `accel_noise_rms_mg` | 1.26 (0.6 to 2.5) | Same rows: 13.5, 11.5, 12.1 mm/s^2 |
| `tof_range_std_fraction` | 0.022 (0.011 to 0.07) | Same rows: 5.2 mm at 237 mm. One distance only |
| `cliff_reaction_pause_ms` | 1220 (800 to 1700) | One event: the wheels were still at 1.19 s after the cliff flag first set and moving at 1.25 s |
| `cliff_reaction_backup_mm` | 29 (20 to 80) | Same event, by the robot's own odometry in a straight line: 29.2 mm from where it stopped to where it came to rest |
| `cliff_reaction_backup_speed_mmps` | 205 (100 to 220) | Same event: mean reported wheel speed in the first 0.1 s of the move, 204.5 mm/s |
| `gyro_bias_dps` | 0.001, 0.009, -0.001 (0.04 either way) | **New.** Mean gyro reading over every still stretch: 695 rows in 31 stretches. What is left after the firmware's own bias removal. The largest mean of any one stretch is 0.038 deg/s |
| `spin_rate_fraction_measured` | 0.50 | **New, and a correction.** Turn rate over the no-slip rate in the six recorded spins: 0.44 to 0.53. A figure to check the model against, not an input. See "The turn in place" |

The noise was measured with the robot asleep; awake and moving it will be
more; the ranges reach up to the datasheet figures or past them. The one cliff event
happened at 54 mm/s during the robot's own drive back to its charger. The
gyro figure is measured only while still: whether the firmware's estimate
drifts in long motion is not in the recording.

### Fitted to the recording

Fitted to the 30 and 100 mm/s spins and the 40 mm/s straight steps (treads)
and to three of the five head and lift moves. The 60 mm/s spins, the 80 mm/s
steps, one head move and one lift move were held out and never scored during
the fit.

| Parameter | Before | Fitted | Range | What in the data fixed it |
|---|---|---|---|---|
| `command_delay_ms` | 30 (assumed) | 60 | 40 to 120 | When reported wheel speed first rises after a command. 60 and 80 fit equally well: the state period is 60 ms |
| `tread_noload_speed_mmps` | 319.5 | 265 | 230 to 320 | The steady reported speed in the 100 mm/s spins, and the rise |
| `tread_friction_power_fraction` | 0.057 | 0.10 | 0.06 to 0.15 | The 30 mm/s spins: how late and how slowly a small command breaks the treads loose |
| `wheel_stall_torque_mNm` | 25 (assumed) | 100 | 50 to 200 | The rise of wheel speed in every segment, together with the next one |
| `wheel_armature_gcm2` | 1000 (assumed) | 2000 | 1000 to 4000 | The same rise, and how the treads slow after the stop |
| `servo_command_delay_ms` | 30 (shared with the treads) | 105 (was 225; see below) | 80 to 130 | When head angle and lift height first change after a command, with the ramp below |
| `head_speed_max_radps` | no limit | 10, no cap (was 1.75) | 3.5 to 10 | No cap below the SDK's requested speed shows once the ramp is in |
| `lift_speed_max_radps` | no limit | 10, no cap (was 2.8) | 6.0 to 10 | The same |

The fit is a coordinate search over those eight values, one at a time, four
sweeps, 129 replays of the tread segments and 87 of the head and lift
segments. The values in the table are the fit's rounded. The search is not
a global optimum: two single changes in the table below lower the fit-set
score a little (to 0.165 and 0.159) while raising the held-out score. They
were not adopted, because choosing by the held-out score would turn it into
a second fit set.

### The head and lift ramp, refitted on 2026-11-18

The first fit moved the head and lift at a constant rate (1.75 and 2.8 rad/s
after 225 ms). The robot does not: in both recordings the longer moves are
faster. The peak head speeds are 92, 144 and 230 deg/s over 15, 32 and
46 degrees on 2026-11-15, and 195 to 283 deg/s over 67 degrees on
2026-11-18. The SDK's `set_head_angle` and `set_lift_height` both ask for
`accel=10.0, max_speed=10.0` by default (rad/s^2 and rad/s; read in the
installed SDK, source SDKPY), and the recorder uses the defaults. So the
model now speeds the command up at that acceleration and slows it down so
it stops on the target (`_profile_step` in `simulacrum/robot.py`). Only the
delay is fitted (`fit_recording.py --servo-delay`, on the same three moves).

| Scored on (mean error / joint range) | Constant rate, 225 ms | Ramp, 105 ms |
|---|---|---|
| 2026-11-15, the three moves fitted | 0.019 | 0.033 |
| 2026-11-15, the two moves held out | 0.052 | 0.035 |
| 2026-11-18 run A, all 15 moves, never fitted | 0.060 | 0.032 |

The constant-rate column is the old values with the ramp switched off
(acceleration 10^6 rad/s^2), which reproduces the old code's 0.060 on run
A. It fits its own three moves better and everything it did not see worse. Fitting the cap as well as the delay,
with the ramp in, gives 1.24 rad/s. That scores 0.026 on the fitted moves,
but 0.065 and 0.081 on the two held-out sets, so it was not adopted. Run A
on its own would choose a delay of 110 to 125 ms: the delay is a round trip
through the wire-pod host and may differ from day to day.

The [stock replay findings and plots](evidence/stock-shell-physics-20261118.md#stock-and-simulator-overlays) compare the 100 mm/s CCW turn and unloaded lift against archived hardware traces. The lift readout uses the calibrated low-dock encoder origin. A faster lift ramp improved the first full raise but increased the mean height error on the separate Nov 18 moving lifts.

Aligning the [unloaded lift traces at first motion](evidence/stock-shell-physics-20261118.md#first-motion-alignment-of-the-unloaded-lift) shows a difference in the recorded raise profiles as well as variation in onset. Matching the first raise’s onset by increasing delay worsened the independent six-move mean error from 2.24 to 3.68 mm. Run `python diagnose_lift_timing.py` for the individual moves.

The [head first-motion overlays](evidence/stock-shell-physics-20261118.md#head-timing-on-the-command-and-first-motion-clocks) compare the closest raises and lowers in those sessions. The Nov 18 head lower also occurs later through mid-stroke. The comparable head raise admits an earlier or zero shift. Increasing the servo delay worsens mean error on all eight independent head moves taken together. Reproduce with `python diagnose_lift_timing.py --joint head`.

The [pitch-gyro speed versus head angle](evidence/stock-shell-physics-20261118.md#head-angular-speed-versus-posture) comparison tests onset timing and posture-dependent load against independent moves. The [acoustic ranging feasibility study](evidence/vector-acoustic-ranging.md) checks Vector’s audio paths, Susurrus timing, and a synthetic four-channel wall-echo detector.

### The spins of 2026-11-18: the centre of mass, again

Run E spun the robot for 3 s each way at 150 and 220 mm/s. Both wheel speeds
match the model within 4 mm/s. The turn rate does not: the robot turned at
184 and 270 deg/s, which is 0.53 of the no-slip rate at both speeds (0.48 to
0.51 at 30 to 100 mm/s on 2026-11-15). The model turns at 0.85 to 0.90 of
the robot's rate (`replay_recording.py --wheels-of`).

| Changed, one at a time, to its range end | Model turn rate / robot's, four spins |
|---|---|
| none (table) | 0.85 to 0.90 |
| `com_x_mm` -30 / -14 | 1.03 to 1.09 / 1.38 to 1.49 |
| `tread_friction` 0.5 / 1.3 | 0.84 to 0.86 / 0.82 to 0.88 |
| `com_z_mm` 15 / 30 | 0.87 to 0.90 / 0.84 to 0.90 |

Only `com_x_mm` matters. At -29 the four spins come out at 0.94 to 1.04,
and the 2026-11-15 wheel scores barely move between -24 and -29 (fit
0.167 to 0.173, held out 0.141 to 0.145). One run on an unrecorded surface,
pointing to the end of the assumed range, is not enough to change a value:
`com_x_mm` stays -24 and assumed. The rear wheelstand decides it: -24
predicts a balance at 64.9 degrees, -29 at 59.4.

**Where the robot turns about rules out -29 (2026-11-20).** Another lane pointed out (the lab's coordination room turn 166) that the centre of mass also
moves the point the robot spins about. A camera followed
the forward end of unit 3's backpack lens through four 150 mm/s spins
(`evidence/camera-spins` in that session's tree, commit 33ff9ea). The lens
sits about 36.3 mm behind the front axle. Within a spin it moves on a
circle about the turn centre. A circle fitted to it per spin, with that
session's calibration and no heading or SDK data, puts the centre 7.4, 10.2,
7.6 and 10.9 mm ahead of the lens and within 2 mm of the centreline: x =
-28.9, -26.1, -28.7, -25.4 mm. A focal prior from 450 to 720 px, or the
lens height 5 mm either way, moves that by at most 0.9 mm. Another substrate
found that dropping the shelf constraint moves the mean from -27.3 to
between -24.9 and -26.3 mm (turn 216). Followed the same way in the model
(2.0 table, a GPU host), with the head and lift where unit 3's were (22.5 degrees,
32.7 mm):

| `com_x_mm` | Turn centre x | Steady rate at 146 mm/s, / unit 3's 3.24 rad/s | Mean roll, pitch rate in the spin |
|---|---|---|---|
| -24 (table) | -25.0, -23.7 mm | 0.79, 0.78 | 0.59 / 0.53, 0.24 / 0.23 rad/s |
| -29 | -36.4, -37.2 mm | 0.93, 0.92 | 0.58 / 0.34, 0.30 / 0.20 rad/s |
| unit 3 | -25.4 to -28.9 mm | 1 | 0.09 to 0.14, 0.07 to 0.10 rad/s |

(A first run had the head at 0 degrees: -24 then turned about -26 mm at
0.86 to 0.90 of unit 3's rate, and -29 about -37 mm at 1.02 to 1.05.)

-24 puts the turn centre where unit 3 turns; -29 matches the rate but turns
about a point 10 mm further back. So the missing rotation is not the
centre of mass. It is in the contact model: see the next section. (The
last column is not a fair comparison, and an earlier version of this
paragraph drew the wrong conclusion from it. The robot's gyro is
low-passed by the sensor, about 18.65 Hz; the model's is read at an
instant. Filtered the same way, the model rocks about as much as the
robot. See "What the rocking was".) Unit 3 turned 312 to 320 degrees per 1.5 s spin by its own
IMU, at 3.24 and 3.23 rad/s. The head-at-0 run's 5 % difference between the
model's two directions was chatter, not a lasting asymmetry: in unit 3's pose
the two directions agree (2.55, 2.54 rad/s), and another substrate found the
true yaw rates differ by 1.5 % with a sign that a solver tolerance flips.
Caveats: one robot, with a printed
lift and no faceplate; the lens's x comes from a geometric model;
the circle fits include the rests, and unit 3 drifted 4 to 9 mm per spin.
(Since ported into `simulacrum/camera_spins.py` and `evidence/camera-spins/`.)

### Where the treads meet the table (2026-11-20)

The model's treads touched the table only under the axles, 48 mm apart. The
further apart the contacts, the more the treads' sideways friction resists a
spin. `tread_contact_inset_mm` moves both contacts inward: the model's four
wheels sit that far inside the wheel base (`mjcf.wheel_x_mm`). The
firmware's `wheel_base_mm` stays 48 for odometry.

`fit_spins.py` (`simulacrum/spin_fit.py`) replays unit 1's four run E spins
from the robot's recorded head and lift, at insets from 0 to 8 mm. It
compares steady turn rates over the second half of each spin. Both the
model's rate and the robot's are read through the IMU chip's filter
("The IMU chip's own filter", below).

| Inset (mm) | 0 | 1 | 2 | 3 | 4 | 5 | 8 |
|---|---|---|---|---|---|---|---|
| Model / unit 1, mean of 4 spins | 0.871 | 0.923 | 0.946 | 0.995 | 1.040 | 1.093 | 1.258 |

The mean crosses 1 at 3.10 mm; spin by spin at 2.88, 3.44, 2.92 and 3.04.
The table holds 3.1, fitted, with 2 to 4 as its range
(`evidence/imu-filter-20261120/fit_spins-output.json`). Before the model's
gyro had the chip's filter, the same fit gave 3.01 mm and the table held 3.0.

That move is smaller than the fit can resolve. Between the 1 mm grid points
the model's rate does not follow the inset smoothly.
`evidence/imu-filter-20261120/inset_scan.py` replays both robots' spins at
21 insets from 2.90 to 3.30 mm, 0.02 mm apart:

- Unit 1's mean ratio wanders from 0.989 to 1.023 and crosses 1 five times,
  between 3.00 and 3.22 mm. Neighbours 0.02 mm apart differ by up to 0.031.
- A straight line through the 21 means rises 0.051 per mm and crosses 1 at
  3.02 mm. The scatter about it, 0.008, is worth 0.16 mm of inset.

So the inset is known to about 0.2 mm either way, and 3.0 and 3.1 are the
same answer. The scatter is the treads' chatter: rigid wheels on a rigid
table. It also sets the allowance of 1 mm either way.

Held out, unit 3's four camera spins on the 2.0 table, which inherits the
value:

| | No inset | 2.90 to 3.30 mm, 21 insets | The table's 3.1 mm | unit 3 |
|---|---|---|---|---|
| Turn rate / unit 3's, each spin | 0.83 to 0.86 | 0.93 to 1.03 | 0.95 to 0.99 | 1 |
| Turn centre x (mm), each spin | -21.4 to -25.2 | -20.9 to -26.8 | -22.5 to -23.6 | -25.4 to -28.9 (camera) |

The rate comes closer to unit 3's at every inset of the scan. The turn centre
is a weaker result than this section first said. One spin's centre moves by
up to 5.7 mm between insets 0.02 mm apart. Averaged over three neighbouring
insets (12 spins), it sits 2.4 to 4.1 mm ahead of unit 3's mean of -27.3 mm.
The test used to check each spin at the fitted inset alone, within 4 mm.
That passed at 3.1047 mm (-24.1 to -25.1) and would have failed at 3.10 or
3.00. It now averages the fitted inset and 0.04 mm either side, within
5 mm. A centre of mass moved back to -29 mm (above) put it 10 mm behind.

The camera's turn centres come from `simulacrum/camera_spins.py` (needs
scipy). It is a port of the circle fit above, with the camera calibration unchanged.

A contact-roller fit ran the other way round (the lab's coordination room turn 221). It
used its own contact rollers with the axles kept, fitted on unit 3 and held
out unit 1, and found 3.75 mm. Its held-out run E improves on all four spins.
The two directions agree within a millimetre.

**What the rocking was.** I reported, in this README and in the room, that
the model's treads chattered at three to ten times the robots' roll rate.
That compared unlike signals, and a lane found it (turn 223). The
robot's gyro is low-passed inside the IMU chip; the model's was read at one
instant per 5 ms tick, so faster vibration aliased into it. A scratch
filter at every 1 ms physics step brought the model's rocking down to the
robots'. Filtering at the 5 ms tick instead removed almost nothing (roll
0.39 to 0.36 rad/s), which is the aliasing. The model now has the chip's
filter ("The IMU chip's own filter", below), and the figures there come
from it.

Moving the wheels inward would also move the edge the robot tips over. The
wheelstand test then balanced at 61.8 degrees against the closed form's
64.9. So the model also has body-fixed tip supports round the real axles:
`tip_support_clearance_mm`, chosen, 0.3 mm smaller in radius than the
wheels. They carry nothing on a flat table and in spins never touched it (0
of 400 ticks). Once the body tilts they take over. A robot balances with its
centre of mass above the centre of the circle it rests on, whatever the
circle's radius, so the tipping angles are the real axles' again.

Two tests had to change, and neither was weakened. "Drawn 2.0 robots spin"
read the turn from the final heading, which wraps at 180 degrees: a draw
that spun 349 degrees read as 11 and failed. It now adds the turn up tick by
tick. "A spin shows on gyro x by the tangent of the head's pitch" averaged
20 state periods, too few for the chatter to average out once the inset
changed it (up - down 40.0 against 37.4). It now averages 120 (38.1).
The same missing sensor filter is why it needed to.

### The IMU chip's own filter (2026-11-20)

The robot's gyro and accelerometer readings are low-passed inside the IMU
chip. The model's were not. That was the rocking above, and most of the
accelerometer's excess shake ("Measured in the model").

What the firmware sets, in `robot/hal/spine/imu.c` (source `IMUC`):

- accelerometer: `ACC_CONF` 0x19 (line 132, written at 241). That is
  200 Hz output, OSR2, undersampling off.
- gyro: `GYR_CONF` 0x09 (line 134, written at 244). That is 200 Hz output,
  OSR4.
- The FIFO carries the filtered data: `FIFO_DOWNS` 0x88 (lines 145 and 249).

Nothing filters the readings again on the way out. `messages.cpp` (lines
94 to 100, source `MSGCPP`) puts the latest accelerometer sample in the
state message, and the gyro less its bias estimate (`imuFilter.cpp` line
950).

One step on that path is not modelled; another substrate (a lane) pointed it
out, and it was read in the file. On a real robot the firmware multiplies
gyro z by 0.989 (`imuFilter.cpp` lines 953 to 958, VIC-285), because "the z
axis gyro usually reports about a 1.03% higher rate than it is actually
experiencing". The model's gyro has neither the chip's error nor the
correction. For a typical unit the two cancel to 0.9992 (1.0103 x 0.989).
How far one robot's chip departs from typical is not known.

What the datasheet says those settings do:

- accelerometer: 80 Hz at 200 Hz in normal mode, halved by OSR2: 40 Hz
  (BMI160DS lines 1044 to 1080).
- gyro: 74.6 Hz at 200 Hz in normal mode, "approximately 4 times smaller"
  in OSR4: 18.65 Hz (lines 1143 to 1165).

It also says OSR2 and OSR4 use "both stages of the digital filter", and it
gives neither stage's shape.

`simulacrum/imu_filter.py` models the two stages as one-pole sections in
series. Their corners are placed so that the pair is -3 dB at the
datasheet's figure. The filter runs at every 1 ms physics step. The robot
reads it once per 5 ms tick, since the chip outputs at 200 Hz. Three new
parameters, all assumed:

| Parameter | Value | Range |
|---|---|---|
| `accel_filter_cutoff_hz` | 40 | 36 to 44 |
| `gyro_filter_cutoff_hz` | 18.65 | 16.8 to 20.5 |
| `imu_filter_second_pole` | 1: two equal sections | 0 (one section alone) to 1 |

The datasheet states no tolerance, so the 10% either way on the cutoffs is
a guess. Two equal sections is taken because nothing favours either stage.
A stage of higher order than one is outside the range.

Checked by running it (`tests/test_imu_filter.py`):

- The discrete filter's gain at its cutoff is 0.709 to 0.711, against
  0.7071, for one or two sections, at 1 ms and 0.5 ms steps.
- A 100 Hz tone on the gyro comes out at 0.080 with two sections (0.0775
  for the continuous filter) and 0.186 with one (0.183).
- In the robot, the filter steps 5 times per tick, and the first reading is
  the settled one, not a ramp from zero.

Scale and offsets are still applied after the filter. That is equivalent,
because the filter is linear with unit gain at rest. The noise is also
added after it, at the level recorded at the robot's output. So the model's
noise is independent from one 5 ms reading to the next, and the robot's is
not. The recording, at one state every 60 ms, cannot tell the difference.

Rocking, from `evidence/imu-filter-20261120/compare_spins.py` in a GPU host.
Mean |roll|, |pitch| in rad/s over the steady half of each spin, at the
table's inset of 3.1 mm:

| | No filter | One section | Two equal sections (table) | Robot |
|---|---|---|---|---|
| unit 3, four spins at 150 mm/s | 0.34 to 0.60, 0.15 to 0.17 | 0.09 to 0.14, 0.08 to 0.11 | 0.06 to 0.08, 0.07 to 0.10 | 0.09 to 0.13, 0.07 to 0.11 |
| unit 1 run E, two spins at 150 mm/s | 0.49 to 0.53, 0.17 to 0.27 | 0.10 to 0.11, 0.06 to 0.08 | 0.07 to 0.09, 0.04 to 0.07 | 0.08, 0.08 to 0.14 |
| unit 1 run E, two spins at 220 mm/s | 0.59 to 0.84, 0.27 to 0.45 | 0.14 to 0.24, 0.15 to 0.16 | 0.11 to 0.20, 0.12 to 0.13 | 0.09 to 0.12, 0.12 to 0.20 |

Either shape takes out most of the vibration:

- With two equal sections the model pitches about as much as unit 3 and rolls
  a little less. Against unit 1 it rolls about as much at 150 mm/s and 1.3 and
  1.7 times as much at 220, and pitches less.
- With one section it rocks about as much as unit 3, and 1.7 and 2.0 times as
  much in roll as unit 1 at 220 mm/s.

The rocking depends on the rigid wheels as much as on the filter, and it
moves with the inset. So these spins cannot settle the filter's shape. The
randomised range covers both.

The filter shifts each spin's measured turn rate a little: by -1.6% to
+1.9% at 3.1 mm, over the eight spins above. The robot's rate is read
through its own filter, so the filtered model is the like-for-like
comparison. That shift is what moved the fitted inset from 3.01 to 3.10 mm
(above).

Shake in a held spin, `evidence/imu-filter-20261120/scatter.py` (the same
measurement as "Measured in the model"): the standard deviation of the
axis with the most, over 21 state messages, head 22.4 degrees up.

| Spin | Gyro (deg/s): no filter, one section, two | Recording, two directions | Accelerometer (mg): no filter, one section, two | Recording |
|---|---|---|---|---|
| 30 mm/s | 9.2, 3.4, 3.0 | 5.6 and 1.9 | 812, 214, 108 | 32 and 19 |
| 60 mm/s | 13.5, 8.4, 8.2 | 11.3 and 3.4 | 949, 252, 144 | 71 and 52 |
| 100 mm/s | 18.5, 10.0, 9.2 | 18.8 and 5.2 | 968, 364, 256 | 185 and 124 |

The accelerometer was 5 to 43 times as rough as the robot's. With the
table's filter it is 1.4 to 5.7 times. The gyro now sits between the two
recorded directions.

Two tests changed with this:

- "The filter takes the vibration out of a spin but not the turn" first
  compared the filtered gyro's mean rate with the unfiltered one's, within
  2%, and failed at 2.4%. The unfiltered reading is the aliased one. Over
  four held spins it gives 0.987 to 1.024 of the true turn rate (from the
  change of heading); the filtered one gives 0.993 to 1.001. The test now
  checks the filtered rate against the true one, within 1.5%.
- "With the reaction off, the robot drives off and falls" read the fall
  flag from the state messages. The flag holds for 5 to 7 control ticks
  before the landing clears it, less than one 60 ms state period, so a
  message catches it or not by phase. At an inset of 3.0 mm one did; at 3.1
  none does, with the filter or without. The test now reads the flag at
  every control tick. That checks less than before: that the robot's own
  loop raises the flag, not that the SDK stream shows it. A real robot's
  stream could miss a short fall the same way; that has not been checked
  on the hardware.

### Vector 2.0: a separate set (2026-11-18)

`parameters-vector2.json` holds the entries where Digital Dream Labs'
Vector 2.0 is known or suspected to differ; everything else is inherited from
`parameters.json`. Load it with `params.load(model="vector_2")`, or set
`VECTOR_MODEL=vector_2` for every tool at once (the MCP `parameters` tool
takes `model`).

What is known to differ, with sources: the display (160x80 against 184x96,
DDL's boot display code), the camera (a 1600x1200 sensor), and the battery
(a new cell in a removable compartment, "up to 30 % longer runtime"). One
community page says the board, IMU, distance and cliff sensors, motors and
gearboxes are the same. No source gives 2.0's mass, dimensions or wheel
geometry. So the 2.0 set replaces seven entries:

| Entry | 1.0 | 2.0 | Why |
|---|---|---|---|
| `display_resolution_px` | 184x96 | 160x80 | Sourced |
| `battery_capacity_mah` | 320 | 416 assumed (320-600) | 320 x 1.3 from DDL's runtime claim; 600 from one unconfirmed listing |
| `total_mass_g` | 170 (150-185) | 170 (150-200) | New battery and compartment; range widened |
| `com_x_mm`, `com_z_mm` | -24 (-30..-14), 22 (15..30) | same, ranges 3 mm wider | The battery is the heaviest part whose place is not known |
| `camera_fov_horizontal_deg` | 90 | 90 assumed (90-120) | One aggregator says 120 |
| `camera_fov_vertical_deg` | 50 | 50 assumed (50-90) | 4:3 sensor; how 2.0 crops it is not known |

The 22 measured and fitted values (tread drive, servo timing, IMU offsets,
heat) come from unit 1, a Vector 1.0. In the 2.0
set each carries `inherited`, so it cannot pass for a 2.0 measurement. Of the
household's other two units, unit 2 is a 1.0 and unit 3 a 2.0; a
sustained-load run on unit 3 is the first measurement the
2.0 set can carry. The robot's factory record says which it is: HW_VER at offset 4
of `/dev/mmcblk0p29`, 0x20 or above for 2.0. On the body, 1.0 prints "S/N"
near the top of the track, 2.0 an "ESN" underneath.

*(Current state: `parameters-vector2.json` now carries **15 overrides** — adding
`camera_resolution_px`, `accel_scale`, `accel_bias_mg`, `imu_pitch_offset_deg`,
and four unit 3 cube-accelerometer overrides: `cube_accel_word_format`,
`cube_accel_scale`, `cube_accel_noise_axis_scale`,
`cube_accel_noise_density_ug` — alongside **27** inherited measured/fitted
entries.)*

### The first run on a Vector 2.0 (2026-11-19)

Unit 3 is not a stock 2.0 body. The operator (via a lane, the lab's coordination room
turn 161): its lift and hubcaps are 3D-printed, its tracks are stock, and its
faceplate is removed. Every unit 3 figure below (top speed, turn rate, the lift
run, the accelerometer) is from that modified body, so a difference from
unit 1 may come from the modifications as much as from the 2.0 hardware.

Run G (`evidence/hw-rec-20261119/run-g-endurance-0a1b2c3d.jsonl`), on
unit 3 (0a1b2c3d), the household's Vector 2.0, reached through its daughter
pod with `VECTOR_IP`: the endurance phase, 300 mm/s commanded. After 9
spins (about 52 s) control was lost. Corrected 2026-11-19 from unit 3's own
`/var/log/messages`, as read by a lane (the lab's coordination room turn 103,
not checked here): an app intent `SleepCommand` reached the robot at about
91.42 s and took control (the recorder saw the loss at 96.66 s); a genuine
front-left cliff stop came at about 119.7 s, 17 s after the recorder's last
sample, with the robot under its own control. The nine spins moved it only
8.21 mm by the SDK's own estimate. So the run was ended by a sleep command
from outside the run, not by drift, and the edge came afterwards. The dock
phase afterwards (run H) did not reach the charger. Over the first 8 full
spins, against unit 1 (1.0) in run F:

| | unit 1 (1.0) | unit 3 (2.0) |
|---|---|---|
| Left tread top speed | 244.3 mm/s | 244.1 mm/s |
| Right tread top speed | 242.1 mm/s | 236.3 mm/s |
| Turn rate | 322.8 deg/s | 326.9 deg/s |
| Turn rate / no-slip rate | 0.533 | 0.546 |
| SDK battery reading | 4.08 V | 3.99 V |

The 2.0's top speed is the 1.0's within 2.4 %, so inheriting the tread
drive values in `parameters-vector2.json` fits the only 2.0 data there is
(8 spins, one unit). The run was too short for a heat fit. Before another
run on a robot that Home Assistant manages, its sleep cycle has to be held
off for the run, as well as quiet mode; unit 1's run F did end at an edge
after 150 s, so a long run still wants walls.

### Lift up against lift down (2026-11-20)

The operator: "try spinning with lift up versus lift down". Run J
(`evidence/hw-rec-20261120/run-j-liftspins.jsonl`, unit 3, recorder phase
`liftspins`): head level, 3 s spins at 150 mm/s each way with the lift down,
up, then down again. Turn rate over the second half of each spin (n=26 each):

| | Lift down (first) | Lift up | Lift down (again) | Up against the mean of the downs |
|---|---|---|---|---|
| Counter-clockwise | 205.2 deg/s | 206.5 | 206.2 | +0.4 % |
| Clockwise | 200.9 deg/s | 204.4 | 203.4 | +1.1 % |

The two lift-down repeats differ by 0.5 to 1.2 %, the same as the lift's
difference, so its effect on turn rate is below about 1 % and not resolved.
The model, asked before the run, predicted -1 to -1.6 %: not contradicted,
direction not confirmed. The model spins at 175 deg/s here, 0.85 of unit 3's
rate, as on unit 1. Before the run unit 3 sat on its charger ramp without
contact; the `nudge` phase seated it on its first 16 mm move back (run I).

Drift, from a calibrated camera over the same run
(P1R4N351/vector-simulacrum PR #1, comment 7025928568; their measurement):
leftward movement per ccw+cw pair, lift down / up / down again:

| Source | Down (first) | Up | Down (again) |
|---|---|---|---|
| Camera | 22.8 mm | 20.1 mm | 24.6 mm |
| Their replay, cylinder wheels | 6.6 mm | -4.5 mm | 5.1 mm |
| Their replay, sphere wheels | -3.4 mm | -12.5 mm | -7.5 mm |
| SDK pose | -0.3 mm | 0.5 mm | 0.3 mm |

No lift effect on drift is resolved either (the three blocks also crossed
different spots on the shelf). The robot moved 20 to 25 mm the same way
every pair. A follow-up by a lane (the lab's coordination room turn 133; eight
lift-down spins at two shelf spots with opposite headings, farther from the
dock) showed whose bias that is: a drift fixed to the shelf predicted the
camera endpoints within 8.3 and 2.9 mm, a drift fixed to the robot's heading
missed by 44.8 and 46.6 mm, and all eight turns went shelf-left; net leftward
movement per pair at spot A facing 0 and 180 deg and spot B likewise: 22.7,
14.4, 22.3 and 19.5 mm (PR #1, comment 7026442869, with footage and inputs). So the
sideways drift is fixed to the shelf, not to the robot's heading; the operator also saw
the robot creep back toward its charger. It is not a slope: the operator measured the
shelf level at every position. Opposite-heading
accelerometer rests read 0.98 and 0.91 deg (turn 137), so that reading has
some other cause and is not a tilt measurement. Part of the head
accelerometer's behaviour is the sensor itself (another substrate, turns 158 and
166): the total gravity it reads changes with head angle (979 to 1001 mg),
which no tilt can do, and a bias plus an x-axis scale of about 1.014 explains
that head-angle excursion (reproduced here; see "The accelerometer's x
scale" below). That does not explain the opposite-heading difference: with no tilt, about 16 mg stays
unexplained at both spots. The chassis itself does not lean (optically -0.07
deg at spot A, turn 163, PR #1 comment 7027738144). So the
opposite-heading signal, like the drift, is still open. The simulacrum models
a per-axis bias and, since 2026-11-20, a per-axis scale. What on the shelf pushes the
robot one way is open: a surface that grips differently in different
directions (tape strips, grain), local unevenness, or something at the dock.

### How far spins move the robot: a camera measurement (2026-11-19)

A camera measured unit 3 (Vector 2.0) with a fixed camera calibrated on a
printed glyph, over four commanded ±150 mm/s turns (P1R4N351/vector-simulacrum
PR #1, comment 7025537786; external measurement, not repeated here):

| Source | Net movement over the four turns |
|---|---|
| Camera | 23.6 mm (20.4 mm left, 12.0 mm away from the camera) |
| Independent optical landmark check | 20.9 mm |
| The robot's own SDK pose | 2.5 mm |
| Their replays, cylinder / round contact, 42 mm span | 9.4 / 8.5 mm |

The SDK pose under-reports spin drift about tenfold, so it cannot score
drift, and the models drift less than the robot. With the model's turn rate
at 0.85-0.90 of the robot's (runs E and F), the real treads slip more than
the model's contacts allow. Spin drift is why both long spin runs ended near
an edge.

### Battery, heat and the top speed (2026-11-18)

The operator: "simulate voltage sag and other factors that affect motor
efficacy/responsiveness ... from real sustained-load tests on multiple units".
What one unit so far gives:

**The top speed.** Run F sent `set_wheel_motors` 300 mm/s both ways in
spins for 150 s (3 s on, 2 s off; it ended when the robot met a cliff).
The robot reached 243 to 247 mm/s cold. The model was capping
`set_wheel_motors` at the firmware's `MAX_WHEEL_SPEED_MMPS` = 220, but the
firmware applies that cap in path following only (`steeringController.cpp`
line 419, `CheckWheelSpeedLimits`), not in direct drive. The cap is gone from
`set_wheel_motors`; the motor sets the ceiling. To reach 245 the no-load speed
had to rise from 265 to 305 mm/s, and the other tread values were refitted on
the 2026-11-15 fit set with it held (`fit_recording.py
--wheels-held-noload`): fit set 0.170 -> 0.143, held out 0.143 -> 0.140.

**The battery.** The tread power is a PWM duty (`hal_motors.cpp`), so the
motor sees duty x battery voltage, and the firmware does not compensate. The
model scales the drive by `battery_rest_voltage_v` / `motor_reference_voltage_v`
(stall torque and top speed fall with voltage; back-EMF damping does not),
draws the voltage per episode between 3.6 and 4.1 V (the SDK's low and full
marks), and sags it by `battery_sag_v_per_power` x summed duty. The sag is
assumed: the SDK's `get_battery_state` returned 4.0797 V 165 times in a row
during run F's full-power spins, so it cannot be measured through the SDK.
The reference voltage, 4.03 V, is the midpoint of the 4.10 and 3.96 V
reported for the 2026-11-15 run; neither reading is in its files.

**Heat.** The left tread's ceiling stayed between 238 and 247 mm/s through run F, with no trend;
the right tread's fell from 250 to 232 (one way) and 241 to 223 (the
other). One battery feeds both, so the fall is in the right motor.
`simulacrum/motor_heat_fit.py` fits a first-order heat state per tread on the
first 10 cycles and predicts the last 5:

| Tread | Held-out error, heat model | Held-out error, constant ceiling | Adopted |
|---|---|---|---|
| right | 1.9 mm/s | 8.5 mm/s | 0.207 %/s of drive lost at full duty, tau 85 s |
| left | 2.8 mm/s | 1.2 mm/s | none (the fit is noise) |

150 s does not pin the time constant: 40 to 600 s fit within 1.5x of the
best error, trading against the total loss (14 to 77 %). What it pins is the
initial rate, 0.13 to 0.34 %/s; those are the ranges training draws from.
The model reproduces the right tread's fall under the same schedule within
the test's +/-50 % (`tests/test_motor_heat.py`).

**Not yet met.** "Multiple units": only one unit has been run; the other two
were offline. A run to the plateau (the full 600 s or more, away
from edges) would pin the heat's time constant. The SDK cannot see motor
temperature, so heat is an inference from one tread falling while the other
holds.

### Fitted without the model: the accelerometer

The gyro and the accelerometer sit in the robot's head. The firmware says so
(`imuFilter.cpp` subtracts the head angle to get pitch) and the recording
shows it: over 369 turning samples with the head between -18 and +22
degrees, gyro x over gyro z follows the tangent of the head angle with
slope 0.9985. So the accelerometer's reading at rest turns with the head.

Whenever the robot stood still on the table (26 stretches, 625 rows, three
clusters of head angle), the reading is gravity as that frame sees it.
`simulacrum/imu_fit.py` fits all 26 at once for the table's tilt (1.4
degrees, as it turned out), the pitch of the accelerometer's frame, and one
offset per axis.

| Parameter | Before | Fitted | Range | Notes |
|---|---|---|---|---|
| `accel_bias_mg` | 0, 0, 0 (assumed, 40 either way) | -47.0, -7.7, +22.3 | 10 either way | Fit's own standard errors 3.0, 0.6, 0.4 mg; residual 17 mm/s^2 rms. x is outside the part's typical 40 mg |
| `imu_pitch_offset_deg` | not in the model | -0.66 | -1.2 to -0.2 | Standard error 0.17. The gyro line fit above gives -1.06 from a different signal. The sum of stance, mounting and head-angle zero error |

What this is worth, and what it is not:

- **Pitch and the x offset trade against each other** (correlation -0.98 in
  the fit). Near level, a small pitch moves the x reading by an almost
  constant amount, which an offset can absorb. A bootstrap over the 26
  stretches (200 draws) puts pitch between -0.79 and -0.56 degrees and the x
  offset between -48.9 and -45.1 mg. Their sum is tight: on a level surface
  with the head at 0 the x axis reads -60.3 to -57.3 mg. Training draws the
  two independently, which is wider than the fit allows.
- Leaving out each cluster of head angle in turn moves the offsets by at
  most 1.2 mg, but that is weak evidence: the 22 degree cluster also faces
  the other way from most of the rest, so head angle and heading are tangled
  in this recording. A grid of head angles at several headings would untangle
  them ("The next recording").
- Gravity is taken as 9806.65 mm/s^2, which leaves about 3 mg of doubt on z.
  The y offset cannot be told from a roll of the frame. Only the x axis's
  scale can be told from its offset, and only because the head turns that
  axis through gravity; see the next section.
- The 10 mg range is the author's allowance for all of that and for
  temperature. It is a fact about one robot: to train for any Vector, widen
  it to the part's 40 mg.

The correlation and the bootstrap are there because an external review lane asked for
them (conferral of 2026-11-15, below); both confirmed its point.

### The accelerometer's x scale (2026-11-20)

The simulacrum multiplies each axis's reading of the true specific force by
`accel_scale` before adding `accel_bias_mg`. The BMI160's sensitivity may sit
anywhere from 0.960 to 1.040 of nominal (BMI160DS:390-393). As the head turns
from -15 to 40 degrees, the x axis's share of gravity runs from -0.26 to 0.64 g,
so an x scale shows as a slope. The z axis's share changes by only a quarter of
g, too little to tell its scale from its offset. Nothing recorded rolls the
robot onto its side, so y's scale is not visible either. y and z stay at 1.

Unit 3 (the 2.0) carries the evidence: unit 3's stationary head sweeps,
analysed in parallel (the lab's coordination room turn 166),
copied to `evidence/head-sweep-unit3-20261120`. `python3 -m simulacrum.imu_fit
--head-sweep evidence/head-sweep-unit3-20261120` fits spot B and scores spot A.
The fit never sees spot A. Heading is the robot's odometry.

| Fitted on B | B (24 windows) | A, headings 0 and 180 (24) | A, heading 90 (6) |
|---|---:|---:|---:|
| x scale fixed at 1 | 6.71 mg | 7.51 mg | 5.82 mg |
| x scale fitted: 1.0140 | 5.31 mg | 6.49 mg | 3.95 mg |

Vector error per settled window, root mean square. A 200-draw bootstrap over
B's windows puts the scale between 1.0085 and 1.0207. With the scale fitted,
the offsets are 13.2, 4.0, -18.1 mg and the frame's pitch is -0.05 degrees
(-0.22 with the scale fixed). Another substrate's own fit, at gravity 9810
rather than 9806.65, gave 1.0138, 13.2, 3.9 and -18.5. These three values
replace unit 1's in `parameters-vector2.json`, marked fitted. Before this, the
2.0 table had inherited them from a 1.0.

What it is not:

- A head angle that reads 1.4 % more or less than the true angle would move
  x in nearly the same way. This fit does not test that alternative; a
  sweep with the angle checked optically would.
- About 15 mg of the opposite-heading difference is still unexplained.
- Unit 1's x scale is not settled. Over table-run1's 26 stretches,
  `imu_fit.fit(..., free_scale=True)` gives 1.0118, standard error 0.0044.
  Another substrate's fit over table-run1 and run A gave 0.999 and no gain on
  held-out windows. That fit gave the two runs' different surfaces one shared
  tilt, which pulls the scale toward 1 (its author, the lab's coordination room turn
  195). Run A cannot be fitted on its own: it has one heading,
  so the table's tilt cannot be told from the offsets. The 1.0 table keeps
  the nominal [1, 1, 1], marked assumed, with the part's range. A head sweep
  of unit 1 at several headings would settle it.

### What the recording could not settle

Checked by changing one value and replaying (fit-set score, held-out score;
the table's values give 0.170, 0.144; lower is better):

| Changed | Scores | Reading |
|---|---|---|
| `total_mass_g` 150 or 185 | 0.172, 0.146 both | **Not identified.** The firmware's loop holds wheel speed whatever the load |
| `tread_friction` 0.6 or 1.2 | 0.175, 0.139 and 0.184, 0.157 | **Not identified.** The lower value is better on one set and worse on the other |
| `com_x_mm` -18 or -30 | 0.189, 0.154 and 0.171, 0.149 | Weak: -18 is worse; -24 and -30 cannot be told apart |
| Stall torque and rotor inertia both x0.5, x2, x4 | 0.215, 0.179; 0.186, 0.144; 0.207, 0.144 | The ratio is fixed; the size is only bounded from below. Twice both fits the held-out set as well as the table |
| Stall torque alone x0.5, x2 | 0.239, 0.200; 0.208, 0.172 | Identified, given the inertia |
| Rotor inertia alone x0.5, x2 | 0.195, 0.175; 0.200, 0.167 | Identified, given the torque |
| No-load speed 230, 320 | 0.165, 0.149; 0.196, 0.159 | 320 is worse; 230 and 265 cannot be told apart. No segment is faster than 100 mm/s |
| Friction share 0.06, 0.15 | 0.218, 0.154; 0.159, 0.158 | 0.06 is worse; 0.10 and 0.15 cannot be told apart |
| Command delay 40, 80 | 0.193, 0.160; 0.172, 0.136 | 40 is worse; 60 and 80 cannot be told apart |

So `wheel_stall_torque_mNm` 100 is the motor's strength relative to an
assumed 170 g robot, not a torque anyone measured. And the delay is the
command's way in and the state message's way back together: the recording
has host times only.

Two limits of the model's shape, visible in the data:

- **Head and lift.** The model ramps at one rate. The robot does not:
  between samples its head moved at up to 92, 144 and 230 deg/s on moves of
  15, 32 and 46 degrees. The fitted rate (100 deg/s) is the one that best
  matches the two moves the fit used, and the held-out 46 degree move is
  fitted four times worse (3.8 against 0.9 and 0.7 degrees).
- **Left and right.** The real robot turned faster clockwise than
  counter-clockwise in all three pairs of spins (for example 130 against
  109 deg/s at 100 mm/s). The model is symmetric.

### Still assumed about the robot (39)

| Group | Parameters |
|---|---|
| Mass | `total_mass_g` 170 (150 to 185), `head_mass_g`, `lift_mass_g`, `wheel_mass_g`, `lift_arm_mass_fraction`, `com_x_mm` -24, `com_z_mm` 22 |
| Contact | `tread_friction` 0.9 (0.5 to 1.3), `chassis_friction` 0.3 |
| Head and lift drive | `head_servo_kp/kv`, `lift_servo_kp/kv`, `servo_torque_max_mNm` 30, `lift_servo_torque_max_mNm` 60 (52 to 250), `servo_armature_gcm2` |
| Lift fork geometry | `lift_fork_side_center_mm`, `lift_fork_side_size_mm` |
| Cliff | `cliff_sensor_height_mm` 8, `cliff_detect_range_mm` 25 |
| Control | `heading_hold_gain_mmps_per_rad`, `drive_done_mm` |
| Battery | `battery_rest_voltage_v`, `battery_sag_v_per_power` |
| Accelerometer | `accel_scale` 1, 1, 1 (the part's 0.960 to 1.040; unit 1's x is not settled, Vector 2.0 fitted) |
| IMU filter | `accel_filter_cutoff_hz` 40 (36 to 44), `gyro_filter_cutoff_hz` 18.65 (16.8 to 20.5), `imu_filter_second_pole` 1 (0 to 1) |
| Light cube | `cube_outer_mm` 45, `cube_mass_g` 50 (30 to 70), `cube_chip_position_mm`, `cube_chip_yaw_deg`, `cube_accel_bandwidth_hz` 250, `cube_accel_frame_period_ms` 2, `cube_accel_filter_second_pole` 1, `cube_accel_noise_axis_scale`, `cube_accel_noise_density_ug` 220, `cube_accel_bias_mg`, `cube_accel_scale` (four accelerometer entries overridden on Vector 2.0 from unit 3 recordings) |

Chosen, not assumed (9): `tip_support_clearance_mm`, `tip_supports_driven`,
`table_length_mm`, `table_width_mm`, `table_height_mm`, `table_thickness_mm`,
`sim_timestep_ms`, `contact_time_constant_ms`, `settle_time_ms`.

(`chassis_ground_clearance_mm` 1.2 and `head_box_mm` are now sourced as `sdk`
from `vendor_animation_rig`; earlier table versions listed them as assumed.)

Resolved in this version, out of the 30 the last version called assumed:
`gyro_bias_dps` (measured), `accel_bias_mg` (fitted), and the seven above
(never unknowns about the robot).

Cautions carried from the published-figures work of the third version:

- The divisors 172.3, 149.7 and 366.211 in the encoder scales are not
  recorded as gear ratios. The source does not say what they are.
- The firmware names the IMU: a comment in `imuFilter.cpp` (line 153) cites
  the "BMI160 datasheet", and `imu.c` heads its register map "BMI160
  REGISTERS" (line 28). The manual does not name it.
- The VL53L0X datasheet was read from a distributor's mirror.
- Mass and outer dimensions were not found on any page that could be read.

## The assumed values, ranked

The operator asked for the assumed values to be resolved in order of their effect on
the simulation, for a policy that has to carry over to the real robot.
`rank_assumed.py` measures that effect; `evidence/rank-20261116-output.txt`
is its output (a GPU host, 24 paired episodes, 63 variants of the table).

Each assumed or chosen value is moved to its low bound and then to its high
bound, everything else at the table's values, and compared with the
unchanged table:

- **Closed loop.** The training task itself with randomisation off, the same
  24 seeds every time, driven twice: by the hand-written no-slip controller,
  which ignores every observation, and by the exported smoke policy of
  2026-11-15, which does not. The figure is the change in reward per step.
  The unchanged table scores 1.1415 (2,223 steps; 5 falls, 17 cliff
  reactions, 2 time limits) and 1.1491 (2,424 steps; 4, 13, 7).
- **Open loop.** Fixed wheel commands with sensor noise off: the mean change
  in true forward speed and turn rate, in units of the reward's own widths
  (40 mm/s and 30 deg/s), and whether any of three runs at a table edge ends
  differently.

**The floor.** The same measurement on six tables that differ only in the
robot's mass, by 0.1 to 0.6 percent (a gram at most), moves the blind
controller's reward by up to 0.0039, the smoke policy's by up to 0.0310 and
the motion figure by up to 0.094. A change that cannot matter does that
much, so nothing below those figures means anything. The model is chaotic
at this level: a first attempt at the floor with changes of parts in a
million gave 0.011 for motion, a ninth of what any real change shows, and
would have passed nearly every entry.

| # | Parameter | Blind controller, low / high | Smoke policy, low / high | Motion | Edge runs that flip | Clears the floor |
|---|---|---|---|---|---|---|
| 1 | `com_x_mm` | +0.017 / +0.095 | -0.070 / -0.067 | 0.652 | 1 of 3 | blind controller, smoke policy, motion |
| 2 | `com_z_mm` | -0.003 / +0.007 | -0.035 / +0.011 | 0.091 | 0 of 3 | blind controller, smoke policy |
| 3 | `table_length_mm` | -0.019 / +0.023 | -0.031 / +0.023 | 0.098 | 0 of 3 | blind controller, smoke policy, motion |
| 4 | `contact_time_constant_ms` | +0.005 / +0.001 | -0.029 / +0.026 | 0.130 | 0 of 3 | blind controller, motion |
| 5 | `head_servo_kv_mNms_per_rad` | +0.001 / +0.003 | +0.013 / -0.028 | 0.085 | 0 of 3 | no |
| 6 | `head_box_mm` | -0.001 / +0.003 | -0.025 / -0.027 | 0.090 | 0 of 3 | no |
| 7 | `table_width_mm` | -0.007 / +0.017 | -0.025 / -0.009 | 0.101 | 0 of 3 | blind controller, motion |
| 8 | `wheel_mass_g` | +0.002 / -0.001 | -0.024 / -0.016 | 0.080 | 0 of 3 | no |
| 9 | `lift_servo_kv_mNms_per_rad` | +0.001 / -0.002 | -0.021 / -0.003 | 0.091 | 0 of 3 | no |
| 10 | `servo_torque_max_mNm` | -0.000 / +0.000 | -0.020 / +0.000 | 0.075 | 0 of 3 | no |
| 11 | `lift_servo_kp_mNm_per_rad` | +0.002 / +0.002 | -0.019 / -0.013 | 0.101 | 0 of 3 | motion |
| 12 | `lift_arm_mass_fraction` | +0.003 / +0.004 | -0.011 / -0.017 | 0.105 | 0 of 3 | blind controller, motion |
| 13 | `head_servo_kp_mNm_per_rad` | +0.002 / +0.003 | -0.013 / -0.015 | 0.095 | 0 of 3 | motion |
| 14 | `settle_time_ms` | +0.001 / +0.003 | +0.005 / -0.015 | 0.068 | 0 of 3 | no |
| 15 | `total_mass_g` | +0.003 / +0.003 | -0.013 / -0.015 | 0.096 | 0 of 3 | motion |
| 16 | `lift_mass_g` | +0.000 / +0.003 | -0.013 / -0.013 | 0.088 | 0 of 3 | no |
| 17 | `table_thickness_mm` | +0.003 / -0.001 | -0.012 / -0.002 | 0.085 | 0 of 3 | no |
| 18 | `tread_friction` | +0.004 / -0.002 | +0.012 / -0.000 | 0.106 | 0 of 3 | blind controller, motion |
| 19 | `sim_timestep_ms` | -0.005 / +0.000 | -0.011 / +0.000 | 0.070 | 0 of 3 | blind controller |
| 20 | `servo_armature_gcm2` | +0.004 / -0.001 | -0.010 / -0.010 | 0.090 | 0 of 3 | no |
| 21 | `chassis_ground_clearance_mm` | +0.002 / -0.000 | +0.004 / -0.007 | 0.092 | 1 of 3 | no |
| 22 | `head_mass_g` | -0.002 / -0.001 | -0.006 / -0.004 | 0.089 | 0 of 3 | no |
| 23 | `chassis_friction` | +0.004 / -0.002 | +0.001 / -0.002 | 0.000 | 0 of 3 | no |
| 24 | `table_height_mm` | +0.000 / +0.000 | -0.002 / +0.000 | 0.000 | 0 of 3 | no |
| 25 | `cliff_detect_range_mm` | +0.000 / +0.000 | +0.000 / +0.000 | 0.000 | 0 of 3 | no |
| 26 | `cliff_sensor_height_mm` | +0.000 / +0.000 | +0.000 / +0.000 | 0.000 | 0 of 3 | no |
| 27 | `drive_done_mm` | +0.000 / +0.000 | +0.000 / +0.000 | 0.000 | 0 of 3 | no |
| 28 | `heading_hold_gain_mmps_per_rad` | +0.000 / +0.000 | +0.000 / +0.000 | 0.000 | 0 of 3 | no |

How to read it:

- **`com_x_mm` is alone at the top.** It clears all three floors, the blind
  controller's by 24 times and the motion floor by 7. With the centre of
  mass forward the robot also falls off the edge at 200 mm/s with its
  reaction on, where the table's value does not.
- **Table size is next on the blind controller** (length 6 and width 4 times its floor):
  a longer table means fewer falls. That is a specification, not an unknown.
- **Everything else is at most about twice a floor**, and the floor is the
  largest of only six runs. `com_z_mm`, `contact_time_constant_ms`,
  `tread_friction`, `lift_arm_mass_fraction` and `sim_timestep_ms` are in
  that band. The order among them is not evidence.
- **The smoke policy is a poor probe.** Its floor (0.031) is eight times the
  blind controller's, and only three entries clear it, one of them by a
  hair. Entries 5, 6 and 8 to
  10 are high in the order on its figure alone and clear nothing.
- **Five entries have no effect at all**, by construction:
  `cliff_sensor_height_mm` and `cliff_detect_range_mm` (the sensor sees
  either the table a few mm away or a drop far beyond any range in the
  table), `table_height_mm`, and `drive_done_mm` and
  `heading_hold_gain_mmps_per_rad` (the task sends wheel speeds and never
  uses the drive actions).

What this is not. It is a screen. It moves one value at a time to bounds the
author chose, so it misses interactions and its order depends on those
bounds. It says what the model is sensitive to, not what is true of the
robot. `total_mass_g` is near the bottom because the firmware's speed loop
holds wheel speed whatever the load; it is still worth a minute on a scale,
because it turns the fitted motor strength from a ratio into a torque.

### What became of the 30

| Outcome | Count | Entries |
|---|---|---|
| Measured from the recording | 1 | `gyro_bias_dps` |
| Fitted from the recording | 1 | `accel_bias_mg` (and `imu_pitch_offset_deg`, which was not in the model before) |
| Never an unknown about the robot: now `chosen` | 7 | 4 table sizes, `sim_timestep_ms`, `contact_time_constant_ms`, `settle_time_ms` |
| Needs hands; high effect | 1 | `com_x_mm` |
| Needs hands; at or near the floor | 6 | `total_mass_g`, `com_z_mm`, `tread_friction`, `chassis_friction`, `chassis_ground_clearance_mm`, `lift_arm_mass_fraction` |
| No effect on this task | 4 | `cliff_sensor_height_mm`, `cliff_detect_range_mm`, `drive_done_mm`, `heading_hold_gain_mmps_per_rad` |
| Head, lift and wheel parts nothing in the task moves or touches | 10 | `head_mass_g`, `lift_mass_g`, `wheel_mass_g`, `head_box_mm`, `head_servo_kp/kv`, `lift_servo_kp/kv`, `servo_torque_max_mNm`, `servo_armature_gcm2` |

The firmware does publish the head and lift control gains (`head_power_gains`
and `lift_power_gains` in the table, from `headController.cpp` and
`liftController.cpp`), but in units of motor power. Turning them into the
model's torques needs the head and lift motors' strength, which no source
gives. They stay assumed. One ratio needs no strength: the lift's integral
term, Ki / Kp of the error summed every control tick, moves the lift servo's
target by that much, because the servo stands for Kp x error. The model runs
it as the firmware does (`tests/test_lift_integral.py`): without it the lift
stopped 2.7° short under the light cube and `set_lift_height` timed out, and
at the end of a move it sagged where the recorded robot holds past its
target (REC2, 60 mm commanded: 61.85 mm recorded, 59.50 mm before, 61.52 mm
with the term).

### The conferral with an external review lane

At its instruction the ranking method and the two recording findings were
put to an external review lane (one ask, answered on the Copilot route, no fall-through
to another provider). `evidence/astra-conferral-20261115.md` holds the ask,
the reply verbatim, both checksums, and what each point led to. In short: it
called the ranking a screen and not a priority list, warned that paired
episodes flip and a weak policy probes its own faults (the floor above
confirms both), said not to fit a physical value to the one turn-rate
number (followed), and said the accelerometer fit cannot cleanly separate
the frame's pitch from the x offset (confirmed: correlation -0.98). Its own
first three things to measure by hand were the cliff trigger margin, the
fore-aft centre of mass and the mass. Not done from its list: elementary
effects over randomised backgrounds, and draws that keep coupled values
together.

## Compared with the real robot

`./run_in_ct.sh replay_recording.py --sensitivity`; the output is in
`evidence/replay-20261115-tuned.txt`. Each commanded segment of the
recording is replayed on its own from rest: the same command at the same
moment, the same stop, and the model's reported signals read at the recorded
sample times. Compared: reported wheel speeds and gyro z for the tread
segments, head angle and lift height for the others.

The score is the mean, over a set of segments, of each error as a fraction
of the segment's own scale: wheel speed error over the commanded speed, plus
for spins the gyro error over the no-slip turn rate; head and lift error over
the joint's range. 0 is perfect.

**The robot's pose and yaw are not used as truth.** They are its own
odometry. Reported wheel speed takes its sign from the command, in the robot
and in the model. Times are receive times on the host.

### Before any tuning

| Model | Treads, fit set | Treads, held out | Head and lift, fit set | Head and lift, held out |
|---|---|---|---|---|
| Second version, as pushed (`e9110340`): ideal speed servo | 0.373 | 0.328 | 0.247 | 0.369 |
| Third version, untuned: the firmware's control law, motor strength assumed | 0.456 | 0.399 | 0.247 | 0.369 |
| Fourth version (the fit) | 0.170 | 0.144 | 0.019 | 0.052 |
| This version (IMU frame pitched; no tread value changed) | 0.170 | 0.143 | 0.019 | 0.052 |

That first row is the answer to "has the model been tested against
hardware": until this recording it had not, and it was off by a third of the
signal. Putting in the firmware's own control law with an assumed motor made
it worse, not better: the assumed motor was too weak (25 mN*m against a
fitted 100), so the treads were slow to break loose.

### Segment by segment

Root-mean-square error; wheel speed in mm/s (and as a fraction of the
command), gyro z in deg/s. "Before" is the untuned third version.

| Segment | Samples | Wheel speed before | after | Gyro z before | after |
|---|---|---|---|---|---|
| Fit set | | | | | |
| `spin_ccw_30` | 58 | 12.4 (0.41) | 2.5 (0.08) | 17.5 | 6.2 |
| `spin_cw_30` | 57 | 12.0 (0.40) | 2.0 (0.07) | 16.2 | 7.1 |
| `spin_ccw_100` | 41 | 24.2 (0.24) | 7.7 (0.08) | 32.7 | 20.3 |
| `spin_cw_100` | 41 | 26.7 (0.27) | 9.1 (0.09) | 41.0 | 19.8 |
| `back_40` | 27 | 13.4 (0.34) | 7.2 (0.18) | - | - |
| `forward_40` | 27 | 13.2 (0.33) | 7.1 (0.18) | - | - |
| Held out | | | | | |
| `spin_ccw_60` | 57 | 23.1 (0.38) | 4.7 (0.08) | 31.3 | 11.8 |
| `spin_cw_60` | 57 | 24.7 (0.41) | 7.1 (0.12) | 35.3 | 16.4 |
| `back_80` | 21 | 14.9 (0.19) | 6.6 (0.08) | - | - |
| `forward_80` | 22 | 13.3 (0.17) | 8.5 (0.11) | - | - |

| Segment | Samples | Before | After |
|---|---|---|---|
| `head_2` (fit) | 21 | 15.1 deg | 0.9 deg |
| `head_4` (fit) | 21 | 6.4 deg | 0.7 deg |
| `lift_1` (fit) | 21 | 25.2 mm | 1.9 mm |
| `head_3` (held out) | 21 | 20.1 deg | 3.8 deg |
| `lift_2` (held out) | 21 | 26.3 mm | 2.9 mm |

What is left: the gyro error in the 100 mm/s spins is about a fifth of the
turn rate, most of it the robot's left-right difference. The straight steps
are 0.4 and 0.7 s long and never reach a steady speed, so they test the rise
only.

### The turn in place

**A correction.** Earlier versions of this file said the robot turned at
0.40 to 0.50 of the no-slip rate (mean 0.46) and that the model agreed. The
first figure was read from gyro z alone. The gyro is in the head, and the
head was 22.4 degrees up in all six recorded spins, so gyro z showed only
the cosine of that, about 93 percent of the turn. From gyro x and z
together the six spins turn at 0.49, 0.51, 0.49, 0.53, 0.44 and 0.52 of the
no-slip rate: **mean 0.50** (`recording.spin_ratios`, 13 to 22 samples each).

The model turns at 0.44 to 0.46. At the recorded speeds it is short of the
recording's mean by 3.3 deg/s at 30 mm/s, 8.6 at 60 and 12.0 at 100, against
a reward width of 30 deg/s. The replay was not misled: it sets the model's
head to the recorded angle and compares gyro z with gyro z, like for like,
which is why the scores did not change. What was wrong was this file's
comparison of the model's true turn rate with the robot's gyro z.

None of the eight fitted values sets this ratio. In the model it follows
from the four wheel contacts and the centre of mass, and it is very
sensitive to `com_x_mm`, in a way that is not monotonic:

| `com_x_mm` | Spin fraction at 60 mm/s |
|---|---|
| -30 | 0.573 |
| -24 (the table's value) | 0.443 |
| -19 | 0.560 |
| -14 | 0.741 |

-24 is the middle of the wheel base. With the load shared equally by the
front and rear wheels the treads scrub most and the robot turns slowest;
move the centre of mass 5 mm either way and it turns a quarter faster. So
the assumed value sits exactly at the model's slowest point, and the
recording's 0.50 could be reproduced by a centre of mass about 2 mm either
side of it. **It was not fitted to that**, for three reasons: one number
cannot choose between two sides or between the centre of mass, the tread's
grip along and across, and the shape of the contact; four point contacts
probably overstate how much the load split matters for a real tread; and
the centre of mass can be measured directly with a straight edge (item 1
of "What hands can settle"). an external review lane gave the same advice unprompted.

An earlier version of this file also said full wheel speed was not in the
recording. It is, briefly, in the robot's own turn after its cliff reaction;
`recording.fast_turn_after_cliff` computes the figures quoted below under
the firmware disagreements.

## What is modelled

- **Body.** A chassis box, a head, a lift arm and fork. The chassis inertial
  is solved so that the whole robot's mass and centre of mass equal the
  table's (checked against the compiled model to 0.05 mm).
- **Treads.** Each tread is a front wheel (radius 14.5 mm) and a rear wheel
  (10.75 mm), 48 mm apart, coupled to turn at one surface speed. Turning is
  skid steering: the wheels slide sideways.
- **Tread drive, as the firmware runs it** (`simulacrum/drive.py`).
  - Encoder: 0.1269 mm of tread travel per count, counted whichever way the
    tread turns. Speed is counts over the time between the edges that bound
    them; no new count for 5 ticks reads zero, so a crawl below about
    5 mm/s reads as standing still. The reported speed is a whole number of
    mm/s.
  - The 0.96 factor: the firmware's scale carries a "96% corrective fudge
    factor" on the 29 mm wheel. The model takes 0.1269 mm per count as the
    true travel. The recording cannot check it: it has no distance measured
    from outside the robot.
  - Control law, every 5 ms: motor power = the firmware's feedforward line
    (0.00313 x speed + 0.057 above 30 mm/s, speed / 150 below) + 0.0005 x
    speed error + 0.00005 x summed error, limited to 0..1 in the commanded
    direction. Power is never applied against the commanded direction, and
    a command of 2 mm/s or less means no power: **the robot does not brake,
    it rolls to a stop on the drive's own drag.**
  - Motor: torque = stall torque x power - drag x wheel speed, against a
    constant friction. Stall torque, no-load speed, friction share and
    rotor inertia are fitted (above). Still assumed: that the motor is
    linear and that it drags at zero power.
- **Head and lift.** Hinges with position actuators and the robot's limits
  (head -22 to 45 degrees, lift 32 to 92 mm), a target that ramps at the
  SDK's trapezoidal acceleration profile (`10.0 rad/s^2`), command delay, and
  the firmware's lift integral term (`Ki / Kp` error sum in
  `simulacrum/robot.py`, tested by `tests/test_lift_integral.py`). Reported in
  whole encoder counts (0.123 and 0.301 degrees). The servo gains are still
  assumed.
- **Sensors.** Four downward range sensors at the cliff sensor positions,
  in the manual's order. One forward time-of-flight sensor, tilted up 6.5
  degrees, with its 25 degree cone and the robot's own reporting rules (see
  "The range sensor" below). Gyro and accelerometer at the
  IMU position in the head and in the head's frame, pitched by the fitted
  `imu_pitch_offset_deg`, with the noise measured at rest, the gyro's
  measured residual offset and the accelerometer's fitted offsets. Both pass
  through the IMU chip's low-pass filter at every physics step and are read
  once per 5 ms tick (`simulacrum/imu_filter.py`). A turn
  therefore shows on gyro z by the cosine of the head's pitch and on gyro x
  by its sine, as on the robot.
  A camera on the head, 50 degrees vertical field of view (and calibrated
  800×600 optics on Vector 2.0). The noise comes from a seeded generator: one
  seed, one run.
- **A free light cube** on the table at world coordinates (100, 0) mm with
  `VectorSim(cube="free")`. Its 62 shell and rubber contact boxes come from the
  [shelf replay fixture](evidence/unit3-cube-contact-replay/fixture.xml) with
  carved notch pockets (`evidence/cube-pickup/README.md`), and all
  `cube_shell_*` geoms use `contact_time_constant_ms` (`solref="0.004 1"`).
  The robot range sensor sees it and the cube sends accelerometer packets as it moves.
  Mass distribution, friction and corner stiffness are estimates. Pass
  `cube_app="mean"` to simulate the modified firmware. Decode its messages with
  `samples_g(rows(sim.cube.messages), counts_per_g, app="mean")` to preserve
  the mean firmware's fractional counts. The default selects shipped firmware.
  See [evidence/cube-pickup](evidence/cube-pickup/README.md) for pickup trials and
  [evidence/unit3-spin-cube](evidence/unit3-spin-cube/README.md) for wheelstand
  mount-impact replay.
- **A carried light cube**, on request (`VectorSim(cube="lift")`). It sits
  in the lift where the engine puts a carried cube. Its accelerometer
  sends the app's messages from the force at its chip
  (`simulacrum/cube.py`): the chip's filter, 12-bit frames every 2 ms, the
  FIFO, the last frame per 10 ms tick, and the tap counter as shipped. The
  cube's lights and radio are not modelled in Python physics.
- **Charger dock & shelf environment**, on request
  (`simulacrum/dock.py`, `simulacrum/autonomy.py`,
  `VectorSim(environment="p2_shelf", body_range_clock=..., charger_contact=...)`).
  Includes the charger mesh, electrical dock contact state, body-clock ToF
  acquisition scheduling, and optical-marker `NavMapGrid` explore and
  self-docking routines.
- **Goal-Masked NoMaD visual navigation** (`simulacrum/nomad_cam.py`,
  `nomad_model.py`, `nomad_onnx.py`, `nomad.py`). Calibrated `64x36` head-camera
  rendering, Transformer + FiLM 1D U-Net + DDIM sampler, ONNX export, and
  topological graph exploration/navigation.
- **Printed replacement parts**, on request
  (`printed_parts.vector_sim(parts=("fork:dauler", "treads:dauler-obar"))`,
  a `VectorSim`, or `printed_parts.build_mjcf` for the model alone). A shared
  printable fork, set of rims or set of treads replaces the stock part's
  contact shape, mass, inertia and rolling radius. The numbers come from the
  mesh (`simulacrum/printed_parts.py`). The meshes are fetched from their
  sources by hash and are not in this repository. Parts, licences and measured
  numbers are in [evidence/printed-parts](evidence/printed-parts/README.md).
  Without `parts` the model is unchanged.
- **Scene.** A table slab with real edges and a floor below; sizes from the
  table (600 by 600 mm, 300 mm high by default). Optional boxes.
- **Timing.** Physics at 1 ms, control at 5 ms (200 Hz), state messages
  every 60 ms, wheel commands delayed 60 ms, head and lift commands 105 ms and
  ramped at the SDK's 10 rad/s^2.
- **SDK surface.** `set_wheel_motors`, `drive_straight`, `turn_in_place`,
  `set_head_angle`, `set_lift_height`, `stop_all_motors`,
  `request_control`; `command_head_angle` and `command_lift_height` return
  at once. State: pose, wheel speeds, head angle, lift height, forward range
  with `found_object` and `is_lift_in_fov`, accelerometer, gyro, and the
  flags cliff detected (one bit for all four sensors), picked up, falling,
  wheels moving, has control.
- **Wheel speed sign.** The tread encoders have one channel, so the robot
  cannot sense direction. The reported wheel speed takes its sign from the
  command, even while the tread is still rolling the other way.
- **Cliff reaction**, with the switch for the two control levels.
  - `CONTROL_DEFAULT`: when a cliff newly appears while the treads are
    moving and not already stopping (the firmware's rule), power to the
    treads is cut, the running action ends with `cancelled_cliff`, control
    is taken from the SDK, and after 1.22 s the robot moves 29 mm away at
    205 mm/s (all three measured on the one recorded event). A cliff under
    the rear sensors makes it drive forward instead. Commands return
    `no_control` until `request_control()`, which is refused while the
    reaction is still running.
  - `CONTROL_OVERRIDE`: no reaction. The cliff bit still sets; the robot
    drives on and falls. This is what the real robot did on 2026-11-14.

## What is not modelled

- The turn of about 170 degrees to face the edge that the real robot made
  after backing away, the reaction's animation, the stuck-on-edge recovery
  and the white-line stop.
- The robot's left-right difference in a spin.
- Session-to-session variation in head and lift motion onset and raise profile
  (see [evidence/stock-shell-physics-20261118.md](evidence/stock-shell-physics-20261118.md)).
- A task built on `mjlab`'s own task classes (see Training below for why).
- Any connection to the Go gRPC simulator.
- Odometry error. The reported pose is the true pose.
- What the range sensor's target is made of, and how it is turned to the
  sensor: the signal depends on distance and on how much of the cone is filled
  only. A carried cube in the beam. The sensor's `rangeStatus`.
- Cliff sensing by brightness. Here a cliff is a range beyond
  `cliff_detect_range_mm`.
- The firmware's pick-up detection from gyro and accelerometer motion. Here
  picked up means three or more cliff sensors see a drop for 40 ticks.
- The battery's sag under load (modelled, but its size is assumed: the SDK's
  reading does not move), capacitive touch, motor stall protection, the lift's
  four-bar linkage physics, and the light cube's LEDs and radio.
- Tread compliance. The wheels are rigid cylinders, so the robot stands on
  four contacts. That is the likely reason the turn rate depends so sharply
  on the fore-aft centre of mass ("The turn in place"); a belt that carries
  load between the wheels would soften it.
- Draws that respect what is known together. Each ranged value is drawn on
  its own: the part masses independently of the total and of the centre of
  mass, and the accelerometer's x offset independently of its frame's pitch,
  although the fit fixes their sum far better than either.
- A roll of the IMU's frame (per-axis accelerometer scale is modelled via
  `accel_scale`, fitted on Vector 2.0 and nominal assumed on Vector 1.0).
- Noise on the cliff sensors, and any noise that changes with temperature
  or with motion. The model's gyro and accelerometer do shake while the
  robot moves, but from its rigid contacts, not from anything measured; the
  IMU chip's filter takes most of it out ("The IMU chip's own filter").
  "Measured in the model" sets that shake beside the recording's. The
  sensor noise is added after the filter, so it is independent from one
  5 ms reading to the next; the robot's is not. Within one
  turn direction the model's gyro x also carries a few deg/s that do not
  reverse with the turn; the mean of both directions is right to about a
  degree of frame pitch. Sensor readings that are refreshed less often than the
  state message.

Assumptions about robot behaviour made by the layer, not taken from a source:

- An action ends with `aborted_off_ground` once the robot reports falling or
  picked up.
- `drive_straight` and `turn_in_place` are steered by the robot's own 200 Hz
  loop after one command delay, on true pose and heading, with the default
  motion profile. The robot's own path follower is not reproduced, and the
  recording holds no such action to compare with.

## Measured in the model (a GPU host, 2026-11-15, table values)

These describe the model, not the robot.

| Run | Result | Third version, untuned |
|---|---|---|
| Drive 200 mm at 100 mm/s | ends at 203.88 mm | 213.88 |
| Drive -100 mm at 60 mm/s | -101.84 mm | -107.35 |
| Drive 50 mm at 20 mm/s | 49.57 mm | 50.78 |
| Drive 250 mm at 160 mm/s | 255.73 mm | 273.18 |
| Turn 90 degrees at 60 deg/s | 88.55 degrees, 3.83 s | 88.18, 9.17 s |
| Turn -180 degrees at 120 deg/s | -179.43 degrees, 3.83 s | -178.16, 7.65 s |
| Turn 30 degrees at 15 deg/s | 28.05 degrees, 6.83 s | 2.95, then out of time |
| Step to 100 mm/s | 90% of speed after 0.310 s | 0.305 s |
| Command cut at 100 mm/s | rolls on 12.3 mm | 20.5 mm |

Drives end within 6 mm; the faster ones overrun because nothing brakes.
Turns end inside the manual's 2 degree tolerance. These nine figures are
unchanged in the 2026-11-16 run (`evidence/measure-20261116-output.txt`).

**The model's accelerometer shakes far more than the robot's.** Scatter in
the second half of a held spin, head 22.4 degrees up, the axis with the
most (21 samples for the model, 13 to 22 for each recorded spin):

| Spin | Gyro, model | Gyro, recording (two directions) | Accelerometer, model | Accelerometer, recording |
|---|---|---|---|---|
| 30 mm/s | 7.3 deg/s | 5.6 and 1.9 | 723 mg | 32 and 19 |
| 60 mm/s | 10.9 deg/s | 11.3 and 3.4 | 884 mg | 71 and 52 |
| 100 mm/s | 22.7 deg/s | 18.8 and 5.2 | 1058 mg | 185 and 124 |

The gyro is of the right size. The accelerometer is 6 to 38 times too
rough: rigid wheels on a rigid table, read once every 5 ms with no filter.
A policy trained here will learn that the accelerometer carries almost
nothing while moving, which is safe to carry over but wastes a signal the
real robot has. Not fixed in this version. The steady offsets fitted above
are small beside this shake, so they matter for a policy mainly when the
robot stands still.

Since 2026-11-20 the model reads its gyro and accelerometer through the IMU
chip's filter. The accelerometer's shake is 1.4 to 5.7 times the robot's;
without the filter, in the same tree, it is 5 to 43 times ("The IMU chip's
own filter").

### The two disagreements with the firmware's own figures

Both are still there. Nothing was tuned toward either figure: the fit scored
reported wheel speeds and gyro against the recording and nothing else.

| Firmware says | Second version | Third, untuned | This version | Real robot, this recording |
|---|---|---|---|---|
| At full wheel speed the robot spins at about 360 deg/s | 242 deg/s | 230 | 250 deg/s, 0.46 of the no-slip rate | 0.44 to 0.53 of the no-slip rate (mean 0.50) in held spins at 30 to 100 mm/s. At full speed only a 0.12 s transient: 276 to 301 deg/s |
| 170 mm/s is the fastest speed that does not fall off most straight edges, reaction on | No fall up to 220 mm/s | Falls from 150 mm/s | No fall at 50 to 220 mm/s; front axle 0.7 to 16.0 mm past the edge | One event at 54 mm/s: no fall, rolled on 1.1 mm after the flag by its own odometry |
| The turn centre is 25 mm behind the front axle | 23.1 mm | 23.7 mm | 23.8 mm | Not measurable: the 25 mm circle in the robot's pose is this same firmware constant |

- **Spin.** The real robot turns at about half the no-slip rate at 30 to
  100 mm/s; the model at 0.44 to 0.46 (see "The turn in place"). If the
  robot's 0.50 held to full speed it would turn at about 274 deg/s, not 360.
  The one glimpse of full speed in the recording is the robot's own turn
  after its cliff reaction: for three samples (0.12 s) the treads report 210
  to 235 mm/s and the gyro, x and z together, 276 to 301 deg/s, which is
  0.49 to 0.56 of the no-slip rate. That is above the model's 0.46 and well
  below the 0.66 that 360 deg/s would need. It is a transient the robot
  commanded itself, so it cannot say what a held spin settles at: a spin at
  150 and 220 mm/s would (sequence below). The firmware's figure is the
  outlier against this robot on this surface, and the model was not moved
  toward it.
- **Edge.** The direction changed twice. With the ideal servo the model
  braked hard and never fell. With the firmware's law and a weak assumed
  motor it rolled on and fell from 150 mm/s. With the fitted motor (more
  drag at zero power) it rolls on less and holds at every speed tried. The
  recording has one edge event at low speed, so it neither supports nor
  contradicts the firmware's 170 mm/s. The model's margin depends on
  `cliff_sensor_height_mm` and `cliff_detect_range_mm`, both assumed.

`measure.py` also prints both figures at other stall torques. Only the
table's value is used anywhere else:

| `wheel_stall_torque_mNm` | Spin at full speed | No fall with the reaction on at |
|---|---|---|
| 50 | 231 deg/s | 50 to 170 mm/s; falls at 220 |
| 100 (the table's value) | 250 deg/s | every speed tried, up to 220 mm/s |
| 200 | 253 deg/s | every speed tried, up to 220 mm/s |

No stall torque in that set comes near 360 deg/s: the spin rate is set by
tread slip, not by motor strength.

One defect found while building the first version, kept here because it
will recur for anyone who lowers `wheel_armature_gcm2`: with a rotor inertia
of 10 g*cm^2 the contact solver applied 0.12 N of tread friction where
0.28 N was available. From about 300 g*cm^2 up the friction is applied in
full. That figure was measured with the first version's drive.

## Training

The stack is the Microduck's except for one piece: PPO from `rsl_rl`, an
actuator model, domain randomisation and ONNX export, but the environment is
this directory's own, not `mjlab`'s.

**Changed in the fourth version: the policy step is 60 ms, not 30.** It follows
`state_period_ms`, which the recording showed to be 60 ms. An episode of
10 s is now 167 policy steps, not 333. The observation layout did not
change: the same 17 numbers in the same order. A policy exported before this
version loads, but it was trained at twice the rate on a different robot and
should be retrained.

### The task

Hold a commanded forward speed (0 to 160 mm/s) and turn rate (-120 to 120
deg/s, the household's directive limits) on the table without falling off.

- **Policy step: 60 ms**, the robot's state period as the SDK delivers it.
  A wheel command takes about that long to act, so a faster policy would act
  on observations it does not have. The robot's own 5 ms loop is not
  reachable through the SDK.
- **Observation (17 numbers):** left and right wheel speed, gyro (3),
  accelerometer (3), the cliff bit, forward range and whether it found
  something, head angle, lift height, the two commanded values, and the
  policy's own last action. Only what the robot reports, scaled by fixed
  numbers from the table. No pose, no true speed, no individual cliff
  sensors, no falling or picked-up flag.
- **Action (2 numbers):** left and right wheel speed, each clipped to
  -1..1 and multiplied by `wheel_speed_max_mmps` (220), sent with
  `set_wheel_motors`.
- **Reward per step:** up to 1.0 for forward speed (error scale 40 mm/s)
  plus up to 0.5 for turn rate (error scale 30 deg/s), minus 0.05 times
  the change in action. A fall costs 20 and ends the episode. The reward
  uses true speeds; the policy never sees them.
- **Episode ends:** a fall (the policy's failure), the robot's own cliff
  reaction (not the policy's doing, passed to PPO as a time-out), or 10 s.
- **Control level:** 75% of episodes at the default level, 25% at the
  override level, where nothing but the policy can stop the robot at an
  edge. Deployment should still be at the default level.

These numbers are design choices in `TaskConfig`, not properties of the
robot. One of them asks for something the model cannot do: it spins at
250 deg/s with the treads at 220 mm/s each way, so a turn rate of 120 deg/s
takes about 106 mm/s each way, and a forward speed of 160 mm/s on top of
that passes the 220 mm/s wheel limit.

### Domain randomisation

Every episode draws a new robot through the same path as
`VectorSim.randomised`: 63 ranged parameters, inside their ranges in
`parameters.json`. That is every assumed parameter but one, all 8 fitted
ones, the 6 measured ones that carry a range, and the 20 published ones
that carry one. The one left out is `sim_timestep_ms`: it must divide the
control period. The start position, heading, head angle, lift height,
command, control level and the noise seed are drawn too.

The ranges on the fitted and measured values are the author's, around one
robot's one run. They are not a spread measured across robots.

### Why not mjlab's task classes

`mjlab` steps many robots as one MuJoCo Warp batch on a GPU. The container host has no
GPU, and what a policy has to be trained against here is the SDK layer:
the command delay, the 60 ms state rate, one cliff bit for four sensors,
wheel speed without a sign, and the robot's own cliff reaction taking
control away. That layer is Python around one MuJoCo model per robot, and
it is the part with tests. So the environment is a small vectorised pool
over `VectorSim` (`simulacrum/vec_env.py`), with up to 4 worker processes.

A move to `mjlab` on the GPU node would need:

1. A dedicated GPU container/environment with GPU passthrough and sufficient disk space.
2. The robot as an `mjlab` entity, built from `mjcf.build_mjcf`, with the
   same actuators.
3. The SDK layer rewritten as batched terms: command delay, state rate,
   the firmware's tread loop and encoder, the cliff flag, the cliff
   reaction (which runs at 5 ms, inside the policy step), the unsigned
   wheel speed, the fall and pick-up flags.
4. Randomisation per robot inside one batch. Mass, friction and gains can
   be set per world; wheel radius, tread spacing and sensor positions
   change the geometry and may need several model variants.
5. A check that the `mjlab` environment and `VectorSim` give the same
   motion for the same actions at table values, and the same replay scores
   against the recording, before any policy trained there is believed.
6. Confirmation that MuJoCo Warp supports what the model uses: range
   sensors against a box, the elliptic friction cone, joint couplings.
7. A MuJoCo that agrees with the reference. mjlab 1.6.0 and 1.6.1 (the
   latest on 2026-11-22) both require `mujoco~=3.11.0` and
   `mujoco-warp~=3.11.0`, so mjlab runs on 3.11.0, a different model from
   the 4.0.1 the table is checked on ("Which MuJoCo: 4.0.1" above). Either
   wait for an mjlab that allows 4.x, or check item 5 against 4.0.1.

### One thing that bites: import order

In a GPU host's environment, importing `mujoco` before `tensordict` ends in a
segmentation fault. Ten import orders were tried: the 4 with `mujoco`
before `tensordict` crashed, the other 6 did not. `simulacrum/training.py`
must be imported before anything that imports `mujoco`, and raises an
error instead of crashing if it is not. `tests/_common.py` loads
`tensordict` first for the same reason.

### Smoke runs

`./run_in_ct.sh smoke.py`: 150 PPO iterations, 32 robots, 32 steps each,
3 worker processes, CPU only, 153,600 environment steps.

**None of these is a trained policy.** They show that every stage runs and
that the reward goes up. Do not put one on a robot.

| Run | Model | Policy step | Reward per step, by thirds of the run | Falls at the override level, last third |
|---|---|---|---|---|
| 2026-11-14 | Second version (ideal servo, no noise) | 30 ms | 0.695, 0.877, 1.002 | 22 of 41 episodes |
| 2026-11-15 | Fourth version | 60 ms | 0.722, 0.894, 0.925 | 55 of 115 episodes |
| 2026-11-16 | This version (accelerometer offsets, pitched IMU frame, narrower gyro offset) | 60 ms | 0.700, 0.855, 0.918 | 39 of 94 episodes |

The most a step can earn is 1.5. The 2026-11-16 run: 1,233 episodes, of
which 122 ended in a fall (281 were at the override level), 505 in the
robot's own cliff reaction and 606 at the time limit; 423 s of wall time,
363 environment steps a second. Curves: `evidence/smoke-20261114-curve.json`,
`evidence/smoke-20261115-curve.json` and `evidence/smoke-20261116-curve.json`;
the last two runs' full outputs are `evidence/smoke-20261115-output.txt` and
`evidence/smoke-20261116-output.txt`.

The exported policy, run with `onnxruntime` only, against two baselines on
16 robots it had not seen (3,200 environment steps each, same seeds):

| Run | ONNX policy | Hand-written no-slip controller | Standing still |
|---|---|---|---|
| 2026-11-14, second version | 1.187 | 1.195 | 0.510 |
| Third version, untuned (run once, not kept) | 0.852 | 1.074 | 0.316 |
| 2026-11-15, fourth version | 1.116 | 1.156 | 0.360 |
| 2026-11-16, this version | 1.029 | 1.125 | 0.360 |

In the last run the policy's forward error was 17.4 mm/s and its turn error
32.6 deg/s, with 3 falls in 18 episodes; the controller's 7.7 mm/s and
33.0 deg/s, with 4 falls in 23.

The 2026-11-16 policy lands further under the controller than the one
before (0.91 of its reward against 0.97). One run each, 16 robots: that is
not enough to say the new sensor values made learning harder. What changed
between the two is only what the accelerometer and gyro report at rest; the
robots move the same.

**Did the smoke result change?** The numbers did, the finding did not. The
reward still rises, the exported policy still lands just under the
hand-written controller (0.97 of its reward; 0.99 before), and it still has
not learned to stop at an edge (about half of the override episodes end in
a fall in both runs). The runs are not comparable number for number: a step
is twice as long, so the same 153,600 steps cover 154 minutes of robot time
instead of 77 and a 10 s episode is 167 steps; and the robot is a different
one (the firmware's drive, the fitted motor, sensor noise). The fall counts
in the comparison (1 and 2 of 20) are too small to compare.

ONNX against torch, in a process with no torch in it: the same recorded
observations give the same actions (120 observations, largest difference
3.0e-7 in the 2026-11-16 run, 4.8e-7 in the one before). A closed loop
repeated from the same seed is reported but is no longer a pass criterion.
In the 2026-11-16 run the first 77 of 120 steps repeated, in the one before
all 120; in an earlier
run of the same check a difference of 1e-7 in one action grew until the two
loops parted within about ten steps. The check that remains is the one that
matters for deployment: same observation, same action.

### What a real training run needs

- Far more steps than a smoke run, on the GPU node.
- A reason for the policy to stop at edges that it can act on. Today it
  sees one cliff bit, after the front sensors are already past the edge.
- The further recording below. Until then the ranges are one robot's.
- A command range the robot can follow (see the task above).
- A longer look at the reward: the weights are first guesses.
- A check on the real robot that is not a drive: replay the recording's
  states through the ONNX policy and look at the wheel speeds it asks for.

## Tests

### Current suite

- **`tests/run_all.py`**: **36** test modules (**485** declared tests), plus
  standalone `tests/test_cube_pickup.py` (**10** tests; **37** `test_*.py`
  files total). On pinned MuJoCo 4.0.1 without Anki's optional proprietary
  `cube.dfu` / `key.h` or optional fetched meshes: `declared 485, ran 485,
  passed 442, failed 0, errored 0, skipped 43 (16 optional)`, and
  `test_cube_pickup.py`: `10/10` passed.
- **`tests/mutate.py`**: **273** fault arms (**93** requiring MuJoCo, **180**
  without MuJoCo) plus **1** neutral control arm (`NEUTRAL`).
- **Browser suite (`web/`)**: `npm test` runs **133** tests across **30**
  `*.test.js` files (**128** passed, **0** failed, **5** skipped without optional
  fetched SDK/firmware files); `npm run mutate` runs **87** arms (**85** faults +
  **2** clean/neutral controls, **87/87** as required); `npm run check`
  (`export_model.py --check`) verifies **28/28** exported files against
  `parameters.json` and `parameters-vector2.json`.

### Historical test runs

`tests/run_all.py`, 348 tests in twenty-five files. Results after the
peer-capture tool, 2026-11-21 (`evidence/peer-capture-20261121/`):

- In a GPU host, with `--require-mujoco`: declared 348, ran 348, passed 340,
  failed 0, skipped 8, all of them optional inputs; exit 0.
- On the Mac, without Anki's cube file and key: passed 235, skipped 113,
  because MuJoCo is absent.
- The mutation check on the Mac: 143 of 143 arms caught, 0 survived, 0 set-up
  errors. That includes the 15 new arms for `capture_peers.py` and
  `save_capture.py`. The 71 arms that need MuJoCo were not run there.

Results after the cube app, 2026-11-21, when the suite was 332 tests in
twenty-four files:

- In a GPU host: declared 332, ran 332, passed 324, failed 0, skipped 8
  (logs in `evidence/cube-firmware-20261121/`).
- On the Mac, with unit 3's `cube.dfu` and `key.h` given: declared 332, ran
  332, passed 226, failed 0, skipped 106, because MuJoCo is absent.
- On the Mac without them: passed 219, skipped 113.

The eight skips in a GPU host are the cube app's image tests, which need
Anki's file and key, and the C tick's host build, since a GPU host has no C
compiler. Skips like these are marked "optional input" (`tests/_common.py`).
`--require-mujoco` still fails on any other skip; `tests/test_run_all.py`
checks that. The runner says which tests it skipped. (The 2026-11-16 version
had 170 tests; the figures below describe what was added at that time.)

The contact trial's run E check holds each spin's mean over eight turns of the
table to 2 %, not one orientation's rate (2026-11-22). Turning the table
changes nothing physical, yet it moves each spin's rate with a standard
deviation of about 1.4 %. One orientation is a draw from that spread; the
mean is what the limit was meant to judge. Spans of 43.5 and 48 mm fail the
averaged check:
[Table turns](evidence/stock-shell-physics-20261118.md#table-turns).

What they cover: the table's rules, with the new ones (a measurement or a
fit must cite the recording; a fit needs a range; the table describes
exactly the classes the code allows); the measured entries
equal what `recording.py` computes from the files; the files' SHA-256; the
MJCF follows the table; the compiled model's mass and centre of mass; the
encoder and the control law against the firmware's arithmetic; straight
drives and turns; the command delays; the 60 ms state rate; head and lift
limits and rates; the range sensors against the geometry; the cliff
reaction on and off, with the measured pause and distance; sensor noise at
the table's level; identical seeds give identical runs; five robots drawn
from the ranges still drive. For the comparison: the replay scores stay
under 0.19 (fit set) and 0.17 (held out) for the treads and 0.03 and 0.07
for head and lift; the values from before the fit score clearly worse on
the held-out segments; the fit set and the held-out set share no segment;
head and lift rates change nothing while neither is commanded. For training: the
observation reads only what the robot reports; the policy step is one state
period; every ranged parameter but the timestep is varied; a fall is a
failure and a cliff reaction is not; worker processes do not change the
result and more than four are refused; a tiny PPO run writes its files; the
ONNX policy gives the torch actions; the torch-free runner really has no
torch and fails when handed a different policy. For the next recording's
script, run against a stand-in SDK and a stand-in clock: the phases, their
order, speeds and durations are the table's below; the reversal sends no
stop between its halves; no wheel command passes 220 mm/s and each is
stopped; nothing is commanded without control; a cliff, a pick-up or loss
of control ends all motion, after which the script only listens; control is
read from the event that clears, not the flag that stays set; the edge
phase refuses to start without the ruler distance.

New in this version. For the recording: a turn is read from gyro x and z
together; the gyro follows the tangent of the head angle; the spin fraction,
the gyro's offset while still and the scatter in motion are what
`recording.py` computes. For the accelerometer fit: the table's values are
the fit's; the fit recovers made-up offsets at the recorded head angles and
headings; an offset on x is not mistaken for a pitched frame; the ranges
cover the bootstrap and the correlation is as reported. For the model: at
rest the accelerometer reads gravity in the head's pitched frame plus the
offsets, at four head angles, to 3 mg; a spin shows on gyro x by the tangent
of the head's pitch. For the ranking: a variant moves one value and nothing
else; change is measured in the reward's widths; the order is by reward
change; an entry clears the floor only by beating the null runs; the same
table twice gives no change; a value the task never reads changes nothing.
For the table: scene and solver settings are `chosen`, and nothing about the
robot is.

`tests/mutate.py` breaks each property once. On 2026-11-20 in a GPU host, after
the listen-only recorder: 173 of 173 injected faults caught, each by a test
named in advance, 0 survived, 0 set-up errors. The 151 tests the arms name
pass on an untouched copy first, and a wording-only change leaves all 275
green. Output: `evidence/mutate-20261120-listen-output.txt`. The three new
arms make `hardware/record_listen.py` ask for control, never put quiet mode
back, and lift quiet mode that was off.

The run before, after the IMU filter, caught 170 of 170
(`evidence/mutate-20261120-imu-filter-output.txt`). Its nine new arms
break the filter's corners, its second section, its units, the typed
cutoff, the robot's reading of it, its step rate, its reset after settling,
the wheelstand's reset, and where the model's turn centre is read. On the
way there, the arm that types the inset in by hand still looked for 3.0
and reported a set-up error, not a verdict; it now looks for 3.1.

The run before, after the contact inset, caught 161 of 161
(`evidence/mutate-20261120-inset-output.txt`). On the way to that one, one
arm survived: with the wheels' friction typed in by hand, the new tip
supports still read the table, so "every ranged parameter changes the
model" stayed green. A test of the wheels' own friction now catches it.
Earlier runs: `evidence/mutate-20261120-output.txt` (152 arms, before the
inset),
`evidence/mutate-20261116-fifth-version-output.txt` (95 arms),
`evidence/mutate-20261116-output.txt` (78) and
`evidence/mutate-20261115-output.txt` (67). The first 2026-11-20 run had one
set-up error. With the x scale taken out of the accelerometer fit, the
tests crashed on a singular matrix instead of failing, so the harness did
not count it as caught. The fit now asserts that every number it fits
moves the readings, and a test holds it to that.

`run_in_ct.sh` holds a lock in the container for the whole run (see its
header). Before the lock, a sibling lane's copy replaced this one in the
middle of a suite run, and 78 tests errored.

That is the second full run. In the first, 94 arms were caught and one, "IMU
fixed to the body, not the head", was a set-up error: its search text held
real line breaks where the source has the two characters backslash and n, so
it matched nothing. The harness said so instead of counting it either way.
While the arms were being written, three more ended as set-up errors for a
different reason (the fit ran in the tests' set-up, so a broken fit stopped
the tests from running at all) and one because its text occurred twice. All
five were faults in the check, not in what it checks, and were fixed there.

## The next recording

The recording settled the timing, the tread response up to 100 mm/s, and
the sensor noise at rest. This sequence would settle most of what is left.
Same recorder, same robot, default control level, a table with a barrier or
a soft landing on every side, state messages logged for the whole run with
2 s of standing still between items.

| # | Sequence | Settles |
|---|---|---|
| 1 | Awake, off the charger, standing still for 20 s | Sensor noise awake; gyro and accelerometer bias |
| 2 | Straight steps held for 2.0 s: forward then back at 50, 100, 150 and 200 mm/s. Mark the start and end of each on the table and measure with a ruler | The steady speed and the rise above 100 mm/s (`tread_noload_speed_mmps`); the roll-on after the stop; the encoder scale and its 0.96 factor against the ruler |
| 3 | Spins held for 3.0 s, both ways, at 150 and 220 mm/s. Count the turns against a mark on the table | The 360 deg/s question; the gyro against ground truth; whether the left-right difference grows |
| 4 | Forward at 100 mm/s for 1.0 s, then straight to back at 100 mm/s for 1.0 s, no stop between | Drag and the absence of braking; the reported sign while the treads still roll the other way |
| 5 | Arcs for 3.0 s: left 60 and right 120 mm/s, then mirrored | Slip when the treads differ but do not oppose |
| 6 | Head from -22 to 45 degrees and back, twice; then to 10 degrees and back. Lift from 32 to 92 mm and back, twice; then to 60 mm and back | The head and lift motion profile (why large moves are faster) |
| 7 | `drive_straight` 100 mm at 50 and 100 mm/s, and `turn_in_place` 90 degrees at 60 deg/s, both ways | The robot's own path follower: overrun, turn time. Nothing in the first recording used these actions |
| 8 | Toward a straight edge, reaction on, at 30, 60, 100 and 140 mm/s, three times each, one approach per run. Before each, with the robot standing square to the edge, measure by ruler from the front wheels' contact line to the edge. Keep recording for 6 s after the cliff flag. Stop raising the speed at the first run that leaves the wheels over the edge | The edge disagreement; `cliff_detect_range_mm` with `cliff_sensor_height_mm`; the reaction at more than one speed |
| 9 | Repeat one 60 mm/s spin and one 80 mm/s step at the very end | Drift with battery and warmth inside one run |
| 9a | **Added.** Set the head to 0 degrees before every spin and say so in the file; one pair of 60 mm/s spins with the head at 0 and one with it at 22 degrees | The turn rate without the head-frame correction; the correction itself |
| 9b | **Added.** Standing still for 3 s at each of head -20, 0, 20 and 40 degrees, at each of four headings a quarter turn apart (16 stops) | Untangles head angle from heading in the accelerometer fit; separates the frame's pitch from the x offset |
| 9c | **Added, if a phone can be fixed overhead.** Video of items 3 and 5 with a mark on the robot | Turn rate and turn centre against ground truth, which no signal in the robot gives |

### Wheelstands: the centre of mass from two balance angles

The centre of mass along the robot (`com_x_mm`) is the assumed value training
is most sensitive to. A wheelstand measures it with the robot's own
accelerometer and nothing else.

A body standing on a round wheel balances when its centre of mass is straight
above the axle, because on flat ground a circle touches under its centre.
Tipped back onto its rear wheels, the robot balances at the pitch where
`tan(pitch) = (com_x + 48) / (com_z - 10.75)`. Tipped onto one tread's outer
edge it balances where `tan(roll) = edge / com_z`, with the edge at half the
width across both treads. The side stand gives `com_z`; the rear stand then
gives `com_x`. The table's assumed (-24, 22) mm predicts **64.9 degrees** on the
rear wheels and **51.8 degrees** on the edge.

Checked in the model (`wheelstand_check.py`, a GPU host). Releasing it from rest
and bisecting which way it falls, the model tips at 64.82 degrees on its rear
wheels against 64.90 from the formula, and at 51.855 on the edge against 51.854.
Nothing but the standing wheels touches the table at either angle. The
model's accelerometer, held still at those angles and read through
`wheelstand.tilt_deg`, gives 64.91 and 51.81.

**What the measurement needs** (executed, `solve_interval`): with the
edge distance from the table's ranges, ±0.5 degrees on both angles gives
`com_x` anywhere from -30.6 to -17.0 mm, no better than the range already
assumed. A ruler across the outside of both treads (to ±0.5 mm per edge)
narrows that to -28.2 to -19.6 mm. The angles alone are worth -25.4 to -22.6;
what remains is the table's ranges for the wheel base and the rear wheel
radius. One degree on the rear stand moves `com_x` by 1.1 mm; one degree on the
side stand moves it by 1.7 mm (and `com_z` by 0.8), as does 1 mm on the edge,
because `com_z` carries into `com_x`. The report gives both intervals.

**How to do it** (a person; the robot drives nothing):

1. On a soft, flat surface, run the recorder with `PHASES=wheelstand`
   (`WHEELSTAND_S`, default 180 s, at most 300). It sets the head level and
   the lift down, then commands nothing and records.
2. Leave it standing for 2 s. Then, for the rear stand: tip it back onto its
   rear wheels to an angle near 65 degrees, hold it still about a second,
   and let go. It falls back onto its treads or over onto its back. Repeat
   on both sides of the balance point, closing in. About eight releases.
3. The same onto one tread's outer edge, near 52 degrees, either side.
4. Measure the width across the outside of both treads with a ruler.
5. `wheelstand_balance` (MCP) or `python3 -m simulacrum.wheelstand <file> <width>`.

Releases that fell each way bracket the balance angle, and the middle of the
bracket is the estimate. Holds where a hand moved the robot away slowly count
as "held" trials and are used only when there is no bracket: a finger pushes.
A trial with the head off level or the lift off its lowest is set aside; the
firmware may move them while the robot is held, which is not known.

Not yet measured: no wheelstand recording exists. The accelerometer reads
1.039 g at rest; a common scale cancels in the angle, a scale that differs
between axes would not, and that is not known either.

### Against the hardware: the run when the robot is back

The operator's go, 2026-11-18: when the robot is back within reach of its cube, this
lane runs the recorder itself on the wire-pod host and adjusts the table.
The order, each a separate run of `hardware/record_barriered.py`:

1. `PHASES=rest,headlift,spins`: the turn rate at 150 and 220 mm/s against the
   model's (`spin_fractions`, `replay_recording.py`).
2. `PHASES=wheelstand`, if a person is there to tip it (the section above).
3. `PHASES=rest,wheelie`: the cube within reach and the room lit. The script
   connects the cube (waits up to 12 s; with no cube it commands nothing),
   records the robot's and the cube's poses, starts `pop_a_wheelie` and then
   only records for `WHEELIE_WATCH_S` (default 8). Once the lift leaves the
   cube the robot swings on its rear wheels, and where its angular
   acceleration changes sign is the balance angle. The analysis of that
   swing waits for the first recording: whether the treads lock and when
   the lift leaves the cube are not known, and a model of the move written
   before seeing one would be a guess.

Every value changed after that is filed `measured` (a direct reading) or
`fitted` (a fit to these runs) with its evidence, and the ranking, replay
and tests are rerun.

### The wheelie on unit 3, and why it lost power undocking (2026-11-20)

The operator gave unit 3 a new cube for the wheelie run above. Six runs, all in
`evidence/hw-rec-20261120/`; none produced a wheelie.

**Runs L and M: the undock is a power loss.** Both started on the charger,
so `record_barriered.py` called the SDK's `drive_off_charger`. In both the
state stream stopped 8.0 s (L) and 8.1 s (M) after the call, and the call
failed about 78 s later with the connection reset. Unit 3's own log after
run M's reboot names the cause. Its PMIC gave the power-on reason as
"Triggered from SMPL (sudden momentary power loss) and 'cold' boot" and the
power-off reason as "Triggered from UVLO (Under Voltage Lock Out)". Its
uptime puts the boot 19 s after the stream stopped. The log shows the
charger contact opening 8 s into a drive-off: the head browns out when the
contacts open and the battery takes the load. The battery was full on the
charger (Home Assistant: "full" all afternoon, 4.02 to 4.06 V after).

The same robot undocks itself without losing power. At 22:31:44Z it drove
off its own charger after an SDK session ended, with the same behaviour the
SDK uses (`DriveOffChargerStraight`, named in `SDKDefault.json`). The SDK's
drive-off lost power in two of three tries (runs L and M; run O kept it).
The robot's own undock kept power all three times (22:31:44Z, the
look-at-me intent below, run Q). The trigger is not isolated. Nothing
upstream matches: purplOS and wire-os-victor have no fix for this.
os-vector/wire-os (`ded117a758`) force-reflashes 2.0 body boards running
`DevBuild9c26f10`; unit 3's body version string reads `Ws00000009c263e`, and
whether that applies is not known.

**The robot's own undock, by intent.** wire-pod's `/api-sdk/cloud_intent`
sends an app intent. Unit 3 drops two kinds of these: an intent with no
`app_intent` entry in its `user_intent_map.json` (`intent_imperative_forward`
is one), and any intent while quiet mode is on, because wire-pod then
holds the behaviour reserve. With quiet mode lifted through the pod,
`intent_imperative_lookatme` undocked unit 3 in 9 s on the same boot (uptime
1689 s before, 1710 s after).

**Runs N and O: the SDK's wheelie.** Both connected the cube
(`02:00:00:00:00:01`). In N, started off the charger after the look-at-me
undock had turned the robot away, `pop_a_wheelie` returned
`VISUAL_OBSERVATION_FAILED`. In O, which started on the charger and
undocked through the SDK without losing power, the robot saw the cube
132 mm ahead and `pop_a_wheelie` returned `BAD_OBJECT` at once.

**Runs P and Q: the robot's own wheelie, listened to.** The wheelie
imperative `intent_play_popawheelie` has no `app_intent` on unit 3, so the pod
cannot send it. The engine's test runner can: a RobotTest script with one
`cloudIntent` command, uploaded to `persistent/robotTestScripts/` and run
with `GET 127.0.0.1:8888/robottest?run=<name>`, sets the intent pending as
a voice answer would (`engine/robotTest.cpp`). `hardware/record_listen.py`
records meanwhile: it connects with no behaviour control, so the robot's
own behaviours run, and lifts quiet mode for the recording and restores it.
Run P listened 240 s for a spoken command that never came. In run Q the
script sent the intent. Unit 3:

1. started its voice-command cube search;
2. undocked itself (contact open at t = 25.0 s, no power loss);
3. connected the cube;
4. started PopAWheelie, which ended in the same tick;
5. failed its paths (`executePath result=1`, then
   `DrivingActionFailedWithAbort`).

Over run Q's 2505 state rows, 60 ms apart, pitch stayed between -11.5
and +9.6 degrees. A retry needs the cube on open surface with room for the
robot to line up and tip back. The script file was deleted afterwards.
The engine keeps the loaded copy in memory until its next restart.

### The cube: held, mapped, and read directly (2026-11-21)

**Holding it.** The robot connects its cube only while something asks it to.
The SDK's `connect_cube` asks for 60 s
(`appCubeConnectionSubscriber.cpp`). After that the connection drops to a
background one and then goes (`cubeConnectionCoordinator.cpp`).
`hardware/hold_cube.py` asks again every 30 s and records the cube's
events.

- **Run R:** both renewals failed, because `connect_cube` needs behaviour
  control.
- **Run S:** connected at `RESERVE_CONTROL`, the lowest level, and held the
  cube for 877.5 s on unit 3's charger. All 29 renewals succeeded. It
  recorded 40 taps, 22 up-axis changes, 11 moved and 11 stopped-moving
  events, and 30 connection-state events. The cube cell read 1.568 V in all
  15 battery rows.

So a docked robot does hear a held cube. The files are in
`evidence/hw-rec-20261121/`.

**Its pins.** `evidence/cube-firmware-20261121/PINMAP.md` maps the cube's
pins from the app the robot sends it, decrypted and disassembled. It also
records the accelerometer setup: ±4 g, 250 Hz bandwidth, FIFO, no
interrupt line.

**Power during capture.** The cube runs on one N/LR1 cell and stays awake during a connection. Brief measurements on disposable power were authorized for this experiment. The recorder logs the power source.

**Reading its accelerometer: `hardware/cube_accel.py`.**

The robot gets the cube's raw samples but keeps them:

- the engine turns each one into mm/s² for its own listeners
  (`cubeAccelComponent.cpp`);
- the SDK passes on only events;
- the engine's "cubes" debug stream carries connection state, battery and
  scan results, not samples (`cubeCommsComponent.cpp`).

So the collector runs on a host with Bluetooth (the Mac) and connects to
the cube in the robot's place. It follows the sequence that Anki's
`robot/cube_firmware/bletool` and the robot's own
`cubeBleClient/bleClient/bleClient.cpp` both use:

1. Read the app version.
2. If it is not unit 3's `2018-08-14T17:40`, write the app file from byte 16
   on, 20 bytes per write, with response. The script refuses a file whose
   SHA-256 is not that of unit 3's `cube.dfu`.
3. Take notifications.

It never writes the lights characteristic.

What the cube sends (`app/accel.c`, `adc.c`, `protocol.h`):

- Every 30 ms, a 20-byte message: the tap count, then three samples of x,
  y and z as little-endian int16. The scale is 0x2000 counts per g, from
  the bletool; the engine's 32767/4 differs by 3e-5.
- About once a second, the battery rail reading: volts = raw × 3.6/1024.

The app reads the whole FIFO every 10 ms tick into one slot, so each
sample is the last frame read in its tick. The 10 ms tick comes from the
radio stack's timer, re-armed after every tick. So the sample clock has to
be fitted from the message count, and the host's arrival times carry the
radio's batching.

How it runs:

- `CUBE_MODE=scan`, the default, only listens to advertisements, which the
  cube sends anyway.
- `CUBE_MODE=collect` requires an explicit `CUBE_POWER` of `supply`, `rechargeable` or `disposable`. It stops after three battery readings below
  `CUBE_LOW_V`, default 1.1 V (chosen, not derived).
- The robot must not hold the cube during a collection. A connected cube
  does not advertise, so the scan would not find it.
- macOS hides Bluetooth addresses. When more than one cube is heard,
  `CUBE_ADDRESS` takes the identifier the scan printed.
- It needs the `bleak` package, which is not installed here. It is written
  to bleak's documented API and has not met bleak.

`tests/test_cube_accel.py` has 17 tests, run against a stand-in bleak, a
stand-in cube and a stand-in clock. Its expected values come from Anki's
sources, not from the script: the `protocol.h` layout packed with
`struct`, the bletool's offsets and scale, the UUIDs as the bletool writes
them, and the service UUID from the advertising bytes in `user_config.h`.
The nine mutation arms are caught (`evidence/cube-accel-20261121/`).

**Spin tests with the cube.** The cube held in the lift is a second
accelerometer, rigid on the robot at a place we can work out. Spinning in
place, it feels ω²r towards the turn centre. On spin-up it also feels αr
along the tangent. Gravity does not change in its frame. So the difference
between a steady spin and the stillness before it gives the vector from
the cube to the turn centre. That is an independent check of the turn
centre `simulacrum/spin_fit.py` and `camera_spins.py` find.

In the model (below), a carried cube's chip circles 42.3 to 44.1 mm from
the centre the robot turns about. That is from circle fits in two turns at
±80 mm/s, a GPU host. The pull that gives:

| Turn rate | ω²r | 12-bit steps (1/512 g) |
|---|---|---|
| 2 rad/s (the planner's default) | 17 to 18 mg | about 9 |
| 120°/s | 19 to 20 mg | about 10 |
| 300°/s (top) | 118 to 123 mg | about 61 to 63 |

That is computed on the Mac, 2026-11-21. The 12 bits are the datasheet's
(BMA253DS:832). This replaces an earlier estimate of 54 to 59 mm, which
used the SDK's drive centre and a guessed cube size.

**The pull is small next to the vibration.** The real robot shakes in a
turn. Unit 1's own accelerometer, behind its chip's 40 Hz filter, scattered
by this much (`recording.py` `moving_scatter`, 13 to 22 state rows per
spin):

| Tread speed | Scatter |
|---|---|
| ±30 mm/s | 19 to 32 mg |
| ±60 mm/s | 52 to 71 mg |
| ±100 mm/s | 124 to 185 mg |

The cube's chip passes up to 250 Hz, and the app keeps one frame per tick
with no filter before it. So the vibration lands in the samples, and a
mean's error is the scatter over the root of the sample count. With 100 mg
of scatter, 1 mg needs 10,000 samples: 100 s of steady turning. With
185 mg it needs 342 s. The model shows what happens otherwise ("What the
model found", below).

The order:

1. Collect still for 60 s, to get the noise and the gravity vector.
2. Turn for 20 s at each of three rates, each way, only to measure the
   cube's scatter at each speed.
3. From that scatter, size the long turns. Hold each until the mean's
   error is under 1 mg, each way. The centripetal part does not change
   sign with direction; the spin-up part does.
4. Collect still again.

Alongside, `record_listen.py`, or the spin recorder, logs the robot's gyro
for ω. The two hosts' clocks are tied by UTC in both files, and then by
the spin-up step, which both accelerometers see.

Two things are confounded and need care:

- **The robot's tilt in a turn.** It also grows with ω² and moves the
  gravity projection: one degree is about 17 mg. The robot's own
  accelerometer sees the same tilt, so the cube's reading is corrected
  with it.
- **The cube's yaw in the lift.** Gravity fixes two of the cube frame's
  three angles. The direction of the centripetal part fixes the third.

As a side benefit, the cube rings when the lift starts and stops. That is
the first data on the lift servo's stiffness and damping, which are
assumed today with no source.

### The cube in the simulacrum (2026-11-21)

`simulacrum/cube.py` is the cube's accelerometer as its chip and app make
their messages. `VectorSim(cube="lift")` carries a cube in the lift, runs
the chip on the specific force at its site at every physics step, and
starts the lift at carry height. Without `cube`, the model is the same as
before.

What was read for it:

- **The chip.** Bosch's BMA253 datasheet, BST-BMA253-DS000-05 rev 1.4,
  is now the source `BMA253DS`. It says:
  - 512 LSB/g at ±4 g in 12 bits, so 8192 per g once left-justified. That
    agrees with Anki's tool.
  - The bandwidth filter is "2nd order".
  - Filtered data comes out at twice the bandwidth, so one frame every
    2 ms at 250 Hz. The comment in `accel.c` says "1000 samples per
    second", which is wrong.
  - Noise density is 220 µg/√Hz, typical.
  - The zero-g offset is ±80 mg, typical, over the chip's life.
  - In FIFO mode the chip stops collecting when the buffer is full at 32
    frames.
- **The app.** `accel.c` matches the shipped binary instruction for
  instruction: the 45 << 6 = 2880 threshold, debounce 90 and 80, and the
  comparison quirk below.
- **The tick.** From `user_custs1_impl.c` and the Dialog SDK: "timer
  slots (10 ms)", re-armed after each tick.
- **Where a carried cube sits.** `carryingComponent.cpp:245-246` puts it
  (22 + 4, 0, −12.5) mm from the wrist. The lift pose is the wrist and is
  kept level (`robot.cpp:1532-1541`). The cube is 44 mm across
  (`BlockDefinitions.h`).

The table gains 14 entries:

- 5 from vendor source: size, carry offset, range, counts per g, tick.
- 9 assumed, each with a range: mass, chip position, chip yaw, bandwidth,
  frame period, filter shape, noise, offset, scale.
- 4 missing measurements.

Two consequences of the tap detector as shipped:

- It counts frames as if they were milliseconds. At 500 frames a second
  its "90 ms" debounce lasts 180 ms and its "10 ms" tap 20 ms.
- After a sharp drop (z falls by more than 2880 counts, 0.35 g), it counts
  a tap at the next frame that does not rise by 90 counts. So a drop alone
  is a tap, while a rise needs a fall within 10 frames. The model keeps
  this.

**What the model found (a GPU host):**

1. **The modelled lift cannot hold the table's cube.** The table's
   `servo_torque_max_mNm` is 30 mN·m, assumed with no source, and shared
   by head and lift. With it, the lift holds a cube of up to 40 g at carry
   height, sagging 1.8 mm. A 50 g cube drops to the bottom within 1.2 s. A
   real Vector lifts its cube, so the assumed 50 g and the assumed
   30 mN·m cannot both be right. Weighing the cube says which one to move.
   Until then the cube tests use 25 g (`diagnose-output.txt`).
2. **Two seconds of cube samples cannot find the turn centre.** Turning at
   ±80 mm/s (2.56 rad/s), the model's chip feels 0.60 to 0.67 g RMS
   between 95 and 270 Hz. Averaged over every physics step, the force puts
   the turn centre 2 mm from where the chip circles. The cube's 600
   samples of the same two seconds miss by about 90 mm. A 10.07 ms tick
   instead of 10 ms changed the miss but not its size, so this is noise,
   not phase lock (`alias-output.txt`). Whether the real lift shakes that
   much is the first thing a real recording will show.

Both files are in `evidence/cube-model-20261121/`, with the scripts that
made them.

**How it is tested.** `tests/test_cube_model.py` has 18 tests:

- Fifteen run without MuJoCo. Their expected values are built apart from
  the code they check:
  - packets with `struct`;
  - frame values coded by the time they were taken: the last frame per
    tick, a frame due with a tick, a full FIFO keeping an old frame, a
    tick with no frame keeping its slot;
  - tap sequences written out from `accel.c`'s rules;
  - the noise level from the filter's own impulse response;
  - the collector reading the model's messages;
  - a constructed spin that comes back through the whole chain.
- Three need MuJoCo:
  - the cube sits where the engine puts it;
  - held still, it reads 1 g up;
  - in a turn, the force fed to the chip gives the turn centre to 5 mm,
    and the cube's own samples agree with it within four standard errors.
    The second check is weak, since the error is tens of mg.

Twelve mutation arms break the model once each; ten of them run without
MuJoCo.

### A new app on the cube: the mean of each tick (2026-11-21)

The operator asked for it, to be flashed through the robot without moving it or
making a sound. Unit 3's cube has run it since 00:46:13 by the robot's clock.

**What it changes.** The shipped app keeps the last FIFO frame of each
10 ms tick, with no filter in front, so vibration lands in the samples. The
new app keeps the mean of the tick's frames, about five frames 2 ms apart.
That is a 10 ms boxcar, with zeros at 100 Hz and its multiples: exactly the
frequencies that alias onto a steady reading.

Everything else is as shipped:

- the message layout, so the robot reads it unchanged;
- the tap detector, compare for compare;
- the battery messages, lights and radio, which are untouched code.

The mean is of the 12-bit values with their fraction in the four low bits,
so samples are finer than the chip's own steps. The model has it as
`CubeApp("mean")` / `CubeAccelerometer(app="mean")`. Its version string is
`build-avg1-261121`.

**How it was built** (`cube_firmware/`):

- `tick_avg.s` is hand-written Cortex-M0 Thumb, replacing the shipped
  `hal_acc_tick` in place: 204 bytes at `0x20005db4`, exactly the old
  function and its literal pool.
- It uses the shipped app's own state block, SPI routines, divide and send
  pointer at the addresses the shipped code uses. Only one call points
  there, at `0x200063ba`.
- `tick_avg.c` is the same tick in C, for reading and testing. GCC makes
  256 bytes of it, too big for the place.
- A larger image placed after the zeroed data was the other way. It would
  need about 386 writes. The stock flash took 5.3 s for 160 writes, and the
  robot allows 15 s for check and upload, so that was too close.

`cube_firmware/dfu.py`:

- decrypts unit 3's `cube.dfu` with the bootloader's own XXTEA (re-encrypting
  gives the shipped file back byte for byte);
- splices in the tick and sets the version;
- encrypts, and refuses any change outside the version and the tick.

The output is 3209 bytes, SHA-256 `10f5ddd8d4f0…`, 160 writes
(`build-output.json`). The file is kept out of the repository: it is
Anki's app, encrypted under their key.

**How it was checked before flashing.** `cube_firmware/thumb.py` is a small
ARMv6-M interpreter. `tests/test_cube_firmware.py` has 10 tests, which need
unit 3's file and `key.h` (`CUBE_STOCK_DFU`, `CUBE_KEY`) and skip without
them:

- The shipped divide at `0x200058ba` is an unsigned divide (400 random
  cases).
- The shipped tick, run from the binary, matches the model of the shipped
  app over 200 messages plus edge cases. This checks the interpreter and the
  model, which came from the C source, against each other.
- The new tick's exact bytes match the model of the mean app over three
  600-tick scenarios plus edge cases.
- Its taps equal the shipped tick's.
- It reads only the image and its stack, and writes only its state, the
  message and its stack.
- It needs at most 16 bytes more stack than the shipped tick.
- The file ends on a write the bootloader treats as the last.
- The C tick, built on the host, matches too.

The edge cases sit on each tap rule's boundary: diffs of ±2880 and ±2881,
89 and 90 after a drop, and a fall in a window's tenth and eleventh frame.
The shipped binary agrees with the model on all of them.
`cube_firmware/mutate_tick.py` breaks the assembly once per arm: 12 of 12
were caught (`mutate-tick-output.txt`). Its first runs caught 7 and then 10.
That exposed two gaps in the checks, both now closed: writes were not
bounded, and nothing sat on the tap rules' edges.

**How it was flashed.** The robot flashes its cube when the cube's version
differs from the version it cached from its own `cube.dfu` when the engine
started. It re-reads the file whenever it flashes (`bleClient.cpp:213, 450`).

1. `/data/cpcs/cube-avg1.dfu` was copied to unit 3 and bind-mounted over
   `/anki/data/assets/cozmo_resources/assets/cubeFirmware/cube.dfu`.
2. The operator took the cube's cell out for a moment. The app lives in the cube's
   RAM, so the cube came back with none.
3. `hold_cube.py` connected the cube at reserve control. The first attempt
   timed out after 20 s. Then the robot saw the mismatch at 00:46:05 and
   flashed: `@cube.firmware_flash_success … build-avg1-261121` at 00:46:13
   (`unit3-log-flash.txt`).

The battery messages kept coming for the rest of the 120 s hold
(`run-t-flash-hold.jsonl`). They are sent from the same tick, so the new
tick returns normally. No engine restart, nothing commanded, the robot
docked throughout.

Not yet seen: the averaged samples themselves, and taps through the new
tick, because nobody touched the cube during the hold.

**What is left in place, and how to undo it.** While the mount stays,
unit 3 keeps the new app on the cube. The mount does not survive a reboot:
after one, the engine caches the stock version again and reflashes stock at
the next connection. To undo it now: `umount` the path and take the cube's
cell out for a moment.

### Teaching the robots to see each other (2026-11-21)

**Anki never built it.** Read in Anki's engine source (`kercre123/victor`,
branch `snowboy`):

- The engine runs one robot: `engine/robotManager.h:73` holds a single
  `std::unique_ptr<Robot>`. A note in `engine/cozmoEngine.h`, dated 12/23/2014,
  says host/client support was "removed until we know more about
  multi-robot requirements", and it never came back.
- There is no robot object type in `objectTypes.clad`, and no robot vision
  mode in `visionModes.clad`.
- No salient-point class covers a robot, and no loaded network is trained
  on one: `vision_config.json` loads a person detector and a hand detector
  on the robot, and a person detector off it.
- Anki's support page says only that Vector "will initially not
  communicate with other Vectors".

**The one outside attempt, and what its numbers are worth.** Amitabha
Banerjee trained YOLOv5 in 2020 and ran it off the robot, on recorded
video. His Drive dataset (`VectorGrandDataset.v2.yolov5pytorch.zip`,
4,394,426 bytes, CC BY 4.0) was measured here:

- It holds 206 images at 416×416, one class, and exactly one box each.
- They come from 96 distinct photos, numbered 1 to 99: 67 photos for
  training (about 2.6 rotated copies each), 19 for validation, 10 for
  testing. None shows no Vector.
- Every validation and test photo is 1 or 2 numbers from a training photo.
  On a 16×16 average hash, 12 of the 19 validation photos are 0 bits from a
  training photo and no test photo is more than 5, against a median of 22
  bits between unrelated training photos.
- So the published mAP@50 of 99.5% measures recall of near-identical scenes,
  not detection of a Vector somewhere new.

**The decision.** First collect and label images of the household's own
Vectors, with and without one in view, and only then choose a model. The
options set aside for now were torchvision SSDlite (code BSD-3; weights 14,069,355 bytes),
Banerjee's YOLOv5 (code AGPL-3.0; `yolov5s.pt` 14,808,437 bytes), or both.

**The capture tool.** `hardware/capture_peers.py` runs on the wire-pod
host, like the other recorders:

1. It holds DEFAULT behaviour control, so the observing robot's own
   behaviours stay off and it stays on its charger. It commands nothing.
2. It streams camera frames as base64 JPEG in JSON lines, at most one new
   frame per tick.
3. Quiet mode gives no frames, so it lifts quiet mode only while control is
   held, and puts it back before letting go.
4. It refuses to connect (exit 6) unless the pod reads Cryosleep's mute off.
   Cryosleep is the household's overnight sleep, and it withholds the camera
   on purpose. A pod with no `cryo_mute` endpoint cannot run Cryosleep, so
   there it goes ahead.
5. It stops on its time limit, a stop signal, lost control, or STALL_S
   without a new frame.

Run A of 2026-11-18 shows frames flow under SDK control: the robot's image
timestamp advanced 82,996 ms in 83.0 s, with 1,006 distinct values.

`hardware/save_capture.py` writes the frames to files and checks each one
against the hash the robot side computed. `IN_VIEW` records what the
operator placed in view: `none` for a negative scene, or the units in
front of the camera. Boxes come later, from labelling.

**Not yet run on a robot.** Cryosleep was on when the tool was finished.
The sessions need two or three robots together in good light; a scene with
no Vector also counts.

### What hands can settle, in order

No connection to the robot is needed for any of these, and each replaces an
assumed value outright. The order is by what the model is sensitive to
("The assumed values, ranked") and by what a measurement costs; an external review lane's
own first three were the cliff margin, the fore-aft centre of mass and the
mass.

| # | Measure | Needs | Replaces |
|---|---|---|---|
| 1 | Balance the robot across a straight edge laid under both treads, fore-aft, head level and lift down; mark the line and measure from the front axle. Repeat with the head fully up | Ruler, straight edge | `com_x_mm`. The model's turn rate moves by a quarter for 5 mm of this |
| 2 | Weigh the robot | Kitchen scale reading to 1 g | `total_mass_g`. Cheap, and it turns the fitted motor strength from a ratio into a torque |
| 3 | Length and width of the table the policy will run on | Tape measure | `table_length_mm`, `table_width_mm`, if training for one table; otherwise leave the range |
| 4 | Where the front wheels' contact line is when the cliff flag first sets: push the robot slowly by hand toward an edge with the state logged (the recorder's edge phase does this with a ruler distance), several headings | Ruler; the recorder | The effective cliff margin. `cliff_sensor_height_mm` and `cliff_detect_range_mm` have no effect at all in this model, so the margin is what to measure |
| 5 | Belly clearance, and length, width and height | Ruler, feeler or folded paper | `chassis_ground_clearance_mm`, chassis sizes |
| 6 | Tilt a board of the same surface with the robot on it, treads not turning, until it slides; forward and sideways | Board, phone inclinometer | `tread_friction`, as a static figure. A sliding figure needs a pull with a force gauge |
| 7 | Balance the robot on its side on the straight edge | Ruler, straight edge | `com_z_mm` |
| 8 | Push against a force gauge at a low commanded speed | Force gauge | `wheel_stall_torque_mNm` as a torque, not a ratio |

Not worth hands yet: the head, lift and wheel masses, the servo strengths
and the head's collision box. Nothing in the task moves or touches them.

One more run on a second surface would say how much of all this belongs to
the table.

Item 8 was wrong in the first version of this list: it asked for the stop
point to be measured after the run. That cannot be done by hand. About 1.2 s
after it stops, the robot backs away by itself, and in the first recording it
then turned to face the edge. So the start distance is measured instead, and
the stop point is that distance less the travel the robot reports; item 2
checks the reported travel against a ruler.

### The script for it

`hardware/record_barriered.py`, written by a lane from the list above.
**It has not been run, and it must not be run on an open-edged table.** It
is kept here so that the sequence the model needs sits beside the model.
`tests/test_recorder.py` runs its own `main()` against a stand-in SDK and a
stand-in clock; that shows what it would send and write, and nothing about
how a robot answers.

Phases are chosen with `PHASES`; `repeat` belongs last. This copy differs
from the original in logging only:

- After a cliff or any other hazard it sends its stop, then keeps recording
  for `ABORT_WATCH_S` (6 s) without commanding anything. The original
  disconnected at once, so the reaction that item 8 exists to record (the
  pause, the move away) would not have been in the file.
- The edge phase needs `EDGE_START_MM`, the ruler distance, and refuses
  before connecting without it. It is written into the file.
- `NOTE` is copied into the first row (surface, barrier, ruler readings).

What the file will not settle, and what the reader still lacks:

- `img_ts`, the time stamp of the robot's last camera image, is the only
  robot-clock field a state message carries. It shows how evenly state
  messages arrive. It cannot split the 60 ms delay into the command's way in
  and the state's way back: the two clocks share no event. Filming the
  treads with the host clock in frame would.
- Battery voltage is not logged. Item 9 is there to show drift; the voltage
  at the start and end of each run would say what drifted.
- The script still sends `stop_all_motors` when it sees the cliff flag.
  Whether that changes the robot's own reaction is not known; the first
  recording's event had no such command. Compare the pause and the distance
  with 1.22 s and 29 mm.
- A blocking action (head, lift, `drive_straight`, `turn_in_place`) has no
  time limit of its own in the script.
- `simulacrum/recording.py` reads the plain wheel segments of the new file
  as it stands (tested). It does not yet read the reversal as one segment,
  the robot's own actions, or head and lift moves longer than 1.1 s. That is
  work for when the file exists.

### Two things the hardware sessions found

Reported by a lane from driving the robot; not observed by this skill,
except where the first recording shows it.

- At the default control level the robot refuses control while its cliff
  flag is set and it is parked. The SDK's connect then returns without
  control, and a blocking action called in that state does not return: a
  `drive_on_charger` blocked for 314 s with the robot motionless. A script
  must check for control before every blocking call. That is also why the
  edge phase is one approach per run: after the reaction the robot has to be
  moved by hand before it will give control again.
- The robot's own `drive_on_charger` failed to dock from about 120 mm in
  front of the charger and ended at a table edge. The first recording shows
  the same thing: the call ran from 58.2 s to 159.8 s, returned 0.06 s after
  the cliff flag first set, and the robot was not on the charger at the end.
  Do not rely on it to bring the robot home on an open table.

`missing` in `parameters.json` carries the same gaps.

## Contributing

If you have a Vector, recordings from your own robot narrow the spread on
the per-unit values in the parameter table. The contribution path is a
`contrib/kit/` recorder, a privacy scrub and a validator that runs locally
and on every pull request.

Read [CONTRIBUTING.md](CONTRIBUTING.md) for the step-by-step; the fastest
first-pass command once you have the Vector SDK set up is:

```bash
python3 contrib/kit/record.py --unit-handle <handle>-1 \
    --vector-serial <8-hex-digits> \
    --phases rest,headlift,spins,straights,actions
```

Contributed recordings and metadata land under CC BY 4.0 by default. The
tracker of contributed units is [`contrib/UNITS.md`](contrib/UNITS.md).

## Sources read

- The recording: `evidence/hw-rec-20261115/`, made 2026-11-15 by a lane
  with its approval, through the wire-pod host's SDK
- Vector Technical Reference Manual, https://randym32.github.io/Vector-TRM.pdf
- https://github.com/kercre123/victor (branch `snowboy`): `cozmoConfig.h`,
  `pathMotionProfile.clad`, `reactToCliff.json`, `behaviorReactToCliff.cpp`,
  `proxSensors.cpp`, `imuFilter.cpp`, `wheelController.h`, `hal_motors.cpp`,
  `wheelController.cpp`, `steeringController.cpp`, `hal.h`,
  `headController.cpp`, `liftController.cpp`, `robot/hal/spine/imu.c`,
  `messages.cpp`
- The same repository, for the light cube:
  - from `robot/cube_firmware/`: `app/accel.c`, `app/adc.c`,
    `app/protocol.h`, `src/user_app/user_custs1_impl.c`,
    `src/user_app/user_custs_config.c`, `src/config/user_config.h`,
    `sdk/app_modules/api/app_easy_timer.h`, `bletool/cube.js`,
    `bletool/factory.js`, `bletool/index.js`, `tools/sign.py`,
    `tools/key.h`;
  - from the engine: `cubeBleClient/bleClient/bleClient.cpp`,
    `cubeBleClient_vicos.cpp`, `engine/components/cubes/cubeAccelComponent.cpp`,
    `cubeCommsComponent.cpp`, `appCubeConnectionSubscriber.cpp`,
    `engine/components/carryingComponent.cpp`, `engine/BlockDefinitions.h`,
    `engine/robot.cpp` (`ComputeLiftPose`), `engine/fullRobotPose.cpp`;
  - the shipped app `cube.dfu`, decrypted and disassembled
    (`evidence/cube-firmware-20261121/PINMAP.md`).
- https://github.com/digital-dream-labs/vector: `simulator/protos/CozmoBot2.proto`, `CozmoIMU.proto`
- Bosch BMI160 datasheet BST-BMI160-DS000-09, Bosch BMA253 datasheet
  BST-BMA253-DS000-05 rev 1.4, and ST VL53L0X datasheet DocID029104 Rev 2
  (the last from a distributor's mirror)
- https://os-vector.github.io/vector-docs/5.-Hardware/index.html, a community page
- private repository `vector-slam-maze` at 5992b24, `movement_directives.py`
- private repository `workspace`, branch `cpcs/vector-cliff-guard` at 17008e7, `skills/vector-sim/vectorsim/world.go`

Exact URLs and SHA-256 of each file are in the `sources` block of
`parameters.json`.
