---
name: vector-simulacrum
description: A MuJoCo model of the Anki/DDL Vector robot for training policies that must transfer to the real robot. Every physical number comes from one table that says where it came from; an SDK-shaped layer drives the model in the robot's own units, with the firmware's own tread control law, and models its cliff reaction. Replayed against one recording of the real robot and fitted to part of it.
---

# vector-simulacrum

This is the physics model that `skills/vector-sim` (the Go gRPC simulator,
branch `cpcs/vector-cliff-guard`) says it is not. It does not replace that
simulator and is not connected to it.

**It has been compared with one recording of one real Vector, and tuned to
part of it.** One robot, one run of 162 s, one surface, one battery charge
(4.10 V falling to 3.96 V), wheel speeds up to 100 mm/s. Nothing here says
anything about another robot, another surface, or faster driving.

Of 160 parameters: 10 are measured on the recordings, 18 are fitted to them, 17
come from the manual, 85 from vendor or household software, 23 are still
assumed about the robot, and 7 are settings of the scene and the solver that
have no real value to find. On the segments the fit never saw, the model's
wheel speed and gyro error fell from 0.33 of each segment's scale (the
version pushed before the recording existed) to 0.14. Mass, tread friction
and centre of mass are still assumed: the recording could not tell their
values apart. The fore-aft centre of mass is the assumed value the model is
most sensitive to, by a wide margin ("The assumed values, ranked" below).

## What is here

| File | What it is |
|---|---|
| `parameters.json` | The single source of every physical number, with provenance |
| `simulacrum/params.py` | Loads and checks the table; draws randomised robots from a seed |
| `simulacrum/mjcf.py` | Generates the MJCF (robot and table scene) from the table |
| `simulacrum/drive.py` | The firmware's tread encoder arithmetic and speed control law |
| `simulacrum/robot.py` | `VectorSim`: SDK-shaped commands and state, sensor noise, cliff reaction |
| `simulacrum/hazards.py` | Cliff, falling and picked-up flags |
| `simulacrum/prox.py` | The forward time-of-flight sensor: its cone, signal quality, and the robot's reporting rules. No MuJoCo |
| `simulacrum/prox_fit.py` | Recomputes that sensor's fitted values from the recordings. No MuJoCo |
| `simulacrum/recording.py` | Reads the recording of the real robot and measures it. No MuJoCo |
| `simulacrum/imu_fit.py` | Fits the accelerometer's steady offsets and the pitch of its frame from the recording. No MuJoCo |
| `simulacrum/wheelstand.py`, `wheelstand_check.py` | The centre of mass from balance angles on the rear wheels and on one tread edge, read by the robot's own accelerometer; the check that the model tips where the formula says (MuJoCo) |
| `simulacrum/replay.py` | Replays the recording's commands through the model and scores the difference |
| `simulacrum/task.py` | The training task: observation, action, reward, episode ends, randomisation |
| `simulacrum/vec_env.py` | Many tasks stepped together, in up to 4 worker processes |
| `simulacrum/training.py` | PPO with `rsl_rl`, ONNX export, the parity recording |
| `simulacrum/onnx_policy.py` | Runs an exported policy with `onnxruntime` only |
| `replay_recording.py`, `fit_recording.py` | The comparison tables below; the fit (prints values, changes no file) |
| `train.py`, `run_policy.py`, `evaluate.py`, `smoke.py` | Train and export; run without torch; compare with baselines; all three in one go |
| `rank_assumed.py` | Moves each assumed or chosen value to its bounds and measures what that does to the training task |
| `build_mjcf.py` | Writes the generated MJCF to a file |
| `demo.py`, `measure.py` | A short run; the model figures quoted below |
| `evidence/hw-rec-20261115/` | The recording, the script that made it, SHA-256 in the table's `REC` source |
| `evidence/` | Smoke run learning curves and outputs; the fit, replay, measurement, ranking and fault-check outputs; the hardware findings the third version started from; the conferral with an external review lane |
| `hardware/record_barriered.py` | The recorder for the next recording. **Not run yet.** Written by a lane, who holds the robot; nothing here runs it against one |
| `tests/` | `run_all.py`, fifteen test files, `mutate.py` |
| `run_in_ct.sh` | Copies this directory into a GPU host and runs a file there |
| `mcp_server.py`, `simulacrum/mcp_service.py` | An MCP server over stdio: the table, the recordings and the recorder's command line as tools. Read-only; no robot, no MuJoCo |

## Use

```bash
./run_in_ct.sh tests/run_all.py --require-mujoco   # all tests, in a GPU host
./run_in_ct.sh tests/mutate.py                      # break each property once
./run_in_ct.sh replay_recording.py --sensitivity    # the model against the real robot
./run_in_ct.sh fit_recording.py                     # refit (4 workers; prints values, changes no file)
./run_in_ct.sh measure.py                           # the model figures in this file
./run_in_ct.sh smoke.py                             # train, export, check without torch, compare
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

- `measured` (9): read directly off the recording, with no model in between.
  Each must cite the `REC` source, and `simulacrum/recording.py` recomputes
  each one from the files, so the table and the recording cannot drift apart.
- `fitted` (10): a value chosen by a fit to the recording. Eight by
  `fit_recording.py`, so that the model reproduces named segments; two by
  `simulacrum/imu_fit.py`, for the accelerometer alone, with no MuJoCo model
  involved. A fit is not a measurement: it depends on the shape of what was
  fitted and, for the eight, on every assumed value that was in place when
  it ran. Each says what it was fitted to and what was held out, and must
  carry a range.
  The validator refuses a `measured` or `fitted` entry that does not cite the
  recording, and a fitted one without a range.
- `manual` (17): the Vector Technical Reference Manual, by section and line.
- `sdk` (74): published in software. `source_kind` says which:
  `vendor_firmware` (62, the robot's released source), `vendor_webots_model`
  (8, Anki's own Webots model) or `household_code` (4, the limits in
  `vector-slam-maze`). Cited by URL, line and SHA-256.
- `assumed` (21): properties of the robot that no source states. Each is a
  measurement still to be taken.
- `chosen` (7): **a class added in this version.** 4 table sizes and 3 solver
  settings. They were filed as assumed before, which made them look like
  unknowns about the robot. They are not: there is no real value to find.
  The validator refuses a robot parameter filed as chosen and a scene or
  solver setting filed as assumed.

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

### Battery, heat and the top speed (2026-11-18)

The experimenter: "simulate voltage sag and other factors that affect motor
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
  The y offset cannot be told from a roll of the frame. Scale errors cannot
  be told from offsets with gravity as the only reference.
- The 10 mg range is the author's allowance for all of that and for
  temperature. It is a fact about one robot: to train for any Vector, widen
  it to the part's 40 mg.

The correlation and the bootstrap are there because an external review lane asked for
them (conferral of 2026-11-15, below); both confirmed its point.

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

### Still assumed about the robot (21)

| Group | Parameters |
|---|---|
| Mass | `total_mass_g` 170 (150 to 185), `head_mass_g`, `lift_mass_g`, `wheel_mass_g`, `lift_arm_mass_fraction`, `com_x_mm` -24, `com_z_mm` 22 |
| Body | `chassis_ground_clearance_mm` 3, `head_box_mm` |
| Contact | `tread_friction` 0.9 (0.5 to 1.3), `chassis_friction` 0.3 |
| Head and lift drive | `head_servo_kp/kv`, `lift_servo_kp/kv`, `servo_torque_max_mNm`, `servo_armature_gcm2` |
| Cliff | `cliff_sensor_height_mm` 8, `cliff_detect_range_mm` 25 |
| Control | `heading_hold_gain_mmps_per_rad`, `drive_done_mm` |

Chosen, not assumed (7): `table_length_mm`, `table_width_mm`,
`table_height_mm`, `table_thickness_mm`, `sim_timestep_ms`,
`contact_time_constant_ms`, `settle_time_ms`.

Resolved in this version, out of the 30 the last version called assumed:
`gyro_bias_dps` (measured), `accel_bias_mg` (fitted), and the seven above
(never unknowns about the robot).

Cautions carried from the published-figures work of the third version:

- The divisors 172.3, 149.7 and 366.211 in the encoder scales are not
  recorded as gear ratios. The source does not say what they are.
- The firmware names the IMU: a comment in `imuFilter.cpp` (line 153) cites
  the "BMI160 datasheet". The manual does not name it.
- The VL53L0X datasheet was read from a distributor's mirror.
- Mass and outer dimensions were not found on any page that could be read.

## The assumed values, ranked

The experimenter asked for the assumed values to be resolved in order of their effect on
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
gives. They stay assumed.

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
  fitted rate, and their own command delay. Reported in whole encoder counts
  (0.123 and 0.301 degrees). The servo gains are still assumed.
- **Sensors.** Four downward range sensors at the cliff sensor positions,
  in the manual's order. One forward time-of-flight sensor, tilted up 6.5
  degrees, with its 25 degree cone and the robot's own reporting rules (see
  "The range sensor" below). Gyro and accelerometer at the
  IMU position in the head and in the head's frame, pitched by the fitted
  `imu_pitch_offset_deg`, with the noise measured at rest, the gyro's
  measured residual offset and the accelerometer's fitted offsets. A turn
  therefore shows on gyro z by the cosine of the head's pitch and on gyro x
  by its sine, as on the robot.
  A camera on the head, 50 degrees vertical field of view. The noise comes
  from a seeded generator: one seed, one run.
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
- A head and lift motion profile that speeds up on larger moves.
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
  reading does not move), charger, cubes, touch,
  the stall protection, the lift's four-bar linkage.
- Tread compliance. The wheels are rigid cylinders, so the robot stands on
  four contacts. That is the likely reason the turn rate depends so sharply
  on the fore-aft centre of mass ("The turn in place"); a belt that carries
  load between the wheels would soften it.
- Draws that respect what is known together. Each ranged value is drawn on
  its own: the part masses independently of the total and of the centre of
  mass, and the accelerometer's x offset independently of its frame's pitch,
  although the fit fixes their sum far better than either.
- A roll of the IMU's frame, and scale errors of the accelerometer.
- Noise on the cliff sensors, and any noise that changes with temperature
  or with motion. The model's gyro and accelerometer do shake while the
  robot moves, but from its rigid contacts, not from anything measured:
  "Measured in the model" sets that shake beside the recording's. Within one
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

`mjlab` steps many robots as one MuJoCo Warp batch on a GPU. a GPU host has no
GPU, and what a policy has to be trained against here is the SDK layer:
the command delay, the 60 ms state rate, one cliff bit for four sensors,
wheel speed without a sign, and the robot's own cliff reaction taking
control away. That layer is Python around one MuJoCo model per robot, and
it is the part with tests. So the environment is a small vectorised pool
over `VectorSim` (`simulacrum/vec_env.py`), with up to 4 worker processes.

A move to `mjlab` on the GPU node would need:

1. a GPU host on the GPU host (it needs disk space first; see
   `skills/a GPU host`), and `the GPU host gpu-setup`.
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

`tests/run_all.py`, 170 tests in eleven files. In a GPU host: declared 170, ran 170, passed 170, skipped 0.
On a machine without MuJoCo: 87 pass and 83 are skipped, and the runner says
so.

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

`tests/mutate.py` breaks each property once: 95 of 95 injected faults
caught, each by a test named in advance, 0 survived, 0 set-up errors. The
80 tests the arms name pass on an untouched copy first, and a change of
wording only leaves all 170 green. Output:
`evidence/mutate-20261116-fifth-version-output.txt` (earlier runs:
`evidence/mutate-20261116-output.txt`, 78 arms, and
`evidence/mutate-20261115-output.txt`, 67).

That is the second full run. In the first, 94 arms were caught and one, "IMU
fixed to the body, not the head", was a set-up error: its search text held
real line breaks where the source has the two characters backslash and n, so
it matched nothing. The harness said so instead of counting it either way.
While the arms were being written, three more ended as set-up errors for a
different reason (the fit ran in the tests' set-up, so a broken fit stopped
the tests from running at all) and one because its text occurred twice. All
five were faults in the check, not in what it checks, and were fixed there.

`python3 skills/a code scanner.py --fail-on warn --fail-on-unscannable
skills/vector-simulacrum`: errors 0, warns 0 (40 files).

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

The experimenter's go, 2026-11-18: when the robot is back within reach of its cube, this
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

## Sources read

- The recording: `evidence/hw-rec-20261115/`, made 2026-11-15 by a lane
  with its approval, through the wire-pod host's SDK
- Vector Technical Reference Manual, https://randym32.github.io/Vector-TRM.pdf
- https://github.com/kercre123/victor (branch `snowboy`): `cozmoConfig.h`,
  `pathMotionProfile.clad`, `reactToCliff.json`, `behaviorReactToCliff.cpp`,
  `proxSensors.cpp`, `imuFilter.cpp`, `wheelController.h`, `hal_motors.cpp`,
  `wheelController.cpp`, `steeringController.cpp`, `hal.h`,
  `headController.cpp`, `liftController.cpp`
- https://github.com/digital-dream-labs/vector: `simulator/protos/CozmoBot2.proto`, `CozmoIMU.proto`
- Bosch BMI160 datasheet BST-BMI160-DS000-09 and ST VL53L0X datasheet
  DocID029104 Rev 2 (the second from a distributor's mirror)
- https://os-vector.github.io/vector-docs/5.-Hardware/index.html, a community page
- private repository `vector-slam-maze` at 5992b24, `movement_directives.py`
- private repository `workspace`, branch `cpcs/vector-cliff-guard` at 17008e7, `skills/vector-sim/vectorsim/world.go`

Exact URLs and SHA-256 of each file are in the `sources` block of
`parameters.json`.
