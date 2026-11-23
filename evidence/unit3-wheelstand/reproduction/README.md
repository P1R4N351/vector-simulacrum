# Reproducing the two physical wheelstands, W4 and W8

The goal is a model that reproduces both recorded wheelstands from their measured starting states, the same way every run. It does not do so yet. This folder records what was changed, what each change did, what is still unmatched and why, and the regression test that holds the current state.

All numbers below come from `replay.py` runs on one CPU container. `runs/` holds the raw output (one JSON line per case), `summarize.py` turns it into the CSV files. Each replay covers four cases, the two release times and two possible command onsets inherited from `W4/free_chassis.py --firmware`. Medians are over those four. W4 starts from its pre-command SDK cube sighting. W8 starts from the camera-registered position in `W8/camera-pose.csv`.

## Physical reference

| | W4 | W8 |
|---|---:|---:|
| Peak chassis pitch | 72.25° | 72.50° |
| Time to peak, from the last raised-lift sample | 0.284 s | 0.275 s |
| Pitch held 0.4–0.8 s after that sample (median) | 70.14° | 70.15° |
| Peak cube specific force, −0.3 to +1.2 s | 4.50 g | 2.36 g |
| Tap increments in the same window | 2 | 2 |

After the peak, both robots stayed between 68° and 71.5° with both treads reporting zero for as long as telemetry lasted, 0.53 s (W4) and 0.68 s (W8). The two held angles agree within 0.01°. That looks like a geometric stop rather than a balance.

## Finding 1: the rear tip supports set a pitch ceiling

`mjcf.py` places body-fixed cylinders round the real axles, slightly smaller than the wheels, because the fitted 3.1 mm contact inset moves the wheels inward. Once the chassis pitches past about 5.5°, the rear supports carry the robot instead of the rear wheels. They do not turn, so the treads cannot push the rear axle forward, and they act as a brake with tread friction.

The new parameter `tip_supports_driven` (default 0, which keeps every generated model byte-identical) puts each support on its own hinge, geared to its tread so that its surface moves at tread speed. The real tread wraps the axle, so it is driven there as well.

| Lift (mN m cap) | W4 fixed | W4 driven | W8 fixed | W8 driven |
|---|---:|---:|---:|---:|
| servo 30 (current) | 32.5° | 32.1° | 32.0° | 24.2° |
| servo 60 | 36.3° | 60.0° | 35.9° | 49.6° |
| servo 100 | 39.9° | 122.6° | 39.3° | 56.1° |
| servo 250 | 39.5° | 122.6° | 44.6° | 123.0° |
| PID + DC motor, 200 mN m stall, 30 rad/s | 41.5° | 122.8° | 43.1° | 123.1° |

These are median peak pitches. With fixed supports, no lift torque from 30 to 250 mN m takes either replay past 47°. With driven supports the ceiling is gone. Peaks near 122° are the model going over onto its back. Full metrics are in [ablation.csv](ablation.csv).

## Finding 2: lift torque cannot be identified from the existing stroke recordings

Every non-wheelstand lift recording (D1, D2, timed-lift, docked-actuator-timing) is unloaded and tracks its commanded profile. None of them saturates the motor, so none of them constrains stall torque. The R5 cube carry gives only a lower bound.

In W4, the loaded descent from 90.6 to 32.0 mm spans 0.168 s between the SDK samples that bracket it, about 45 ms apart. The firmware profile for that move (10 rad/s, 200 rad/s², 0.961 rad) takes 0.146 s, so within sampling resolution the hardware lift kept to its profile while levering up the robot. That is a lower bound on available torque, not a value for it. At the current 30 mN m cap the model takes 1.2–1.7 s for the same descent.

`--lift motor:S:W` replaces the servo with the public lift PID (hardware gains Kp 3, Ki 0.1, Kd 3000) driving a linear DC motor. In an earlier grid on W4 (stall 40–300 mN m, no-load 12, 20 and 40 rad/s), the lift loop diverged at 170–300 mN m with 12 rad/s and at 300 mN m with 20 rad/s. The lift then ran away from its target, which the real controller does not do, so those pairs are outside the plausible region. With the parameter unidentifiable, the fitting below uses the existing servo with its cap as the single lift parameter.

## Finding 3: nothing in the model stops the stand at about 70°

A Latin hypercube of 96 points was run on W4 with driven supports. It covered stall torque 60–400 mN m, no-load speed 15–60 rad/s, `com_x_mm` −30 to −14 and `com_z_mm` 15 to 30, every range inside the parameter table's declared bounds. In 64 of the 96 points at least one case went over onto its back, and in 51 all four did. None reached a median peak of 69–75° together with a median hold of 67–73° ([lhs.csv](lhs.csv)). The best median pitch RMSE was 12.2°, with a 69.7° peak but the peak 0.68 s late and the cube pushed 32.9 mm.

Contact dumps show why. In the model, the fork leaves the cube when the lift reaches its stop at 33–41° pitch, and from then on the robot either rides the cube face, pushing it 20–40 mm, or rotates over. On hardware the lift read 29.3 mm (W4) and 30.3–31.0 mm (W8) during the stand, 1–2.7 mm below its 32 mm stop. That points to a load still on the fork, perhaps the crossbar caught on the cube's lower overhang, but no recording shows the contact. The overhead camera cannot see under the cube lip (see "Does the overhead camera resolve fork seating in W4?" in the parent README). A side view of a new physical wheelstand is the measurement that would settle it.

## Fit on W4, hold out W8

With driven supports, the servo cap was chosen on W4 alone from {30, 60, 100, 250} mN m and the PID + DC motor case, by median pitch RMSE over −0.1 to +1.0 s. 60 mN m gave the lowest value, 18.4°. That is inside the declared 10–80 mN m range, but not identified, for the reasons in Finding 2. W8 was then replayed with nothing changed.

| | W4 (fit) | W8 (held out) | Physical |
|---|---:|---:|---:|
| Median peak pitch | 60.0° (58.8–122.8) | 49.6° (45.9–53.5) | 72.3 / 72.5° |
| Median time to peak | 0.56 s | 0.56 s | 0.28 s |
| Median held pitch | 58.3° | 48.7° | 70.1° |
| Median pitch RMSE | 18.4° | 30.0° | — |
| Median cube travel | 38.7 mm | 19.5 mm | 9.5–10.1 mm (W4) |
| Median cube yaw | −1.2° | −11.8° | −3.0 to −3.7° (W4) |
| Median peak cube force | 2.43 g | 2.64 g | 4.50 / 2.36 g |
| Median taps | 2.5 | 2.5 | 2 / 2 |
| Same metrics, current model (fixed supports, 30 mN m) | 32.5°, 1.50 s, RMSE 52.9° | 32.0°, 1.32 s, RMSE 53.8° | |

Pitch RMSE falls from 52.9° to 18.4° on W4 and from 53.8° to 30.0° on the held-out W8. Peak pitch, timing, hold, cube travel and yaw all remain outside any tolerance that the measurement error would justify. The model does not reproduce these wheelstands.

## Reproducibility

- **Same inputs, same output.** Two runs of the same case gave byte-identical traces and cube packets for W4 and W8. The same case on a second host gave a byte-identical trace as well. The hosts were an AMD Ryzen R2544 with Python 3.11 and an Intel i7-8700B with Python 3.13, both on MuJoCo 3.11.0 and NumPy 2.4.6. The cross-host comparison used this harness with the supports driven by an XML edit equivalent to the new parameter, before the parameter existed.
- **Sensor seeds.** The seed drives only the cube accelerometer's noise, so physics is identical. Across seeds 1–4, peak cube force moved by at most 0.006 g and tap counts did not change.
- **Placement sensitivity.** With the starting cube moved 0.2–1 mm along the heading, the peaks that did not go over spanned 52.5–59.8° on W4 and 44.5–53.2° on W8. In 1 of 24 shifted W4 cases, the robot went over instead of riding the cube ([placement.csv](placement.csv)). Because the onset timing alone (65 ms apart) already splits outcomes, a single replay is not a stable estimate. Compare medians over cases, and over physical repeats.

## Regression test

`tests/test_wheelstand_replay.py` replays W4 and W8. It needs the printed-fork fixture XML, so set `WHEELSTAND_FORK_FIXTURE`. Without it the tests skip as an optional input. It checks four things:

- one case run twice gives the same bytes;
- fixed supports keep peak pitch below 48° and driven supports exceed 50° (W4 and W8, 100 mN m);
- with the reference configuration (driven supports, 60 mN m), median pitch RMSE and median peak error may not rise more than 2° and 3° above the values recorded here (W4 18.40° and 13.03°, W8 29.97° and 22.91°);
- an expected failure checks the physical tolerances: peak pitch ±3°, time to peak ±0.08 s, held pitch ±3°. The recordings agree with each other to 0.25° on peak and 0.01° on hold. The 0.08 s covers the 65 ms onset ambiguity plus state receipt jitter. When the model meets these tolerances the expected failure becomes an unexpected success, the suite turns red, and the tolerances move into the ratchet.

Fault validation: each arm breaks one thing in a fresh copy and runs the module. The needle must occur exactly once, and the file must change.

| Arm | Result |
|---|---|
| Unchanged copy | 3 pass, 1 expected failure |
| Comment added to `mjcf.py` (neutral) | 3 pass, 1 expected failure |
| `_tip_supports_driven` always false | ceiling test and ratchet fail |
| Tip-support gear set to 0 | ceiling test and ratchet fail |
| Reference lift cap returned to 30 mN m | ratchet fails |
| 0.1 µm random offset on the starting cube | determinism test fails |

The whole module ran in 68 s on the CPU container.

## Reproduce

```
python evidence/unit3-printed-fork-contact/build_fixture.py forks7.stl fixture.xml
python evidence/unit3-wheelstand/reproduction/replay.py fixture.xml --trial W4 --driven-tips --lift servo:60
python evidence/unit3-wheelstand/reproduction/summarize.py
WHEELSTAND_FORK_FIXTURE=fixture.xml python tests/run_all.py --only test_wheelstand_replay
```

The fixture used here had SHA-256 `4867e81c8d8d92ad3abc621d920af9b550c001e5d8f81d8443d1e84a069b8230`. The LHS points are in `runs/lhs-points.txt`, drawn with seed 1.
