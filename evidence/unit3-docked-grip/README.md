# Docked cube contact with a missing lift tooth

Unit 3 remained on the charger while the experimenter placed a marked face of the cube against its installed printed fork. The experimenter reports that the fork is missing a tooth. Which tooth, the fracture shape, and the date it broke are unverified. The intact STL in earlier contact replays is a comparison geometry. Historical footage establishing a physical carry does not establish full seating.

[Two-sided clip](docked-contact.mp4) · [Contact sequence](contact.jpg) · [Encoder and accelerometer](sensors.png) · [Cube samples](cube.csv) · [Encoder samples](encoder.csv)

The first 25% raise requested a 47 mm target over 1.0 s. The encoder started at 33.02 mm, peaked at 45.42 mm and settled back at 31.32 mm. At 1.518 s the SDK returned MOTOR_STOPPED_MAKING_PROGRESS. The routine stopped; the subsequent 50% and lower commands were never issued. Wheel speed readings stayed at zero. Both cameras show transient cube movement and the cube back down beside the fork. The contact point is occluded.

There were 95 native cube accelerometer notifications (285 samples). Relative to the median pre-command three-axis vector, the largest vector change was 0.010 g before the command and 2.421 g during the action. The latter includes gravity, rotation and transient motion; it is not a grip-force measurement. The three samples in each notification share a packet receipt timestamp. The pre-command packet baseline is short (about 0.20 s). The cameras use V4L2 timestamps with a host clock mapping. Interhost and SDK receipt latency limit event alignment. This single failed hold cannot identify the cause of the stall or quantify a stock fork's capacity.

The archived run.jsonl, packets.tsv, A.nut, B.nut and clock.csv regenerate these exports with analyze.py and make_clip.py. The cube was disconnected and quiet mode restored. Postflight reported full battery, dock contact and stationary robot.

### Optical motion and the free-cube replay

[Glyph, encoder and acceleration overlay](motion-overlay.png) · [Frame measurements](glyph-track.csv) · [Simulation replay](face-replay.png)

Each opposing view tracks a face glyph against the near wheel hub in the published 20 fps clip. Both glyphs reached their highest image position at +1.15 s (16.29 px in A and 12.92 px in B), the same playback sample as the first encoder maximum. At +1.35 s both remained near that peak. At +1.45 s both were within 1.1 px of baseline. The strongest cube acceleration packet arrived at +1.429 s; the action returned its stall result at +1.518 s. These are recorder and playback times, with camera sampling, interhost clock and Bluetooth receipt uncertainties. No force or metric lift distance is inferred from the glyph positions.

The current Vector 2.0 model received the same 47 mm, one-second command. The cube was free and face-square on level ground. Its bounding face began 0, 1 or 2 mm ahead of the projected front of the simulator's three-box fork before settling. The shelf's dock geometry, broken tooth and actual seating depth are unmeasured in this setup. The predicted lift settled at 46.46 mm in every case. The physical lift fell back to 31.32 mm. The zero-gap replay produced a 1.19 mm transient cube-center rise; the other gaps produced at most 0.012 mm. This flags a missing load/release response under these placements. It does not isolate the tooth geometry, compliance or stall controller as the cause. Run track.py, then face_contact.py with the repository on PYTHONPATH.

### Docked motor power and the action result

[Encoder and torque-cutoff comparison](dock-power.png) · [Physical status words](dock-status.csv) · [Reproduction](dock_power.py)

The reference engine routes a normal SDK SetLiftHeight request to MoveLiftToHeightAction. Its CheckIfDone tests for a lift that has started moving and then stopped before the engine has latched completion. That condition returns MOTOR_STOPPED_MAKING_PROGRESS. The action result is not a current measurement. Separately, the robot lift controller enables DISABLE_MOTORS_ON_CHARGER and calls DisableInternal when on the charger and internally in position, clearing commanded power. Burnout protection has a separate two-second sustained-power threshold. Source paths in the kercre123/victor reference are engine/robotEventHandler.cpp, engine/actions/basicActions.cpp and robot controllers/liftController.cpp. The running unit's binary and motor power were not instrumented.

In this recording the SDK LIFT_IN_POS bit cleared and then returned between +1.373 and +1.431 s (last clear, first set by packet receipt). The measured height was below 34 mm by +1.498 s, before the SDK result at +1.518 s. Switching off only the simulated lift actuator, with no cube attached, at either end of that bracket produced a below-34 mm crossing at +1.440 or +1.500 s and a final height of 31.66 mm. The powered control stayed at 46.46 mm. This uses a measured switch window; it is a mechanism check, not a prediction of release time.

Four unloaded physical 25% holds on the same unit had LIFT_IN_POS set throughout the 120–600 ms settled window and median heights of 45.07–46.80 mm. Simply dropping all simulated holding torque when that bit appears would incorrectly collapse the unloaded fork. Passive drivetrain holding torque and the cube load at release remain unmeasured. The runtime controller is unchanged.

### Passive lift holding under load

[Load sweep](passive-load.png) · [Simulated holds](passive-load.csv) · [Friction solver sensitivity](passive-sensitivity.csv) · [Reproduce](passive_load.py)

We settled the simulated fork near the measured 45.42 mm peak, removed lift actuator torque, and applied a constant downward force on the fork. The model assumes a 50 g cube; fractions of its weight represent equivalent vertical loads, without cube inertia, contact geometry or release. The grid tested passive joint friction from 0 to 30 mN·m in 2 mN·m increments. The criterion was at most 1 mm unloaded drop in 0.6 s. With the stiff friction constraint, 6 mN·m was the first tested value to pass. The same value held the other three unloaded starting heights (62, 77 and 89.5 mm) within 0.2 mm. With a quarter cube weight (0.123 N) it crossed below 34 mm by the first 60 ms reported state. This is consistent with the physical distinction between empty holds and a loaded fall; it does not estimate the actual force, exact crossing time, or tooth shape.

The solver matters: at 6 mN·m, the default MuJoCo friction constraint let the empty lift creep from 44.80 to 37.13 mm. Tightening solreffriction and solimpfriction reduced this to 45.34 to 45.07 mm. The strength of the real unpowered drivetrain remains unidentified. Neither the constant load nor the fitted friction was installed in runtime.

### Does the unpowered friction candidate transfer to powered moves?

[Two-sweep overlay and residuals](powered-transfer.png) · [All 16 moves](powered-transfer.csv) · [Reproduce](powered_transfer.py)

We froze the previous bench candidate at 6 mN·m with the same stiff friction constraint and ran the original two unloaded staircase schedules through the powered actuator. Everything else, including the original 0.7 s command duration, stayed fixed. On sweep one, mean per-move RMS error rose from 1.31 to 1.41 mm and mean absolute settled error rose from 1.10 to 1.23 mm. On sweep two these rose from 1.30 to 1.44 mm and 0.90 to 1.16 mm respectively. Full raise improved on the second sweep, but the intermediate holds generally worsened. The two recorded sweeps are repeats from one session. The passive load experiment and this transfer test answer different questions: an effective passive hold can account for load-dependent collapse, while applying that friction throughout commanded movement worsens the existing calibrated servo response. This experiment does not establish an appropriate motor compensation law.

### SDK lift-in-position state

[Status against both physical sweeps](status-transfer.png) · [Per-sweep agreement and recall](status-transfer.csv)

The Python and browser simulators now expose lift_in_pos in the robot state. A fresh target clears it. Once the commanded profile ends and measured joint angle stays within the configured firmware tolerance for more than 100 ms, the bit latches. Repeating an already reached target preserves it. This adds a sensor-state output and does not change actuation.

On 266 and 269 SDK receipts from the two unloaded unit 3 staircases, agreement was 80.5% and 87.7%; a constant set bit scored 48.9% and 56.1%. In the independent failed-grip recording, first clear and set were observed at +0.185 and +1.431 s versus +0.133 and +1.211 s predicted when sampled at those same receipts. The physical bit then cleared and set again after the failed action and stop command. The nominal one-command replay produces only the first pair. Receipt latency, action handling and contact physics still limit agreement. Run status_transfer.py from the repository environment to reproduce.
