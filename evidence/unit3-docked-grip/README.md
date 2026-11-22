# Docked cube contact with a missing lift tooth

Unit 3 remained on the charger while the operator placed a marked face of the cube against its installed printed fork. The operator reports that the fork is missing a tooth. Which tooth, the fracture shape, and the date it broke are unverified. The intact STL in earlier contact replays is a comparison geometry. Historical footage establishing a physical carry does not establish full seating.

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

On 266 and 269 SDK receipts from the two unloaded P2Y8 staircases, agreement was 80.5% and 87.7%; a constant set bit scored 48.9% and 56.1%. In the independent failed-grip recording, first clear and set were observed at +0.185 and +1.431 s versus +0.133 and +1.211 s predicted when sampled at those same receipts. The physical bit then cleared and set again after the failed action and stop command. The nominal one-command replay produces only the first pair. Receipt latency, action handling and contact physics still limit agreement. Run status_transfer.py from the repository environment to reproduce.

### Does a common receipt-time offset transfer?

[Physical transition offsets and held-out scores](status-lag.png) · [Per-transition timing](status-lag.csv) · [Per-recording agreement](status-lag-agreement.csv)

A median 115.5 ms offset was estimated on the first eight staircase moves using transitions with one physical and one simulated edge of the same polarity within the move. Physical edge times use the midpoint of consecutive SDK receipts; simulated edges use published state time. All SDK samples in the scoring windows were retained, including unmatched transitions. Applying the frozen offset raised agreement from 87.7% to 91.8% on sweep two and from 86.3% to 91.1% in the independent interleaved full-raise session. In the failed grip, agreement over the post-command recording rose from 73.8% to 83.3%; the nominal replay still misses the second clear/set cycle. The first raise's set edge still lagged the shifted prediction by 855 ms. The adjustment changes only the analysis timebase. These host-timed captures combine command delivery, controller dynamics, publication and receipt; they do not identify a universal status transport delay.

### Does an explicit stop reset lift status?

[Four physical stops and four sham events](stop-control.png) · [Measurements](stop-control.csv) · [SDK capture](stop-control.jsonl.gz) · [Failed-action replay](stop-replay.png)

With unit 3 stationary on its dock and the fork lowered, four stop_all_motors commands each cleared the previously set lift-in-position bit, even though the encoder did not move. It first read clear 163–233 ms after the command and set again at 278–297 ms. Four interleaved sham timestamps produced zero clears. All wheel readings were zero. The robot finished on dock, battery full, with quiet active.

The simulator now clears the bit when stopping all motors and lets the usual settling logic set it again. For the failed grip, replaying the stop immediately after the recorded failure response produces the second cycle. With the previously frozen 115.5 ms receipt offset, post-command agreement went from 35/42 to 38/42 states (83.3% to 90.5%). The second set is still early (1.914 s vs 2.044 s). The original script called stop during cleanup; exact SDK dispatch was not timestamped separately. This check predicts telemetry given the command history. It does not predict loss of cube support or the initial failure.

### Stationary stop timing

[Four per-stop overlays](stop-clock.png) and [sample brackets](stop-clock.csv) compare the existing Python model with a direct lift-hold counterfactual. All replays use the prior 115.5 ms observation offset. Each black dot is a received SDK state. A bit transition can fall anywhere between adjacent opposing samples. The control-path A/B compares the SDK call timestamp and its reply timestamp as possible clock origins. Neither is a firmware timestamp.

The frozen model clears the bit for 215 ms after a stationary stop. The physical clear interval was at most 178, 195, 144 and 107 ms, respectively (rounded conservative upper bounds from the surrounding set samples). A constant clock shift cannot repair the pulse width. Bypassing the simulated lift command queue on stop reduces the pulse to 110 ms, still slightly longer than the last trial's receipt-time upper bound. Reply-anchored direct hold captures four of six clear samples and falsely clears two set samples. A firmware timestamp or independently measured observation jitter is needed to resolve the remaining edge differences. The counterfactual is isolated to the replay. These four stops come from one docked Vector 2.0 session with stationary encoders.

Run from the repo root with `PYTHONPATH=. python evidence/unit3-docked-grip/stop_clock.py`.

### Second docked stop recording

P2Y8 received twelve stop-only calls interleaved with six sham events under one SDK override. Both wheel speeds stayed zero. Eleven stop windows had constant lift height. Event 8 spanned 0.339 mm. All stops produced a clear lift bit and all shams stayed set. The end-of-test battery response reported docked, level 3, charging false, and Home Assistant reported quiet on before and after.

[Per-event overlays](stop-holdout.png) · [Counts by event](stop-holdout.csv) · [Monotonic callback recording](stop-holdout.jsonl.gz). Score window −100 to +450 ms. Of 109 stop-window SDK samples, 23 had the bit clear.

| Stop prediction | Correct clear | False clear | Missed clear | Overall correct |
|---|---:|---:|---:|---:|
| Existing 215 ms pulse, dispatch anchor | 23 | 20 | 0 | 89/109 |
| Direct 110 ms hold, dispatch anchor | 12 | 7 | 11 | 91/109 |
| Direct 110 ms hold, reply anchor | 12 | 9 | 11 | 89/109 |
| Direct 110 ms hold, first-session phase | 22 | 1 | 1 | 107/109 |

The last row is exploratory. After reviewing the second-session errors, we tested a stop-specific observation phase. Its numeric value (146.86 ms from dispatch) comes exclusively from the first session, using the median of its interval-midpoint onset estimates and its return-edge midpoint estimates minus 110 ms. Selection of this variant occurred after inspecting the second session. A prospective third session should test it as frozen. It predicts the received SDK flag on a docked P2Y8. The SDK descriptor carries an image timestamp but no lift-controller timestamp, so the relative contributions of command, controller and observation delay remain unresolved. No runtime parameters changed.

### Prospective validation of the stationary pulse

Before the third physical run, the vector-navigation room recorded the exact prediction and scoring window. We reversed the event order from session two and varied the inter-event spacing. Neither clock nor pulse duration was adjusted. Black dots in the [overlays](stop-prospective.png) are SDK receipts on the pod's monotonic clock. All twelve stops cleared the bit with constant lift height. All 54 samples in the six sham windows stayed set.

| Prediction | Correct samples | Clear precision | Clear recall | Stops with any correctly predicted clear |
|---|---:|---:|---:|---:|
| 215 ms pulse starting at +115.53 ms | 91/109 (83.5%) | 24/42 (57.1%) | 24/24 (100%) | 12/12 |
| 110 ms pulse starting at +146.86 ms | 105/109 (96.3%) | 22/24 (91.7%) | 22/24 (91.7%) | 12/12 |

This supports the shorter pulse for predicting pod receipts during a stationary docked stop on this unit. It also gives a concrete limit on transferring the rule to a loaded fork. In the earlier [failed-grip data](dock-status.csv), the post-stop clear samples extend from 1.675 to 1.985 s, spanning at least 310 ms on the receipt clock. The encoder rose from 30.31 to 31.32 mm during that interval. A single 110 ms pulse cannot cover those observations at any offset. We retain the stationary rule in the replay and the settling criterion for moving or loaded stops.

P2Y8 finished docked, battery level 3, charging false, with quiet mode on. [Raw capture](stop-prospective.jsonl.gz) · [Per-event scores](stop-prospective.csv). Reproduce with `python evidence/unit3-docked-grip/stop_holdout.py --prospective`.

### Can the runtime distinguish a stationary stop from the failed grip?

[Encoder and status overlay](stop-posture.png) · [Pre-stop encoder travel](stop-posture.csv). The failed-grip fork fell from 45.42 to 30.98 mm in the final readings before cleanup. The powered Vector 2.0 replay reported a constant 46.46 mm. A rule selecting the stationary pulse on the basis of the simulated encoder would therefore select it incorrectly. On the 16 post-response SDK receipts, that rule scores 12/16 and the existing settling path scores 13/16. The stop call is approximated by the logged failure response at 1.518 s.

For comparison, 25 of 28 physical stop-only events had usable encoder coverage and zero pre-stop travel. For 100, 150 and 200 ms histories we include the last sample at or before the left boundary and subsequent samples received by dispatch. Boundaries older than 100 ms or fewer than two samples produce a missing value. The same histories captured the 14.44 mm physical drop in the failed grip. Two counterfactual runs disabled lift torque within the previously measured first status-return bracket after subtracting the frozen 115.5 ms receipt offset. Both produced 16.82 mm of encoder travel before cleanup. These runs are conditioned on a measured event and do not predict when the real motor released torque. The remaining problem precedes the stop. An encoder-gated pulse in the runtime depends on first reproducing the fork's loss of support.

### Did the cube unload as the fork fell?

[Camera and accelerometer comparison](release-accel.png) · [Windowed packet scores](release-accel.csv) · [Reproduce](release_accel.py). The face glyph returned toward its starting height in both cameras between 1.35 and 1.45 s. Four received BLE packets in that interval included four successive slots below 0.6 g in order of transmission, followed by one slot at 2.22 g. The minimum was 0.37 g. Magnitude avoids interpreting a change in gravity direction alone as unloading. The observations establish that the cube itself underwent a transient consistent with loss of support and impact. They leave residual fork contact, cube rotation and the impact surface unresolved. The next packet arrived 270 ms later, so the intervening motion cannot be integrated from receipt times.

Running the existing free-cube 0/1/2 mm initial fork-gap cases through the simulated cube packet encoder gave 0.968–0.973 g during the same interval for all three. Widening the comparison to 1.25–1.55 s still produced no unloading or impact. As a positive control we repositioned the simulated cube clear of the fork and released it from an additional 10 mm above its resting position. Three releases spaced 10 ms apart relative to the packet clock each produced a minimum below 0.002 g and a peak near the 4 g per-axis limit. The 10 mm input is a sensitivity experiment, not a reconstructed physical drop height. The simulated readout can show a drop. The ordinary fork-contact replay failed to generate one, and a clean simulated free drop overshot both physical excursions. The printed fork with the missing tooth and the real seating depth remain unrepresented in these three-box fork trials.

### Did the raised cube rotate?

Both face glyphs rotated as the fork rose. At +1.35 s a similarity fit to tracked glyph features gave +3.09° in camera A and −2.77° in opposing camera B (43 and 45 inliers). The 10th–90th percentile rotations of long feature pairs were +1.70° to +4.01° and −3.60° to −2.07°. These intervals describe feature agreement within each image. In the pre-command frames each fitted angle stayed within 0.07° of the reference. At +1.45 s both were within 0.23°. The intermediate +1.40 s frame is poorly resolved, particularly in B with ten inliers. These are projected image angles.

In the raised interval (+1.10 to +1.35 s), the physical cube's mean accelerometer direction changed by 5.06° relative to its resting direction. Restricting the window to its last 150 ms gave 5.10°. After settling the change was 0.04°. Acceleration can contribute to this angle. The zero-gap replay briefly tilted 3.17° but all three gap cases had less than 0.001° simulated inclination throughout the physical raised interval. Contact in the zero-gap replay ends too early to account for the sustained physical tilt. Opposite camera signs and the change in mean force direction provide separate checks on a translation-only interpretation. Neither identifies which fork tooth was carrying load.

[Opposing views and feature displacements](release-rotation.png) · [Frame fits](release-rotation.csv) · [Force direction and simulated inclination](release-inclination.csv) · [Reproduce](release_rotation.py)
