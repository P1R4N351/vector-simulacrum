# Vector physics overlays

## Shelf tilt from stationary accelerometer samples

![Tilted and flat replays on held-out spot A](shelf-tilt-heldout.png)

[Opposite-heading estimate, contribution and replay errors](../shelf-tilt/README.md)

## Opposite headings at two shelf spots

![Camera displacements in shelf and starting robot coordinates](heading-crossover-frames.png)

[Eight measured turns, both 42 mm overlays, corrected See3CAM footage and calibration](../heading-crossover/README.md)

## Lift-down, lift-up, lift-down spins

![Camera and model displacements by lift posture](lift-camera-drift.png)

[Six-turn measurements, sensitivity and upright video](../lift-camera/README.md)

## Independent camera tracking

![Camera, odometry and two contact models after each spin](camera-rest-drift.png)

![Camera trace through four alternating spins](camera-spin-traces.png)

[Calibration, upright video, independent optical checks and per-turn vectors](../camera-spins/README.md)

## Contact orientation and integration step

![Constant-speed migration and recorded-turn checks](pressure-sweep.png)

![Transfer to five SDK turns](pressure-transfer.png)

[Pressure geometry experiments and per-turn data](../endurance-drift/pressure.md)

## Balanced-wheel pivot and contact geometry

![Pivot tracks during nine alternating turns](balanced-pivot.png)

![Contact geometry transfer and native gyro check](pivot-transfer.png)

[Pressure measurements, held-out results and pivot coordinates](../endurance-drift/pivot.md)

## Alternating spins and edge detection

![Vector 2.0 turn-by-turn translation](endurance-drift.png)

![Yaw replay and cliff event timing](endurance-timeline.png)

[Run G analysis, input data and turn-by-turn results](../endurance-drift/README.md)

## Turning and tread contact

![Hybrid encoder replay of Vector 2.0 stops](hybrid-encoder-replay.png)

[Held-out hybrid results and one-frame switch sensitivity](../hw-rec-20261119/hybrid-replay.md)

![Native encoder replay with 42 mm and 48 mm patch spans](native-span-replay.png)

[Held-out timing test and raw-count input comparison](../hw-rec-20261119/native-replay.md)

![Native encoder and yaw braking overlays](native-wheel-stop.png)

[Four Vector 2.0 stops on the robot clock](../hw-rec-20261119/native-stops.md)

![Firmware sensor timing through the held-out stops](sensor-timing-braking.png)

[Firmware and SDK trace, phase assumptions, and held-out scores](../hw-rec-20261119/sensor-timing.md)

![Measured and simulated braking in both directions](repeated-braking.png)

[Held-out braking scores and signed wheel and gyro traces](../hw-rec-20261119/repeated-braking.md)

![Encoder-to-gyro timing calibration and held-out turn](turn-lag-transfer.png)

[Timing methods, before/after scores and per-sample data](../hw-rec-20261119/unit2-ccw100.md#encoder-to-gyro-timing-test)

![Held-out 42 mm contact span and 48 mm control](unit2-heldout-contact.png)

![The same turn on two robots](unit2-unit1-ccw100.png)

![Original counterclockwise turn and contact-model variants](vector-100-ccw-gyro.png)

![Incline sensitivity](vector-turn-incline.png)

## Head and lift

![Speed against head angle, by starting posture](vector-head-gyro-angle.png)

![Head movements aligned at first motion](vector-head-aligned.png)

![Lift movements aligned at first motion](vector-lift-aligned.png)

![Unloaded lift timing](vector-unloaded-lift.png)

[Head, lift, shell and tread findings](../stock-shell-physics-20261118.md)

[unit 3 lateral friction and head posture](../lateral-friction/README.md) compares the B friction sweep with held-out A camera endpoints and gyro traces, including a level-shelf control.

[unit 3 stationary head sweeps](../head-sweep/README.md) separate head posture, chassis offset, approach direction and a shelf-frame component, with opposite and sideways heading tests at held-out A.

[Optical head and chassis check](../face-tag-trial/README.md) includes matched stock-crown and simulator views, the held-out vent-placement check, side-insert tracking from opposite headings and the fixed-backpack pitch comparison.

The [unit 3 three-camera survey](../three-camera-survey/README.md) includes the full orbit coverage, held-out silhouette and head-angle checks, crown and microphone reprojections, and the revised Vector 2.0 crown rendering.

### Longitudinal contact inset at the recorded posture

The unit 3 yaw fit selected 3.75 mm per end and reduced yaw error on all four held-out unit 1 spins. The camera pivot and roll/pitch measurements still disagree with the model. [Overlays, selection sweep and replay data](../recorded-pose-inset/README.md)
