# Does the stationary cube packet model transfer?

![Packet excursion survival curves and variation by sample spacing](packet-noise-transfer.png)

Each BLE packet contains three XYZ slots. We divided successive slot differences by the per-axis RMS differences in the 643-packet resting capture. The score is the larger of the two resulting vector lengths. This eliminates the gravity offset and avoids assumptions about interpacket timing. It also removes information about slow motion.

With the current Vector 2.0 cube parameters frozen, eight simulation seeds produced 5,328 packets. The simulated median and 95th percentile were 1.87 and 3.03. Held-out pre-lift windows gave medians of 1.86 and 1.84 and 95th percentiles of 2.91 and 3.15. Both held-out 95th percentiles are within 5% of the simulated value. The 99th percentiles differed by more than 5% in both sessions. Those estimates depend on just a few packets.

The resting 99th-percentile threshold of 3.61 was exceeded in 39/5,328 simulated packets (0.73%), 0/218 D1 packets and 3/149 D2 packets (0.82% pooled). The undock-check recording crossed it in 31/253 packets (12.25%). That run included a physical undock attempt on the shelf. A high packet excursion alone cannot identify fork contact. The per-axis two-slot to one-slot RMS ratios were 0.98–1.01 at D1 and 1.00–1.15 at D2. These observations apply to this cube, surface and packet format.

[All scores](packet-noise-transfer.csv) · [Reproduce](packet_noise_transfer.py) · [Original stationary fit and physical pickup](README.md)

## Does lift vibration transfer by cube axis?

![Within-packet acceleration variation during held-out lift strokes](lift-packet-transfer.png)

We measured successive XYZ differences inside each radio packet in the encoder-defined 10–90% stroke interval and in the 0.7 s quiet window before the command. Twelve docked D1 strokes supplied a median increase in variance for each direction and axis. Adding that variance to the pre-stroke reading predicted the seven timing-valid D2 strokes beside the cube. Each axis and stroke had equal weight. No D2 vibration samples entered the estimates.

Overall mean absolute error improved from 0.94 mg using pre-stroke scatter alone to 0.78 mg. For raises, Z error fell from 2.46 to 1.06 mg, while Y error increased from 0.57 to 1.45 mg. For lowers, Z error fell from 0.97 to 0.43 mg and Y rose from 0.39 to 0.82 mg. The cube's quiet gravity vector lay within 2° of +Z in both sessions. Rotation about gravity, mechanical coupling to the shelf and the source of vibration remain unresolved. D1's measured gain on all three axes would overstate the reproducibility of lateral vibration at D2. This is an empirical candidate for shelf vibration during full strokes and has not been enabled in the simulator.

[Per-stroke measurements and predictions](lift-packet-transfer.csv) · [Replay](lift_packet_transfer.py) · [Physical D1 control](../unit3-wheelstand/D1/) · [Physical D2 repeat](../unit3-wheelstand/D2/)

## Does the robot's own accelerometer predict cube vibration?

![Cube Z scatter with lift-direction and onboard-acceleration predictions on D1 and D2](onboard-vibration.png)

We compared the variance of the robot head IMU's acceleration magnitude during each stroke with its pre-command variance. Magnitude avoids rotating the head axes into the shelf frame. We used the positive excess to predict the cube's within-packet Z variance with a single nonnegative gain trained on the 12 D1 strokes. Each D2 scoring window had only 9–10 robot samples, so this tests stroke-level amplitude at SDK cadence.

| Mean absolute error in cube Z scatter | D1 fit | D2 transfer |
|---|---:|---:|
| Pre-stroke baseline | 2.35 mg | 1.61 mg |
| Median by lift direction | 0.66 mg | 0.70 mg |
| Onboard acceleration gain | 1.82 mg | 1.30 mg |
| Direction means plus onboard gain | 0.37 mg | 1.78 mg |
| Direction means alone | 0.80 mg | 0.75 mg |

The combined model fit D1 more closely but transferred poorly when unit 3 was off the dock. D1 used slow and fast strokes, while D2 used an intermediate commanded duration. These data leave dock coupling, location, speed and acceleration bandwidth confounded. The robot IMU at this cadence does not justify a shared gain for predicting vertical cube vibration in both settings. [Per-stroke inputs and errors](onboard-vibration.csv) · [Analysis](onboard_vibration.py)

## Can lift speed or travel explain the partial raises?

![Cube vibration predictions on off-dock full raises and native partial raises](lift-reach-transfer.png)

We used only the six full D1 raises to estimate vibration above the pre-stroke baseline. On six native partial raises, a linear speed fit to excess variance gave 3.12 mg mean absolute error. Scoring only the D1 packet samples from the same 10–90% height interval above the starting pose gave 3.22 mg. The original full-stroke estimate gave 3.61 mg. In particular, R6 rose at 52.3 mm/s, close to the slow D1 raises at 49–50 mm/s, yet its cube magnitude SD was 3.09 mg versus 6.69–6.87 mg on D1.

Scaling the added RMS vibration amplitude in proportion to total lift travel reduced partial-raise error to 0.62 mg, versus 0.76 mg using the pre-stroke baseline alone. This assumed proportionality has no exponent fitted on the partial raises. It still overpredicted all six. Across all ten evaluation raises (three valid D2, six partial, one full native), none of those travel-scaled predictions was within 5% of the measured SD. The lower portion of a commanded full stroke already vibrates more than a whole native undock raise. Stroke length, the motor profile and mechanical setup remain confounded. Keep the proportional rule offline.

[Per-stroke predictions](lift-reach-transfer.csv) · [Reproduce](lift_reach_transfer.py)

## Could cube heading explain the lateral vibration discrepancy?

We took successive XYZ accelerometer differences within each three-sample BLE packet and projected them onto the plane perpendicular to each stroke's pre-command mean gravity. We used the same rest and 10–90% lift windows as the original comparison. The sum of the two horizontal mean squared steps, divided by two, is invariant under any rotation about that gravity direction. Its square root gives the plotted vibration amplitude. This assumes gravity direction stays effectively fixed during each stroke.

Twelve docked D1 strokes supplied the added horizontal variance separately for raises and lowers. On all seven timing-valid off-dock D2 strokes the prediction exceeded the observation. Across the three raises D2 measured 3.74–4.60 mg against 5.96–6.07 mg predicted. Across the four lowers it measured 3.00–3.75 mg against 6.18–6.55 mg. Taking the median of the individual D1 variance increments reduced mean absolute amplitude error from 2.43 to 1.26 mg. Using each D2 stroke's quiet baseline alone gave 0.76 mg. Errors weight each stroke equally.

A cube heading change can redistribute vibration across its axes but preserves this horizontal total. Even allowing a separate best-case horizontal rotation on each D2 stroke leaves a residual in the second-moment matrix (reported in mg²). Dock coupling, lift profile and cube location still differ between sessions. These recordings support keeping the D1 horizontal vibration gain out of the shared sensor model.

[Overlay](horizontal-vibration.png) · [Per-stroke scores](horizontal-vibration.csv) · [Reproduce](horizontal_vibration.py)

### Does matching encoder speed explain the difference?

For each lift direction we interpolated added horizontal variance between the median slow and fast docked strokes, using median measured speed at each endpoint. Each off-dock speed fell between every slow and fast training speed. Training uses three docked repetitions per direction and speed. Off-dock variance comes from the original seven timing-valid strokes in a separate recording. We also interpolated each of the nine possible slow/fast endpoint pairs to check sensitivity to individual docked strokes.

At 85–90 mm/s, the three off-dock raises produced 3.74–4.60 mg horizontal vibration, versus 6.12–6.32 mg predicted. Even the least energetic pair of docked endpoints overpredicted each raise. Lowering at 81–88 mm/s produced 3.00–3.75 mg versus 3.94–4.54 mg predicted. One lower came within 0.063 mg² of the least energetic pair; this small difference is unresolved. The nine-pair envelope describes observed endpoint sensitivity and carries no statistical confidence level.

Mean absolute amplitude error was 2.04 mg on raises and 0.78 mg on lowers. Predicting the pre-stroke reading gave 1.26 and 0.38 mg respectively. Linear interpolation using mean stroke speed does not explain the discrepancy. A nonlinear response between the two docked speeds, stroke acceleration, and differences in mechanical coupling remain possible. [Speed comparison](horizontal-speed.png) · [Per-stroke values](horizontal-speed.csv) · [Reproduce](horizontal_speed.py)
