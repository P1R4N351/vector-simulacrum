# Physical cube pickup and carry on the shelf

Unit 3 is a Vector 2.0 with a printed lift. After a baseline stock pickup, three further pickups visibly carried the cube during motion. The See3CAM overhead view establishes physical engagement. The robot's own carry flag disagreed with the footage once.

| Maneuver with cube | Measured yaw | SDK pitch range | Largest sampled lift drop |
| --- | --- | --- | --- |
| Initial short wheel pulse and stop | See pulse trace | 1.58° | 4.60 mm |
| Left first, then right | +79.47°, −78.40° | 0.41°, 0.31° | 0 mm |
| Right first, then left | −78.99°, +77.01° | 0.62°, 0.56° | 0 mm |
| Four forward/reverse steps, left-first run | See CSV | 0.19° | 0 mm |
| Four forward/reverse steps, right-first run | See CSV | 0.25° | 0 mm |

The first pulse took the lift from 91.79 to 87.20 mm. The minimum came about 0.66 s after the command, after the all-motors stop returned. The lift recovered to 90.31 mm. The empty comparison used a lowered fork. Load, posture and the stop command therefore remain confounded.

The two longer sequences used opposed 80° commands followed by four alternating wheel-speed steps at 45 mm/s magnitude. Wheels-only stops kept the lift engaged. One sequence included an empty raised-fork control. Empty pitch ranges were 0.49° and 0.42° during the turns and 1.32° during reversals. The loaded reversal ranges were smaller in both sequences. A guard interrupted the second empty control after release. Pitch is the SDK robot pose estimate. Independent cube rocking and motor torque were not measured. Wheel speed shortfall during short reversals includes acceleration and sampling effects.

![Pitch, wheel speed and lift during turns and reversals](rotation.png)

[Baseline pickup footage](pickup.mp4) · [Short pulses](first-pulses.mp4) · [Short pulse traces](pulses.png) · [Left-first carry and raised-fork control](turn-left-first.mp4) · [Right-first carry](turn-right-first.mp4) · [SDK carry with the cube visibly on the shelf](false-carry.mp4)

The false carry was excluded from loaded comparisons. A separate pickup ended with VISUAL_OBSERVATION_FAILED. The final place action reported BAD_OBJECT although the overhead view showed release. Action success, carry ID, and visual engagement are recorded separately.

The compressed JSONL files contain robot states and action events. rotation.csv and pulses.csv contain per-action measures. The CSV beside each video lists camera timestamps. Videos pair frames by host time at 8 fps. A blank onboard pane indicates missing fresh imagery. The cameras have independent clocks and receipt latency. The raw measurements apply to this robot, fork, cube and shelf.
