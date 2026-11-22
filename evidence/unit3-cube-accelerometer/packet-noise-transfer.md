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
