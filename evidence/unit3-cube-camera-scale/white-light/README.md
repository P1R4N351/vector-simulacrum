# Cube white LEDs as seen by docked unit 3

![Onboard brightness and exact-stamp cube detections](light-response.png)

With the head at −8° and the cube stationary, we alternated the cube's white LEDs off and on across seven three-second blocks. We discarded the first second of each block for optical scoring. The camera returned 44 frames in total, with four scored frames per block. The cube sent 581 accelerometer packets (1,743 XYZ samples). No wheel or cube motion was commanded. At handback the cube LEDs were off, the cube was disconnected, and unit 3 was charging at 4.06 V with quiet enabled.

In the three white blocks, mean JPEG grayscale over the cube's upper strip was 41.4–41.6 on a 0–255 scale. The four dark blocks were 26.5–28.5. The front glyph region stayed at 76.6–76.7. The SDK reported the cube at the exact timestamp of all 28 scored images. This placement therefore measures how the light appears but supplies no detection advantage. The accelerometer's mean vertical component ranged from 970.1 to 971.0 mg across settled blocks. Ambient illumination and camera autoexposure were uncontrolled.

The repo contains the SDK image/report timestamps, light command times, raw cube packet payloads, and per-frame luminance. Run `python summarize.py` to regenerate the plot, or add `--images DIRECTORY` to remeasure the fixed pixel regions on the original JPEGs.

### Stationary accelerometer holdout

![Stationary cube gravity, noise and packet timing](stationary-holdout.png)

We ran the frozen Vector 2 cube model upright under gravity with seed zero and compared the first 581 packets with this separate physical recording. Within-packet noise on X/Y/Z was 1.98/2.29/2.65 mg RMS physically and 2.00/2.23/2.72 mg in simulation. All signed axis words in both streams had remainder 1 modulo 4. Neither tap counter changed. Mean Z agreed within 0.30 mg, but mean X differed by 18.35 mg. A single resting orientation cannot separate inclination from accelerometer bias.

The 580 physical interarrival intervals spanned 20.86 s (27.8 packets/s at the host), compared with 17.40 s for simulated emission (33.3 packets/s). The longest host gap was 275 ms. These are timestamped at different stages, so this recording alone cannot distinguish missed packets, transport delay and a different production rate. We retain the existing calibration. Reproduce with `python stationary_holdout.py`.

### Transfer from robot receipt to host receipt

A receiver model fitted earlier on the robot's Bluetooth daemon timestamps predicts 21.123 s for this packet count. The host measured 20.863 s, a +1.24% prediction error. The ideal emission timeline predicts 17.400 s (−16.60%). The host trace has 17 gaps longer than 150 ms, spanning 226–275 ms. Using the first restart as a timing anchor and the 1.22746 s period from the earlier resting recording, median absolute timing error for the other 16 restarts is 38 ms, maximum 65 ms. This recurring receiver pattern explains most of the apparent rate difference across these two observation points. The cause of the pauses and what happened to chip samples during them remain unresolved.

[Timing overlay](receiver-holdout.png) · [Reproduce the comparison](receiver_holdout.py) · [Earlier robot-side evidence](../../unit3-cube-accelerometer/receipt-transfer.md)
