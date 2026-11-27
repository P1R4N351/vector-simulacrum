# Three docked Vectors, 2026-11-26

Unit 2 executed a conditioning move to −10° and three commanded round trips to +25° and back. All seven SDK actions returned success. It began and ended on the platform and charging at 3.966 V. The capture restored the initial head-angle target. Unit 3 (3.592 V) and Unit 1 (3.548 V) were on their platforms, below the pod's active-trial voltage threshold. Their recordings are stationary controls. Both head encoders stayed constant, and both reported zero wheel speed throughout.

![Unit 2 head angle and gyro Y on the charger](head-overlay.png)

The first observed encoder displacement of at least 2° occurred 245, 307 and 321 ms after the three raise commands. The two lowers starting from rest crossed that threshold at 303 and 304 ms. On the first lower the encoder had already fallen from +25.19° to +6.26° before the command. The gyro also records this movement. Its cause is unresolved. The dashed trace is retained and excluded from the timing estimate.

Times are SDK host receipt times on a monotonic clock, with a median state interval of about 60 ms. Millisecond values identify individual packets. Precision of physical onset is limited by sampling and transport. No fit to simulator parameters was attempted.

State rows contain angles and wheel speeds. Short command and return rows carry command index and target, and the fifth field of a return row is the SDK result code. Run analyze.py alongside these files to regenerate the figure and moves.csv. The capture used the stock SDK and an existing paired pod.
