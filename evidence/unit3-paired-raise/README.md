# Interleaved physical full raises, unit 3

[Physical traces and holds](paired.png) · [Physical and frozen simulation endpoints](replay.csv) · [Raw SDK events](paired.jsonl.gz)

Unit 3 completed four unloaded raises from bottom to full and four from 75% to full, interleaved in one session. Every command requested 0.7 s duration and used the same 0.85 s post-success dwell. Both schedules reached 91.55–92.04 mm. The first five raises ended on the same encoder value, followed by three on the next lower value, across both starting heights. All eight were within 0.25 mm of the frozen Vector 2 model’s settled reading of 91.79 mm.

The earlier 89.29–89.54 mm quarter-stroke endpoints did not repeat. In this experiment the 75% preparation approached from above. The earlier staircase approached 75% from below. Head pose also differed (about +35° here versus −22° at the start of the earlier staircase). These recordings leave reversal history, head pose and session effects unresolved. The measurements are encoder heights and cannot establish absolute fork-tip accuracy.

The corrected shelf view showed the cube clear before testing. All 17 actions succeeded and all 588 SDK states reported charger contact and stationary wheels. The robot finished docked with a full battery and active quiet mode. Hold measurements use the median 120–600 ms after action success. Reproduce with analyze.py, then run replay.py with the repository on PYTHONPATH.
