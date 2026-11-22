# Docked lift approach history on P2Y8

[Full-raise pairs](paired.png) · [Intermediate and final measurements](holds.csv) · [Frozen replay](replay.csv) · [75% endpoint transfer](transfer.png) · [Raw physical SDK events](physical.jsonl.gz)

Each of four blocks used both 50→75→100% and 100→75→100% sequences with identical 0.7 s move duration and 0.85 s post-success dwell. The head targets were −22°, +35°, +35°, −22°. Within-block order reversed at each head pose on its repeat. Holds use median SDK heights 120–600 ms after success. Each three-command simulation starts from the recorded head and lift pose. The cube stood clear, wheels remained stationary and the robot stayed on its dock. Quiet was verified active after the experiment.

All four blocks reproduced a directional split at 75%: approaching from below ended 1.83–2.42 mm higher than approaching from above. The subsequent full raise reached 92.04 mm seven times and 91.06 mm once. Neither head posture nor approach sequence reproduced the earlier 89.29–89.54 mm full-height endpoints.

An endpoint-only correction fit on blocks 1–2 adds 1.21 mm to the frozen 75% prediction after a raise and subtracts 0.92 mm after a lower. On blocks 3–4, mean absolute error fell from 1.14 to 0.23 mm. Applied unchanged to the two earlier staircase sweeps, it fell from 0.76 to 0.30 mm. Those sweeps had previously motivated the hypothesis. This comparison concerns an unloaded, settled encoder reading at one intermediate height on one unit. It does not determine physical fork deflection, encoder bias or transfer under load. The runtime model remains frozen pending those distinctions.

Run analyze.py, then replay.py with the repository on PYTHONPATH, then transfer.py.
