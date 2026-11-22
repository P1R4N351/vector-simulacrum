# unit 3 shelf dock contact release

![Physical and simulated wheel travel and charger flag transitions for twelve shelf departures](release.png)

Unit 3 cleared its charger flag earlier than the nominal MuJoCo dock in every moving departure. The physical reference is the interval between the last SDK report with status bit 0x1000 set and the first report with it clear. Integrating the average of both reported wheel speeds with the trapezoid rule gives tread travel at both ends of that interval. This estimates wheel travel. It does not measure chassis translation or the instant of electrical disconnection.

Each replay begins settled on the shelf dock, with head and lift at their recorded starting poses. The browser takes recorded wheel speeds as motor setpoints and head and lift reports as posture targets. The simulated wheel loop introduces its own response delay. The blue shading brackets the observed SDK transition. Orange dashes mark nominal simulated release. Green dots mark release after shifting the initial seat toward the lip, settling again, and replaying the same inputs. The orange travel curve belongs to the nominal replay.

Using W2, W3 and W4, we estimated the seat shift as the median nominal excess wheel travel to release relative to the midpoint of each physical observation interval. We applied this **7.94 mm** shift to eight other moving departures without refitting. All recordings are from unit 3 on the same shelf.

| Held-out result (8 moving departures) | Nominal seat | Shallower seat |
|---|---:|---:|
| Mean absolute error relative to midpoint of observed tread-travel interval | 6.52 mm | 1.73 mm |
| Predicted tread travel inside observed interval | 0/8 | 2/8 |
| Mean simulated delay after first SDK report off charger | 416 ms | 167 ms |
| Mean simulated settled chassis pitch | −7.84° | −8.28° |

Mean initial SDK pitch was −4.31° across those eight departures. It is a firmware estimate from a head-mounted IMU. A shallower seat improves release travel but moves predicted pitch farther from that estimate. The pads' electrical reach, dock shape, starting depth, and slip can all influence this comparison. The candidate remains an experiment; the production dock retains its nominal seat. R11 is shown separately because its first loss of charger status occurred with approximately zero net wheel travel. Neither replay reproduced that transition during the plotted window.

Reproduce with Node and the web simulator dependencies installed, then run `python score.py` with NumPy and Matplotlib. The trial list refers to the archived state and action logs under [unit3-wheelstand](../unit3-wheelstand/). [Per-trial results](release.csv)
