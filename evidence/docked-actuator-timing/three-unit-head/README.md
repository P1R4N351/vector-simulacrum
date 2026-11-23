# Repeated head strokes, November 22

Unit 3 completed three cycles of +40°, −18°, +15° and −8° at default speed under continuous SDK override on the dock. Initial head angle was +1.59° for the first move. Subsequent cycles began near −8°. The recording contains all 12 successful commands and 373 state callbacks (median interval 61 ms). The SDK confirmed charging at 4.004 V after restoring the head, which measured +0.73°. The recording host reported quiet mode on after release.

![Three physical cycles and their frozen Vector 2 replays](unit3.png)

Dots and solid curves show measured head encoder and pitch gyro. Dashed curves show each replay from the measured starting posture with the previously estimated 91.6 ms receipt offset. Horizontal bars on the gyro panels bracket the physical head-in-position transition. The replay uses level-table support. Across the 12 moves the median per-move encoder RMS was 5.87° on the unshifted host clock and 1.19° with the prior offset. Pitch gyro RMS was 38.21 and 11.20°/s respectively. No parameter was fitted on this session.

[State and command log](unit3.jsonl) · [Per-stroke transition brackets and errors](unit3.csv)

Unit 1 and unit 2 were unavailable from both pods. The workshop PodNet access point had zero associated stations at the final check. No workshop trials are included.
