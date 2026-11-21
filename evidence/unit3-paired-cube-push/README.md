# Paired optical measurement of five free-cube pushes

unit 3 on his shelf with printed fork and stock rubber-corner cube. Each cycle approached 42 mm from the observation standoff, pushed another 6 mm at 20 mm/s, raised and lowered the lift and withdrew 42 mm. All five ran under a single override. The table compares settled positions across each complete cycle. Trial 1 displaced the cube sideways during the lift. The robot turned to face it again before trial 2.

| Trial | Onboard + odometry (mm) | See3CAM (mm) | Difference between displacement vectors (mm) |
|---|---:|---:|---:|
| 1 | 24.85 | 27.23 | 8.17 |
| 2 | 4.99 | 5.16 | 1.05 |
| 3 | 7.91 | 7.09 | 0.98 |
| 4 | 6.10 | 6.37 | 0.65 |
| 5 | 7.58 | 7.00 | 0.71 |

The independent camera reader ran without behavior control, retained a single subscription and periodically re-requested streaming when idle. Unlike still-photo RPCs in the movement loop, it never held up the next command. The dock check yielded successive frame IDs, then the same reader ran through the trials. This tests the capture workaround; the intermittent gateway stall's internal cause is unresolved. The recorder stores capture timestamp, frame ID and host receipt time. In-motion observations use receipt time. Settled endpoints use median marker picks from 1.5-second windows.

The onboard estimate uses lines fitted to the four outer printed edges of the front glyph and a square-marker perspective pose solve. It assumes a 23 mm marker, 22 mm from glyph plane to cube center, a centered principal point, square pixels, and 20 mm forward camera offset. The robot pose rotates and translates the estimated center into the odometry frame. Effective focal length (537 px) was fitted only on the stationary cube before the trials while odometry changed by about 20 mm. Calibration-position RMS was 0.45 mm. A separate 18 mm retreat after trial 5, with the cube stationary, yielded 4.33 mm apparent onboard travel and 0.06 mm overhead. Similar-standoff endpoint agreement is better than cross-standoff accuracy.

The overhead estimate uses the intersection of the top glyph diagonals. A local projective mapping uses the nominal 44 mm cube envelope in the initial pose, square constraints on the top glyphs at each settled observation, and the rear shelf edge for orientation. The overhead reconstruction does not use robot odometry or onboard displacements. Reported physical distances are approximate. Colors in the endpoint overlay compare vectors in dock-forward and dock-left coordinates. The time plots exclude oversized or occluded onboard glyphs and raised-fork onboard picks. During lifting, apparent marker motion also includes changing projection. Gaps indicate missing or rejected observations.

The CSVs contain millimetres. endpoint-picks.json records both camera picks and robot state. The image contact sheet shows before and after for all five trials. These are physical validation references, not a simulation accuracy score.

[Physical pickups, turns and rapid reversals with Unit 3](../unit3-native-pickup/README.md)
