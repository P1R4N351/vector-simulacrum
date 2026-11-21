# unit 3 shelf repeatability

Seven sequential cycles with the same robot, printed fork, free cube, and surface. One SDK override covered the household undock imperative and all shelf movements. Each cycle used a 20 mm/s forward command, a 55% lift and lower, a 10 mm retreat, and a 10 mm reapproach. The first five strokes were 6 mm, followed by 4 mm and 8 mm. unit 3's heading stayed within about 1.5° across the seven pushes. The starting cube pose evolved from one cycle to the next.

| Stroke | Runs | Wheel odometry during forward stroke | Net cube displacement over whole cycle |
|---|---:|---:|---:|
| 6 mm | 5 | 6.19 ± 0.25 mm | approx. 4.61 ± 2.61 mm |
| 4 mm | 1 | 4.38 mm | approx. 4.39 mm |
| 8 mm | 1 | 8.31 mm | approx. 8.46 mm |

Plus/minus denotes sample SD. The sequential trials measure within-session repeatability, including the changing contact geometry. Single observations at the two other strokes cannot establish a slope or repeatability at those settings.

The See3CAM measured the top-face glyph in the rotated, cropped image. Cube travel is the magnitude of the difference between median marker centroids before the push and after reapproach. Five nominally identical cycles yielded 1.35–7.34 px. During a 4 s stationary window at the trial position, 95% of the 14 marker detections lay within 0.114 px of their median. The analysis tracked the glyph in 342 of 360 frames. A local affine conversion from the two projected edges of the nominal 44 mm top face gives approximate shelf-plane distances. The intermediate traces include changes in apparent position from tilt/lift. Raw image-space endpoints are included for independent evaluation.

The proximity signal returned a median of 30 mm before four of the five equal strokes and 31 mm on the first. These readings do not resolve the differences between trials. A robot-camera still was obtained before the sequence. The second still request stalled after withdrawal, so there is no before/after onboard-camera estimate. A separate control session completed docking.

For the eventual hardwood trial, use onboard marker images at matched stationary standoffs before and after each cycle, logging common-frame robot pose and lift position. Validate repeatability of that onboard measurement on the shelf first. Changing camera method and surface simultaneously would confound the comparison.
