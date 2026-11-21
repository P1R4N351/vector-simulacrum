# P2Y8 pushes the free cube

One Vector 2.0, printed lift, one upright rubber-corner cube, shelf surface. A continuous SDK override spanned the Vector SLAM undock imperative, approach, push, lift, withdrawal and final dock. The camera observed the top glyph independently of wheel odometry. Times in the film and telemetry are relative to the first straight command. The film omits the pauses between actions.

| Movement | Observed response |
|---|---|
| 17 mm approach at 20 mm/s | Top marker stayed within 0.1 px between the two settled frames. |
| Additional 12 mm at 20 mm/s | Robot odometry advanced 12.8 mm. The cube marker moved 32.7 px toward the camera. |
| Lift from 31.3 to 65.8 mm, then lower | The cube stayed on the shelf. The fork missed or slipped; the view does not resolve contact within the recess. |
| Reverse 30 mm | Cube remained at its pushed position, within 2 px comparing the settled endpoints. |

Estimated post-push displacement is 10.7–12.2 mm using a local planar homography from the four outer corners of the nominal 44 mm horizontal face. This range varies the near corner line across the visible rounded rim; it is not a confidence interval. Raw full-resolution, 180°-corrected marker centroids before and after the push were (762.50, 956.00) and (762.77, 988.69) px. No global fisheye calibration was applied. Use this as a reference for reproducing the contact sequence; a displacement-controlled push without contact-force measurements cannot identify rubber friction on its own.
