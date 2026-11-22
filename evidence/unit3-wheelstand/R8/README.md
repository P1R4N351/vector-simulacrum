# Autonomous cube roll on the shelf

After a native undock P2Y8 acquired a cube glyph at 95.4 mm and issued roll_cube with zero retries. The previous approach (R7) missed the glyph and made no contact. R7 supplies a search-only comparison.

The SDK reported taps at 3.285 and 4.059 seconds after the roll command. Cube acceleration magnitude peaked at 1.43 g. The first reported cliff flag arrived at 4.363 seconds with SDK pitch at 10.1°. Pitch reached 31.3°. The guard interrupted the action before an SDK result arrived.

The 69 cube samples in the last 0.9 seconds before command averaged (0.016, −0.018, 0.971) g. A separate docked recording starting 197 seconds after command yielded 834 samples over ten seconds averaging (0.009, 0.970, −0.009) g. The angle between mean gravity vectors is 91.58°, consistent with settling on an adjacent face. The gap includes redocking, so the two rest windows alone cannot establish the instant the cube changed face. A gravity reading also leaves yaw undetermined.

[Contact video](roll.mp4) · [Sensor overlay](roll-traces.png) · [Post-roll gravity samples](settled.csv.gz)

Time is in seconds from the undock command on the receiving host. Three cube samples share each notification arrival time and retain their packet order in the slot column. Accelerometer axes are signed counts at 8192 counts/g. The cropped overhead clip spans the approach, contact and release. The plot shows SDK wheel speeds, lift, pitch and raw cube acceleration on the receiver clock. No simulator parameters were fitted.
