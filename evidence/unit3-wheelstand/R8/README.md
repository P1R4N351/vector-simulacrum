# Autonomous cube roll on the shelf

After a native undock Unit 3 acquired a cube glyph at 95.4 mm and issued roll_cube with zero retries. The previous approach (R7) missed the glyph and made no contact. R7 supplies a search-only comparison.

The SDK reported taps at 3.285 and 4.059 seconds after the roll command. Cube acceleration magnitude peaked at 1.43 g. The first reported cliff flag arrived at 4.363 seconds with SDK pitch at 10.1°. Pitch reached 31.3°. The guard interrupted the action before an SDK result arrived.

The 69 cube samples in the last 0.9 seconds before command averaged (0.016, −0.018, 0.971) g. A separate docked recording starting 197 seconds after command yielded 834 samples over ten seconds averaging (0.009, 0.970, −0.009) g. The angle between mean gravity vectors is 91.58°, consistent with settling on an adjacent face. The gap includes redocking, so the two rest windows alone cannot establish the instant the cube changed face. A gravity reading also leaves yaw undetermined.

[Contact video](roll.mp4) · [Sensor overlay](roll-traces.png) · [Post-roll gravity samples](settled.csv.gz)

Time is in seconds from the undock command on the receiving host. Three cube samples share each notification arrival time and retain their packet order in the slot column. Accelerometer axes are signed counts at 8192 counts/g. The cropped overhead clip spans the approach, contact and release. The plot shows SDK wheel speeds, lift, pitch and raw cube acceleration on the receiver clock. No simulator parameters were fitted.

### Held-out event timing

Replaying the recorded packets through the frozen listeners yields two tap candidates. Applying the 145.906 ms receiver delay estimated in earlier experiments predicts the SDK tap reports 11.0 ms early and 8.1 ms late. Movement onset is 24.7 ms early. Both replay and SDK report zero up-axis changes during the captured roll interval. The last recorded packet arrived 5.266 s after the command, while the replay filter still classified +Z as up. The separate rest sample establishes +Y up later.

This tests the packet-to-event path given measured acceleration and the physical firmware tap counter. It does not test whether simulated fork contact generates those accelerations or increments that counter. Times use host receipt; samples in a packet share one timestamp.

[Event timing overlay](event-replay.png) · [Timing table](event-replay.csv) · [Reproduce the replay](replay_events.py)
