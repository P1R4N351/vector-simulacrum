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

### Does the printed fork generate a roll?

The free-cube MuJoCo replay uses the first pre-command SDK cube pose for horizontal position and yaw, places the cube flat on the shelf and drives the printed wrist from observed chassis translation, yaw, pitch and lift. The SDK poses share one odometry origin. The 16 mm pitch pivot height is assumed. The measured pitch is an input, so this isolates fork–cube geometry without predicting chassis pitch from the contact forces.

| Frozen geometry, same initial pose | First fork contact | Maximum cube tilt | Tilt at last chassis sample |
|---|---:|---:|---:|
| Measured pitch | 3.991 s | 91.53° | 90.00° |
| Chassis held level | 3.994 s | 6.20° | <0.01° |

The physical cube tap counter first changed at 3.128 s. Modeled fork contact begins 0.863 s later. The pitched replay crosses 45° at 4.598 s and tips toward the robot (top-face normal ends 11.5° from the horizontal cube-to-robot bearing). The physical packet replay still reports +Z at 5.266 s and the later rest measurement gives +Y. A matching adjacent-face endpoint leaves a discrepancy in the sequence. The source of the first real tap is unresolved. The fixture omits contact with chassis and arms. The prescribed rigid wrist also omits motor compliance. No contact or sensor parameter was fitted to this run.

[Measured pitch replay](contact-replay.png) · [Level control](contact-level.png) · [Recorded pose inputs](roll-poses.csv)

Build the fixture with evidence/unit3-printed-fork-contact/build_fixture.py and forks7.stl, then run replay_roll.py with the fixture XML path. Add --level for the control. The fixture used here had SHA-256 4867e81c8d8d92ad3abc621d920af9b550c001e5d8f81d8443d1e84a069b8230.
