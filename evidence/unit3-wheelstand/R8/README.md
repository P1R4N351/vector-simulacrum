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

### What can reach the cube at the first tap?

With the cube fixed at each of four pre-contact SDK observations, the simulated front-right tipping cylinder overlaps the cube shell by 2.09–2.44 mm at the first transmitted counter change (3.128 s). This cylinder represents the tread near the actual axle. The inset wheel collision geometry is within 0.2 mm of touching in all four cases. At the same instant the chassis box is at least 5.7 mm clear, head 10.6 mm, lower links 28.6 mm and upper links 35.2 mm. These are signed distances at corresponding receiver times. The four observations share camera calibration and odometry errors; their spread is a sensitivity check.

Adding just the driven front-right cylinder to the free-cube fork replay yields first support contact at 3.086 s, 42 ms before the physical counter change. The fork follows at 3.987 s. The cube peaks at 43.18° and returns to its original face by the final chassis sample. The fork-only replay reached 90°. Thus the front-right tread offers a geometrically plausible source for the first disturbance, while the extra contact materially worsens the endpoint prediction. No firmware tap was predicted from that contact force and no physical contact point was identified in the video. Neither replay extends beyond the recorded chassis motion.

Run early_contact.py for the clearance plot (requires the viewer SDK assets and web dependencies used by replay_links.py). Pass --front-right to replay_roll.py for the free-cube experiment, using the same printed fixture as above.

[Part clearance](early-contact.png) · [Geometry and distances](early-contact.csv) · [Free cube with tread contact](front-right-replay.png)

### Does the tread surface speed explain the roll?

The paired replays use the same translating, pitching front-right tread cylinder, printed fork, cube, and contact material. A free joint is driven from consecutive axle poses with explicit linear and angular velocity. One replay locks the tread rotation relative to the chassis; the other applies measured right-wheel speed divided by cylinder radius. Both impose the trajectory every step. The proxy has assumed 180 g inertia mass and diagonal inertia of 0.0001 kg m² per axis. This change of driving method matters: the stationary-rotation control now also rolls the cube. Comparing these two cases isolates the effect of rotation within this fixture.

At 0.25 ms the control reaches 170.3° peak tilt and the rolling tread reaches 154.8°. Both end at 90° by the last recorded chassis pose. At 0.5 ms the peaks are 170.1° and 155.2°. The rolling case first touches the cube at 3.086 s across all three step sizes. It predicts four packet counter changes at the two finer steps versus two observed from 2.5 s onward. Its packet acceleration magnitude peaks at 5.39 g at the finest step versus 1.43 g measured in the common interval; individual axes can saturate. Packet peak magnitude is timestep sensitive. At the final simulated packet the rolling case reports −Y up, while the physical receiver still reports +Z and the later physical rest window measures +Y up. The comparison uses the model's nominal sensor mounting and assumes the SDK cube yaw registers the same sensor frame.

The replay uses the existing accelerometer, packet format, and filter on the MuJoCo cube chip site with a fixed seed. The robot's pose and fork motion remain prescribed. The several-fold acceleration error and different indicated face argue against adopting this driver as a calibrated model.

[Tread-speed overlay](tread-speed.png) · [All six runs](tread-speed.csv) · [Reproduce with printed fixture XML](tread_speed.py)
