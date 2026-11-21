# Four physical shelf recordings with one robot and one cube

Four separate continuous-override sessions used the native undock behavior and returned to the dock. Each directory includes overhead video, action times, SDK object events, wheel speeds, head and lift position, odometry, robot IMU and proximity feedback. Cameras and radio events are aligned by host receipt. The overhead videos are 5 fps resamples; independent raw frames typically arrive more slowly. The onboard capture preserves original JPEG bytes. No changes to simulator parameters were fitted on these runs.

| Session | Robot states | Cube XYZ observations | Physical outcome |
|---|---:|---:|---|
| [A](A/) | 1,906 | 4,737 | Cube was visible during undocking but the pickup search lost it. The robot displaced and rotated the cube during its later docking maneuver. |
| [B](B/) | 1,536 | 0 | Connection request returned without an active cube stream. Returned to dock before the contact sequence. |
| [C](C/) | 862 | 0 | Retreated for a wider field of view and searched optically. No fresh sighting before docking. Radio connection was conditional on a sighting in this run. |
| [D](D/) | 1,946 | 0 | With the cube placed to the side, seven optical sightings during the interior heading sweep, at two commanded head angles. Bluetooth connection request did not produce raw packets. The run was an optical survey. |

### Real contact during the dock return

![Cube movement, notifications and raw accelerometer in A](A/contact.png)

In A the robot generated four SDK tap events and one moved/stopped pair. The XYZ magnitude peaked at **1.84 g** between 60 and 68 s and **2.39 g** between 88 and 96 s. The early and late optical sightings share an odometry origin and differ by about **86 mm** in the horizontal plane. Camera footage independently shows translation and rotation. This is an incidental contact recording with multiple encounters, not a commanded stroke or an isolated friction measurement. The radio continued to deliver packets after the disconnect request. **None of these four runs reached the pickup command.**

### Seeing the stationary cube from the shelf interior

![Onboard cube sightings during D](D/native-sightings.jpg)

D preserves [81 native 800 × 600 camera frames](D/native-views.zip) from the sweep, [image timestamps and nearest telemetry](D/native-index.csv), and [the object sightings](D/object-events.csv). The reconstructed cube centers span 0.94 mm along odometry X and 1.28 mm along Y in that one origin. That is repeatability of SDK object localization during this sweep, not independent world-position accuracy. A missing notification on a frame does not prove the cube was invisible.

The robot, range and cube CSV files are compressed with gzip. Cube axes are signed words in nominal units of 8192 counts/g, with three samples sharing each radio receipt timestamp. Empty cube files mark sessions without radio data. Read the robot status word alongside the recorded carry ID when interpreting telemetry. The full-resolution originals and acquisition logs were retained locally.

### Side-lit shelf oval, sessions E and F

[E](E/) adds 6,477 cube XYZ samples and 2,030 robot states. [F](F/) adds 2,232 cube XYZ samples and 3,114 robot states. Both recordings used the physical cube and P2Y8's printed fork. Neither reached a pickup. In E the engine saw the glyph during undocking and lost it before pickup. F connected the cube before undocking. Its second head-angle request never returned, and telemetry subsequently showed wheel motion without a matching drive request from the trial. Exclude this interval from commanded-motion fits. A separate native docking action recovered P2Y8 and charging and active quiet mode were verified.

![Cube accelerometer and optical movement during E](E/tap-overlay.png)

| E tap time after undock (s) | Peak acceleration magnitude within ±1 s (g) | Overhead glyph shift, x and y (pixels) |
|---:|---:|---:|
| 50.98 | 1.08 | −7, −5 |
| 65.45 | 1.22 | 0, 0 |
| 87.24 | 0.98 | −2, 0 |
| 94.00 | 1.14 | −13, −2 |

Each image shift compares median glyph centers in the two seconds before and after notification. The overlay uses three manually centered orientation templates in one fixed camera view. The 5 fps video can reuse a source frame. Small shifts include template centering and pixel uncertainty. The center moves under cube rotation as well as translation. Acceleration magnitude includes gravity and three samples share a packet receipt time. The four events occurred during aiming or dock return and are not independent, commanded push replicates.

[Overhead close-ups around all four events](E/tap-frames.jpg) · [full E video](E/overhead.mp4) · [onboard E survey](E/onboard-survey.jpg) · [raw optical track](E/glyph-track.csv) · [F robot and cube traces](F/traces.png)

### Held-out cube accelerometer and SDK event checks

Replaying E's captured packets through the frozen listeners recovered all four SDK taps. F had zero predicted and zero observed events during radio coverage. **A exposes a discrepancy**. Its packet counter changed six times while the SDK delivered four tap callbacks. Two of the packet changes followed an earlier change by 146 and 130 ms. The data do not establish whether the engine suppressed events or the SDK observer missed them. The eight taps in the earlier pickup recording gave a median packet-to-SDK host receipt delay of 146 ms. Applying that fixed delay to E gave signed errors of +4, +31, −2 and −12 ms. This transfer tests the reporting path with real packets as input. Generating taps directly from simulated contact is a separate test.

After standoff, the quiet head-look windows in E and F gave physical X/Y/Z within-packet scatter of 1.99/2.34/2.80 and 2.03/2.21/2.80 mg. The frozen simulator gave 2.27/2.34/2.37 mg. It underestimated Z scatter by about 15% in both windows. Both observations use the same cube and shelf orientation. A second orientation would help distinguish an axis effect from vibration. The model parameters are unchanged.

[Sensor and event comparison](oval-sensor-check.png) · [Event times](oval-event-check.csv) · [Held-out tap timing residuals](tap-receipt-transfer.csv) · [Reproduce](oval_sensor_check.py)

### Which tap survived in the two close pairs?

The original SDK payload carries the robot's own message timestamp. In A, both pairs of counter changes were **150 ms apart on robot uptime** (10 ms recorder resolution). The two SDK stamps fall **30 and 19 ms before the second change**, versus **120 and 131 ms after the first**. Across all twelve independently paired taps in pickup and E, stamp-minus-radio offsets range from −25 to +35 ms. The ordinary offset range supports matching A's two callbacks to the later changes. The earlier pickup also includes **two separate callbacks for changes 170 ms apart**. The previously measured 146 ms pod-receipt lag therefore must not be used as a tap suppression rule.

The public [stock packet handler](https://github.com/kercre123/victor/blob/snowboy/engine/components/cubes/cubeAccelComponent.cpp) stamps a counter change with the latest robot message time. The [physical engine filter](https://github.com/kercre123/victor/blob/snowboy/engine/components/blockTapFilterComponent.cpp) starts a group at the first input, then releases its last payload on an engine update after the deadline. The source default is 75 ms. Substituting radio arrival times for engine arrival times and applying a punctual 75 ms timer would split both 150 ms pairs. The engine enqueue/update times and effective setting for these trials were not captured. A missing first callback remains possible. Keep the packet-level simulator behavior and published comparison distinct from an SDK event prediction.

[Device-clock overlay](tap-clocks.png) · [Per-change timing](tap-clocks.csv) · [Original SDK tap stamps](sdk-tap-clocks.csv)

### How far did the cube rotate during E?

The first and last contacts rotated the cube in opposite directions. For each tap, the table compares top-glyph heading in two windows 0.8–1.6 seconds on either side. Positive means counterclockwise viewed from above. One method fits and rectifies the four white edges. The other registers the printed glyph texture after correcting local foreshortening. Both assume the cube settled on the same face.

| Tap time (s) | Peak specific force (g) | Outline yaw (°) | Texture yaw (°) |
|---:|---:|---:|---:|
| 50.98 | 1.08 | +14.1 | +18.0 |
| 65.45 | 1.22 | +1.0 | +1.3 |
| 87.24 | 0.98 | −2.3 | −2.3 |
| 94.00 | 1.14 | −10.4 | −11.1 |

At 75, 79 and 83 seconds, texture registration gave absolute changes below 0.1° in three no-tap comparisons. The outline fit alone gave a spurious-looking 5.9° at 83 seconds as the cube LEDs changed. A fourth no-tap window lacked enough usable segmentations. Treat the exact magnitudes of the smaller tap rotations as provisional. These results supply physical contact-replay targets. They do not identify friction or fork geometry by themselves.

[Before/after outline overlays](E/tap-heading.png) · [Angles and fit sensitivity](E/tap-heading.csv) · [Analysis](oval_rotation.py)
