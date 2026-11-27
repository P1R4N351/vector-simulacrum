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

[E](E/) adds 6,477 cube XYZ samples and 2,030 robot states. [F](F/) adds 2,232 cube XYZ samples and 3,114 robot states. Both recordings used the physical cube and unit 3's printed fork. Neither reached a pickup. In E the engine saw the glyph during undocking and lost it before pickup. F connected the cube before undocking. Its second head-angle request never returned, and telemetry subsequently showed wheel motion without a matching drive request from the trial. Exclude this interval from commanded-motion fits. 

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

### Replaying the first shelf contact with the free cube

The frozen Vector 2 scene, driven by E's measured robot XY, yaw, wheel speeds, head angle and lift height, predicted a **9.3° counterclockwise rotation** from the original SDK cube sighting. The video estimates were 14.1° and 18.0°. The simulated packet peak was **1.90 g** against **1.08 g** recorded within one second either side of the tap. The nominal replay produced one packet tap in that window, matching the physical packet count.

All nine initializations (−5, 0, +5 mm along each horizontal axis) began clear of robot penetration and struck the cube first with the right side of the simulated fork. Yaw ranged from 0.3° to 12.2°. Three positions started contact roughly 3.6 seconds before the physical tap. The offsets illustrate sensitivity; their range is not a measured localization error bound. The last SDK cube sighting preceded the tap by 44 seconds. The scene uses the default fork collision geometry, while this robot has a printed lift. The replay prescribes a level chassis following SDK odometry at each physics step, leaves the cube free, and uses one sensor noise seed and sampling phase. Robot odometry, actual contact geometry, and sensor phase remain distinct possible contributors. No friction or impact parameter was adjusted.

[Yaw sweep](E/first-contact-replay.png) · [Physical and simulated accelerometer packets](E/first-contact-force.png) · [Per-pose outcomes](E/first-contact-replay.csv) · [Reproduce](oval_replay.py)

### Does the printed fork explain the first contact?

Replacing the three collision boxes with the registered 58-piece forks7 mesh raised the nominal yaw prediction from 9.31° to 10.40° (camera estimates 14.15° and 18.01°). It lowered peak specific force from 1.90 to 1.68 g (recorded 1.08 g), and changed the packet tap count from one to two (recorded one). Across the nine starting cube positions the printed fork produced 3.86–13.05° of yaw and peaks of 1.34–1.99 g. The mesh had previously improved endpoint error on short pushes. This turning contact tests its transfer to a different motion. It helps two nominal metrics, worsens the third, and leaves substantial error. The exact fork file installed on unit 3 remains unconfirmed.

The comparison retains fork inertia, contact material, prescribed trajectory, cube, and sensor seed and phase. To reproduce, run the default replay, then run it with `--printed-fixture` pointing to the output of [the printed-fork fixture builder](../unit3-printed-fork-contact/build_fixture.py). Only the convex mesh assets are imported into the normal Vector 2 scene.

[Geometry comparison](E/fork-transfer.png) · [Printed-fork accelerometer overlay](E/printed-contact-force.png) · [Results for all placements](E/printed-contact-replay.csv)

### Could the cube's sampling phase explain the peak?

At the nominal SDK cube pose, each fork generated one fixed 1 ms physical trajectory. We passed each trajectory through the existing cube accelerometer, firmware packetizer and tap listener 80 times. Frame phases covered 0–1.5 ms and app tick phases covered 0–9.5 ms, both in 0.5 ms steps, with the same noise seed. Phase zero reproduced both previously published nominal peaks and tap counts. In the two-second physical comparison window the recorder captured 60 packets and a maximum magnitude of 1.078 g.

| Collider | Range of simulated packet peaks | Tap candidates across the 80 phase combinations |
|---|---:|---|
| Three boxes | 1.505–2.301 g | One in 40 combinations, two in 40 |
| Printed fork | 1.489–2.729 g | Two in all 80 |

Sampling phase affects the apparent peak substantially. Even the lowest printed-fork result was 38% above the recorded peak, and the cube's real tap counter incremented once. Host packet receipt had a 280 ms gap starting at 50.950 s; the on-robot monitor had a 260 ms gap between those same two receipts. The recording therefore bounds the delivered peaks, not the continuous physical maximum. Packet loss cannot be inferred from that gap alone. This sweep holds the app and its clock periods fixed; it measures phase sensitivity for this contact only.

[Sampling phase overlay](E/phase-sweep.png) · [All phase combinations](E/phase-sweep.csv). To reproduce, run each version of the first-contact replay with `--export-force` and pass the resulting two NPZ files (boxes first) to `contact_phase.py`.

### Did receipt-time jitter generate the extra simulated tap?

Two successive SDK state messages arrived 108.5 ms and 12.8 ms apart during the first shelf contact. Differentiating their yaw using the latter interval gives about −3.6 rad/s, while the head gyro Z reports about −0.8 rad/s and wheel speeds remain comparable across those samples. Over 149 states spanning 45–54 s, a straight-line fit to arrival time versus sample index gives a 59.99 ms period. Treating those same poses as uniformly sampled reduces the yaw-rate RMS difference from the gyro from 0.76 to 0.28 rad/s during the contact turn and from 0.50 to 0.19 rad/s during the earlier turn. This clock is an estimate from arrivals. The gyro is head-mounted, and its samples are compared by state index.

The printed-fork replay with that cadence retained **both original packet tap candidates at 50.57 and 50.75 s** and added a third at 50.90 s (nominal cube pose, original noise seed and sensor phase). The physical cube reported one counter change. Predicted yaw increased from 10.40° to 11.30° and peak packet magnitude fell from 1.68 to 1.61 g. The physical measurements were 14–18° and 1.08 g recorded. The clock correction removes a spurious angular-speed spike, but that spike alone does not account for the extra tap. No production contact parameters were fitted.

[Chassis gyro, wheel speeds and simulated taps](E/motion-clock.png) · [Cadence-adjusted cube acceleration](E/printed-contact-cadence-force.png) · [All nine starting positions](E/printed-contact-cadence-replay.csv). Reproduce with `oval_replay.py --printed-fixture FIXTURE --cadence`, then `state_timing.py ORIGINAL_PRINTED_FORCE.npz` using the force export from the original receipt-time replay.

### Are the three simulated taps separate impacts?

In the nominal cadence-adjusted printed-fork replay, the same fork hull exerted a positive normal load at every 1 ms step from first touch (50.527 s) through the third app increment (50.899 s). The counter incremented at 50.559, 50.729 and 50.899 s. Over that interval the cube tilted less than 0.13° and its summed floor normal load briefly fell below 1 µN on 43 steps. The simulation produces several vertical acceleration transients during one sustained fork contact. There is no corresponding physical force measurement.

| Physics step (ms) | Cube yaw (°) | Peak packet magnitude (g) | Packet taps |
|---:|---:|---:|---:|
| 1 | 11.30 | 1.61 | 3 |
| 0.5 | 11.37 | 1.84 | 4 |
| 0.25 | 11.22 | 1.62 | 1 |
| 0.125 | 11.18 | 1.66 | 2 |
| 0.0625 | 11.14 | 1.61 | 2 |
| Physical | 14–18 | 1.08 recorded | 1 |

Only the physics integration and prescribed-pose update interval changed. Geometry, material settings, solver time constants, the 1 ms sensor integration interval, firmware clocks and random seed stayed fixed. The first three rows include the original nine cube poses; the finest two use only the nominal pose. Yaw varied by less than 0.24° across the five nominal replays. The coarser steps disagree on tap count, and the one-tap result at 0.25 ms disappears at both finer steps. This limits the conclusions one can draw about real fork impacts from a single simulated counter.

[Contact loads and chip Z](E/contact-load.png) · [Time-step comparison](E/contact-steps.png) · [1 ms load trace](E/contact-load.csv). Generate the trace with `oval_replay.py --cadence --printed-fixture FIXTURE --contact-log E/contact-load.csv`, then run `contact_load.py`. Use `--substeps 2` and `4` for the placement sweep, and `--substeps 8 --nominal-only` and `16 --nominal-only` for the two finest comparisons. Run `contact_steps.py` for the comparison figure.

### Separating rubber-on-shelf contact from fork contact

During the three simulated counter increments (50.527–50.900 s), all solved fork normal load was on the plastic shell and all floor normal load was on rubber corners. We varied only the rubber-to-table contact solver time constant. The explicit 8 ms control reproduced the original nominal peak and tap count at both finer test steps. Friction, fork collision properties and recorded motion stayed fixed.

Increasing the time constant from 8 to 32 ms reduced the packet peak from **1.62–1.66 g to 1.19–1.21 g** across 0.25 and 0.125 ms physics steps. Both steps produced one tap at 32 ms. The physical packet peak was 1.08 g with one counter change. Yaw remained around 11° against the camera's 14–18°. The time constant is a solver parameter; these runs do not measure a rubber material property.

On five separately recorded pushes, trial 1's endpoint error increased from 26.43 to 26.85 mm and its already underestimated tilt fell from 2.29° to 1.36°. Mean endpoint error across trials 2–5 was 1.24 mm at both settings (rounded). These push recordings check motion transfer; they lack matched raw cube acceleration for testing the peak improvement. The 32 ms setting remains experimental.

[Pad–shelf sweep](E/pad-shelf.png) · [Per-setting metrics](E/pad-shelf.csv) · [Five transfer trials](E/pad-transfer.csv). Run `pad_shelf.py FIXTURE` and `pad_transfer.py FIXTURE` using the registered printed-fork fixture.

### Did the cube slide far enough during the first tap?

Registering the outer top glyph before and after contact at 50.98 s gives **8.0–9.5 mm** of planar displacement with projective rectification, or **7.1–7.6 mm** with a local affine map. Both methods assume a level top face and use Anki's nominal 23 mm marker width for the segmented outline. They share the same corner fits; their spread tests sensitivity to projection and edge selection, not absolute calibration accuracy. No-tap comparisons show up to 0.94 mm apparent projective displacement. The last tap, at 94.00 s, moved the glyph another 9.1–9.5 mm by the projective estimate.

At the nominal SDK placement, the printed fork moves the cube **6.52 mm** with the frozen 8 ms pad setting, **6.62 mm** at 32 ms. The softer shelf contact had a much larger effect on packet acceleration than on net travel. Starting the cube 5 mm farther along odometry Y gives 9.34 and 9.02 mm at zero X offset, but both simulations then first touch at 47.21 s, well before the measured tap at 50.98 s. Neither setting brings those runs to the two central optical yaw estimates (14.1° and 18.0°). These are before/after displacement magnitudes, not total path lengths. All simulations in this comparison use 0.25 ms physics steps.

[Rotation versus translation at nine starting positions](E/slide-transfer.png) · [Camera measurements and controls](E/tap-translation.csv) · [Replay endpoints](E/slide-transfer.csv). Run `oval_rotation.py` and then `slide_transfer.py FIXTURE`.

### When did the cube visibly move?

A separate registration of the glyph texture in 31 timestamped overhead captures brackets the **main motion between frame receipts at 50.274 and 50.537 s**. The cube was about 0.47 mm and +1° from its starting pose at the former frame, 2.87 mm and +7.5° at the latter, and 8.20 mm and +19° at 51.057 s. The SDK tap arrived at 50.978 s. These are host receipt times from two acquisition paths. Camera exposure and transport latency limit any inference about the instant of first touch.

There is a much smaller image change between 47.121 and 47.380 s (0.47 mm, from −0.5° to +1°) as the corner LEDs change. Its magnitude is comparable to the earlier no-tap control variation; the footage does not resolve a light brush there. In the +5 mm odometry-Y replay the cube has already moved 2.04 mm and rotated **−2.07°** by 50.274 s. The nominal replay stays still over that interval but has barely begun moving at 50.537 s, by which point the camera sees substantial rotation. Matching the final displacement with the Y offset conceals a discrepancy in the preceding trajectory.

The crop tracker uses a fixed planar rectification, the nominal 23 mm glyph, and a joint position/rotation search on the inner glyph. All 31 texture correlations exceeded 0.93. Angles are quantized at 0.5° and position at one third of a millimeter. These resolutions are not accuracy bounds. The comparison uses the printed fork, 8 ms pad reference, cadence-adjusted robot trajectory, and a 0.25 ms physics step. No timing offset or pose was fitted to these images.

[Original frames and trajectory overlay](E/camera-onset.png) · [Timestamped camera crops](E/onset-crops.zip) · [All nine replay errors](E/onset-errors.csv). Reproduce with `onset_camera.py E/onset-crops.zip`, the cadence-adjusted printed-fork replay with `--substeps 4 --summary-only --camera-trace E/onset-replay.csv`, then `onset_compare.py`.

### Does camera versus SDK delivery timing explain the late replay?

The earlier turn provides a timing reference independent of cube motion. On original camera crops we measured optical-flow displacement in upper and lower robot regions using forward/backward-checked tracks. We compared each interval's fraction of total image motion with the fraction of SDK yaw change over the same interval. Each turn has seven camera intervals. The fit adjusts only a constant offset in the yaw lookup. Head and lift telemetry were stable during the active portions of both turns.

The earlier turn gave +45 ms from lower-body flow and +95 ms from upper-body flow (look up SDK yaw later than the camera receipt). On the held-out cube turn the lower-body estimate improved normalized interval RMS error from 0.041 to 0.037; the upper-body estimate worsened it from 0.052 to 0.069. Refitting each region on that second turn would yield +30 ms for both. The frame spacing is about 260 ms, the lower region had only seven reliable tracks in its sparsest interval, and optical flow contains translation and projection effects. These offsets are sensitivity estimates, not measured camera latencies.

At the 50.537 s frame, applying the transferable +45 ms offset to the nominal printed-fork replay gives 0.49 mm slide and 0.49° yaw versus 2.87 mm and 7.5° in the image. Even +95 ms yields only 1.50 mm and 1.65°. Over ten main-motion frames, +45 ms reduces slide RMS from 1.76 to 1.66 mm and yaw RMS from 7.75° to 7.61°. Receipt alignment accounts for part of the onset error; it leaves the shortfall in settled rotation.

[Robot motion timing overlay](E/chassis-sync.png) · [Calibration and transfer scores](E/chassis-sync.csv) · [Original robot crops](E/chassis-sync-crops.zip). Reproduce with `chassis_sync.py E/chassis-sync-crops.zip` and the nominal onset replay with `--camera-offset-ms 45` or `95`.

### Cube position immediately before the first contact in E

The onboard camera captured the upper-left outer glyph corner at 48.870 s and again at 50.203 s, before the first clear overhead movement at 50.274–50.537 s. The head changed from −12.2° to −0.9° with almost no chassis translation. We matched robot state by the JPEG's native image timestamp and picked the intersection of the outside white edges, extrapolating across the rounded corner. The [overlay](E/precontact/overlay.png) shows the two native frames. The [script](precontact_pose.py) reproduces the projections from the [inputs](E/precontact/inputs.json) and the unit's SDK camera intrinsics.

Conditioning on a level chassis, upright 44 mm cube, centered 23 mm glyph and the +Y-facing side at the SDK yaw, the head-down ray meets the nominal 33.5 mm corner plane at a cube offset of (−12.2, +21.0) mm from the early sighting in SDK world coordinates. Transferred to the raised-head image, this predicts the corner 4.7 px right and 47.7 px low. The raised-head frame by itself gives (−6.3, +15.5) mm. A 1 mm change in relative corner height moves the head-down solution by 5.5 mm horizontally. These two views share an odometry trajectory and a lens and are a posture check within one contact.

As a cross-check, intersecting the horizontal bearings from the 6.837 s and 48.870 s images, conditional on matching the same face corner across the approach, gives (+21.2, −10.7) mm. That places the held-out corner 2.6 px left and 66.8 px high. Matching horizontal position alone allows incompatible depths. The onboard data establish a failure of the nominal projection across head postures, but do not isolate chassis pitch, camera extrinsics, lens distortion, marker plane, corner association or accumulated odometry. Neither correction was adopted for contact fitting. There is one physical first contact in this comparison.

[Spine proximity range and return signal across all six recordings](spine-range/README.md)
