### unit 3 wheelstand series on the shelf

One Vector 2.0 with a printed fork and one retail light cube. In the docked baseline, the cube sent 828 acceleration samples during 10.016 seconds. Its median specific-force magnitude was 0.9703 g and the 5th–95th percentiles were 0.9655–0.9748 g. The tap counter held at 18. The individual samples in stationary.csv.gz share receipt times in groups of three.

**W1 stopped during undocking.** The Bluetooth connection succeeded and the native undock imperative started under continuous override. Both telemetry streams stopped about 8.4 seconds later. The last wheel reports were 33 and 30 mm/s. Robot uptime was 8,264 seconds immediately before telemetry loss and 84 seconds at the subsequent SSH check, establishing a reboot. Its cause remains undetermined. The wheelstand action never started and this attempt contributes zero wheelstand outcomes. There are 145 robot states, 729 individual cube samples and a [See3CAM recording](W1/undock.mp4). The [trace](W1/traces.png) and CSVs use seconds from the undock command.

### Held-out stationary sensor check

The 828 pre-wheelstand samples independently test the existing unit-3 cube sensor settings. With a level, upright cube and sixteen simulator seeds, mean specific-force magnitude is 1.00012 g in the frozen simulator and 0.97027 g on the shelf. Applying the common gain of 0.970709 estimated from the earlier resting.csv gives 0.97083 g, an absolute error of 0.57 mg. The new recording was used only for scoring. This pose gives little leverage for distinguishing a common gain from a Z-axis offset. The alternative gain was kept out of the production model.

The magnitude standard deviation was 2.88 mg on the shelf and 2.35 mg in simulation. Differences between successive samples within each three-sample packet gave physical RMS changes of 2.92, 3.18 and 3.99 mg along X, Y and Z, compared with 3.28, 3.38 and 3.32 mg in simulation. These comparisons use packet order, independent of host receipt jitter. [Overlay](rest-transfer.png) · [per-seed results](rest-transfer.csv) · [reproduction](rest_transfer.py).

**W1 reboot diagnosis.** A subsequent read of the current boot's [PMIC journal](W1/boot-reason.txt) identifies undervoltage lockout at power-off and a cold boot after sudden momentary power loss. This narrows the failure to the power supply dropping below the PMIC threshold during the undock sequence. Robot uptime ties this boot to W1. The wall clock changed across boot. The available journal contains only the current boot and pstore was empty. The readings do not distinguish the cell, its connections, the dock-to-battery transition, or another supply fault. No minimum loaded voltage was recorded.

### W2–W4 physical wheelstand retry

unit 3 was charging at 3.901 V before the tests. The cube stayed in its shelf placement. In W4 the native SDK PopAWheelie command started 22.330 s after the undock command. The overhead view shows the approach and tip-back. SDK chassis pitch peaked at 72.25° at +4.30 s relative to the wheelstand command. The cube's largest recorded specific-force magnitude was 4.50 g across three axes (−1.47, −2.30, +3.58 g). Its tap counter advanced from 19 to 21. W4 contains 2,247 cube samples and 782 robot state messages. Three cube samples share each packet receipt time. The reader guard interrupted the SDK action during tip-back, so the experiment records physical wheelstand contact without an SDK completion result. The raw metrics describe this single contact.

[Contact video](W4/wheelstand.mp4) · [Full overhead recording through redock](W4/overhead.mp4) · [Pitch and accelerometer overlay](W4/contact.png) · [Cube samples](W4/cube.csv.gz) · [Robot state](W4/state.csv.gz)

W2 produced 3,594 cube samples, but the recorder missed the valid zero-valued object ID in a JSON conversion and returned to dock before commanding a wheelstand. W3 returned to dock after two Bluetooth connection failures. For W4 the recorder read the protobuf ID directly and verified fresh accelerometer packets on the dock before undocking. After the maneuver, SDK pitch settled near 1.4° with cliff and pickup flags clear. 

### Held-out wheelstand tap timing

The packet listener maps the two W4 hardware counter changes to two events. With the 145.906 ms median receipt offset from the earlier pickup applied unchanged, the predictions were 10.3 ms late and 12.3 ms early relative to the two SDK tap receipts. These are pod receipt times, and the comparison starts with a counter computed by the physical cube. It checks downstream delivery timing only.

To test upstream observability, the same firmware counter implementation also ran on the retained physical Z readings without access to the transmitted counter. At the model's nominal five FIFO frames per reported sample, two alternative reconstructions filled the four missing frames by holding the previous reading or by linear interpolation. Both reproduced one tap over the whole recording and missed the second. The transmitted samples alone therefore provide a poor reference for reconstructing the high-rate shock waveform with these assumptions. A dynamic simulator can be scored directly against both observed counter changes and the reported acceleration. [Event overlay](W4/tap-transfer.png) · [times](W4/tap-transfer.csv) · [replay](wheelstand_taps.py).

### W4 cube displacement from overhead

The cube moved in two stages. The 5 fps overhead recording resolves a first shift near +4.1 s from the wheelstand command. From +5 to +8 s the template fit measured 10.1 mm displacement and 3° clockwise yaw (median across 11 frames with correlation above 0.7). A second shift appeared between +10.5 and +10.9 s, after robot and cube telemetry ended. From +24 to +32 s the fit measured 13.6 mm total and 5° clockwise (40 frames). Treat the first displacement as the wheelstand-contact target and the second as later movement. Camera sampling and receipt timestamps cannot resolve the movement of the cube at each individual tap.

A four-corner hand measurement on separate picks gave 9.5 mm / 3.7° clockwise after the first shift and 12.5 mm / 5.4° clockwise after the second. Both methods use a nominal 23 mm top glyph and assume it is level at the measured endpoints. They share perspective and scale error. Their differences are a method sensitivity check, not an absolute accuracy bound. [Displacement and yaw](W4/optical-travel.png) · [registration checks](W4/optical-checks.jpg) · [per-frame measurements](W4/optical-travel.csv).

### W4 free-cube replay with the printed fork

The fixed forks7 proxy was driven through the physical SDK approach, heading, lift height and chassis pitch in the existing MuJoCo free-cube fixture. The starting cube XY and yaw came from the sighting immediately before the command. The cube started upright on the shelf. Chassis pitch rotates the fork about the nominal rear axle, with its center held at axle height. Because odometry continues integrating as the treads unload, a second run froze planar translation once pitch exceeded 5°. Both runs hold the final measured pose after +4.836 s while the cube settles. Physics uses four substeps per cube sensor step. No W4 contact outcome was used to tune parameters.

| Input after tip-back begins | Top glyph travel | Yaw (CCW positive) | Reported peak | Taps |
|---|---:|---:|---:|---:|
| Physical first displacement | 9.5–10.1 mm | −3.0 to −3.7° | 4.50 g | 2 |
| SDK XY throughout | 1.57 mm | +2.49° | 1.75 g | 1 |
| Freeze XY at 5° | 0.78 mm | +2.25° | 1.60 g | 1 |

The replay underpredicts travel and shock and reverses the yaw direction in both cases. Measured chassis pitch is an input, so these runs cannot establish whether the model can lift the robot. They isolate fork contact, omit body and lift-arm contacts, and use SDK odometry without external fork-tip registration. The actual printed fork geometry is unconfirmed. These findings motivate checking fork position against the camera at first contact before changing cube friction. Reproduce with the XML generated by evidence/unit3-printed-fork-contact/build_fixture.py from forks7.stl and pass that XML path to [fork_replay.py](W4/fork_replay.py). [Physical and simulated overlay](W4/fork-replay.png) · [metrics](W4/fork-replay.csv).

### W4 fork and shelf force balance

The SDK-XY replay had one uninterrupted interval of loaded fork contacts, +4.052 to +4.160 s. The hardware cube delivered tap-counter changes at +3.983 and +4.274 s. The first lies within the coarse optical-motion bracket around +4 s. Packet receipt and camera time have different delays, and neither reveals the physical contact point.

The simulated contact positions, weighted by normal force over the interval, averaged (+2.8, −6.0, +18.3) mm from the simulated cube center in its body axes. This centroid can lie between separate contacts. The fixture produced **4.01 kN peak downward fork load** and almost the same upward shelf load. Integrated over contact, the opposing vertical impulses were −250.14 and +250.20 N s. The fork's positive yaw impulse (+0.14559 N m s) was almost entirely canceled by the shelf (−0.14559 N m s). These loads are simulator diagnostics. No force sensor measured them on P2Y8.

Prescribing a moving fork against a cube backed by a rigid shelf lets the solver generate huge opposing reactions while the accelerometer reports only modest net acceleration. The resulting travel and yaw cannot identify real friction or available lift force. A finite-force lift with chassis recoil is a higher-priority physics check than a friction refit. [Load and impulse overlay](W4/contact-balance.png) · [summed forces](W4/contact-balance.csv) · [individual contacts](W4/contact-loads.csv.gz).

### Free chassis and torque-limited lift on W4

The full Vector 2 dynamics model starts from measured pre-contact XY and yaw with the lift raised, the cube independently located at its pre-command sighting and the forks7 proxy installed. After a short settling interval, wheels and planar speed initialize from the corresponding encoder observations. The chassis and cube are then free in all six degrees of freedom. The recorded wheel speeds and head/lift positions supply proxy controller targets. Original motor duty and firmware servo goals were not captured. The drive uses tread speed feedback and a fixed 3.92 V. The model holds the last recorded lift position and commands zero wheel speed after the telemetry ends. No W4 outcome was used to fit any parameter.

At the nominal assumed lift torque cap of 30 mN m, the two pre-contact starts (3.50 and 3.80 s) produced 27.2° and 14.8° peak pitch, 13.7 and 16.0 mm settled travel, 2.0° and 2.2° clockwise yaw, 2.84 and 3.38 g, and three and two taps respectively. Peak downward fork loads were about 2 N in both cases. The cube now turns clockwise, as seen in the recording. The chassis tips too little and the cube slides too far. The differing pitch predictions show substantial sensitivity to approach initialization.

At the 3.80 s start, the previously declared low/high torque bounds of 10 and 80 mN m gave 4.3° and 42.6° peak pitch. The 80 mN m case lands close to the first optical travel at 10.1 mm, but still falls short of the physical 72.3° pitch and 4.50 g peak and underpredicts clockwise yaw. This single contact cannot select an actuator limit. The real fork shape, approach velocity and controller demand remain uncertainties. [Measured and simulated traces](W4/free-chassis.png) · [all four evaluations](W4/free-chassis.csv) · [replay script](W4/free_chassis.py).

### Second docked cube recording

With P2Y8 docked and the cube stationary on the same shelf after W4, a fresh Bluetooth connection delivered 828 samples in a 10.014 s window. The tap counter stayed at 23. Mean specific-force magnitude was 0.97036 g, within 0.00009 g of the earlier docked window. Standard deviation was 0.00266 g. The earlier recording gave 0.00288 g. This verifies repeatability of the upright reading across sessions on this cube. The 828 samples are one stationary trial and contribute no additional wheelstand outcomes. Run rest_transfer.py with --repeat for the independent score against the frozen sensor and earlier gain. [Recording](stationary-repeat.csv.gz) · [overlay](rest-transfer-repeat.png) · [metrics](rest-transfer-repeat.csv).

### W4 replay with firmware-derived targets

The public supervisor requests 150 mm/s on both wheels and a 32 mm lift height with limits of 10 rad/s and 200 rad/s². The wheelstand phase ends at 1.2 rad of estimated pitch or after 500 ms. W4 lacks the internal command timestamp. We tested two possible onsets 65 ms apart around the last raised-lift observations and the same two pre-contact initialization times used above. Each candidate starts its lowering profile at the observed lift height at onset, assuming zero initial lift speed. The simulation uses chassis pitch for the cutoff. This approximates the public controller and does not establish the firmware build on P2Y8. [Action source](https://github.com/kercre123/victor/blob/669ec7499a7084640b57706a7937884a6568be50/robot/supervisor/src/pickAndPlaceController.cpp#L606-L614).

At the frozen 30 mN m lift limit, all four simulations reached the timeout. Peak pitch was 16.4–28.9° against 72.25° on the shelf. Cube travel was 15.4–17.8 mm against 9.5–10.1 mm for the physical first shift. Peak cube acceleration was 2.37–2.73 g against 4.50 g. All four reproduced the two tap increments. That agreement coexists with a large error in the underlying motion. These four cases are sensitivity tests on a single physical contact. To discriminate contact geometry from actuator response, the next physical replicate needs a side view resolving fork insertion and lift height at first cube motion. [Overlay including lift height](W4/firmware-drive.png) · [metrics](W4/firmware-drive.csv). Reproduce by adding --firmware to the free_chassis.py invocation.

### Does the overhead camera resolve fork seating in W4?

Five consecutive original See3CAM frames cover +3.603 to +4.661 s relative to the command. The crops preserve the captured pixels and use local luminance contrast enhancement. At +3.879 s the fork is still raised. At +4.143 s the fork overlaps the cube in projection and motion blur obscures its tip. The 264 ms between these frames spans a measured drop in lift height from 90.8 to 39.5 mm and a rise in chassis pitch from approximately 0 to 15° (nearest SDK samples). By the next frame at +4.392 s the chassis has already passed peak pitch. Frame labels are host capture midpoints; SDK and Bluetooth timestamps are host receipt times. This view establishes the order of the approach and tip-back. It cannot distinguish whether the fork seated below the lip or pressed on a corner, or determine insertion depth. [Five original capture times and crops](W4/contact-native-frames.jpg).

### Faster overhead recording for wheelstand repeats

With P2Y8 on the dock, an eight-second shelf-camera test saved 160 distinct full-resolution JPEGs on the camera host (20.1 frames/s). The median gap between receipt times was 48 ms and the longest was 67 ms. W4's two frames bracketing the initial fork descent were 264 ms apart. The new recorder polls the existing snapshot endpoint locally, discards consecutive byte-identical images and saves the original JPEGs plus request/receipt times. Camera exposure times and offsets to the robot and cube clocks remain unmeasured. A faster overhead sequence can narrow the motion interval; the viewing angle still limits visibility beneath the cube lip. Invoke overhead_capture.py on the camera host with a new destination directory, --url for the live JPEG endpoint, and --seconds for the capture duration. [Test manifest](overhead-timing.csv) · [recorder](overhead_capture.py).

### Optical timing cross-check with three cube light pulses

P2Y8 stayed on the dock while the SDK switched the connected cube's lights green and off three times. In the camera-host frames, a crop around the cube supplied the 99th percentile of green minus the larger of red and blue. We used the midpoint between the first off and on reference windows (6.25 levels) and recorded the previous and first changed frame for each edge. The first changed frame arrived 180–463 ms after the command began across the six edges. This measures the entire SDK, wireless light control, light response, image and detection path. It is threshold dependent, and cannot be used as an accelerometer time correction.

Immediately beforehand, 50 paired exchanges bracketed each remote clock against the same Mac clock. Assuming stable relative rates over the short test, the camera clock minus the pod recorder clock lay between −6.52 and +10.91 ms. This limits the contribution from host clock offset. Camera exposure and cube sampling still have their own unknown delays. The physical synchronization signal for a moving trial should be visible cube motion paired with accelerometer response. [Pulse trace](optical-sync.png) · [edge times](optical-sync-edges.csv) · [frame scores](optical-sync-frames.csv) · [clock exchanges](clock-brackets.csv).

### Does the rest-based packet cadence transfer across recordings?

Across six further shelf sessions with 6,672 received acceleration packets, assuming one packet per 30 ms shortened the reconstructed recording by 17.0–17.7%. Using the previously estimated normal interval, pause interval and pause period from resting.tsv brought endpoint errors to +0.18% through +1.60%. Each session starts at the first observed packet with phase zero. No timing parameters were estimated on these sessions. B, C, D and W3 contained no usable packets and have no score.

The largest within-session timing error was 0.92 s. For individual impacts use the actual packet receipt timestamps. Matching overall delivery rate does not establish the physical time of a particular sample or how many measurements the link may have omitted. This checks the timing of data available to the robot on this cube and pod; it does not score grip or impact dynamics. [All six overlays](cadence-holdout.png) · [per-session results](cadence-holdout.csv) · [reproduction](cadence_holdout.py).

### R5 inward transport, operator-observed improper engagement

The SDK sighted the cube at 113 mm and the native pickup raised it. The operator identified the engagement as improper during this run. The overhead view shows the cube clear of the shelf during transport. P2Y8 commanded four 15 mm reverse moves and set the cube down farther inward. The 60 mm is commanded tread travel, not measured cube displacement. Placement returned VISUAL_OBSERVATION_FAILED although the overhead view shows the cube released. P2Y8 returned to his dock.

During pickup the maximum reported cube acceleration was 2.01 g and the tap counter advanced three times. During reverse transport the peak was 1.58 g with zero increments. Set-down reached 4.02 g with two increments. These windows follow SDK action boundaries on the receiving pod. Exact fork seating remains unmeasured. [Carry video](R5/carry.mp4) · [sensor traces](R5/sensor-traces.png) · [cube packets](R5/cube.csv.gz) · [SDK actions](R5/actions.csv).

On retry W5, P2Y8 undocked and searched but never obtained a recent cube glyph sighting. No wheelstand command was issued. [Search actions](W5/actions.csv).

The [four-stage photo and telemetry overlay](R5/rectified-carry.jpg) uses CPCS's feed-annotator rectifier on the original 1920 × 1080 See3CAM frames. It uses the provisional equidistant lens and a 130° horizontal view tipped 10°. Time is seconds from the undock command. Lift comes from the closest SDK report, acceleration from the peak reported magnitude in the preceding 300 ms. No arena-anchor registration was available for this run.
