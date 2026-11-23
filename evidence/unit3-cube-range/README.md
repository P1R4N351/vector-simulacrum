# Can the shelf replay predict the distance that unit 3 reports?

![Robot and simulated ToF range, including rejected observations](range.png)

The 19-ray production proximity model observes the simulated free cube and shelf in each printed-fork replay. Each replay starts from the camera-derived cube location and overhead-derived yaw and follows the measured chassis and lift trajectory. The chassis is assumed level. The stateful sensor runs at the configured state period with a fixed noise seed per trial. Each SDK row is compared with the most recent simulated sample. There is no time alignment fitted to the range data.

| Trial | Paired SDK rows | Both report a target | Mean absolute range error on those rows | Found flag agreement |
|---|---:|---:|---:|---:|
| 1 | 130 | 125 | 13.54 mm | 100% |
| 2 | 133 | 126 | 6.50 mm | 97.7% |
| 3 | 126 | 121 | 8.73 mm | 100% |
| 4 | 129 | 124 | 11.15 mm | 98.4% |
| 5 | 128 | 123 | 9.59 mm | 98.4% |

The last three runs have 368 paired positive readings. The model reads short by 9.80 mm on average. As a diagnostic, the median physical minus simulated range in the first two runs is +2 mm. Adding that fixed offset on the last three lowers mean absolute error from 9.83 to 7.94 mm, with 31.5% within 5% of measured distance. Adjacent SDK samples are correlated. These are repeat trials in one session and the upper sensor flag agreement alone does not establish cube fidelity. The physical endpoints include cube motion the replay misses, so this does not isolate sensor calibration, initial placement, or material response. No global range bias was applied.

Run score.py with the repository path and the printed-fork fixture XML built as described in [the fork comparison](../unit3-printed-fork-contact/). [Per-sample data](samples.csv)

## Range change before fork contact

![Range versus time during stationary-cube approaches](approach.png)

Each curve starts from that trial's median recorded range before the drive command. This removes initial distance error and tests *change* in range, a narrower question than predicting absolute range. The reference predicts a one-for-one reduction with forward odometry, bounded at the sensor's 30 mm reporting floor. The scored interval is 1–2.75 seconds after the approach command. Across those approaches the maximum observed overhead displacement from the starting position was 0.76 mm or less. Overhead frames are sparse, so this bounds sampled movement only.

We fitted lag alone, travel gain alone, or both on trials 1–2 and froze them for trials 3–5. Mean absolute error over 51 held-out rows was 6.92 mm with the one-for-one model, 5.71 mm with the 71 ms lag alone, 6.16 mm with gain alone, and 5.84 mm with both. Only 19 of 51 lag-corrected values fell within 5% of the measured distance. Different starting offsets and later fork contact cannot explain that residual. The fitted lag is relative to SDK odometry and includes transport and reporting effects.

As a separate check, apparent motion from the widening camera glyph used the previously published scale and image timing fit. At the eleven available held-out moving frame times, the mean optical travel exceeded the lag-aligned decrease in range by 4.33, 3.22 and 2.26 mm in trials 3, 4 and 5 respectively. Odometry gave the same sign in each trial. Oblique faces and partial illumination by the range beam remain plausible contributors. The sample cannot identify either. The diagnostic lag has not been applied globally.

[All held-out errors](approach-scores.csv) · [Frame-by-frame optical cross-check](approach-optical.csv) · [Reproduction script](approach.py)

## Would a closer starting cube explain the apparent fork reach?

The 2 mm offset comes from fitting fork reach to push displacement on trials 2–3. As a separate hypothesis, we moved the cube 2 mm toward the robot along its starting heading and kept the wrist and sensor fixed. We compared the 19-ray model against the first 1.5 s of actual range reports before the drive command. Both placements used identical sensor noise seeds, timestamps and initial cube orientation. All 15 samples per trial reported a target in both replays.

Moving the cube closer reduced the range error in trial 2 alone. On trials 4–5, which were excluded from reach selection, mean absolute range error increased from 3.50 to 5.47 mm. The previous fork reach result therefore has little independent support as a **uniform cube-depth correction** on this shelf. This test cannot establish the true wrist offset: absolute ToF bias, face orientation, the camera pose and printed-fork registration still enter the comparison. The bars in the figure show the middle half of repeated readings at each starting pose; they describe within-trial variability.

![Measured and simulated pre-motion range by trial](initial-depth.png)

[Per-trial range statistics](initial-depth.csv) · [Mechanical comparison of both changes](../unit3-printed-fork-contact/depth-check.csv). Reproduce with `python initial_depth.py ../.. /tmp/unit3-fork.xml` using the registered fixture.

On trials 4–5, extending the wrist gave pre-lift displacement errors of 1.32 and 0.10 mm. Moving the cube closer gave 1.32 and 0.07 mm. Push displacement alone barely distinguishes the hypotheses. The independent range check favors keeping the camera placement on those trials.

## Vector 2 browser replay before cube contact

![Recorded and browser range for each camera-registered approach](browser-range.png)

The browser's Vector 2 free-cube scene starts each replay from the cube center
estimated from the onboard glyph and its heading from the overhead camera. We
move the cube relative to the level chassis according to recorded odometry,
hold its orientation fixed in shelf coordinates, and supply the measured lift
height. The browser casts its 19 rays and samples at its configured 60 ms
state period. Each physical receipt is paired with the latest simulated
report. Neither real range nor an offset fitted to range sets the starting
position. These are registered poses with the stock simulated scene background.
They omit any shelf motion between overhead frames, chassis pitch, and
occluders unique to the recording.

On trials 3–5, the mean absolute range error before driving was **3.22 mm**
(45 readings). From 1 to 2.75 seconds into the approach it reached **9.00 mm**
(51 readings); all 51 browser ranges were at or below the measured distance.
Both sides reported a target throughout those windows. Only 3 of 51 moving
readings were within 5% of measured range. Applying the 71 ms delay previously
estimated on trials 1–2 reduced moving error to **7.88 mm**, with 7 of 51
within 5%. This shift is a diagnostic on the reporting time axis. The remaining
error and trial 2's opposite bias make a global range correction premature.

[Paired readings](browser-range.csv). Run `python browser_range.py` with the
web dependencies installed. No sensor or contact parameters are fitted.

### Isolating camera placement from approach travel

![Range at the moving upper-edge observations](edge-range.png)

A second reconstruction uses the two visible ends of the glyph's 23 mm upper
edge. At each image it solves for positive depths along the two calibrated
camera rays, using the edge direction from the initial overhead heading and
instantaneous SDK chassis yaw. The midpoint, measured head angle, camera mount
and 22 mm face-to-center offset give a fresh robot-relative cube center. Wheel
translation and measured range do not enter this optical position estimate.
An anchored version adds a constant XY translation per trial to match the
earlier four-corner registration at rest. All three alternatives use the same
browser ray fan, lift height and simulated noise draws at each image. Real
range is interpolated at the previously estimated 71 ms range delay; the image
clock uses the previously fitted 236 ms capture alignment.

| Pose input | Mean absolute error, moving trials 3–5 | Mean signed error |
|---|---:|---:|
| Initial four corners plus odometry | 7.30 mm | −7.09 mm |
| Upper edge at each frame, anchored at rest | 5.94 mm | −5.36 mm |
| Upper edge at each frame, absolute position | 3.83 mm | −0.82 mm |

The same 11 held-out images enter each column comparison. Updating travel
alone recovered part of the short reading. Changing the initial registration
as well recovered more. Absolute edge placement also over-read the moving
ranges in trials 1 and 2, with mean errors of +6.09 and +9.06 mm. The
comparison cannot identify a universal placement offset or a sensor bias.
The two endpoints constrain position conditional on the overhead yaw and level
chassis. The images are sparse and the available calibration assumes pinhole
optics. No production coefficients changed.

[Frame scores](edge-range.csv). Reproduce with `python edge_range.py`.

## Docked range timing on the shelf

The shelf scene now samples forward range on the body controller clock. The published controller checks tread power on both sides. On a stationary charger it schedules an acquisition every 200 body ticks (1.024 s). Off the charger at rest it uses 12 ticks (61.44 ms), and driven treads use 6 ticks (30.72 ms). Head and lift power alone leave the slow cadence active. [Range scheduler](https://github.com/kercre123/victor/blob/669ec7499a7084640b57706a7937884a6568be50/robot/syscon/src/opto.cpp#L78-L89) · [Tread power determination](https://github.com/kercre123/victor/blob/669ec7499a7084640b57706a7937884a6568be50/robot/syscon/src/motors.cpp#L145-L150)

In seven recorded sessions, we anchored phase at the second of two docked stationary range/quality changes spaced 0.8–1.2 s apart. Over the following three changes per run, the body-clock prediction gave 19.6 ms mean absolute arrival error, compared with 48.8 ms for a rounded one-second clock. Twenty of 21 observed timing brackets included the body-clock prediction. These are SDK receipt times. Absolute acquisition latency remains unknown. The physical comparison validates the slow cadence. A separate browser regression exercises head and lift power on the simulated dock, followed by right tread power.

[Timing overlay](dock-clock.png) · [All 21 forecasts](dock-clock.csv) · [Reproduce from the recordings](dock_clock.py)

## What changes while Vector rolls off the charging contacts?

Seven recorded departures have consecutive SDK states with nonzero wheel speed while the charger flag is still set. Over those intervals (149–421 ms), the range/quality pair changed 2–7 times per departure. A 1.024 s acquisition interval permits at most one change within each window at the recorded times, regardless of phase. A 30.72 ms driven interval permits all observed counts. This supports switching cadence before the charger flag clears. Batching and unknown acquisition phase limit timing inference from SDK receipt times. Run K's brief loss of the flag with both wheel speeds zero was a different event and is excluded.

Position behaves differently. The supervisor suppresses planar odometry increments while electrically docked to prevent drift when treads slip. In six separate physical pickup sessions A–F, SDK XY held exactly constant through the last continuously docked rolling interval. Integrated mean wheel speed indicated 5.37–7.57 mm of wheel travel over those samples. The first off-contact report displaced XY by only 0.13–1.78 mm. All six stayed in the same pose origin across release. In F the charger flag briefly cleared earlier, so each session is referenced to its first rolling sample within its final on-contact interval. Integrated wheel travel is not an external measurement of chassis displacement. [Supervisor localization](https://github.com/kercre123/victor/blob/669ec7499a7084640b57706a7937884a6568be50/robot/supervisor/src/localization.cpp#L442-L452)

The simulator now excludes on-contact translation from SDK XY and resumes reporting incremental travel after release. A browser regression drives the simulated shelf robot off the dock and checks that the chassis moves, fresh range samples arrive, SDK XY holds, and leaving the contacts preserves the accumulated offset. Ground-truth chassis pose remains available from the physics model. This validates the reporting rule; the physical contact-release distance still needs an independent optical measurement.

[Range overlay](departure-clock.png) · [Range scores](departure-clock.csv) · [Wheel and SDK position overlays](departure-pose.png) · [Pose scores](departure-pose.csv). Reproduce with departure_clock.py and departure_pose.py in this directory.

## How far does the robot visibly travel before the dock releases?

We recovered the native See3CAM captures for pickup sessions A–F. In A and C, one capture timestamp falls between the final SDK on-contact report and the first off-contact report. The two fork-tip patches give **10.25 mm forward in A** (8.02–12.72 mm across projection assumptions and tips) and **8.09 mm in C** (6.22–10.19 mm). In both sessions the reference frame precedes the first continuously rolling docked sample. The other four sessions have no camera timestamp inside the release bracket. Interpolating to the middle of each bracket gives 7.14–10.93 mm across all six. Those estimates depend on motion between frames. In E the reference frame already follows the first rolling report.

We corrected the native crops using the fitted See3CAM division lens and subtracted the apparent translation of two stationary fasteners. Both fork tips are visible and the reported lift height is constant across each accepted segment. A 46 mm upright dock glyph and dock lip locate the camera separately for the observed mount positions. We projected each tip at starting heights of 33 and 43 mm with descent slopes of zero and 0.17. The range across these choices is a sensitivity check. The fork is a chassis proxy: pitch, compliance, initial seating depth and the sensor-to-camera clock offset all affect this comparison. Frame times are host capture-request midpoints; the exposure time and SDK report latency are unknown.

The canonical browser physics, starting docked at 33.4 mm lift and −9.5° head, releases after **14.01–14.28 mm** of forward chassis travel across commands of 25, 40 and 60 mm/s (last on through first off). Its chassis origin drops 2.44–2.45 mm and pitch changes −0.81°. That is longer than either in-bracket optical estimate. These observations constrain total travel from the initial seating to electrical release. They do not isolate pad length from initial pose or establish the correct replacement dimension.

[Optical displacement and simulated release](release-optics.png) · [Tracked landmarks](release-landmarks.png) · [Session scores](release-optics.csv) · [Per-frame scores](release-frames.csv) · [Sim runs](release-replay.csv). Run release_replay.mjs with Node and the web dependencies, then release_optics.py with NumPy, OpenCV and Matplotlib. The timestamped rectified grayscale crops used by the analysis are in release-optics.zip.

## When the printed fork crosses the range beam

![Ray intersections versus lift height and measured ranges on three shelf pushes](fork-beam.png)

In a quarter-millimetre sweep, at least one of the 19 browser rays struck the printed fork between 35 and 49.5 mm of lift height. The three collision boxes gave 34.5 to 50.5 mm. The firmware flags a wider 34–56 mm band. The geometric sweep measures line of sight with the wrist level and counts each ray equally.

During the nine in-band samples on held-out push trials 3–5, unit 3 reported the 30 mm sensor floor three times. At the reported encoder time, the printed fork intercepted **zero rays** for all three floor readings. Using lift height interpolated **71 ms earlier**, those same readings intercepted 19, 11 and 16 rays. Each of the six longer-range readings intercepted at most one ray. The delay was taken from the earlier approach analysis on trials 1–2.

A majority-of-rays rule classified 3/9 floor versus longer readings at zero delay and 9/9 at 71 ms. Including the adjacent SDK sample before and after each in-band sample gives 14/21 and 17/21, respectively. Trials 1–2 together gave 9/15 and 7/15 on that expanded window. The earlier lift height explains a timing pattern on three pushes. It does not yet predict continuous range or transfer across all five trials. A cube close to the fork can also return the minimum distance. The recorded lift is sampled roughly every 100 ms, so the 71 ms alignment depends on interpolation.

Generate the local printed scenes as described in the web README, then run `node fork_beam.mjs > fork-beam.csv` and `python fork_beam.py` from this directory. The physical observations come from samples.csv. No production sensor settings changed.

### Independent dock-motion check

![Archived dock range reports versus lift, and classification across seven runs](dock-beam.png)

Seven other unit 3 recordings (G, J, K, L, M, O and Q) contain lift height, range, signal quality and charger status. We selected SDK rows with dock contact, both reported wheel speeds zero, and lift between 34 and 56 mm. The dedicated docked actuator timing logs omit range. These seven recordings include dock departure or arrival behavior; the object in the beam is unlabelled.

The unchanged majority-of-19-rays rule classified **125/182** minimum-distance versus longer reports at the recorded lift height and **142/182** at the lift height 71 ms earlier. Six recordings improved; J tied. At 71 ms the rule missed 10 of 75 minimum readings and predicted 30 additional minimum readings. Run K (return to dock) was the weakest, with every one of its five minimum readings missed. This is a test of the 30 mm reporting floor only.

In 96/182 rows the reported distance and signal quality had both been constant for at least 250 ms. Restricting the score to the 57 rows where either field actually changed gives **43/57** at zero delay and **49/57** at 71 ms. An unchanged pair can reflect either a repeated report or a fresh equal measurement. SDK arrival time does not identify which. These observations support testing report cadence separately from optical geometry.

[Per-run confusion counts](dock-beam.csv). Run `python dock_beam.py` to recreate the comparison from the archived JSONL and fork-beam.csv.

**Encoder validity changes the interpretation of this result.** The published firmware assigns status bit 0x00400000 to an invalid lift encoder. All 19 selected rows of K and all 28 of J carry that bit. All 135 selected rows from the other five runs clear it. Restricting the comparison to these 135 improves the score from 99/135 at zero delay to 113/135 at 71 ms. K provides evidence about ranging cadence, but its reported wrist height cannot validate fork geometry. [Firmware status definitions](https://github.com/kercre123/victor/blob/669ec7499a7084640b57706a7937884a6568be50/robot/clad/src/clad/types/robotStatusAndActions.clad)

In K the last rapid range/quality change occurred at 78.546 s. The next four changes arrived 997, 1023, 1038 and 1003 ms apart. The published body controller schedules a new front range measurement every 200 body ticks while docked with tread drive inactive, every 12 ticks while off dock with tread drive inactive, or every six ticks with tread drive active. At 5 ms per tick these correspond to 1000, 60 and 30 ms. Tread drive uses motor power, which can differ from measured wheel speed at zero. The one-second sequence agrees with docked slow ranging. The SDK trace lacks the raw acquisition counter and motor power needed to reconstruct every transition. [Body range scheduler](https://github.com/kercre123/victor/blob/669ec7499a7084640b57706a7937884a6568be50/robot/syscon/src/opto.cpp#L76-L100) · [Tread drive determination](https://github.com/kercre123/victor/blob/669ec7499a7084640b57706a7937884a6568be50/robot/syscon/src/motors.cpp#L145-L151)

### The body timer predicts a 1.024 s dock period

![Held-out range change times against the nominal clock and the audio-derived timer](dock-clock.png)

The 5 ms body tick used above was rounded. The public syscon uses a 48 MHz clock and schedules each main loop from an 80-sample microphone frame, with decimation 96 and clock division by 2 × 16. That gives **5.12 ms per body tick**. The front range scheduler's 200, 12 and 6 tick settings therefore correspond to **1024, 61.44 and 30.72 ms**. This derivation concerns the published body timer; engine and SDK reporting have separate clocks. [Timer setup](https://github.com/kercre123/victor/blob/669ec7499a7084640b57706a7937884a6568be50/robot/syscon/src/timer.h#L13-L22) · [Audio frame size](https://github.com/kercre123/victor/blob/669ec7499a7084640b57706a7937884a6568be50/robot/syscon/schema/messages.h#L227) · [System clock](https://github.com/kercre123/victor/blob/669ec7499a7084640b57706a7937884a6568be50/robot/syscon/common/hardware.h#L20)

In each of G, J, K, L, M, O and Q, the first two observed range/quality changes 0.8–1.2 s apart during continuous dock contact with zero wheel speeds established the slow regime and its phase. Each subsequent 3.25 s window contained three pair changes. We predicted all 21 times from the second anchor without further synchronization or fitting. Mean absolute arrival error was **48.8 ms at 1.000 s** and **19.6 ms at 1.024 s**. Each pair change occurred sometime between the preceding SDK row and the first changed row, subject to transport latency. Allowing that interval at both anchor and forecast placed **20/21** changes within the audio-derived clock bounds versus **13/21** at the nominal period. The largest residual at 1.024 s was 78.7 ms. Selection conditions on an already-observed slow interval; entry into that mode, distance accuracy and the two faster periods remain untested by this forecast.

[Per-update timing and bounds](dock-clock.csv). Reproduce with `python dock_clock.py`.

### Can the existing off-dock recordings distinguish 30.72 from 61.44 ms?

![Observed range changes and candidate timing bounds](fast-clock.png)

The native braking CSV preserves the body frame counter but omits the proximity acquisition counter. The firmware increments the latter on a rising ready-status bit and transmits it separately. The archived pickup SDK events expose range and quality. They omit that counter too. [Firmware fields](https://github.com/kercre123/victor/blob/669ec7499a7084640b57706a7937884a6568be50/robot/syscon/src/opto.cpp#L102-L118)

We split seven recordings into disjoint windows of 17 consecutive off-dock SDK reports with both measured wheels stopped or at least one wheel at 30 mm/s or faster. Windows exclude receipt gaps above 200 ms and changes of speed class. For each candidate period we maximized the number of observable changes over all acquisition phases at the host receipt times. An unchanged pair can span multiple acquisitions. Wheel speed is an imperfect proxy for drive power.

During motion, 35/87 windows exceeded the 61.44 ms ceiling; **13/87 also exceeded the 30.72 ms ceiling**. At rest the counts were 52/206 and 20/206. Packet batching is visible on an independent robot clock. In one stationary run G window, 14 range/quality changes arrived in 236 ms while the latest camera frame timestamp advanced 957 ms. In a moving G window, all 16 pairs changed in 634 ms of host time while the camera timestamp also advanced 957 ms. Camera timestamps constrain transport behavior but their ages relative to range acquisitions are unknown. The SDK receipt stream cannot independently establish the fast acquisition period. A discriminating measurement would log the body frame index, proximity sampleCount and tread drive power on the same clock.

[Window-level counts and both clocks](fast-clock.csv). Reproduce with `python fast_clock.py`.

### When does Unit 3 report contact on the way back?

[Camera, SDK and canonical MuJoCo returns](return-optics.png) · [Camera landmarks and simulated wheel profiles](return-landmarks.png) · [Trial scores](return-scores.csv) · [Starting-depth estimates](seating-depth.csv)

Five sessions contain a sustained electrical return (SDK status bit 4096). The first on-report came 55.17, 29.93, 17.94, 30.97 and 52.11 seconds after the dock command in A–E. The preceding off-report was only 34–94 ms earlier. In A the two wheels still read −15/−19 mm/s, and in E they read −18/−22 mm/s. Both read zero in B, C and D. F contains an isolated on-report during departure and ends off the charger. It is excluded from return scoring.

Each plot uses forward-positive mm and aligns on its own first on-contact sample. SDK XY is projected along the reported heading at contact. One origin spans each window. Between the final off-report and the first on-report the SDK pose jumped forward by 8.02, 11.40, 0.18, 6.28 and 7.44 mm (A–E), despite the preceding reverse wheel speeds. The camera trace approaches the dock continuously at its available frame rate. Treat the sampled SDK step as a change in reported pose; the images cannot resolve the motion within a single report interval. The canonical simulated SDK omits a comparable forward step. SDK displacement along that heading changed by at most 0–1.18 mm during the next second. Camera points track the two raised fork tips against fixed fasteners, requiring correlation ≥0.80 and ≤0.11 mm encoder variation. The shaded camera range spans two tip heights (45 and 55 mm), two ramp slopes (0 and 0.17) and the two tips. Each candidate trace is independently zeroed by interpolation at the first on-report. The camera frames bracketing this report are 233–270 ms apart. Their timestamps are camera-request midpoints. Estimated additional backing after the report was 0.16–1.95 mm across sessions. Submillimetre travel is unresolved at this cadence.

The MuJoCo curves use the unmodified Vector 2 shelf XML and browser runtime from bfd532c. Drive forward 55 mm from the canonical docked start, brake, reverse at 25/35/45 mm/s, command zero at the first 5 ms tick with electrical contact, settle for 1 s, then drive forward at 35 mm/s. These are straight final approaches at representative speeds. They omit the recorded search and alignment phases. Simulated chassis travel between first contact and rest was 2.12/3.16/4.29 mm. The first off-contact sample of the following departure occurred after 2.24/3.38/4.68 mm. The canonical preset departure at the previously measured lower lift takes 14.01–14.28 mm. The return replay uses a raised lift for the full cycle, including the outward control leg, which also takes about 14 mm from the preset. Electrical geometry is shared throughout.

**Does the starting pose account for the gap?** The preset starts with its chassis origin 29.9 mm inward from the lip. The simulated release boundary is near 16 mm. Using that boundary with the observed 7.1–10.9 mm departure travel requires starts near 23–27 mm inward. To cross-check independently, the red marks in the landmark sheet locate the lower visible tread edges in a frame roughly one second before departure. We projected nominal front tread sections (axle at body x=0, z=14.5 mm, lateral centers ±23.95 mm) through the calibrated dock camera. Cyan shows the preset and orange the depth required by the measured departure. Both projections end above the observed edges. The nominal image fit gives 14.3–15.4 mm. Scoring each side separately and varying radius by 1 mm, pitch from 0.10 to 0.18 rad, height by 2 mm and picks by 2 px gives 6.9–23.4 mm across the six sessions. These are sensitivity limits for that projection. The profiles also miss laterally, and yaw, tread shadows and camera calibration can bias absolute depth.

A shallower start shortens the simulated departure in the right direction. Neither the default start nor a stop immediately on simulated contact reproduces the recorded departure travel. The fixed-dock silhouette check does not independently confirm the specific depth required to close the gap. This comparison leaves the absolute contact boundary and initial seating unresolved. No pad or start-pose parameter was fitted.

To reproduce, install the canonical web dependencies, run `node return_replay.mjs > return-replay.csv` in this directory, then run `python return_optics.py` with NumPy, OpenCV and Matplotlib. The [rectified native crops](return-optics.zip), [camera values](return-frames.csv) and [5 ms simulation states](return-replay.csv) accompany the original SDK inputs under unit3-physical-repeats.

### What causes the position step at charger acquisition?

[SDK position and yaw relative to the predicted mount pose](mount-target.png) · [scores](mount-target-scores.csv) · [charger observations](mount-target-inputs.csv) · [analysis](mount_target.py)

On a successful reverse mount the supervisor sends a completion message when it detects electrical contact. The engine then sets the robot pose relative to the located charger. Its target is 30 mm inward from the charger origin at a relative yaw of 180°. This operation updates the estimated pose. Manual placement and passive pad contact have a separate code path. [Mount detection](https://github.com/os-vector/wire-os-victor/blob/3cec4844d24c4719b5b3892e4979494bcd260424/robot/supervisor/src/pickAndPlaceController.cpp#L791-L819) · [message handler](https://github.com/os-vector/wire-os-victor/blob/3cec4844d24c4719b5b3892e4979494bcd260424/engine/robotToEngineImplMessaging.cpp#L137-L147) · [engine pose update](https://github.com/os-vector/wire-os-victor/blob/3cec4844d24c4719b5b3892e4979494bcd260424/engine/robot.cpp#L1796-L1814) · [charger transform](https://github.com/os-vector/wire-os-victor/blob/3cec4844d24c4719b5b3892e4979494bcd260424/engine/charger.cpp#L121-L128).

For each return we selected the last CHARGER observation in the robot's origin before contact (7.98–8.82 s old). Composing its recorded quaternion and translation with the published transform predicted all five first on-contact SDK positions within 0.58 mm and headings within 0.53°. No trajectory offsets or headings were fitted. E observed its charger in a different map position, yet the same transform predicted its reported position to within 0.02 mm. This establishes what the recorded position step measures. It supplies no new constraint on the physical pin release position.

The browser now localizes its reported planar pose on acquisition during the backing phase of the native docking action, using the charger's last mapped pose. Subsequent translations follow the corrected heading frame. The physics pose continues evolving from contacts and motor forces. The direct motor return replay above has no mount-completion event and retains its original meaning.

### Why is the final reverse mount so fast in the simulator?

[Wheel-speed and ramp-travel overlay](mount-speed.png) · [five physical return medians](mount-speed-scores.csv) · [all ramp contacts](ramp-backup.csv).

From 1.00 to 0.15 s before the first on-contact report, the five recorded median wheel speeds (averaging left and right at each report) were −32.25, −31.00, −33.25, −32.25 and −34.50 mm/s. The native Vector 2 browser routine's successful diagonal start gave −105 mm/s. The stock supervisor starts both treads at −30 mm/s and allows 7 s to back up. Its cliff-assisted mode can slow an individual side to −10 after the robot pitches onto the charger and crosses a reflectance boundary. [Supervisor constants and command](https://github.com/os-vector/wire-os-victor/blob/3cec4844d24c4719b5b3892e4979494bcd260424/robot/supervisor/src/pickAndPlaceController.cpp#L110-L114) · [mount entry](https://github.com/os-vector/wire-os-victor/blob/3cec4844d24c4719b5b3892e4979494bcd260424/robot/supervisor/src/pickAndPlaceController.cpp#L368-L376) · [alignment logic](https://github.com/os-vector/wire-os-victor/blob/3cec4844d24c4719b5b3892e4979494bcd260424/robot/supervisor/src/pickAndPlaceController.cpp#L821-L867).

**Isolated mount.** Start 78 mm forward of the dock lip facing outward, lift at 47 mm, on the canonical table and dock. At offsets of 0 and 2 mm, both −30 mm/s fixed-guard trials advanced about 43 mm and stalled with the origin 35 mm ahead of the lip. The wheels still reported −30 mm/s. Both rear tipping cylinders touched the apron. Those cylinders are attached to the chassis and carry no belt motion. Lowering their friction to 0.01 still stalled both approaches. Disabling them allowed contact at 3.47 s in both trials. Keeping their geometry and giving each a hinge driven at the rear wheel's peripheral speed also acquired contact at 3.51 s in both trials. These deterministic counterfactuals isolate a contact-model defect on the ramp. They do not validate free tipping or flat-floor skid steering with altered geometry.

**Full native action.** Tested at source revision 27555cd, including find, turn, reverse, verify and retries. Direct starts undock first. Diagonal starts begin at (120, 80) mm and 45° in the table scene.

| Vector 2 candidate | Direct start | Diagonal start |
|---|---|---|
| Canonical controller and geometry | Timeout | Docked |
| −30 mm/s, 7 s, canonical geometry | Timeout | Timeout |
| −30 mm/s, 7 s, driven rear tread surfaces | Docked | Timeout |

The driven direct mount reached a median −30 mm/s in the same precontact window. Its trace and the canonical diagonal trace are from different successful starts. On the driven diagonal run the first backup reached roughly (−118, 8) mm with yaw 0.20 rad against a mapped outward heading of 0.02 rad. Verification and retries followed, and the routine timed out. The first straight ramp obstruction and final alignment are separable failures. The checked-in runtime retains its existing geometry and controller.

To reproduce in an experimental checkout with web dependencies installed, run `node ramp_backup.mjs` here for straight contact traces. Run `node mount_speed.mjs gen2 canonical` for the original full action. Apply [the speed and timeout change](mount-speed-trial.patch) from the repo root, then repeat with `gen2 canonical` and `gen2 driven_rear_envelope`. Each run reports both outcomes on stderr and emits traces for successful runs on stdout. The [experimental geometry](rear_envelope.mjs) is composed in memory. Run `python ramp_score.py` to regenerate the figure from the committed traces.

### Can an angled approach reach both dock pins?

[Recorded yaw, simulated paths and contact outcomes](guide-heading.png) · [trial endpoints](guide-entry-outcomes.csv) · [complete simulation traces including pad residuals](guide-entry.csv) · [physical heading scores](guide-heading.csv).

The rear tread surfaces move at belt speed in all 57 simulated trials. The robot starts 78 mm ahead of the charger lip on the Vector 2 table scene and backs at 30 mm/s for up to 7 s. With the original guides, all six starts at ±3° and lateral offsets −2, 0 and +2 mm miss. Removing the funnels, rails or both, or reducing their sliding friction, still gives six misses per variant. Guide contact does change the trajectory. For example the +3° center start finishes at 59.6° with the guides and 0.04° without them. In the latter case it passes the pins too far sideways. At 100 ms sampling, when both longitudinal and vertical pad residuals are within the canonical limits, the smallest maximum lateral error is 7.10 mm (+3°) or 6.96 mm (−3°), against a 3.8 mm tolerance.

For the next trial we projected the pad midpoint through the starting line: roughly 94 mm of travel to the contact boundary plus 35.5 mm for the pads behind the chassis origin. Applying an initial lateral offset of 129.5 × tan(entry yaw) gives ±6.79 mm at ±3°. Both entries acquire contact with intact guides and with the guides removed. Reusing that rule without refitting, ±1.5° with intact guides succeed at ±3.39 mm and ±5° miss at ±11.33 mm. A centered chassis at the start of a reverse command does not guarantee that the pads will cross the pins.

We also tested the predicted pad line as steering feedback during the full autonomous action, using the optically mapped charger, reported planar pose, driven rear envelope and 30 mm/s base speed. Vector 2 completed the direct return and timed out diagonally, as it had without this feedback. The first-generation model completed both (previously only the diagonal case). The steering experiment does not reproduce the stock cliff-reflectance controller. The canonical model and controller retain their existing settings.

**Physical comparison.** Across five recorded returns, the last SDK heading scored 0.12–0.21 s before the first contact report was −7.56°, +5.87°, +8.21°, −10.36° and +7.07° from the mapped charger heading. These precede the localization jumps in all five sessions. Over roughly the preceding 0.9 s the SDK yaw change and integrated head gyro Z agreed in sign in all five and differed by at most 1.05°. The independent gyro check supports the short-term changes; it cannot establish absolute heading relative to the physical pins. Gyro Z also depends on head pitch. The five observations do not validate a physical ±10° capture range.

Reproduce the grid with `node guide_entry.mjs > guide-entry.csv` in this directory, then run `python guide_heading.py`. To reproduce the steering experiment, apply [the experimental controller patch](guide-steering-trial.patch) to a clean checkout of 15c89b1 and run `node mount_speed.mjs gen2 driven_rear_envelope` and the corresponding `gen1` command.

### Does the observed fork heading support the SDK approach pose?

[Five precontact camera overlays](fork-heading.png) · [angles and sample times](fork-heading.csv) · [analysis script](fork_heading.py) · [contact-aligned travel comparison](return-optics.png)

In each last available overhead frame before sustained contact (73–245 ms before the first on-report), we measured the yaw of two transverse chords on the raised fork relative to the fixed dock. The nominal angles from the inner tips and the lower highlights agree within 1.1°. In A and D both ranges include the preceding SDK heading. In B, C and E the SDK gives +5.9°, +8.2° and +7.1° while the nominal tip estimates are −7.3°, −4.0° and −6.0°. Even the upper ends of the tested tip ranges are only +1.6°, +5.3° and +2.5°. Using the SDK approach angle as a physical constraint on seating depth would introduce an unverified assumption.

The points are brightness centroids near manually selected symmetric features. Both chords share the same fork and camera. Bars show extrema over focal length (450–500 px), feature height (±5 mm), differential feature height (±1 mm), differential image row (±1 px), hand-picked versus fitted glyph corners, and a glyph skew (±1 px per side) with dock lip row shifted ±3 px. These are conditional sensitivity ranges. Unknown fork asymmetry, chassis bank, reflections and exposure timing can still bias both measurements together. The contacts themselves are hidden. Run `python fork_heading.py` alongside the archived crops to reproduce the figure. Rerunning `return_replay.mjs` on b28b3f3 reproduced the published return-replay.csv byte for byte.

### How much travel spans the two images bracketing contact?

[Wheel integration, camera travel and measured approach-speed MuJoCo replays](return-braking.png) · [trial-level distances](return-braking.csv)

We used the last camera frame before the first sustained charger report and the first frame after it, a 233–270 ms interval. Subtracting their forward positions eliminates the interpolated camera origin. Across A–E the camera measured 3.27, 4.65, 0.94, 2.99 and 7.08 mm of backing. Linear interpolation and integration of the two reported wheel speeds on their host receipt clock predicted these distances with 0.48 mm mean absolute error. The largest difference was 1.05 mm in E. A constant time shift selected on the other four returns by minimum mean squared error (±150 ms search) gave 0.53 mm held-out mean absolute error. The right-hand plot marks each held-out prediction. This test provides no support for applying a common timing correction.

We also ran the canonical shelf model at each return's median measured approach speed (31–34.5 mm/s over −1.00 to −0.15 s), commanded a stop on the first contact tick, and sampled chassis travel at the same offsets from its contact. Every replay backed farther than the camera, by 1.91–4.76 mm. Matching the approach speed alone leaves the braking or contact timing unresolved. The wheel integral assumes equal contribution from both tracks, ignores pitch projection and slip, and uses receipt times. The camera tracks fork motion and has uncertain exposure times. These data identify the discrepancy over an interval spanning contact, without isolating a motor delay or an electrical threshold.

Run `python return_braking.py` with the canonical web dependencies installed. The script runs the additional MuJoCo speeds directly. The default three-speed replay still reproduces the original CSV byte for byte.

### Does the onboard charger stop need the external wheel-command delay?

[Wheel-speed reports through contact](contact-stop.png) · [frame intervals, simulated tread travel and chassis travel](contact-stop.csv) · [reproducer](contact_stop.py)

The simulated external wheel command traverses a 75 ms queue. On a stock successful mount the onboard supervisor detects electrical contact and stops the reverse maneuver locally ([WireOS controller](https://github.com/os-vector/wire-os-victor/blob/3cec4844d24c4719b5b3892e4979494bcd260424/robot/supervisor/src/pickAndPlaceController.cpp#L791-L819)). We tested clearing the two queued tread targets on that event. The wheels then decelerate under the same motor, friction and encoder model. This change runs only on contact during autonomous reverse mounting.

For a straight return at each measured approach speed, both simulated encoders first reported zero 70–80 ms after contact with the local stop, versus 150–155 ms with the queued stop. At zero reporting delay simulated tread travel and chassis travel differed by at most 0.19 mm across the camera intervals in either condition. Most of the earlier travel excess was already present at the wheels.

Electrical contact can precede its first recorded SDK report. Each ribbon shifts acquisition across the trial's preceding off-to-on receipt interval (34–94 ms), a sensitivity assumption that omits unknown common transport latency. At the midpoint the mean absolute displacement error fell from 2.82 to 0.69 mm across five trials. Assuming zero reporting delay, it fell from 3.48 to 1.70 mm. Four camera distances fall inside the local-stop sensitivity ranges. In C the range begins 0.29 mm above the nominal camera measurement. The model still takes time to stop, and the true contact phase was unobserved. We used the existing queue length and motor parameters throughout.

Reproduce with `python contact_stop.py` and installed canonical web dependencies. The straight-return fixture isolates the braking change. Full native behavior still depends on successful ramp entry. In the separate direct/diagonal native benchmark both generation 1 cases and the generation 2 diagonal case completed; the generation 2 direct case timed out.

### Can the recorded approaches climb the simulated dock?

[Five complete final backing profiles](recorded-mount.png) · [fifteen replay outcomes](recorded-mount.csv)

For each return we located the final turn with opposing wheel speeds of at least 15 mm/s, then the first three consecutive samples with both wheels below −12 mm/s. Replay begins at the last nearly stationary report (both magnitudes below 3 mm/s) between those two events. The recorded head and lift positions initialize the two joints. Transforming the SDK position and yaw into the last observed charger's frame puts the starts 72–96 mm in front of the lip. These are estimated poses. The known fork-heading disagreement and SDK localization around arrival limit physical interpretation of the absolute tracks.

We held each recorded wheel speed as a command until the next report and let the canonical Vector 2 motor and contact physics evolve the robot independently. Average wheel-speed mean absolute error versus the inputs was 6.3–8.4 mm/s across the two original-profile variants. This replay includes motor response and SDK sampling error. Recorded head and lift motion after the start and the physical robot's sensor-dependent steering decisions were not replayed as feedback.

All five canonical starts stalled with the chassis origin within 0.08 mm of 35 mm in front of the lip at recorded contact time. The driven rear support from the earlier ramp experiment reached contact in B, C and E, 44–150 ms before the first physical on-report. A and D missed. With the same driven support and identical mean speed profile but equal left/right targets, A, C and E reached contact; B and D missed. D missed in every replay. A common contact offset alone cannot account for the different outcomes. These are five replayed approaches from one unit. The candidate still needs independent tests on flat-ground turns and tipping before use in the canonical physics.

In the full native routine the direct case traveled about 49 mm outward during its final half-turn and began backing with the origin about 138 mm ahead of the lip. Its first reverse attempt hit the 1.60 s controller limit. Extending that limit alone to 7 s recovered the direct start after retries but lost the previously successful diagonal start. The evidence therefore points to both starting pose and tread contact before electrical acquisition.

Reproduce the recorded-pose comparison with `python recorded_mount.py` and the canonical web dependencies installed. The zero on each time axis is the first recorded SDK on-contact report.

### Does the driven rear support transfer to flat spins and wheelstands?

[Native gyro overlays](support-transfer.png) · [Wheelstand pitch and cube accelerometer overlays](support-wheelstand.png) · [Per-turn errors](support-transfer.csv) · [Tipping thresholds](support-balance.csv) · [Table-orientation sensitivity](support-sensitivity.csv) · [Wheelstand scores](support-wheelstand.csv) · [Four-support dock returns](support-dock.csv)

We compared the fixed supports in the current Vector 2 model, the rear-only ramp experiment, and the existing four-support option. With no refitting, rear-only supports recovered 3/5 recorded docks (B/C/E); four driven supports recovered 2/5 (B/E). Both missed D. The four-support option also changes the joint gearing implementation and support mass, so this is a comparison of complete candidates.

For flat turns we replayed the published hybrid shaft speeds and 17 ms gyro offset at each recorded starting head angle, using the exported default collision model. Wheel tracking stayed within 0.01 mm/s. On the previously held-out CCW stop the braking gyro RMSE was 9.18°/s for both fixed and rear-driven supports. On the held-out CW stop it rose from 10.48 to 12.52°/s. Repeating both stops at tabletop azimuths of 0°, 15°, 30° and 45° yielded zero rear-support contacts above 1 µN in all 24 runs. The gyro differences coexist with zero measured rear-support load and sensitivity to the numerical contact solution. They provide no estimate of rear tread friction. Rear and side tipping thresholds changed by less than 0.006° across the three configurations in the gravity-release simulation. These angles are an internal regression check.

We reused the two physical cube wheelstands, the 58-piece printed-fork fixture with its published SHA-256, and the existing 60 mN·m experimental lift cap. Each replay spans four combinations of starting time and lift-command onset. These four cases describe uncertainty in one recording. Medians of peak pitch and cube specific force are shown below.

| Configuration | W4 pitch | W8 pitch | W4 cube peak | W8 cube peak |
|---|---:|---:|---:|---:|
| Recorded | 72.25° | 72.50° | 4.50 g | 2.36 g |
| Fixed | 45.02° | 43.48° | 3.36 g | 4.32 g |
| Driven rear | 36.02° | 36.29° | 2.42 g | 2.28 g |
| Four driven | 52.60° | 33.46° | 2.32 g | 3.22 g |

The W8 cube peak is within 0.08 g with the driven rear supports even though chassis peak pitch is short by 36°. Neither driven candidate transfers consistently across both wheelstands and the dock. We retain these as experimental configurations. Wheelstand numbers here use the model and MuJoCo version of this comparison; earlier ablations in the wheelstand archive used their contemporary model.

Run `support_transfer.py`, `support_sensitivity.py`, and `support_dock.py` with Python, MuJoCo 4.0.1, NumPy, OpenCV, Matplotlib, Node and installed canonical web dependencies. Run `support_wheelstand.py /path/to/fixture.xml` for the two dynamic wheelstands ([fixture reproduction](../unit3-wheelstand/reproduction/README.md)).

### Does matching the loaded lift stroke recover the wheelstand?

[Lift, pitch and cube accelerometer overlays](loaded-stroke.png) · [Sixteen replay scores](loaded-stroke.csv) · [Reproducer](loaded_stroke.py)

We aligned W4 and W8 at the last raised-lift SDK sample and replayed each from the two archived starting times, keeping the same fork fixture, tread controller and cube pose. The physical lift descended through 40 mm at an estimated 124 ms (W4) and 129 ms (W8), using linear interpolation between SDK reports. At those crossings the reported chassis pitch was about 15° and 24°. The canonical position profile with the 60 mN·m lift limit reached 40 mm at 359–392 ms and 522–541 ms respectively. Both simulated robots had already pitched about 28° at that lift height. Timing uncertainty between samples limits the precision of the physical crossings.

Sending the interpolated measured height as a target every 5 ms still left 10–28 mm of lift RMS error over the first 250 ms, depending on the starting condition. To isolate the effect of following the stroke we then increased position gain to 12 N·m/rad, damping to 0.035 N·m·s/rad and the torque ceiling to 0.5 N·m at onset. These diagnostic settings were identical in both recordings. They reduced early lift RMS error to 2.0–2.3 mm and put the 40 mm crossing within 6 ms of the nominal physical crossing. The replay obeys the model's lift limits; some recorded heights extend past those limits. The instantaneous torque from the assisted drive is a simulated quantity.

With this same assisted stroke the fixed-support bodies peaked at 34.5–36.9°, while four driven supports tipped them onto their backs at 121.2–121.4°. The recorded peaks were 72.25° and 72.50°, followed by a sustained pitch near 70°. The similar lift trajectories and divergent body trajectories isolate a large contact-dependent residual. Which physical contacts arrest the body at 70° remains open. Matching the recorded stroke alone does not identify the real lift motor torque.

Reproduce with `python loaded_stroke.py /path/to/fixture.xml` using MuJoCo 4.0.1 and the [published fork fixture recipe](../unit3-wheelstand/reproduction/README.md).

### Does centering the recorded backup improve acquisition?

[Approach paths](mount-alignment.png) · [Contact outcomes and entry poses](mount-alignment.csv) · [Reproduction](mount_alignment.py)

Each of the five recorded wheel profiles was replayed from four poses. We kept the recorded longitudinal start, lift, head and wheel inputs and independently set initial lateral offset, yaw, or both to zero. The canonical model missed all 20 returns. With the experimental driven rear support, B, C and E acquired from the recorded poses. All 15 modified starts missed. At the first crossing 35 mm ahead of the lip, the two misses from recorded poses were only 1.55 and 0.23 mm off center, whereas successful B and C were 6.43 and 4.30 mm off center. See the CSV for signed offsets and headings. The paths stop at first electrical acquisition or 0.5 s after the recorded on-report.

These are open-loop counterfactuals. The physical wheel profiles include steering responses to the original poses. Reusing them after centering can introduce a steering error. The result rules out simply centering these replay starts as a remedy. It does not estimate the success rate of a robot steering with live feedback. Run `python mount_alignment.py` from this directory with the canonical web dependencies installed.
