# Hold-out roll approach with no measurable cube movement

A native undock and cube connection succeeded. The robot saw the cube at 79.2 mm and issued a single native roll. The SDK returned VISUAL_OBSERVATION_FAILED after 8.58 s. The recorder kept the cube connected through eight seconds of passive observation after motor stop. The robot completed native docking. Charger and quiet-mode status were verified after release.

From command through the end of passive observation, 1,569 acceleration samples arrived in 523 packets. The tap counter stayed at 3, with zero changes and zero SDK tap or movement reports. The cube remained +Y up. Mean gravity changed by 0.011° between the one-second pre-command window and the final two seconds of observation. That value describes agreement between sample means; true pose precision is lower. Accelerometry alone cannot exclude slow level sliding or yaw.

The rolling-tread replay from the preceding successful physical roll was frozen before this experiment. We supplied the new chassis and lift trace, observed horizontal cube center, and pre-command SDK cube quaternion corrected to the nearest flat resting face. Since the cube started on another face, orientation is now an explicit replay input. The physical accelerometer and SDK cube pose use different conventions; this comparison scores acceleration magnitude, tilt from the starting vertical and tap changes. No coefficients were fitted. With a 0.25 ms step the simulated tread never contacted the cube. Predicted peak tilt was less than 0.001°, with zero tap-counter changes. Peak packet magnitude after 0.5 s was 1.007 g simulated and 0.982 g measured. The run tests rejection of a false disturbance. It adds no positive flip or impact benchmark.

[Overhead clip](approach.mp4) · [Physical traces](post-stop.png) · [Held-out replay](holdout.png)

Run holdout.py with the printed fixture XML described in ../R8. Run poststop_metrics.py with this directory as its argument. Times in actions, state, cube and events are relative to the undock command; roll-poses uses seconds after the roll command.

### Sensor frame and gravity gain on the sideways face

The earlier +Z-rest capture yields a common gain of 0.970709 using 1,929 samples. Applied without refitting, it recovers the +Y-up gravity magnitude to within 0.29 mg after the roll and 0.97 mg before the next approach. Applying the upright correction to Z alone leaves errors of 29.57 and 30.23 mg. The two sideways windows share the same physical pose. The X gain and independent offsets remain unconstrained.

In the pre-command sideways sighting the nominal SDK-cube-to-sensor transform predicts gravity 175.8° from the raw observation. Rotating the chip frame 180° about cube Z reduces this to 5.0°. This is a candidate registration for this cube when its initial pose comes from the SDK. The two recordings and the preceding settled window use the same cube. They do not provide independent calibration for other cubes, for a carried cube whose face registration is unknown, or for all six faces.

Applying the registration to the preceding rolling-tread replay gives +Y as the final filtered up-axis, matching the later physical rest face. The real cube still reported +Z during the simultaneous captured interval, so the simulated change is early. The vector RMS error over the common dynamic packet window falls from 0.775 g to 0.588 g; adding the earlier gain reduces it to 0.580 g. The adjusted simulation still produces four tap-counter changes versus two measured and peaks at 5.32 g versus 1.43 g. This distinguishes an axis registration error from the remaining contact dynamics and timing errors.

[Sensor comparison](frame-gain.png) · [Resting gain](side-gravity.csv) · [SDK pose registration](frame-sign.csv) · [Dynamic transfer](dynamic-frame.csv) · [Reproduce with printed fixture](frame_gain.py)

### Independent check of cube placement

The camera saw the dock and cube in both sessions. We fit camera pose to the dock glyph and lip using the existing lens correction, then added the nominal 23 mm square on the top of the first cube as a local scale constraint. The second cube view was held out. Its reconstructed glyph measures 22.0 by 24.8 mm; with the dock alone it measures 21.4 by 28.5 mm. The remaining depth error limits absolute registration.

The fitted camera sees 13.8 mm forward and 8.8 mm left between the two cube centers. Subtracting the two SDK sightings gives 15.1 and 8.4 mm respectively, assuming the dock-based odometry origins coincide. The apparent two-axis discrepancy is 1.4 mm. The first view's optical grid differs from SDK yaw by 7.4° modulo 90°; the second differs by −1.2°. Bars on the plot show 5th–95th percentiles under random ±1 px landmark and ±1 mm height variations, not overall measurement confidence. The cube moved between runs; this agreement in displacement cannot identify a shared absolute translation error or establish where the fork touched.

[Landmark picks](registration-picks.png) · [Camera versus SDK](optical-registration.png) · [Calibrations](optical-registration.csv) · [Reproduce](optical_registration.py)

### Transfer the starting pose into the articulated-lift replay

In the physical roll (R8), the camera's +7.41° yaw correction gives a simulated peak of 1.57 g and two packet counter changes, compared with 1.43 g and two measured. With SDK yaw, the same servo fixture gives 1.87 g and three changes. Across the optical yaw sensitivity range the highest simulated tilt remains below 0.2°. No tested yaw produces the physical quarter-turn. All held-out R9 cases predict no taps and no tread contact, as observed in the accelerometer record.

The SDK cube center also appears 6–7 mm below its nominal resting height relative to chassis odometry. Reprojecting its reported center along the head-camera ray to a 22 mm shelf height shifts it 29.8 mm closer in R8 and 22.5 mm closer in R9. The resulting proximity predictions (33.6 and 30.0 mm) disagree with the independently recorded 61 and 51 mm. The check uses the firmware camera and range sensor positions, measured head angle and pitch, a flat 44 mm cube envelope and the model's 19-ray fan and signal weights; the shelf and surrounding geometry are excluded.

Alternatively, retaining lateral position and using the optical yaw, a fore–aft shift of 4.51 mm toward the robot in R8 and 6.58 mm away in R9 matches each reported range by construction. This produces four simulated tap changes and 1.98 g in R8, still without a flip. Between-run displacement then differs from the overhead estimate by 10.9 mm versus 1.4 mm for SDK XY, assuming coincident dock odometry origins. A single range observation cannot independently establish cube center. These results give no basis to adopt a common depth shift.

[Yaw and range replay](yaw-transfer.png) · [Ten replay results](yaw-transfer.csv) · [Range observations](pre-roll-ranging.csv) · [Range model](range-check.csv). Run range_check.py first and yaw_transfer.py with the printed fixture path to regenerate.
