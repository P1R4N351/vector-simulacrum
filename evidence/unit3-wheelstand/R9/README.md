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
