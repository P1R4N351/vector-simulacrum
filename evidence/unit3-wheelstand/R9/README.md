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

### Lift torque and chassis support during the roll

The recorded pitch, applied about the original fixed front pivot, places the rear tread up to 24.2 mm below shelf level in R8 and 0.4 mm in R9. A paired replay moves the chassis vertically until the lower of the two nominal wheel envelopes touches the shelf. It preserves measured XY and attitude and assumes at least one tread remains supported. Chassis height was not independently measured.

At the nominal 30 mN·m limit this reduces peak lift tracking error (expressed as wrist height in chassis coordinates) from 19.4 to 6.1 mm. Simulated cube tilt increases from 0.16° to 0.47°. The two simulated taps persist and peak acceleration rises from 1.57 to 1.71 g, farther from the observed 1.43 g. The recorded cube rolled 91.6°, but chassis telemetry ends before it settles. Held-out R9 still gives zero taps. Raising the torque ceiling alone from 10 to 80 mN·m with the original pivot leaves peak tilt at 0.16°. Actual lift torque was not recorded.

The lower panel compares the quasi-static support required to balance gravity and robot contact with the cube footprint. Crossing its edge briefly does not establish a tip. Force and actuator summaries use +3.00 to +5.21 seconds, clipped to available replay time. Run lift_leverage.py with the printed fixture XML to reproduce.

[Contact force and lift tracking](lift-leverage.png) · [Eight replay results](lift-leverage.csv)

### The roll discrepancy is visible before chassis telemetry ends

The successful roll's cube packets show median Y–Z acceleration direction changes of 20.4° from +4.60 to +5.00 s and 28.8° from +5.00 to +5.208 s, relative to the pre-command rest face. The nominal shelf-supported replay gives 0.02° in each window. Median physical packet norms are 0.966 and 0.962 g (rest about 0.971 g). An accelerometer responds to both orientation and translation, so these directions measure a sensor discrepancy directly but do not independently measure geometric tilt. Receiver timestamps include Bluetooth delivery jitter. The held-out missed approach stays within 0.06° of its initial direction in both windows, measured and simulated.

The last three physical packets after the R8 chassis cutoff have median direction change 41.3°. The overhead view reaches its final appearance at approximately +5.46–5.66 s. Extending the replay might capture the endpoint, but the earlier packet discrepancy already exists while measured drive inputs are available. The discrete up-face indicator masks this gradual change.

[Packet direction overlay](roll-onset.png) · [Window medians and counts](roll-onset.csv) · [Reproduce with printed fixture](roll_onset.py)

### Does the position servo lag account for the missing roll?

A second replay adds the encoder trajectory's angular velocity times the existing damping-to-stiffness ratio (0.020 s) to the servo target. This compensates the actuator's velocity-dependent force when the simulated lift follows the reference. Both variants have the same 30 mN·m limit, joint travel, shelf support, initial pose and contact geometry. The interpolated encoder trajectory is known offline. The real motor target was not logged.

At the encoder sample times from +4.00 to +5.208 s within replay coverage, successful-roll height RMS error drops from 3.43 to 3.42 mm. Peak cube tilt drops from 0.47° to 0.16°. Peak acceleration is 1.71 g and two taps in both cases. The held-out missed approach stays at 1.41 mm RMS error and zero taps. The real encoder drops below the 32 mm model travel limit in both recordings. Compensating nominal servo damping explains little of the loaded tracking error and does not recover the physical roll. The default replay still uses the original servo.

[Encoder tracking overlay](lift-tracking.png) · [Four replay scores](lift-tracking.csv) · [Reproduce with fixture XML](lift_tracking.py)

### Below the 32 mm low-dock target

The public firmware clamps height-to-angle commands at low-dock and carry heights. Its reverse conversion from measured angle to reported height has no such clamp. Calibration resets the current angle to the low limit. The five-degree calibration margin is a fault-detection margin and does not specify mechanical travel. The reference source does not establish this unit's physical fork height. [Reference geometry and conversions](https://github.com/kercre123/victor/blob/669ec7499a7084640b57706a7937884a6568be50/robot/include/anki/cozmo/shared/cozmoConfig.h#L133-L146) · [Calibration reset](https://github.com/kercre123/victor/blob/669ec7499a7084640b57706a7937884a6568be50/robot/supervisor/src/liftController.cpp#L306-L314).

The minimum reported height in the independent unloaded unit 3 docked recording was 30.645 mm. Two diagnostic replays transfer this number into both roll trials. In the travel case the model's lower joint and replay target limits both move to that height. In the encoder-zero case the nominal travel is preserved and a constant 1.202° is added to the measured angle to obtain the physical target; the same offset is subtracted before scoring simulated encoder height. Both use the existing servo and shelf support.

The failed approach spends much of the comparison at its lower lift endpoint. Either hypothesis reduces its encoder RMS error from 1.41 to 0.40 mm. During the successful roll, the travel case gives 3.41 mm versus 3.43 mm nominal; the zero offset gives 2.41 mm but decreases peak cube tilt from 0.47° to 0.14°. All three predict two taps and 1.71 g. Neither correction explains the roll, and encoder agreement alone cannot select the physical geometry. No model default changed. Scores use encoder samples from +4.00 to +5.208 s within the available replay, as in the preceding comparison.

[Lift encoder overlay](low-stop.png) · [Six replays](low-stop.csv) · [Reproduction script](low_stop.py)

### Which omitted surfaces can reach the cube?

We sampled the free cube and robot from the shelf-supported replay every 20 ms between +4.00 and +5.208 s and measured signed distances to the detailed cube collision geometry. The coarse chassis box overlaps it by up to 5.36 mm at +4.20 s. The smallest positive gaps in R8 were 2.84 mm for the inset wheels, 3.77 mm for the head, 10.40 mm for the other tread supports, 13.30 mm for the lower lift links and 25.53 mm for the upper links. All six groups stayed at least 55 mm clear on held-out R9. These are distances on the simulated trajectory. A positive sampled gap is not proof of physical clearance once the real and simulated cube poses diverge.

We then enabled chassis-box contact from the start of each replay, leaving the free cube, articulated fork, rolling tread and measured chassis trajectory intact. The box first touches at +3.348 s in R8 with 0.783 s of total contact. Maximum cube tilt decreases from 0.47° to 0.13°, and the packet tap count increases from two to three versus two observed. R9 still predicts no chassis contact or taps. The overlap identifies a missing interaction in the reduced fixture, but adding this rectangular body does not recover the physical roll. The chassis is driven by the measured pose throughout; the box shape and reaction to contact are not independently validated. The historical driver remains the default.

[Part clearances](late-clearance.png) · [Sampled distances](late-clearance.csv) · [Paired chassis contact overlay](chassis-contact.png) · [Replay metrics](chassis-contact.csv). Run late_clearance.py and chassis_contact.py with the printed fixture XML path.

### Does the overlap survive the curved shell geometry?

On the frozen fork-and-tread trajectory, we checked the viewer's photo-derived front apron, SDK body and two fixed track bases against all 62 cube boxes. Each 20 ms sample transforms the chassis triangles into the cube frame. A separating-axis test checks each candidate triangle against the box. The shell export uses the viewer's front axle registration and centimetre scale. The check covers +3.00 to +5.208 s within recording coverage. It uses surface triangles and does not generate contact forces.

In R8 the apron intersects at every sampled time from +3.28 through +4.46 s (60 samples). The SDK body intersects from +3.38 through +4.28 s (46 samples). Neither fixed track base intersects. On this trajectory, the interference therefore survives replacing the chassis box with curved surfaces. It ends before the +4.60 to +5.208 s late sensor comparison. Physical positions may diverge as soon as the first contact is missed. All four tested surfaces stayed clear on held-out R9. Shading on the region map marks entire triangles touched at any sampled instant, so it overstates the area of contact; triangle counts depend on tessellation.

[Contact region in top and side views](shell-regions.png) · [Timing](shell-overlap.png) · [Per-part samples](shell-overlap.csv). Run shell_overlap.py with the printed fixture XML and the viewer's locally installed SDK geometry.

### Dynamic contact with the curved front apron

The registered viewer apron was decomposed into three convex collision pieces (CoACD threshold 0.025, seed 0). The paired run enables these pieces on the prescribed chassis with the existing chassis contact material. The other two variants use the original fork-and-tread setup or add the chassis box. Cube pose, lift torque, motor trajectory, shelf support and tread input are identical across variants. The apron-only variant isolates that surface; it omits the rest of the SDK shell.

Apron contact starts at +3.271 s and lasts a cumulative 0.654 s during R8. The cube tilts at most 0.126° and the packet tap counter changes three times. The first contact agrees to one 20 ms sample with the earlier triangle-intersection check. Earlier geometric overlap did not imply that the apron would supply sufficient leverage. Compared with the two physical taps and measured 1.43 g peak, the apron replay gives an extra tap and 1.81 g. All held-out R9 variants stay at zero taps and 0.98 g. These results do not support adopting the apron alone as a roll correction. Full shell shape, fork purchase and the reaction of an independently moving chassis remain open.

[Cube tilt and Y acceleration](apron-contact.png) · [Six replay results](apron-contact.csv) · [Reproduce with fixture XML and CoACD](apron_contact.py)

### Resting output through the normal free-cube simulator

The unit 3 shelf-cube profile now uses the previously fitted effective gain of 0.970709 on Y and Z. X remains nominal because neither recorded gravity direction constrains its sensitivity. The fit used 1,929 upright samples. Four other physical windows cover two positive-gravity faces. The post-roll window and the next approach share one resting orientation, so they do not constitute independent orientations.

We placed a free cube upright or sideways in VectorSim, let it settle and decoded three seconds of its normal packets with the standard noise and word format. Across the four windows the absolute error in the magnitude of the mean acceleration vector drops from 29.66–30.32 mg to 0.37–1.03 mg. The generic Vector 1 profile retains nominal gain. This transfer validates resting magnitude for this cube; negative faces, X gain and impact response still need separate measurements. A bias could account for part of the apparent gain.

[Free-cube output comparison](free-sensor.png) · [Window errors](free-sensor.csv) · [Simulator entry-point check](free_sensor.py)

### When does the receiver stop seeing cube packets?

Long intervals between incoming packets recur regularly in the robot-uptime trace. Using only the original rest recording, we fit 1.22746 s between returning packets (linear fit across 18 gaps) and a median gap width of 0.260 s. In each of five separate sessions we used the first return to establish phase and predicted subsequent gaps without refitting. A gap is an interarrival interval over 150 ms. Scoring excludes the phase-setting gap and uses complete return times within the trace.

The schedule predicted the count of subsequent returns in every session (179 overall). Per-session median absolute return error ranged from 6 to 25 ms; maximum error reached 113 ms. Time overlap between observed and predicted gap intervals, divided by their union, ranged from 0.832 to 0.948. In R8, on the command-relative host receipt clock, no new packet arrived between +3.128 and +3.372 s or between +4.327 and +4.599 s. The latter interval ends just before the previously scored late acceleration rise. Interpolating the data across it cannot locate motion onset within the gap.

The simulation emits one packet every 30 ms at its firmware interface. This result describes downstream availability to the recorder, and does not establish whether frames were lost, deferred or never generated. Keep raw simulation and actual observations on separate clocks; use measured arrival times where available. The interval predictor is useful for sensitivity checks where only an initial observed phase is available.

[Held-out timing overlays](receipt-gaps.png) · [Interval scores](receipt-gaps.csv) · [Reproduce](receipt_gaps.py)

### Transfer of the revised cube collision boxes

We swapped only the free cube's box positions and sizes from the revised SDK surface into the printed-fork replay. Both variants use the same recorded chassis motion, articulated lift, rolling tread, shelf support, material settings and 0.25 ms step. The chassis box is disabled. Scores use the paired replay from the same checkout. The sensor peak window starts at +2.5 s and ends at the last chassis sample.

| Recording | Quantity | Physical | Original boxes | Revised boxes |
|---|---|---:|---:|---:|
| R8 roll | Peak acceleration (g) | 1.433 | 1.706 | 1.570 |
| R8 roll | Tap changes | 2 | 2 | 3 |
| R8 roll | Change of face | Yes, adjacent face in later rest capture | No | No |
| R9 missed approach | Peak acceleration (g) | 0.982 | 0.978 | 0.978 |
| R9 missed approach | Tap changes | 0 | 0 | 0 |

The revision roughly halves the R8 peak error. It also adds a spurious tap and the cube tilts only 0.25° during the replay. The missed-approach result transfers. This experiment leaves the cause of the missing roll open.

[Acceleration and tilt overlays](surface-transfer.png) · [Replay metrics](surface-transfer.csv) · [Reproduction script](surface_transfer.py). Pass the printed-fork fixture XML (SHA-256 `79641b7fe78c9ef47293f069002474be7d1e004d16f4acf6300fa90b14b321d9`).
