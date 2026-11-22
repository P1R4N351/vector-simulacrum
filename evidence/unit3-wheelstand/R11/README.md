# Native roll attempt after acquiring the cube at 70 mm

The same robot undocked under continuous override and acquired the cube after turning toward its last observed position. The SDK estimated a 70.3 mm target distance. The native roll command ran for 3.916 s and returned LAST_PICK_AND_PLACE_FAILED. The paired video shows the lift rising while the cube remains on the shelf. Both the image and the sensor outcome support a missed engagement. The 324 accelerometer slots received during the command peaked at 0.992 g and the tap counter stayed at nine. Mean gravity changed by 0.005° between the short pre- and post-command windows, below what should be interpreted as a resolved physical angle.

One later counter change, 23.68 s after the roll command, occurred during docking. There were 5,757 cube acceleration slots and 1,975 robot states in all. The robot completed docking. Telemetry confirmed charging at 3.92 V with quiet mode enabled, active and muted after the experiment. This is a held-out failed engagement and contains no successful landing.

[Corrected twin-camera clip](paired.mp4) · [Cube and robot traces](contact.png) · [Roll result](roll-outcomes.csv) · [Cube samples](cube.csv.gz) · [SDK poses](poses.csv.gz) · [Robot states](state.csv.gz) · [Command log](actions.csv) · [SDK events](events.csv). Camera frames are sampled at the closest original presentation timestamp to each 12 fps preview frame. A fresh cross-host check during the run bounded camera minus telemetry wall-clock offset to −12.60 to +10.90 ms. The clip alignment uses zero offset, within that bracket. Host clock alignment does not remove firmware or BLE delays.

### Held-out replay of the missed engagement

We initialized the cube from the last pre-command SDK observation, leveled it onto the nearest resting face, and replayed the measured chassis and lift motion with both R8 fork models. At solver steps of 0.25 and 0.50 ms, both models produced zero fork–cube and tread–cube contact steps and zero taps. Peak simulated acceleration magnitude was 0.977–0.979 g. The 324 physical slots peaked at 0.992 g. The replay reproduces the visible miss. The physical spread around rest remains larger. This trial tests a single approach given the observed trajectory and supplies no grip-force or slip measurement.

[Accelerometer overlay](miss-replay.png) · [Per-model results](miss-replay.csv) · [Replay script](replay_miss.py)

### Cube noise while idle and during the miss

Two eight-second passive windows each contain 217 complete packets. Acceleration magnitude had a standard deviation of 2.71 mg (R10) and 2.67 mg (R11). During the R11 roll miss that rose to 4.70 mg across 108 packets. The strongest change occurred on cube Z: the RMS of adjacent-slot differences divided by √2 rose from 2.81 to 4.42 mg. The post-command window provides a same-session comparison with the cube on the same face.

An upright, motionless virtual sensor with the earlier resting gain and frozen noise parameters gave 2.37–2.50 mg magnitude standard deviation across duration-matched windows. Its mean magnitude was within 0.001 g of each recording. The physical excess during the miss warrants testing for shelf vibration or light contact. These recordings cannot distinguish them. Increasing stationary noise to match this maneuver would overstate variation in both idle recordings.

[Axis comparison](noise-transfer.png) · [Window statistics](noise-transfer.csv) · [Analysis](noise_transfer.py)

### Does wheel motion predict the excess?

Using only R10, we fixed the resting mean and variance from passive windows, then fitted one nonnegative slope relating excess cube acceleration variance to squared wheel speed during search turns. Each quarter-second window needed at least 12 received acceleration slots. Speed uses the faster tread in absolute value, interpolated at packet receipt. The coefficient was 0.0190 mg²/(mm/s)² above a 7.38 mg² resting variance. This is a shelf-specific empirical prediction of signal magnitude.

On the five eligible R11 search-turn windows, RMS prediction error fell from 2.20 to 0.62 mg. Across 14 windows of the R11 missed roll it fell from 2.68 to 2.16 mg. Two lift-motion windows with zero reported wheel speed still measured 9.00 and 7.32 mg against 2.72 mg predicted. The two moving-wheel windows measured 5.29 and 8.36 mg against 6.77 and 7.28 mg predicted. Windows within a maneuver share a robot and shelf and do not constitute independent trials. The simulator's defaults remain frozen pending repeat trials and a second surface.

[Wheel speed and cube acceleration](wheel-vibration.png) · [Per-window measurements and predictions](wheel-vibration.csv) · [Reproduce](wheel_vibration.py)
