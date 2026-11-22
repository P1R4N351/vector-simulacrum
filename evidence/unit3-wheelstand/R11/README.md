# Native roll attempt after acquiring the cube at 70 mm

The same robot undocked under continuous override and acquired the cube after turning toward its last observed position. The SDK estimated a 70.3 mm target distance. The native roll command ran for 3.916 s and returned LAST_PICK_AND_PLACE_FAILED. The paired video shows the lift rising while the cube remains on the shelf. Both the image and the sensor outcome support a missed engagement. The 324 accelerometer slots received during the command peaked at 0.992 g and the tap counter stayed at nine. Mean gravity changed by 0.005° between the short pre- and post-command windows, below what should be interpreted as a resolved physical angle.

One later counter change, 23.68 s after the roll command, occurred during docking. There were 5,757 cube acceleration slots and 1,975 robot states in all. The robot completed docking. Telemetry confirmed charging at 3.92 V with quiet mode enabled, active and muted after the experiment. This is a held-out failed engagement and contains no successful landing.

[Corrected twin-camera clip](paired.mp4) · [Cube and robot traces](contact.png) · [Roll result](roll-outcomes.csv) · [Cube samples](cube.csv.gz) · [SDK poses](poses.csv.gz) · [Robot states](state.csv.gz) · [Command log](actions.csv) · [SDK events](events.csv). Camera frames are sampled at the closest original presentation timestamp to each 12 fps preview frame. A fresh cross-host check during the run bounded camera minus telemetry wall-clock offset to −12.60 to +10.90 ms. The clip alignment uses zero offset, within that bracket. Host clock alignment does not remove firmware or BLE delays.

### Held-out replay of the missed engagement

We initialized the cube from the last pre-command SDK observation, leveled it onto the nearest resting face, and replayed the measured chassis and lift motion with both R8 fork models. At solver steps of 0.25 and 0.50 ms, both models produced zero fork–cube and tread–cube contact steps and zero taps. Peak simulated acceleration magnitude was 0.977–0.979 g. The 324 physical slots peaked at 0.992 g. The replay reproduces the visible miss. The physical spread around rest remains larger. This trial tests a single approach given the observed trajectory and supplies no grip-force or slip measurement.

[Accelerometer overlay](miss-replay.png) · [Per-model results](miss-replay.csv) · [Replay script](replay_miss.py)
