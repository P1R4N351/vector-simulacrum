# Cube displaced during the return to dock

This is a physical contact recording with a Vector 2.0 and the shelf cube. The native undock completed. The active cube connected over BLE, but the visual search ended without a recent cube sighting. No roll command was issued. During the autonomous return to dock, both cameras show the turning robot hit the cube. Its position and visible side marking changed. The BLE disconnect had returned REQUEST_PROCESSING; the monitor continued receiving cube data during contact.

The cube tap counter incremented five times between 4.31 and 5.12 s after the dock command. In the first 24 s after that command the measured magnitude ranged from 0.055 to 1.779 g. From 6 to 7 s, the mean gravity vector was within 1.02° of its pre-command direction. This records displacement with nearly the same final up direction. Gravity alone cannot resolve yaw. There were 7,137 accelerometer slots and 2,056 robot states across the full recording. Docking completed.

[Corrected twin-camera clip](paired.mp4) · [Cube and robot traces](contact.png) · [Cube samples](cube.csv.gz) · [SDK poses](poses.csv.gz) · [Robot states](state.csv.gz) · [Command log](actions.csv) · [SDK events](events.csv). The clip uses the original camera presentation times, local clock pairs and nearest frames at 12 fps. Roll and dock command times refer to host receipt. The clip ends 16.83 s after the dock command when camera coverage ends.

### Cube displacement during docking

The two opposed corrected views independently place the settled top-glyph center **39.8 mm (A) and 42.9 mm (B)** from its pre-contact position. This is net displacement between endpoints. We used the nominal 23 mm outer glyph width, the rectified camera intrinsics scaled to the published clip, and a shared level plane for each pair of endpoints. The fits use eight manually identified corner intersections per camera. Coordinate reprojection RMS was 0.24 and 0.36 pixels respectively. The endpoint orientations differ by 25° and 19° modulo 90°; the square outline cannot distinguish quarter turns.

Adding independent Gaussian corner perturbations of 0.75 preview pixels gave 5th–95th percentile displacements of 37.7–42.3 mm and 31.6–47.1 mm. Those ranges describe pick sensitivity only. Common scale, lens calibration and any endpoint tilt remain systematic uncertainties. The shallower, smaller marker in camera B gives weaker depth resolution. The approximately 41 mm slide provides a physical holdout for chassis-to-cube contact even though final gravity changed by only about one degree.

[Endpoint overlays](planar-slide.png) · [Per-view estimates](planar-slide.csv) · [Reproduce and inspect corner picks](planar_slide.py)

### Direction of the collision turn

Eight consecutive camera A frames show the original near face swinging toward image right, with the adjacent face emerging on the left and taking its place. This selects a counterclockwise turn viewed from above. Applying that quarter-turn correspondence to the two settled top-glyph fits gives 64.8° (A) and 70.9° (B). The perturbation ranges, 62.5–66.8° and 68.7–82.3°, exclude shared calibration and endpoint tilt errors and do not agree at their closest edges. Treat the result as an approximate signed turn. In addition to the roughly 41 mm displacement, this gives the simulator a rotational endpoint to reproduce. The face change is visible by +4.67 s, before the last three tap-counter increments. The timestamps label camera frames and packet receipt; taps do not mark equal increments of rotation.

[Frame sequence and accelerometer](collision-turn.png) · [Angular estimates](collision-turn.csv) · [Reproduce](collision_turn.py)

### Where the cube was before contact

At +4.25 s the two views independently locate the cube center 38.7 and 41.4 mm forward of the front axle, and 47.7 and 48.3 mm to the robot's right. Cube edges were oriented −20.9° and −23.9° relative to forward, modulo 90°. The last visible stationary cube pose and both axle centers locate the initial condition without using the eventual cube displacement, tap times or SDK endpoint.

We fitted camera pose using top and side 23 mm markings on a nominal upright 44 mm cube. The picks are from the stationary pre-contact image. In frame 75 the visible hub center rays were intersected at 14.5 mm (front) and 10.75 mm (rear) above the shelf. The visible hub plane was assumed 28 mm off center; true cap extension, tread compression and chassis attitude are sources of systematic error. As an independent scale check the recovered axle spans were 47.3 mm (A) and 45.3 mm (B), versus 48 mm nominal. The CSV includes perturbations of marker and axle picks and a 28–30 mm hub offset. This approximate starting pose is suitable for replay sensitivity tests.

[Hub and glyph picks](hub-registration.png) · [Both camera estimates](hub-registration.csv) · [Reproduce](hub_registration.py)
