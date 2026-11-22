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

### Frozen contact replay

We put each optical registration into the SDK frame at +4.25 s, then replayed the measured robot XY, yaw, pitch, lift and right wheel speed. We used the R8 printed fork, torque-limited arm, front-right tread proxy, chassis collision and shelf support. Starting at +4.25 s already penetrates the fork by 4.36 mm (A) or 0.76 mm (B). The primary replay starts at +3.75 s with the same stationary cube position. First contact now occurs at +4.22 or +4.24 s with less than 0.011 mm penetration.

At a 0.25 ms timestep, net travel was 44.2 and 37.0 mm, bracketing the two physical estimates. Both predictions turn clockwise (60.4° and 61.7°), opposite the physical counterclockwise turn. They produce four and three tap increments versus five observed. At 0.50 ms, camera A changes to 71.4 mm travel and 168.9° clockwise, showing strong numerical sensitivity. This maneuver fails the signed-yaw comparison. The fixture represents only one tread and drives the chassis along SDK odometry. Which surface supplied the incorrect moment remains unresolved.

[Replay curves and physical endpoint bands](dock-replay.png) · [Early-start results](dock-replay.csv) · [Late-start diagnostic](dock-late-start.csv) · [Replay script](dock_replay.py). Run with the R8 fork XML fixture; optional second argument 4.25 reproduces the late start.

### Test of the recorded chassis trajectory

In seven successive camera A frames (+4.25 through +4.75 s), we picked the right-side front and rear hub centers, intersected their rays at nominal axle heights using the pre-contact camera registration, and transferred the front hub to the axle midpoint. In the robot's initial axes, the camera gives 9.3 mm forward and 30.7 mm right. SDK odometry gives 10.3 mm forward and 50.0 mm right. Clockwise chassis heading changes were 63.1° from camera A, 79.7° from odometry, and 74.2° integrating head gyro Z at host receipt times. The gyro is head mounted. The optical path is sensitive to pick error, occlusion, pitch, camera latency and the level-shelf assumption; reconstructed axle length varied from 46.0 to 48.9 mm.

We replaced SDK XY and heading within that interval with the optical path. Before the interval we retained SDK motion, and afterward we continued its relative motion from the corrected endpoint. The recorded lift, wheel speeds, chassis pitch, pre-contact cube registrations, and collision parameters remained fixed. At 0.25 ms, A predicts 52.8 mm travel and 65.1° clockwise, B predicts 39.2 mm and 50.2° clockwise. Both predict six taps. The physical cube turned about 65–71° counterclockwise and advanced its tap counter five times. The measured path discrepancy does not resolve the reversed contact moment in this test.

[Hub picks](chassis-picks.png) · [Optical path, odometry and gyro](chassis-path.png) · [Trajectory samples](chassis-path.csv) · [Replay plot](camera-path-replay.png) · [Per-model results](camera-path-replay.csv). Run chassis_path.py, then dock_replay.py with the fork XML followed by 3.75 --camera-path.

### Which contact turns the cube?

With the optical chassis path and original cube fixture, removing either tread or chassis contact alone reproduced all complete-model endpoints at both timesteps. Removing the fork reduced the 0.25 ms clockwise rotation from 65.1° to 8.0° (A) and 50.2° to 8.4° (B), and delayed first contact by 0.34–0.35 s. Disabling all three left the cube stationary within 0.00002 mm. This isolates the fork as the dominant simulated contributor in the full-contact run. It does not identify the contacting surface on the physical robot.

The legacy replay XML has 62 cube colliders. Replacing these with the current 74-box SDK-derived cube surface while retaining the same robot contacts, drive path and material settings produced different outcomes. At 0.25 ms camera A still gave 63.5° clockwise and 48.0 mm. Camera B now gave 28.2° **counterclockwise** and 10.0 mm. Its result at 0.50 ms was 28.6° counterclockwise and 10.0 mm. The physical band is 64.8–70.9° counterclockwise with 39.8–42.9 mm of displacement. The B sign is repeatable across timesteps, but the cube travels only about one quarter of the observed distance. This comparison makes the fork–cube contact location a higher priority than adjusting uniform friction.

[Contact ablations](contact-ablation.png) · [All 20 comparisons](contact-ablation.csv) · [Current cube surface replay](current-surface.png) · [Current surface results](current-surface.csv). Use dock_replay.py with the fork XML and 3.75 --ablation or 3.75 --camera-path --current-surface.

### Does identifying the upward face resolve the docking impact?

The two side glyphs and camera A's top glyph were scored against the six rendered SDK faces at four quarter turns. Only the 24 right-handed rigid orientations were considered. The four highest joint scores all put the SDK mesh's −Z face upwards. Their mean fit correlations were 0.636, 0.608, 0.595 and 0.581. Camera B's withheld pre-impact top crop preferred +X at 0.465, narrowly above −Y at 0.463. Both post-impact crops favored −Z after rectification, but preferred different quarter turns. The mesh axis is distinct from the accelerometer's axis convention. The observations leave the initial quarter turn unresolved.

![Measured side glyphs and three best individual template matches](face-match.png)

We replayed all four shortlisted orientations plus the previous upright control, under each camera registration, with the measured chassis path, 74 cube boxes, printed fork, and unchanged material parameters. No physical endpoint entered the optical ranking. At a 0.25 ms step, the best jointly scored orientation gave 26.8 mm and 12.4° clockwise from A, versus 31.3 mm and 32.5° counterclockwise from B. Physically the cube traveled 39.8–42.9 mm and rotated 64.8–70.9° counterclockwise. None of the shortlisted orientations met both endpoint bands at either step size. In all four, the first simulated fork contact hit a rubber corner from A and a shell box from B. That distinction is a simulation prediction; the footage does not resolve the contact material.

For a numerical control we reversed the XML declaration order of the same cube colliders. Dimensions, world pose, inertia, trajectory, and friction stayed fixed. The B rank-four prediction shifted from 75.4 to 62.8 mm and from 128.6° to 96.7° clockwise. The B rank-one result shifted by under 0.2 mm and 0.2°. The size of this sensitivity depends on the contact. These data do not support choosing a cube friction coefficient from this impact.

![Endpoint comparison across physical motion, camera registration, initial orientation and collider order](face-endpoints.png)

[All 24 rigid orientations](rigid-faces.csv) · [Individual glyph correlations](face-match.csv) · [Both integration steps](face-replay.csv) · [Reversed collider order](face-replay-reverse.csv)

To reproduce, generate the six corrected projections with the [optical mesh render](../../cube-optical-mesh/README.md). Run `face_match.py /path/to/projections` here, then `dock_replay.py /path/to/fork.xml 3.75 --face-test` with the R8 fixture. Repeat with `--reverse-box-order` appended, then run `face_endpoints.py`. The CSV records the first fork contact position in the simulated cube's local coordinates.

### Does tightening the contact solver stabilize the impact?

We selected the most order-sensitive case from the previous comparison (B, orientation 4). A with the same orientation tests transfer to the opposite camera registration; B with orientation 1 is the control that was previously nearly insensitive to box order. All three reuse the same physical docking impact.

The four runs per integration step cross original/reversed box order with default/tighter MuJoCo Newton solver settings. The default solver uses 100 maximum iterations and tolerance 1e-8, with 50 line-search iterations and tolerance 0.01. The tighter settings use 300 and 1e-12, with 100 and 1e-6 for line search. We test 0.25 and 0.125 ms steps. Integration step changes also change how often the kinematically prescribed robot trajectory is applied. All runs retain the same printed fork, measured arm/chassis path, cube geometry, and material properties.

![Free cube endpoints across integration steps and solver settings](solver-check.png)

[Default, original order](solver-default-original.csv) · [Default, reversed order](solver-default-reverse.csv) · [Tight, original order](solver-tight-original.csv) · [Tight, reversed order](solver-tight-reverse.csv). Run the face replay described above with `--solver-check`, optionally adding `--tight-solver` and/or `--reverse-box-order`; run `solver_plot.py` after all four. Solver iteration statistics record the maximum across islands at each step, summarized over the replay.

With tighter settings, the B4 order gap in final yaw was 0.8° at 0.25 ms, 17.3° at 0.125 ms, and **72.5° at 0.0625 ms**. At the finest step both first contacts fell on the same printed fork piece and the same cube shell patch at 0.4925 s. The two patch coordinates agreed to far below 0.001 mm. Both replays traveled about 61 mm and recorded seven simulated tap events, yet their peak tilts were 93.2° and 44.7°. Thus agreement on contact onset, displacement and taps can conceal a different tipping outcome. The physical estimate was 39.8–42.9 mm and 64.8–70.9° counterclockwise; both fine-step results turned clockwise. No solver run reached its iteration cap (at most nine iterations in all 26 replays).

Across the two main steps and both solver settings/orders, A4 varied by under 1.5° and B1 by under 1°. The default 0.25 ms runs exactly reproduced the previously published endpoint, first patch and tap counts. Tightening the solver failed to establish convergence for B4. Production settings are unchanged.
