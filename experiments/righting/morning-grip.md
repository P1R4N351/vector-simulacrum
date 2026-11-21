# Sideways fork retention and rescuer leverage

## Start with the grip already engaged

For the leverage-only simulation, the cube is rigidly mounted at the patient wrist's carry transform. The patient's head, lift and chassis remain articulated, its wheel motors receive zero drive, and its original mass and the cube inertia both load the rescuer. After one second of settling, position the rescuer above the adjacent face and run the existing low-roll approximation. Per-side placement retreats in 1 mm increments if needed to avoid an initial intersection. The unloaded control starts from the same cube presentation with the patient moved clear; both settle under gravity independently. All remaining contact with the rescuer and floor is free. A rigid mount supplies an ideal patient grip for this diagnostic.

From the repository root, with the dependencies in the [replay guide](README.md):

```sh
python experiments/righting/run.py /tmp/mounted --cube-obj "$VECTOR_SDK_ASSETS/cube.obj" --predocked --fixed-grip
python experiments/righting/run.py /tmp/unloaded --cube-obj "$VECTOR_SDK_ASSETS/cube.obj" --predocked --presentation carry-pose --cube-control
node experiments/righting/render.cjs /tmp/mounted
```

| Patient side | Rescuer fork contact, s | Cube net rotation, unloaded | Cube net rotation, attached | Closest patient tilt to upright after contact | Final patient tilt |
|---|---:|---:|---:|---:|---:|
| Right | 1.020 | 90.04° | 1.86° | 89.86° | 90.03° |
| Left | 1.026 | 90.04° | 1.08° | 89.85° | 90.04° |

Tilt is 0° upright and 90° on the side. Cube rotation is the shortest three-dimensional rotation from its settled starting orientation. The rescuer reached the reverse phase on both sides. The nominal 50 g cube and the fork collision shapes are assumptions. These results describe this approach, contact point, controller and surface. They do not establish a hardware torque limit.

## Close-up hardware capture

Use the same patient, fork, cube, battery and surface on both sides. Record the robot identifier and whether its fork is stock or printed. Keep the rescuer out of these shots to isolate the patient's grip.

1. Frame the fork tip, cube lip, one fixed chassis landmark and the table in a single close-up. Set a millimetre reference alongside the lip in the same plane. Check focus on the seam before the pull. Use diffuse side lighting so fingers and highlights do not hide first slip. Preserve original frames.
2. Start recording. Lay the patient on its right side, offer a marked cube face, engage the fork, then remove the offering hand. Hold still for two seconds. If the cube releases under its own weight, record that outcome. Establish a fresh engagement for the next take.
3. Pull gently from the adjacent face in the direction of the rescuer's back-out. Keep the hand clear of the fork. Leave the chassis free to roll and record continuously through the first slip, release or chassis motion. Record the final pose for two seconds. Mirror the geometry with the patient on its left side.
4. Record three independent reseatings per side if engagement is repeatable. Log side, cube faces, fork position, pull direction and take number. A visible force gauge, if available, adds force at first slip. With ordinary video, report displacement and release angle only.
5. Annotate engagement, hand-off, start of pull, first fork–lip slip, first chassis roll and release by frame timestamp. Track the fork tip and cube corner relative to the fixed chassis landmark. An obscured seam or a release between frames defines an interval. Use the first two right-side takes to refine the simulated contact position and lip clearance; check the remaining right take and the mirrored left takes without another fit. Keep the rigid-mount test as a separate leverage control.

The See3CAM bridge exposes a native Motion JPEG stream. From a host with FFmpeg, set VECTOR_CAMERA_STREAM to the bridge's live.mjpg URL. Run the capture once per take:

```sh
experiments/righting/record-grip.sh right-01.mkv 30
```

The script keeps the native JPEG frames and host arrival timestamps, refuses to overwrite an existing clip and stops after the specified duration. This baseline camera path delivered 30 frames at 1920 × 1080 in the three-second preflight. A higher-frame-rate phone close-up can resolve slips faster than that roughly 100 ms interval. The fixed overhead installation may require a closer camera or reframing before the fork seam is visible. Store raw footage locally and publish only robot crops and derived traces.
