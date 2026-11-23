# Cube pickup

The simulated robot could not pick up its light cube. Docked square to the cube with the lift at the bottom and raised to the carry height, the fork slid up the cube's face and the cube stayed on the table, in all 5 lateral offsets tried (−2 to +2 mm). Three things were missing, and the pickup needs all three.

| | Before | Now | Source |
|---|---|---|---|
| Fork | a crossbar and two flat side plates | a barbed hook ahead of each upright, and the upright behind it | the stock fork in Anki's animation rig (`Victor_rig_01.obj`) |
| Cube | each vertical edge's notch closed above and below by a solid ledge | a 1.5 mm lip on each face over a hollow pocket, as on the real cube | Anki's animation cube (`Victor_cube_rig.obj`); TRM 15 |
| Lift torque limit | 30 mN·m, shared with the head | 60 mN·m, its own entry, at least 52 | TRM 15 and 114.5: the lift picks up a cube and lifts the whole robot; the holding torque below |

## How the real lift holds a cube

The cube's four vertical edges are notched over their middle: "The cube has holes near the corners to allow the lift to engage" (Vector TRM, section 15). Docking slides a hook on each fork upright into a notch. Each hook is a barb. Its upper surface dips into a valley next to the upright (z −6.2 mm in the fork's frame, x 4.8 to 6.5 mm) and rises again to the tip (z −2.55 mm at x 10.25 mm). Above the opening, the notch is not a solid ledge: the cube's face continues as a lip 1.5 mm thick, with a pocket behind it up to 13.7 mm from the cube's centre. When the lift rises, the lip drops into the hook's valley and the tip rises into the pocket. The cube's weight pulls its far side down, so it presses its lower front on the crossbar and tries to slide forward off the hooks. The barb's tip stops the lip. A flat-topped hook, or a hook under a solid ledge, holds the cube by friction alone, and the cube slides off.

## Measurements

`measure_hook.py` cuts the fork out of the pinned rig (the registration and frames of `simulacrum/body_mesh.py`), samples its surface every 0.1 mm and measures each upright in the fork's frame. The two sides agree: upright front x 4.73 mm on both, hook y 15.55 to 19.27 mm on both, tip x 10.26 mm on both. The model takes 11 slices 0.5 mm wide from x 4.5 to 10.0 mm, each the box inside both sides' upper and lower surfaces, plus the upright from x 1.0 to 4.5 mm (`hook.json`, `lift_fork_hook_*`).

`measure_notch.py` fetches the animators' cube at the commit and sha256 that `web/tools/fetch_cube_model.py` pins, in that tool's frame (z up, centred, 44 mm on each axis, the frame of `simulacrum/cube_surface.csv`). All 16 cases agree within 0.01 mm: 4 edges, both ends, and the lip on each of the 2 faces.

| | Measured |
|---|---|
| Lip underside | 9.04 mm from the centre |
| Lip's inner face | 20.50 mm |
| Pocket ceiling behind the lip | 13.69 mm |
| Back wall behind the opening | 14.01 mm |

`mjcf.cube_boxes` carves |x| and |y| from 14.0 to 20.5 mm and |z| up to 13.7 mm out of the SDK cube's boxes at each vertical edge (`notch.json`, `cube_notch_pocket_*`). The SDK's `cube.obj` has 128 vertices and fills the pocket in.

`holding_torque.py` sets the lift at each height with the fork level and reads the gravity torque the lift motor must supply. The fork is kept level by a joint equality, so that torque is the lift joint's share less the wrist's.

| Load | Worst holding torque |
|---|---|
| Empty lift | 5.8 mN·m |
| Cube at 30 g (the table's low) | 25.2 mN·m |
| Cube at 50 g (the table's value) | 38.2 mN·m |
| Cube at 70 g (the table's high) | 51.1 mN·m |

All four are worst at 47 mm. The old 30 mN·m limit holds up only the lightest cube the table allows. `lift_servo_torque_max_mNm` is 60 mN·m, the cap `evidence/unit3-wheelstand/reproduction` chose by replay error, with its low bound at 52. It stays assumed: no recording loads the lift enough to identify it.

## The trial

`trial.py` starts the robot square to the cube's near face, 15 mm from it, lift at the bottom. It drives 22 mm at 20 mm/s, raises the lift to 92 mm, holds 1.5 s, and backs away 40 mm. A pickup counts when the cube is at least 40 mm up both while held and after backing away.

| Model | Lift limit | Picked up | Cube rise | Note |
|---|---|---|---|---|
| Before: plates only, solid ledges | 30 mN·m | 0 of 5 | 0.0 mm | the fork pushes the cube 6.3 mm and slides up its face |
| Flat-topped hook (3 or 6 steps), solid ledges | 30 to 80 mN·m | 0 of 5 | 0.0 mm held, in all but one run (6.3 mm at 50 mN·m, 19.6° tilt) | traced at 80 mN·m, the cube rises 21 mm mid-lift, tilts 16° and slides off as the fork swings back |
| Barbed hook and pocket | 30 mN·m | 0 of 5 | 0.0 to 1.5 mm | the lift stalls under the cube |
| Barbed hook and pocket | 52, 60 or 100 mN·m | 4 of 5 | 50.7 mm | carried at 3.0° tilt at offsets −1, 0 and +1 mm and 6.7° (52.0 mm rise) at +2 mm; at −2 mm the hook misses the notch |
| The same, with the lift's integral term | 60 mN·m | 5 of 5 | 51.7 mm | the lift completes at 90.9 mm at offsets −1, 0 and +1 mm, carried at 3.0 to 3.1° tilt; 51.9 mm rise at 4.8° at ±2 mm, lift at 91.1 and 90.8 mm |

The flat-topped and stalled rows are diagnostic runs on intermediate models; the first row and the last two are `trial.py` on the model before the hook and pocket, after them, and after the lift's integral term.

The browser's JS sim, on the exported scenes the page loads, does the same (`web/test/cube-pickup.test.js`). That includes the keyboard's way: from the default start pose in the default cube-and-dock scene, treads held at 100 mm/s until the fork is 7 mm past the cube's face, then released, then the lift key held. In an exploratory run of the keyboard way at 100 and 20 mm/s, pushing 3, 7 or 15 mm past the face, all 6 lifted the cube 50.7 mm and kept it after backing 40 mm. Against the exported model from before this change, all 3 test cases fail with the cube 0.0 mm up.

## Not settled

- The lift's droop under the cube is taken out by the firmware's integral term (`tests/test_lift_integral.py`). Without it the loaded lift stopped at 89.8 mm, 2.68° short, and `set_lift_height` timed out after 5.3 s; with it the action completes in 0.63 s. It does not stay at 92 mm. Holding its target, the firmware bleeds the sum while the lift's power is above 0.1 (LIFTC:858-866), and the cube costs about 0.14 at the top (28.1 mN·m on the servo untouched, at 200 mN·m per unit of power), so the lift works along the edge of its tolerance (1.5°, 1.23 mm there): over the 1.5 s hold its error runs from −0.51° to 1.77°, within tolerance on 217 of 300 ticks, and it ends at 90.7 to 91.1 mm. The firmware allows 0.24 while the engine reports a carried block, which SDK lift calls never set. The 200 mN·m per unit of power is an assumption: the model's lift servo (600 mN·m/rad, no source) stands for the firmware's Kp of 3 power per rad.
- Held at offset 0, the cube's centre sits at (27.1, 0, −16.3) mm from the wrist in the fork's frame. The engine places a carried cube at (26, 0, −12.5) (`cube_carry_offset_mm`): 1.1 mm nearer and 3.8 mm higher. In the model the lip rests in the hook's valley, which with these two rig measurements puts the centre at −15.45 mm before the 3° tilt. The engine's −12.5 is its own constant, and nothing measured settles which is right.
- The cube's mass is assumed (50 g, 30 to 70 g). No source found online or in the TRM gives it. The holding torque scales with it.
- The lift's torque limit is assumed: it must exceed the holding torque, and nothing measured bounds it from above.
- The SDK cube's notch opening goes 12.25 mm from the centre where the animators' cube's back wall is at 14.01 mm. The pocket is carved from the animators' cube; the opening below it is left as the SDK has it.
