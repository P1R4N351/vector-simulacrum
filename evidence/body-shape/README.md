# The robot's real outer shape

The parametric model draws Vector as boxes and cylinders sized from Anki's Webots model. At rest it measures 86.1 × 56.0 × 66.0 mm. Retail listings give the robot as 9.98 × 5.99 × 6.93 cm. `simulacrum/body_mesh.py` builds the real outer shape instead, from the rig Anki's animators used. `mjcf.build_mjcf(v, body=...)` and `VectorSim(body=...)` use it when asked. With no body the generated model is byte for byte what it was.

![Parametric vs rig body envelope, joint pivots, printed references, and CoACD surface fidelity](envelope_and_joints.png)

Breakdown table and reproduction: [`measurements.csv`](measurements.csv) · [`plot_breakdown.py`](plot_breakdown.py)

## Where the shape comes from

[`Victor_rig_01.obj`](https://github.com/kercre123/vector-animations-raw/blob/59b198d4e6fb7394ae4c3d832dea2c4d28866ab0/assets/rigs/Base/obj/Victor_rig_01.obj) in kercre123/vector-animations-raw, commit `59b198d4e6fb`, 13,309,832 bytes, sha256 `59890fafc1c122726d73f49fab66913d45d87426143107d9359e6ca81e7fad60`. That repository carries no licence (GitHub reports NOASSERTION), so the rig and everything cut from it stay out of this tree: `python3 -m simulacrum.body_mesh fetch` downloads it at that commit and hash, and `build` writes the meshes to `~/.cache/vector-simulacrum/body/`.

The rig is in Maya centimetres, +Z forward, +X to the robot's left, +Y up. `register` puts the origin on the table (the lowest tread vertex) under the front axle (a circle fitted to the front rims). It also centres the rig between its outer rims, which sit 0.36 mm off the robot's centre line. The rig is level: the rear axle sits 3.746 mm below the front one, and the wheel radii differ by 3.75 mm.

## The rig's joints against the parameter table

`build` refuses a rig whose joints sit more than 1 mm from the table. Fitted in robot mm:

| Feature | Rig | Table (firmware) | Difference |
|---|---:|---:|---:|
| Front axle height | 14.51 | 14.5 (FW:23) | 0.01 |
| Rear axle x, height | −47.99, 10.77 | −48, 10.75 (FW:28, WEBOTS:32) | 0.01, 0.02 |
| Neck pivot x, z (centre of the head's round side frames) | −12.92, 49.71 | −13, 49 (FW:117) | 0.08, 0.71 |
| Upper lift pivot x, z (rear end cap of the upper links) | −41.26, 45.04 | −41, 45 (FW:123) | 0.26, 0.04 |
| Lift link length, upper / lower links | 65.05 / 65.39 | 66 (FW:84) | 0.95 / 0.61 |
| Camera lens centre from the rig's neck pivot, x, z | 18.91, −8.02 | 18.84, −7.96 (FW:120, optical centre) | 0.07, 0.06 |

End caps are fitted as half-rings beyond their centres (rms 0.18 to 0.25 mm), wheels and the head's side frames as full circles. The camera row shows the rig's head is level: a head turned by h would move the lens 20.54·sin h vertically, so the 0.06 mm bounds h to 0.17°.

The lift is a parallelogram, though not an exact one in the rig. The lower links pivot 0.05 mm ahead of and 12.12 mm below the upper ones; the SDK viewer has 12.0 mm. Their front caps are 12.41 mm apart, and the two links differ in length by 0.34 mm. The model turns the lower links about their own rear cap, at the lift's angle. The rig holds the lift at −11.14°, near the low dock (FW:77, 32 mm).

The links are 0.6 to 0.95 mm shorter than the firmware's 66 mm. The model keeps the firmware's arm, because the robot reports lift height through it. The fork is placed exactly where the rig has it at rest, and then follows a 66 mm arc. At full lift, after a 56.5° swing, that puts it 0.90 mm from where a 65.05 mm arm would put it, or 0.58 mm for a 65.39 mm arm. These numbers are computed from the geometry, not measured on a robot.

## The rig against the real robot

These checks do not come from Anki's CAD.

**Retail size.** Several listings give 9.98 × 5.99 × 6.93 cm (3.93 × 2.36 × 2.73 in); no vendor spec sheet was found. The extents run from the body's rear to the fork's front, across the outer wheel rims, and from the table to the top of the head. They are measured in the compiled MuJoCo model at rest (lift at the low dock, head level) by `validate.py`:

| Model | Length × width × height (mm) | Against retail |
|---|---|---|
| Rig shape, visual meshes | 100.6 × 60.3 × 69.3 | +0.8, +0.4, +0.0 |
| Rig shape, collision pieces | 100.9 × 60.9 × 69.7 | +1.1, +1.0, +0.4 |
| Parametric model, contact geoms | 86.1 × 56.0 × 66.0 | −13.7, −3.9, −3.3 |

**Parts printed to fit a real Vector.** Each was designed by a hobbyist against a physical robot, and none was taken from the rig. Files are read from the Internet Archive mirrors of their Thingiverse pages, measured, and not kept: most are CC BY-NC.

| Part (designer, Thingiverse id) | Theirs | Rig | Parametric model |
|---|---:|---:|---:|
| Front wheel rim diameter (Dauler 3764077 "stock", 3298622) | 26.0 | 26.0 | (29.0 cylinder incl. tread) |
| Rear wheel rim diameter (Dauler 3764077, 3298622; FabioDB 4444528) | 18.5; 18.59 | 18.5 | (21.5 cylinder incl. tread) |
| Tread width (Dauler 3241305, 3464746; madizmo 4287628) | 11.6 | 11.5 | **10.0** |
| Tread loop outer perimeter (printed as a ring 56.0 mm across) | 175.9 | 175.7 | — |
| Fork width (Dauler 3731746, forks7) | 48.0 | 48.2 | 47.0 over the side plates |
| Lower rear panel width (Steve/M 3964462, butt plate) | 32.4 | 32.9 | 32.2 (chassis box) |
| Light cube outer (Dauler 3239449 cap, 3405048 box) | 45.0 | 42.2 (cube rig) | **44** |

**Unit 3's iPhone LiDAR scans** ([iphone-lidar-orbits](../iphone-lidar-orbits/README.md)) **do not resolve the shape at the millimetre.** Distance from scan points to each surface, for the body and head inside the treads and behind the fork, as registered (median / p90 mm):

| Surface | level | lift up | head up |
|---|---|---|---|
| rig | 3.6 / 9.8 | 3.8 / 10.7 | 3.4 / 9.7 |
| rig scaled ×1.10 | 4.5 / 11.3 | 4.3 / 11.4 | 4.5 / 11.3 |
| rig scaled ×0.90 | 3.4 / 9.1 | 3.7 / 12.1 | 3.1 / 8.7 |
| rig moved 5 mm back | 3.4 / 9.5 | 3.9 / 10.9 | 3.5 / 9.5 |
| parametric boxes | 4.8 / 12.8 | 5.2 / 15.0 | 4.9 / 14.3 |

The scans rank the rig ahead of the boxes, but they cannot tell it from the rig shrunk 10 % or moved 5 mm. This null result stands: the scans' 3 mm voxels and 2 to 9 mm pose error are coarser than the question. `lidar_vs_rig.py` reproduces the table.

## What the cube rig gets wrong

The animation cube is 42.2 mm. Two parts printed to fit the real cube are 45.0 mm, which matches the Webots `Block1x1` value. The engine's 44 mm (BLOCKDEF:6) is the size it gives the marker-bearing faces. The rig is trusted for the robot, where the checks above agree, and not for the cube.

## Dimensions the parametric model has wrong

| Parameter | Table | Real | Evidence | Effect if changed |
|---|---:|---:|---|---|
| `tread_width_mm` | 10.0 | 11.5–11.6 | rig, three printed treads | contact patch of the wheel cylinders, so spin friction; refit `tread_contact_inset_mm` and the drive fit |
| `tread_spacing_mm` (as built) | 46.0 | 47.9 centre to centre | rig chains at y −23.98 and +23.90 | turning; FW:26 uses 46 in the robot's own odometry, which must stay |
| `head_box_mm` | 34 × 40 × 34 | 39.7 × 46.4 × 38.8, centred 1.5 mm ahead of and 1.0 mm above the neck | rig head | head inertia; contact only in the parametric model |
| chassis box | 56 × 32.2 × 24.2, x −43 … 13 | body x −66.7 … 14.2, up to 67.9 mm high at the back | rig body | contact when tipped; the box ends 5 mm ahead of the rear axle |
| `chassis_ground_clearance_mm` | 3.0 (assumed) | 1.2 | rig body's lowest point; WEBOTS:97,185 gave 1.2 | none on a flat table |
| `cube_size_mm` (as a physical size) | 44 | 45.0 | two printed cube parts | carried and free cube contact |

None of these is changed here. The drive, spin and inset fits were made with the current values, so changing them without refitting would move every replay. With `body=...`, contact everywhere except the wheels comes from the rig. The wheel cylinders keep the fitted contact model.

## How the collision shape is built

The rig is a visual model: most of its groups are open shells, and it has no faces where parts meet. `build` cuts it into the five moving parts, in each part's body frame at joint angle zero:

| Part | Rig groups | Model body |
|---|---|---|
| Chassis | body, backpack, tread chains, wheels | `chassis` |
| Head | head and face | `head` |
| Upper lift links | both upper links | `lift_arm` |
| Lower lift links | both lower links | `lift_lower` (new) |
| Fork | crossbar and side plates | `lift_fork` |

Each part is then made solid:

1. The part is voxelised at 0.5 mm. A voxel counts as inside when the surface lies beyond it in at least 5 of the 6 axis directions: a hole in a shell lets one direction out, not five.
2. Gaps up to 3 mm are closed and enclosed cavities filled.
3. The chassis is filled with the head and treads closing its open front and sides. Then the head (one voxel wider) and both whole tread hulls (one voxel wider) are carved back out.
4. Slivers one voxel thin and components under 2 % of the largest are dropped.
5. CoACD (concavity 0.02, seed 20261122) splits the solid into convex pieces within a budget per part: chassis 24, head 10, upper links 8, lower links 12, fork 8.

Each side's treads and wheels become one convex hull, cut 0.3 mm above the table as `mjcf._tip_supports_xml` does, so the wheels alone carry the robot on a flat table. The built body has 64 collision pieces.

In the model the pieces carry contact, with the table's friction (tread hulls `tread_friction`, the rest `chassis_friction`) and no mass. The primitive geoms keep their names and masses but lose contact and drawing, so total mass and every part's mass are unchanged. The upper and lower links share the arm mass equally. The lower links turn on their own hinge, held to the lift's angle by an equality. Visual meshes are the rig's own triangles in geom group 2.

## Fidelity of the built body

Scored over the whole robot at the rig's pose:

- **Protrusion** is rig surface lying outside every piece: shape the collision model lacks.
- **Excess** is exposed piece surface, measured from the nearest rig surface: filling where the robot has none.

Samples are 0.35 mm apart. Piece surface within 1.5 mm of another piece counts as an interface, not exterior. The limits were fixed before any build was scored: protrusion p99 ≤ 1 mm and excess p95 ≤ 2 mm.

| Part | Pieces | Protrusion p99 / max | Excess p50 / p95 / max |
|---|---:|---|---|
| Chassis | 26 | 0.20 / 0.60 | 0.63 / 1.40 / 4.71 |
| Head | 10 | 0.00 / 0.16 | 0.72 / 1.00 / 1.91 |
| Upper links | 8 | 0.00 / 0.00 | 0.69 / 1.93 / 2.47 |
| Lower links | 12 | 0.00 / 0.07 | 0.77 / 1.82 / 2.48 |
| Fork | 8 | 0.09 / 0.59 | 0.52 / 1.59 / 2.88 |

`python3 -m simulacrum.body_mesh check` prints this from a build. A body that misses a limit exits 1. Build time on an 8-core x86_64 Linux host: 19 min.

## Checks of the model

`validate.py BODY_DIR` checks four things:

- **Envelope.** It produced the envelope table above.
- **Drag.** Over 900 control ticks of driving straight, spinning, stopping and reversing on the table, no piece other than the wheels touched it.
- **Faults.** The fidelity measure on the fork, with its largest piece:
  - moved 2 mm: protrusion p99 1.49, excess p95 2.50, fails;
  - removed: protrusion p99 7.73, fails;
  - grown 20 %: excess p95 2.54, fails;
  - unbroken: passes. 4 of 4 as expected.
- **Cost.** Driving at 150 mm/s on an idle 8-core x86_64 Linux host took 246.3 ms per simulated second with the rig shape and 206.9 with the primitives, 19 % more.

## Tests

`tests/test_body_mesh.py` has 20 cases. None needs the rig: every shape is made up in the test. They cover:

- circle and end-cap fits;
- the hinge sign;
- the table check;
- voxel surfaces closed and exact;
- a shell with an 8-voxel hole filled, and a pocket open on two sides left empty;
- scrap removal;
- the clipped tread hull;
- the fidelity measure on known shapes;
- the mesh-mode MJCF from a body of boxes. It has the lower-link equality, carrier geoms without contact, the arm mass split, and the table friction on pieces; it compiles to the primitive model's mass, and the lower links follow the lift in `VectorSim`.

`tests/mutate.py` gains 9 arms, one per property. Each was run through the harness's own control and arm logic (MuJoCo 3.11.0): 9 of 9 caught, 0 survived.

Full suite on that 8-core Linux host (MuJoCo 3.11.0): 390 declared, 390 ran, 381 passed, 8 skipped for optional inputs. One failed: `test_replay.Replay.test_contact_trial_medium_ccw_trace_and_fast_turn_rates`, segment `spin_cw_220`, ratio error 0.0356 against 0.02. It fails with the identical value (0.03564888850321879) on a pristine `git archive` of HEAD `d1e2ee6`, so it predates this change. The default MJCF is unchanged byte for byte (`MeshModeModel.test_default_model_is_unchanged`).

## What the checks caught while this was built

1. **The fidelity measure caught a real decomposition defect.** The first chassis fill leaked through the gap between body and treads: 16 cm³ of solid inside a 169 cm³ hull. With six-direction voting it reached 130 cm³, but the scraps left at the tread edges used up CoACD's budget. The result was one hull of 224 cm³ over the whole robot and 23 slivers. The build reported it as excess p95 3.85 mm, outside the limit; cleaning the solid brought it to 1.40.
2. **An instrument fault, reported as such.** The first envelope check gave the rig shape 95.3 mm of height. MuJoCo re-centres a mesh on its principal axes, and that bounding box, turned back to the world, overstates the extent. The check now uses the mesh's vertices.
3. **Excess measured per part alone flagged hidden faces.** These were the head's underside and the recess it sits in, which the rig has no surface for. On the same build, scoring the whole robot with a 1.5 mm interface band took the head's excess p95 from 9.05 to 1.02 mm. The lower links then still missed (2.38 mm with 8 pieces) and were given 12 (1.82 mm). The limit was not moved.

## Using it

```bash
python3 -m simulacrum.body_mesh fetch
```

```bash
python3 -m simulacrum.body_mesh build
```

```python
from simulacrum import body_mesh
from simulacrum.robot import VectorSim
sim = VectorSim(body=body_mesh.load(body_mesh.DEFAULT_OUT))
```

The build needs numpy, scipy and CoACD (`pip install coacd`). Without CoACD it falls back to one hull per rig group, which fills the links' and fork's concavities: excess p95 9 to 13 mm in the first build, scored per part.

## Not done

- The dimension corrections in the table above. Each needs a refit, since the drive and spin fits were made with the current values.
- Mass and inertia still come from the parameter table and the primitive boxes, not from the shape.
- The cube keeps the SDK shell scaled to 44 mm.
- This build applies on top of `d1e2ee6` and `main`; combined integration with `dock` or other scene modes remains a separate step.
- The lift-link length (65.05 to 65.39 mm in the rig, 66 in firmware) is the one rig dimension a printed part could still settle. TheDimensionalForge's replacement upper arms are on Cults3D only, behind a Cloudflare check, and were not read.
