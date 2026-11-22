# Printed replacement parts

People print replacement forks, rims and treads for Vector and share the
meshes. `simulacrum/printed_parts.py` can put one of each kind in the model in
place of the stock part:

```python
from simulacrum import printed_parts
sim = printed_parts.vector_sim(parts=("fork:dauler", "treads:dauler-obar"))   # a VectorSim
```

`vector_sim` passes everything else to `VectorSim`. It is a post-pass:
`VectorSim` and `mjcf.build_mjcf` have no keyword of their own for it yet. The
body-mesh mode (`body=`) is adding one to the same two functions, and the two
will be folded together once both land. Until then a printed fork is refused on
a body-mesh model, which builds its own fork. Rims and treads are allowed,
since that mode leaves the wheel bodies alone. Everything printed parts change
is found through the wheel and fork bodies in the model, not through the table,
so a refit that moves the wheels (for example to 47.9 mm tread spacing) moves
the printed treads' masses with them.

```python
from simulacrum import printed_parts, params
v = params.values(params.load())
xml = printed_parts.build_mjcf(v, parts=("rims:10zz",))
report = printed_parts.part_report(v, ("rims:10zz",))   # every number it used
```

With no parts the model is the stock one, byte for byte. The meshes are not in
this repository. They are fetched from their sources and checked against the
SHA-256 recorded for each file:

```sh
python3 -c 'from simulacrum import printed_parts as p; [p.fetch_part(n) for n in p.PARTS]'
```

The cache is `~/.cache/vector-simulacrum/print-parts`, or `VECTOR_PRINT_PARTS`.
Each part keeps its source's licence. Most are non-commercial. That covers the
mesh and anything built from it, including the contact boxes the model makes
at run time.

## The parts

| Name | What it replaces | Source | Author | Licence |
|---|---|---|---|---|
| `fork:dauler` | fork | [Thingiverse 3731746](https://www.thingiverse.com/thing:3731746) `forks7.stl` | Dauler | CC BY-NC 4.0 |
| `fork:10zz` | fork (lift hook) | [Printables 1777652](https://www.printables.com/model/1777652) `Lift Hook.stl` | 10ZZ | Open Community License 1.1 |
| `rims:dauler-stock` | outer wheels, stock design | [Thingiverse 3764077](https://www.thingiverse.com/thing:3764077) `wheel_front_stock.stl`, `wheel_back_stock.stl` | Dauler | CC BY-NC 4.0 |
| `rims:dauler-spoke` | outer wheels, spoked | Thingiverse 3764077 `wheel_front_spoke.stl`, `wheel_back_spoke.stl` | Dauler | CC BY-NC 4.0 |
| `rims:10zz` | outer wheels | Printables 1777652 `Front Wheel.stl`, `Back Wheel.stl` | 10ZZ | Open Community License 1.1 |
| `treads:dauler-obar` | treads, original bar | [Thingiverse 3464746](https://www.thingiverse.com/thing:3464746) `tread_oBar.stl` | Dauler | CC BY-NC 4.0 |
| `treads:dauler-diag` | treads, diagonal bar, left and right | Thingiverse 3464746 `tread_diag_left.stl`, `tread_diag_right.stl` | Dauler | CC BY-NC 4.0 |
| `treads:dauler-sq-knobby` | treads, square knobs | Thingiverse 3464746 `tread_sq_knobby.stl` | Dauler | CC BY-NC 4.0 |
| `treads:madizmo` | treads | [Thingiverse 4287628](https://www.thingiverse.com/thing:4287628) `anki_vector_tracks_V2.stl` | madizmo | CC BY 4.0 |

Thingiverse files come from the Internet Archive's copy of each Thing, the same
route `web/tools/fetch_print_parts.py` uses. Printables files come from its
public download API.

## What each kind changes

- **Fork.** The fork's contact shape is rebuilt from the mesh as boxes. The
  mesh is voxelised at 1 mm and the voxels are merged greedily. The stock
  contact proxies stop colliding, so the opening between the side plates stays
  open. The fork body's mass, centre of mass and full inertia come from the
  mesh's closed surface. They are exact for the mesh, scaled by a filament
  density and fill. The wrist joint, the arm and the lift's level coupling do
  not change.
  - The decomposition is this module's own, not CoACD. Voxels at
    `fork_voxel_mm` (default 1.0) are merged greedily along x, then y, then z.
    It is deterministic for a given mesh and pitch and depends on no library
    version: `fork:dauler` gives 106 boxes, `fork:10zz` 177. `part_report()`
    returns the boxes, the pitch and the voxel volume, so a replay can check its
    piece count. CoACD-based fixtures (PR #91) depend on CoACD's version.
    The body-mesh work reports forks7 at 58 pieces in that fixture and 52
    under coacd 1.0.14, from the same STL and script. Not re-measured here.
  - `fork:dauler` uses the print-bed registration that `experiments/righting/printed_fork.py`
    and the browser use.
  - `fork:10zz` is registered by matching. It is turned into the same axes,
    centred on Dauler's fork, and each of the eight mirror choices is scored by
    voxel overlap. The best must score at least 1.5 times any choice that
    differs in x or z. A fork is symmetric left to right, so a choice and its
    mirror in y tie.
- **Rims.** Each wheel's mass and inertia become the printed rim's. The rolling
  radius becomes the rim's outer radius plus the tread thickness.
- **Treads.** The rolling radius becomes the rim radius plus the band's
  thickness. The thickness runs from the inner surface to the furthest lug,
  since the lugs touch the table. The contact width becomes the band's width,
  and the rear wheel's coupling becomes the ratio of the new radii. The loop's
  mass is split along its mid-line. The arc round each wheel turns with it, and
  the two straight runs ride with the chassis. The stock treads' mass is taken
  back out of the chassis at the same places, since the table lumps it there
  through `total_mass_g`.
- **Tip supports** (`mjcf._tip_supports_xml`) follow the new rolling radius
  and keep their clearance.

The table's values (`v`) are untouched. The robot's firmware still believes in
a 29 mm wheel, so a bigger rolling radius shows up as the robot covering more
ground than its encoders say. That is what happens on the real robot too.

## Measured (2026-11-22)

All figures are from `python3 evidence/printed-parts/measure.py`, MuJoCo not
needed. Material is PLA (1.24 g/cm^3) for forks and rims and TPU (1.21) for
treads, solid. Both figures are assumed from datasheets; override them with
`materials=`.

Forks:

| Part | Mesh volume | Mass | Stock fork in the table | Boxes | Voxel volume | Registration |
|---|---|---|---|---|---|---|
| `fork:dauler` | 2030 mm^3 | 2.518 g | 6.0 g (assumed) | 106 | 1956 mm^3 | print bed |
| `fork:10zz` | 1844 mm^3 | 2.287 g | 6.0 g (assumed) | 177 | 1957 mm^3 | overlap 0.517, next 0.202 |

Rims:

| Part | Front diameter | Rear diameter | Front mass | Rear mass | Front axial inertia |
|---|---|---|---|---|---|
| `rims:dauler-stock` | 26.002 mm | 18.500 mm | 1.352 g | 0.751 g | 135.7 g mm^2 |
| `rims:dauler-spoke` | 26.002 mm | 18.500 mm | 1.328 g | 0.818 g | 133.1 g mm^2 |
| `rims:10zz` | 25.982 mm | 18.500 mm | 2.158 g | 1.140 g | 218.5 g mm^2 |

The two stock-shaped prints agree on the stock rims, 26.0 by 18.5 mm. The table's
wheel radii, 14.5 and 10.75 mm, then leave 1.50 mm for the stock tread at both
ends. That figure fills `STOCK_RIM_DIAMETER_MM`. A front rim's axial inertia (1.33 to 2.19 g cm^2) is under a
thousandth of the wheel joint's fitted armature (2380 g cm^2). Its mass matters,
its spin does not.

Treads, one band; both sides are the same mesh except `dauler-diag`:

| Part | Inner radius | Outer radius | Thickness | Width | Mass | Rolling radius, front / rear |
|---|---|---|---|---|---|---|
| `treads:dauler-obar` | 26.10 | 28.01 | 1.91 | 11.60 | 2.721 g | 14.914 / 11.164 mm |
| `treads:dauler-diag` | 26.10 | 28.88 | 2.78 | 11.60 | 3.354 g | 15.782 / 12.032 mm |
| `treads:dauler-sq-knobby` | 26.10 | 28.22 | 2.12 | 11.60 | 2.525 g | 15.122 / 11.372 mm |
| `treads:madizmo` | 26.10 | 28.04 | 1.94 | 11.60 | 2.533 g | 14.941 / 11.191 mm |

Stock tread (assumed, not weighed): 1.50 mm thick, 10.0 mm wide (the table's
`tread_width_mm`), silicone 1.10 g/cm^3, 2.815 g a side.

The printed bands are 26.10 mm across the inside, 163.99 mm round. The path
round the stock rims is 2 x 48 + pi x (13.0 + 9.25) = 165.90 mm, so a band
stretches 1.17 % when fitted. The model ignores the stretch and the thinning
that goes with it.

In the model (MuJoCo 4.0.1, macOS arm64, nominal table, `set_wheel_motors(60, 60)`
for 3 s):

| Parts | True travel / stock travel | Rolling radius / 14.5 | Rest height change |
|---|---|---|---|
| `treads:dauler-obar` | 1.0287 | 1.0286 | +0.414 mm (+0.414 expected) |
| `treads:dauler-diag` | 1.0886 | 1.0884 | +1.282 mm (+1.282 expected) |
| `treads:madizmo` | 1.0305 | 1.0304 | |
| `rims:10zz` | 0.9994 | 0.9994 | |

Before the rear coupling followed the new radii, the obar run gave 1.0336. The
rear wheels scrubbed at the old ratio. That run is why the coupling is now
rewritten.

## Limits

- Densities and fill are assumed. A real print with 20 % infill weighs much
  less. Weigh the part and pass the measured fill.
- The fork boxes over-fill thin walls by up to half a voxel. Dauler's voxel
  volume is 3.6 % under the mesh, the 10ZZ hook's is 6.1 % over. Mass comes
  from the mesh, not the boxes.
- Treads are still two cylinders per side, not a band: no lugs, no flex, no
  stretch.
- The stock tread and wheel masses in the table are assumed, so the swap's
  change in total mass is too.
- The chassis inertia stays diagonal. The tread masses added and removed are
  mirror pairs in y, so their xy and yz products cancel; their xz products are
  left out.
- Parts on Cults3D, MyMiniFactory and MakerWorld are not registered here. Those
  sites need an account or a person to get past their bot checks to download,
  or the licence does not allow it.
