# Vector Dock (Charger) Simulacrum Evidence

## Asset Provenance and Mesh Repair

- Source repository: `kercre123/vector-animations-raw` (`assets/models/Vector/obj/`), under the Digital Dream Labs Software Asset License v1.0 (`DDL-ASSET-LICENSE.txt`). Raw proprietary OBJs (`Vic_charger_hp_geo.obj`, `vic_charger_midPoly.obj`) and MTLs are downloaded locally into git-ignored `web/public/robot/sdk/` via `web/tools/fetch_dock_model.py` (`dock_manifest.json` SHA-256 verified) and are never committed.
- Raw `Vic_charger_hp_geo.obj` contains `6,136` non-triangular polygons and `370` inverted vertex normals. `repairDockNormals()` in `web/src/game/dock.js` triangulates polygons, drops zero-area slivers (`<= 1e-10`), computes 45-degree creased normals, and aligns vertex normals outward (`alignGeometryNormals`), eliminating all inverted/degenerate normals (`0` inverted across both `Vic_charger_hp_geo.obj` and `vic_charger_midPoly.obj`).
- Cyclic right-handed Maya-to-charger transform (`CHARGER_FROM_MAYA_M`, determinant `+1e-9` in metres):
  `x_c = Z_maya + 81.2401 mm`, `y_c = X_maya mm`, `z_c = Y_maya + 0.5586 mm`.

## Collision Decomposition Fit

- `simulacrum/dock_surface.csv` decomposes the dock into `13` oriented boxes (`3` ramp, `6` guide, `2` shell, `2` pin) with exact bilateral reflection across `y = 0`.
- Across all `11` non-pin boxes (`55` pairs), 3D oriented-bounding-box SAT overlap is `0.0 mm`.
- Across `144` continuous ray-triangle samples along the ramp tread lanes (`x in [5, 74] mm`, `y = +-20, +-24 mm`), top-surface Z matches `Vic_charger_hp_geo.obj` with **0.0409 mm MAE** and **0.1119 mm max error**.
- Pin tops sit at `(59.710, +-9.850, 13.671) mm`; the Home Marker back wall stands at `x = 93.205 mm`.

## Physics and Browser Verification

`views.png` (and `measurements.csv`) show four headless Chromium WebGL captures from `web/dist`:

1. **Panel 1 (`?dock=scene`)**: Vector v1 at `(0, 0) mm` (`4.03 V`) and high-poly `Vic_charger_hp_geo.obj` at `(-120, 0) mm`, yaw `180 deg`, with procedural Home Marker (`chargerHome.png` contour polygons) and white alignment stripes.
2. **Panel 2 (`?robot=2&dock=docked&cube=free`)**: Vector 2.0 settled on charger pins at `(-149.43, 0, 6.56) mm`, `ON CHARGER` HUD chip, `4.20 V` resting supply voltage, and free Light Cube (`800x600` head camera active).
3. **Panel 3 (`?dock=scene`, reset to `(-25, 0) mm`, heading `180 deg`)**: Head camera frames the Home Marker and ramp stripes directly ahead while the ToF fan detects the dock (`prox_found=true`, `prox_mm=154`).
4. **Panel 4 (`?dock=docked`, `B` overlay)**: Green wireframe collision boxes (`13` OBBs from `dock_surface.csv`) overlaid in world registration with `Vic_charger_hp_geo.obj`.

Offline MuJoCo tests (`tests/test_dock.py` + `web/test/dock.test.js`) verify:
- Docked start settles on pins with `4.20 V` supply; forward `drive_straight(120, 80)` undocks (`on_charger=False`, `on_charger_platform=False`, `< 4.10 V`).
- Skewed backup inside the cheek mouth (`(-55.0, 3.5) mm`, `-3.0 deg`, `lift=45.0 mm`, `contact_model`, `-45/-45 mm/s`) self-centers onto the pins (`[-150.67, 3.78, 6.77] mm`, `+1.10 deg`, `4.20 V`, `0` cliff reactions).
