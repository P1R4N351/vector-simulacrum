# Unit 1 controlled reverse ramp climb vs. canonical MuJoCo 4.0.1

To isolate the `~3.5×` backing speed gap noted in [return-contact](../workshop-spins/return-contact/README.md), `unit 1` (Vector 1.0) ran open-loop reverse climbs (`−25, −35, −50, −70 mm/s`) onto the charger ramp from ahead of the apron under synchronized ZED 2 stereo tracking (`1048` SDK state rows, `939/958` detected ZED 2 VGA frames).

| Attempt | Cmd (mm/s) | Dur (s) | Phys mounted | Wheel odo (mm) | ZED `dock_x` (mm) | ZED Δx (mm) | Pitch start → min → end (deg) | Yaw start → end (deg) | Sim mounted | Sim Δx (mm) | Sim pitch end (deg) |
|---|---:|---:|---:|---:|---:|---:|---|---|---:|---:|---:|
| `climb_25` | −25 | 4.05 | 0 | −89.5 | 159.4 → 93.9 | −65.5 | −3.86 → −16.58 → −11.87 | +4.5 → +2.4 | 1 | −83.3 | −8.50 |
| `climb_35` | −35 | 2.89 | 1 | −86.5 | 137.0 → 83.7 | −53.3 | −3.98 → −15.24 → −12.12 | +1.6 → +1.8 | 1 | −84.5 | −8.43 |
| `climb_50` | −50 | 2.24 | 0 | −94.6 | 145.8 → 114.1 | −31.8 | −3.94 → −13.84 → −13.30 | +10.7 → +18.4 | 1 | −86.3 | −8.32 |
| `climb_70` | −70 | 1.65 | 0 | −98.3 | 111.5 → 111.6 | +0.1 | −10.01 → −13.51 → −11.43 | +17.8 → +27.0 | 1 | −88.8 | −8.18 |

## Key findings

- **Physical mount threshold lies between `−25` and `−35 mm/s`:** On squarely aligned climbs (`yaw = +1.6°..+4.5°`), `−25 mm/s` cleared the `−16.58°` ramp crest with `24.0 mm` of tread slip (`−89.5 mm` wheel odometry vs `−65.5 mm` ZED optical travel) and stopped just short of the pins (`dock_x = 93.9 mm`), whereas `−35 mm/s` mounted onto the charger pins (`dock_x = 83.7 mm`, `pitch = −12.12°`, `33.2 mm` slip) inside `2.89 s`.
- **Canonical MuJoCo ramp climb fix (`tip_supports_driven = 1`, `chassis_ground_clearance_mm = 2.5`, `tread_contact_inset_mm = 3.5`):** Prior to these updates (`tip_supports_driven = 0`, `chassis_ground_clearance_mm = 1.2`), canonical `vector_1` stalled at the ramp lip on `−25`, `−35`, and `−50 mm/s` (`pitch = −4.36°..−4.40°`, `Δx ≈ −30.5 mm`, `0` mounts) and climbed only partially to `−9.19°` (`Δx = −65.0 mm`, `0` mounts) at `−70 mm/s`. With driven tip supports and realistic `2.5 mm` belly clearance, canonical `vector_1` mounts cleanly across `−35..−70 mm/s` (`−25 mm/s` also mounts in aligned MuJoCo, whereas physical `climb_25` stopped at `dock_x = 93.9 mm` with `+4.5°` initial yaw).
- **Off-axis grazing on `climb_50`/`climb_70`:** During the `−50 mm/s` reset undock, heading drifted to `+10.7°`; `climb_50` and `climb_70` grazed the side of the charger apron (`yaw` rotating to `+27.0°`), after which `climb_70` tripped `cliff_detected` (`status = 7357184`) and the guard halted actuation safely without damage. Subsequent native `robot_dock` and one `DEFAULT_PRIORITY` `drive_on_charger` left `unit 1` off-dock at `3.9047 V`, and the session stopped per safety protocol.

## Reproduce

```bash
python3 evidence/v1-ramp-climb-sweep/analyze.py
```

Reads [unit1-ramp-climb.csv](unit1-ramp-climb.csv) and [visitA-zed.jsonl](visitA-zed.jsonl) and writes [ramp-climb-scores.csv](ramp-climb-scores.csv).
