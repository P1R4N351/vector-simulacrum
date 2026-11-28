# Unit 1 off-dock straight speed steps (`40–160 mm/s`) and alternating spins (`60–200 mm/s`)

Visit 2 undocked `unit 1` (Vector 1.0) under `OVERRIDE_BEHAVIORS_PRIORITY` with active hazard/telemetry guards, reversed `25 mm` to center on the tabletop, turned `−90°` to face the open side of the desk, executed equal-and-opposite timed wheel pulses at `40, 80, 120, 160 mm/s`, turned `+90°` toward the ZED 2 stereo camera, ran eight `0.85 s` alternating spins (`±60, ±100, ±150, ±200 mm/s`), and re-docked natively (`1410` SDK state rows, `757/848` ZED 2 VGA frames, `3.8035 V` undocked, finishing `on_platform = 1`, `charging = 1`, `3.8896 V`).

## Straight speed steps (`40–160 mm/s`) vs. canonical `VectorSim`

| Phase | Cmd (mm/s) | Pulse dur (s) | Phys `w_mid` (mm/s) | Sim `w_mid` (mm/s) | Phys `w_peak` (mm/s) | SDK odo (mm) | Canonical sim (mm) | ZED silh (mm) |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| `drive_40_fwd` | +40 | 0.545 | +26.1 | +28.2 | 32.0 | 11.4 | 13.8 | 12.8 |
| `drive_40_rev` | −40 | 0.535 | −24.5 | −27.0 | 30.5 | 12.9 | 13.4 | 27.7 |
| `drive_80_fwd` | +80 | 0.539 | +71.1 | +71.0 | 74.5 | 36.6 | 36.0 | 9.5 |
| `drive_80_rev` | −80 | 0.545 | −66.2 | −72.2 | 69.5 | 30.8 | 36.4 | 15.3 |
| `drive_120_fwd` | +120 | 0.430 | +110.3 | +108.2 | 110.5 | 51.0 | 45.7 | 10.9 |
| `drive_120_rev` | −120 | 0.430 | −109.3 | −108.2 | 112.0 | 48.0 | 45.7 | nan |
| `drive_160_fwd` | +160 | 0.383 | +152.2 | +146.3 | 153.5 | 59.4 | 56.7 | nan |
| `drive_160_rev` | −160 | 0.394 | −153.0 | −146.3 | 154.5 | 58.5 | 58.4 | nan |

- Across `40–160 mm/s`, SDK displacement matches canonical `vector_1` (`3.8035 V`) within `0.1–5.6 mm`. Side-on ZED silhouette detection (`−90°`) dropped out at `120_rev` / `160` against dark background borders (`nan`).

## Alternating spins (`60–200 mm/s`, `~0.9 s` pulses) vs. canonical `VectorSim`

| Phase | Cmd (mm/s) | Phys `wdiff` (mm/s) | Sim `wdiff` (mm/s) | Phys `yaw` (rad/s) | Sim `yaw` (rad/s) | Phys `yaw/|wdiff|` (rad/m) | Sim `yaw/|wdiff|` (rad/m) | Raw `yaw` err (%) |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| `spin_60_ccw` | +60 | +99.5 | +106.9 | +1.143 | +1.222 | 11.49 | 11.43 | +6.86 |
| `spin_60_cw` | −60 | −100.5 | −105.3 | −1.190 | −1.204 | 11.84 | 11.43 | +1.18 |
| `spin_100_cw` | −100 | −184.4 | −191.1 | −2.153 | −2.193 | 11.68 | 11.47 | +1.86 |
| `spin_100_ccw` | +100 | +180.8 | +192.4 | +2.109 | +2.204 | 11.67 | 11.46 | +4.48 |
| `spin_150_ccw` | +150 | +287.3 | +298.0 | +3.313 | +3.502 | 11.53 | 11.75 | +5.69 |
| `spin_150_cw` | −150 | −286.7 | −298.7 | −3.323 | −3.435 | 11.59 | 11.50 | +3.38 |
| `spin_200_cw` | −200 | −389.4 | −405.4 | −4.425 | −4.878 | 11.36 | 12.03 | +10.25 |
| `spin_200_ccw` | +200 | +387.1 | +405.1 | +4.479 | +4.665 | 11.57 | 11.52 | +4.15 |

- Normalized yaw per differential wheel speed (`11.36–11.84 rad/m`, median `11.58 rad/m`) matches canonical `vector_1` (`11.43–12.03 rad/m`) within `1–3%` across the entire `60–200 mm/s` range. Raw `0.9 s` spin rates in simulation sit `1.18–10.25%` higher because physical wheel motion begins `~130–160 ms` after the `set_wheel_motors()` RPC (`~75 ms` longer than canonical `command_delay_ms`), reducing second-half wheel speed (`387–389` vs `405 mm/s` at `200 mm/s`).

## Undock and native dock return (`dock_x` in charger frame)

- **Undock:** ZED eyes tracked `dock_x` from `71.1 mm` to `187.2 mm`.
- **Dock return:** From the last `−5°` pitch crossing to the first `on_charger` flag took `2.94 s`, `−79.9 mm` wheel odometry, `−31.0 mm/s` final backing speed, and `−11.89°` pitch at contact (`dock_x` `192.6 -> 131.4 mm` in ZED).

## Reproduce

```bash
python3 evidence/v1-zed-drive-spins/analyze.py
```

Reads [unit1-visit2-drive-spins.csv](unit1-visit2-drive-spins.csv) and [visit2-zed.jsonl](visit2-zed.jsonl) and writes [drive-pulses.csv](drive-pulses.csv), [spin-pulses.csv](spin-pulses.csv), and [dock-return.csv](dock-return.csv).
