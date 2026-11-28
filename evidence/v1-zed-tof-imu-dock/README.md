# Unit 1 off-dock multi-distance ToF, 2-heading IMU fit, and ZED-tracked dock return

Visit 1 drove `unit 1` (Vector 1.0) off its workshop charger under `DEFAULT_PRIORITY`, held at the undocked spot and four `−20 mm` reverse steps facing the stationary ZED 2 stereo camera (`head = 0°`, `lift = 32 mm`), drove `+40 mm` forward, executed paired head sweeps (`0, 25, −15, 40, 10, 0°`) at two opposite headings (`0°` and `180°`), and returned to the charger. Across `1874` SDK state rows and `1274/1292` detected ZED 2 VGA frames, all drive, turn, and head actions returned code `0`, battery stayed `3.9730 -> 3.9703 V` off-dock, and `unit 1` re-docked and began charging (`on_platform = 1`, `charging = 1`).

## Multi-distance ToF range, noise, and pitch cutoff vs. ZED 2 stereo

| Phase | ZED `x` (mm) | ZED `dock_x` (mm) | SDK odo `x` (mm) | Pitch (deg) | ToF mean ± SD (mm) | ToF SD / mean | `prox_q` med | Phys `found` | Canonical `found` |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| `dock_pre` (`head = −21.4°`) | 208.3 | 70.3 | 0.5 | −12.12 | 182.33 ± 1.53 | 0.0084 | 0.0737 | 0.00 | 0 |
| `tof_step_0` | 88.6 | 190.1 | 124.6 | −3.90 | 80.05 ± 1.32 | 0.0165 | 0.5594 | 1.00 | 1 |
| `tof_step_1` | 108.6 | 170.2 | 104.2 | −3.78 | 98.51 ± 1.78 | 0.0181 | 0.2351 | 1.00 | 1 |
| `tof_step_2` | 129.5 | 149.3 | 83.7 | −3.66 | 122.69 ± 1.71 | 0.0139 | 0.0899 | 1.00 | 1 |
| `tof_step_3` | 145.3 | 133.6 | 63.7 | −8.27 | 139.62 ± 1.50 | 0.0107 | 0.1105 | 0.00 | 0 |
| `tof_step_4` | 162.2 | 116.7 | 43.6 | −11.84 | 155.78 ± 1.53 | 0.0098 | 0.0825 | 0.00 | 0 |

- **Linearity vs. ZED stereo:** Across `tof_step_0..4` (`Δzed_x = +73.6 mm`), ToF distance increased by `+75.73 mm` (`2.9%` difference), with a `5.7–10.1 mm` face-to-ToF offset and stationary range SD of `1.32–1.78 mm` (`1.0–1.8%` of range).
- **Pitch cutoff (`found_object`):** Backing onto the charger apron (`tof_step_3` at `pitch = −8.27°` and `tof_step_4` at `−11.84°`) crossed `tof_found_pitch_limit_deg = 5.0°`, switching `found_object` from `1.00` to `0.00` despite `prox_q = 0.08–0.11` (`8–11×` `tof_quality_min`), matching `simulacrum.prox.ProxSensor` on every step.

## Level-table 2-heading IMU fit (`simulacrum.imu_fit`)

Twelve settled holds (`6` targets × `2` headings, `~180°` apart) separated desk slope (`tilt = (−0.97°, +0.36°)`), IMU pitch mounting offset, and accelerometer biases:

- Nominal `scale_x = 1.0`: `rms = 2.951 mg`, `pitch_off = −0.684°`, `bias_mg = (−36.87, −13.29, +25.09)`.
- Free `scale_x = 1.01587`: `rms = 1.302 mg`, `pitch_off = −0.499°`, `bias_mg = (−42.46, −13.28, +25.61)`.

## Undock and native dock return (`dock_x` in charger frame)

- **Undock:** ZED tracked `unit 1` from `dock_x = 70.0 mm` to `187.7 mm` (`+117.7 mm` face advance).
- **Dock return:** From the last `−5°` pitch crossing to the first `on_charger` flag took `2.82 s`, `−82.3 mm` integrated wheel travel, median final backing speed `−30.5 mm/s`, and `−11.57°` pitch at contact, while ZED measured `dock_x` retreating from `178.2 mm` to `96.6 mm` (`81.6 mm` optical travel, within `0.7 mm` of wheel odometry).

## Reproduce

```bash
python3 evidence/v1-zed-tof-imu-dock/analyze.py
```

Reads [unit1-visit1-tof-imu-dock.csv](unit1-visit1-tof-imu-dock.csv) and [visit1-zed.jsonl](visit1-zed.jsonl) and writes [tof-vs-zed.csv](tof-vs-zed.csv), [level-imu-fit.csv](level-imu-fit.csv), and [dock-return.csv](dock-return.csv).
