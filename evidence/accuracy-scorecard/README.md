# Canonical Sim-to-Real Accuracy Scorecard (`main` + Targets A–C)

This directory scores the canonical MuJoCo 4.0.1 simulacrum (`parameters.json`, `simulacrum/`, and `web/src/sim/`) against physical captures on `unit 1` (Vector 1.0 under ZED 2 stereo tracking) and `unit 3` (Vector 2.0) across all measurable operational domains, separating fit splits from held-out captures.

## Summary scorecard (`>= 95%` across all measurable domains)

| Domain | Metric | Split | Accuracy (%) | Physical observation | Canonical MuJoCo 4.0.1 | Evidence source |
|---|---|---|---:|---|---|---|
| `1_straight_drives` | Mid-pulse speed (`40–160 mm/s`, `n=8`) | held-out | `96.29%` | `mean_err = 3.71%` cmd | Full-trace `REC1` `87.2%` incl. wireless lag | [drive-pulses.csv](../v1-zed-drive-spins/drive-pulses.csv) |
| `2_open_loop_spins` | Normalized yaw gain (`60–280 mm/s`, `n=12`) | held-out | `97.05%` | Median `23.01 rad/m` | Median `23.41 rad/m` | [steady-spins.csv](../v1-highspeed-spins-reversals/steady-spins.csv) |
| `2_open_loop_spins` | Speed ceiling (`240/280 mm/s`, `3.95 V`) | held-out | `99.04%` | `235.5–238.7 mm/s` (`312–315 dps`) | `235.0–235.5 mm/s` (`309–317 dps`) | [steady-spins.csv](../v1-highspeed-spins-reversals/steady-spins.csv) |
| `3_unit3_camera_spins` | Unit 3 turn-rate RMS (`150 mm/s`, `n=4`) | held-out | `99.67%` | Camera ratio `1.000` | Ratios `[1.0053, 0.9980, 1.0030, 1.0019]` | [fit_spins.json](../body-shape/refit/fit_spins.json) |
| `4_head_lift_strokes` | `REC1` held-out servo RMSE + Visit B settles | held-out | `96.82%` | Lift `50.95–52.33 mm`, head `+6.75..+8.97°` | Off-dock settles `99.14%` (lift), `98.91%` (head) | [simulacrum/replay.py](../../simulacrum/replay.py) |
| `5_imu_and_calm` | Static IMU gravity vector fit (`7`-param) | train/held-out | `99.87%` | `REC1=1.69`, `V1=1.30`, `U3_A=6.49`, `U3_A90=3.95 mg` | `accel_scale_x = 1.0118` (`g = 9806.65`) | [level-imu-fit.csv](../v1-zed-tof-imu-dock/level-imu-fit.csv) |
| `5_imu_and_calm` | Calm-mode IMU (`240 ms`) & ToF (`1024 ms`) cadence | held-out | `99.85%` | `23.88% / 23.71%` IMU, `20 / 59` ToF | `23.88% / 24.00%` IMU, `20 / 59` ToF | [calm-summary.csv](../v1-docked-fork-tof-sweep/calm-summary.csv) |
| `6_tof_occlusion` | Off-dock `tof_near_asc` (`72.5 mm`, `n=27`) | train | `96.81%` | Wall `72.5 mm`, `fov = 100%` | `MAE = 2.31 mm`, `fov = 100%` | [tof-near-far.csv](../v1-offdock-tof-and-joints/tof-near-far.csv) |
| `6_tof_occlusion` | Off-dock `tof_far_asc` (`200.5 mm`, `n=13`) | train | `97.80%` | Wall `200.5 mm`, `fov = 100%` | `MAE = 4.41 mm`, `fov = 100%` | [tof-near-far.csv](../v1-offdock-tof-and-joints/tof-near-far.csv) |
| `6_tof_occlusion` | Off-dock `tof_near_desc` (`73.0 mm`, `n=13`) | held-out | `97.85%` | Wall `73.0 mm`, `fov = 100%` | `MAE = 1.56 mm`, `fov = 100%` | [tof-near-far.csv](../v1-offdock-tof-and-joints/tof-near-far.csv) |
| `6_tof_occlusion` | Docked sweep (`n=38`) & pitch cutoff (`n=6`) | held-out | `97.57%` | `MAE = 4.37 mm`, `found = 100%` | `tof_lift_beam_window_mm`, `found = 100%` | [tof-curve.csv](../v1-docked-fork-tof-sweep/tof-curve.csv) |
| `7_dock_idle` | `70.7 s` idle contact hold & creep | check | `97.82%` | `0` disconnects on charger | `0` disconnects, `2.07 mm` creep (`< 2.5 mm`) | [simulacrum/autonomy.py](../../simulacrum/autonomy.py) |
| `8_dock_ramp_return` | Ramp climb (`-35 mm/s`) + `3`-pose dock return | held-out | `100.00%` | `climb_35` mounted (`1`), native dock (`1`) | `climb_35` mounted (`1`), auto-dock `3/3` (`4.20 V`) | [ramp-climb-scores.csv](../v1-ramp-climb-sweep/ramp-climb-scores.csv) |
| `9_wheelstand_cube` | Cube BMA253 gravity + wheelstand equilibria | held-out | `99.95%` | `0.97036 g` on shelf (`unit 3`) | `0.97083 g`; `rear = 64.9°`, `side = 53.5°` | [rest-transfer-repeat.csv](../unit3-wheelstand/rest-transfer-repeat.csv) |

### Key model updates closing sub-95% discrepancies

1. **Target A (`tip_supports_driven = 1`, `chassis_ground_clearance_mm = 2.5`, `tread_contact_inset_mm = 3.5`):** Eliminates the zero-mount dock ramp stall (`0/4 -> 4/4` aligned/straight ramp mounts; `climb_35` physical match `1/1`), keeps `70.7 s` dock idle creep within `< 2.5 mm` (`2.07 mm`), and improves held-out `unit 3` camera spin RMS to `0.33%` (`99.67%`).
2. **Target B (`tof_lift_beam_window_mm = [37.5, 39.8, 46.0, 51.0]`):** Replaces the flat `[34, 56] mm` optical clamp with the physical trapezoid beam occlusion and photon-weighted multi-target mixing, raising ToF accuracy from `~68–84%` to `96.81–97.85%` while keeping firmware flag `is_lift_in_fov` exact (`100.0%`).
3. **Target C (`accel_scale = [1.0118, 1.0, 1.0]`, `accel_bias_mg = [-50.0, -7.9, 22.2]`, `imu_pitch_offset_deg = -0.59`):** Improves `unit 1` static IMU RMS across `REC1` (`1.77 -> 1.69 mg`), held-out Visit 1 (`2.95 -> 1.47 mg`), and held-out docked head sweep.
4. **Target D (Verified invariant):** Visit C proved cold-start onset latency (`median 176 ms`) equals zero-pause reversal zero-crossing latency (`median 176 ms`), isolating the excess full-trace delay on `REC1` (`87.2%` in `simulacrum.replay`) to wireless SDK bridge scheduling rather than tread breakaway stiction.

## Hardware-Gated / Unmeasured Parameters

The following physical parameters remain unmeasured on a precision scale, tilt rig, or feeler gauge and keep their documented `parameters.json` priors (`provenance = "assumed"`):

| Parameter | Canonical value | Range (`low..high`) | What is missing / hardware gate |
|---|---|---|---|
| `total_mass_g` | `170.0` | `150.0..185.0` | Direct precision scale weighing of `unit 1` / `unit 3` |
| `com_x_mm` | `-24.0` | `-30.0..-14.0` | Hand-held released wheelstand rear-balance bracketing ([simulacrum/wheelstand.py](../../simulacrum/wheelstand.py)) |
| `com_z_mm` | `22.0` | `15.0..30.0` | Hand-held released wheelstand side-balance bracketing |
| `cube_mass_g` | `50.0` | `30.0..70.0` | Direct precision scale weighing of the Light Cube |
| `cliff_detect_range_mm` | `25.0` | `15.0..45.0` | Feeler-gauge height calibration over table step (`unit 1` / `unit 3`) |
| `tread_friction` | `0.9` | `0.5..1.3` | Controlled tilt-board slip angle measurement on desk surface |

## Reproduce

```bash
python3 evidence/accuracy-scorecard/scorecard.py
```

Reads the evidence CSVs/JSONs and outputs [scorecard.csv](scorecard.csv).
