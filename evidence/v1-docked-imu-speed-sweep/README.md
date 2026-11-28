# Unit 1 docked head gravity sweep and joint multi-speed matrix

Two docked `unit 1` (Vector 1.0) captures at `3.5479 V` (on charger throughout every state packet, zero wheel commands, all actions returning code `0`).

## Awake stationary baseline and head gravity-vector sweep (`-20°` to `+44°`)

- **Awake vs asleep IMU cadence**: Before the head sweep (`15 s` stationary awake baseline, `lift = 31.66 mm`, `head = -4.18°`), `unit 1` delivered `240/240` (`100%`) fresh IMU samples at `60.07 ms` median spacing (`X/Y/Z SD = 1.481 / 1.195 / 1.300 mg`), versus `23.88%` (`241.79 ms`) when asleep.
- **Gravity-vector sweep (`25` settled holds across 3 sweeps)**: Sweeps 1 (`-20°..+44°`) and 2 (`+44°..-20°`) served as training holds; Sweep 3 (`0, +40, -20, +20, -10, +44, +10, +30, 0°`) was held out. Fitting effective dock+IMU pitch, 3-axis bias, and `scale_x` on Sweeps 1+2 yields:
  - Nominal `scale_x = 1.0`: train RMSE `3.068 mg`, held-out Sweep 3 RMSE `3.450 mg`.
  - Free `scale_x = 1.0127`: train RMSE `1.532 mg`, held-out Sweep 3 RMSE `3.387 mg`, with `pitch0 = -9.45°` and `bias_mg = [-41.7, +10.0, +22.6]`.
  - **Charger posture shift**: `|a|` drops monotonically with upward head tilt (`1042.4 mg` at `-21.14°` to `1000.3 mg` at `+44.23°`), while `pose_pitch` shifts from `-10.25°` (`sweep3_holdout_2`, fast `-20°` return) to `-12.05°` (`+30°`), explaining the `-14.19 mg` X / `-6.37 mg` Z held-out outlier on `sweep3_holdout_2`.
- **Canonical Sweep 3 MuJoCo 4.0.1 replay**: Docked `vector_1` maintained charger contact across all `1718` packets with held-out Sweep 3 per-move median `head_RMSE = 2.45°` (`1.08°–4.96°`) and median `gyro_y_RMSE = 0.316 rad/s` (`0.166–0.473 rad/s`).

## Head and lift multi-speed matrix (`max_speed = 1.0, 2.0, 4.0, 10.0 rad/s`)

Two cycles each for head (`-10° ↔ +30°`, `accel = 10 rad/s²`) and lift (`35 ↔ 85 mm`, `accel = 10 rad/s²`) compared against canonical docked `vector_1`:

| Joint & `max_speed` | Phys 10–90% (ms) | Sim 10–90% (ms) | Phys onset (ms) | Sim onset (ms) | 1.5 s RMSE range |
|---|---:|---:|---:|---:|---:|
| Head `1.0 rad/s` | 530–560 | 517–564 | 286–328 | 230–275 | 1.55–3.22° |
| Head `2.0 rad/s` | 300–312 | 313–320 | 284–325 | 229–239 | 2.40–3.83° |
| Head `4.0 rad/s` | 280–309 | 259–348 | 291–318 | 225–285 | 2.68–5.94° |
| Head `10.0 rad/s` | 239–305 | 296–319 | 261–315 | 227–234 | 2.07–2.82° |
| Lift `1.0 rad/s` | 592–659 | 628–660 | 268–291 | 240–250 | 2.22–2.60 mm |
| Lift `2.0 rad/s` | 309–348 | 337–371 | 288–311 | 232–264 | 2.60–4.02 mm |
| Lift `4.0 rad/s` | 272–322 | 315–356 | 246–317 | 219–240 | 2.00–3.23 mm |
| Lift `10.0 rad/s` | 272–347 | 279–317 | 271–293 | 238–261 | 2.16–3.83 mm |

Canonical PD + inertia dynamics reproduce both the `1.0 rad/s` rate-limited strokes and the physical speed saturation between `2.0` and `10.0 rad/s` without retuning; the remaining trajectory RMSE is dominated by `~55 ms` host-to-motion onset lag (`246–328 ms` measured vs `219–285 ms` simulated).

## Reproduce

```bash
python3 evidence/v1-docked-imu-speed-sweep/analyze.py
```

Writes [gravity-holds.csv](gravity-holds.csv), [sweep3-replay.csv](sweep3-replay.csv), and [speed-matrix-scores.csv](speed-matrix-scores.csv) from [unit1-head-imu-sweep.csv](unit1-head-imu-sweep.csv) and [unit1-joint-speed-matrix.csv](unit1-joint-speed-matrix.csv).
