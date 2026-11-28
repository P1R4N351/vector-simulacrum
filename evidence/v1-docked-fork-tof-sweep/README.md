# Unit 1 docked stock-fork ToF occlusion, lift hysteresis, and passive calm-mode cadence

Three docked recordings captured zero wheel commands while both units remained on their chargers (`unit 1`, Vector 1.0 stock fork at `3.5479 V` charging inactive; `unit 3`, Vector 2.0 at `3.5930 V` calm passive only). Every SDK action returned result code `0`, `on_charger` stayed set in every packet (`335/335`, `1004/1004`, and `2232/2232`), and both lifts returned to `32 mm`.

## Passive calm-mode IMU and ~1 Hz body-clock ToF cadence

Both sleeping robots refresh IMU samples near `240 ms` and update the forward ToF sensor near `1020 ms` (`~0.98–1.00 Hz`, matching Syscon's `200 × 5.12 ms = 1024 ms` body-clock period). Even awake on the charger during the `133.9 s` fork sweep, `unit 1` produced `132` unique ToF updates (`median dt = 1020.6 ms`).

| Recording | Packets | Phys fresh IMU (%) | Phys IMU dt (ms) | Phys accel X SD (mg) | Phys ToF updates | Phys ToF dt (ms) | Sim fresh IMU (%) | Sim X SD (mg) | Sim ToF updates |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| `unit 1` (1.0, 20 s asleep) | 335 | 23.88 | 241.79 | 1.529 | 20 | 1020.2 | 23.88 | 1.998 | 20 |
| `unit 3` (2.0, 60 s calm) | 1004 | 23.71 | 243.25 | 1.383 | 59 | 1018.8 | 24.00 | 1.241 | 59 |

On the stationary shelf (`unit 3`), forward ToF remained in the weak-signal regime (`found = 0`, median distance `93.0 mm`, SD `10.88 mm`, median quality `0.00275`).

## Stock-fork ToF occlusion fine scan (`32–92 mm`)

Four ascending/descending stepped sweeps on `unit 1` (`head = 0°`, `1.0 s` holds) yielded `110` stationary SDK packets immediately following fresh ToF updates (`max(lift) - min(lift) < 0.4 mm` over `200 ms`).

- **Firmware flag boundary (`is_lift_in_fov`)**: `0` at `<= 32.00 mm`, `1` across `35.41–55.07 mm`, and `0` at `>= 56.44 mm`, consistent with `tof_lift_occluding_min_mm = 34.0` and `tof_lift_occluding_max_mm = 56.0`.
- **Actual optical beam interception on stock fork**: Inside `[34, 56] mm`, the physical beam cleared the fork at both edges (`35.41–36.44 mm`: `179.0–179.5 mm`, `q = 0.066`; `54.73–55.07 mm`: `178.0–179.0 mm`, `q = 0.067`). Grazing began at `37.47 mm` (`167.5 mm`, `q = 0.071`), locked onto the `30.0 mm` clamp across `40.92–49.92 mm` with two quality lobes (`q_med = 19.73` at `40.92 mm`, `1.59` at `43.00 mm`, `17.52` at `44.73 mm`, decaying to `0.85` at `49.92 mm`), and exited at `50.61 mm` (`38.0 mm`, `q = 0.678`). Canonical `vector_1` assigns constant `(30.0 mm, q = 11.70)` across the full `34–56 mm` flag band.

## Full-stroke `0/25/50/75/100%` hysteresis (`duration = 0.7 s`)

Replicating the `0 → 25 → 50 → 75 → 100 → 75 → 50 → 25 → 0%` cycle twice on `unit 1`:

- Full raise (`100%`, `92.0 mm`) settled slightly above target (`92.76` and `92.28 mm`; canonical `92.04 mm`), unlike `unit 3`'s lower settle (`89.29–89.54 mm`).
- Cycle 2 `62.0 mm` holds showed a `3.35 mm` direction offset (`63.52 mm` approaching from below vs `60.17 mm` from above), compared with canonical replay (`62.52 mm` vs `61.52 mm`, `1.00 mm` difference).

## Reproduce

Run from repository root with MuJoCo 4.0.1 and NumPy:

```bash
python3 evidence/v1-docked-fork-tof-sweep/analyze.py
```

Outputs [tof-curve.csv](tof-curve.csv), [hysteresis-replay.csv](hysteresis-replay.csv), and [calm-summary.csv](calm-summary.csv) alongside raw captures [unit1-fork-tof-sweep.csv](unit1-fork-tof-sweep.csv), [unit1-passive-asleep.csv](unit1-passive-asleep.csv), and [unit3-passive-calm.csv](unit3-passive-calm.csv).
