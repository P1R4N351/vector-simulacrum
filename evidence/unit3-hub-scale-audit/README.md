# unit 3 Fore-to-Aft Hub Spacing, Dock Glyph & FCC Ruler Scale Audit

Strictly unforced multi-camera bundle adjustment and FCC external-photo verification evaluating unit 3's fore-to-aft hub spacing (`41.45–41.99 mm` in `surveygeom.py` vs DDL `CozmoBot2.proto` `48.0 mm`) without forcing `48 mm`, printed cap rims, or a `580 px` See3CAM focal prior into the calibration target.

## Key Results (All Executed via `check_hub_scale.py`)

| Stage / Configuration | `f_main / f_ultra / f_see` (px) | Dock RMS `main / ultra / see1` (px) | `Bm-15` Hubs (`tri_xy` / `3d`, mm) | `Bp0` Hubs (`tri_xy`, mm) | Held-out `Am-15` Hubs (`tri_xy`, mm) | `Bm-15` Mic (`rms`, mm) | Held-out `S-Am-15` Mic (`rms`, mm) | Held-out `Am-15` 2-Cam Mic |
|---|---|---|---|---|---|---|---|---|
| **0. Baseline `survey.json` (`wh(l)` circular + `lip` on tire + `SHELF` orthog.)** | `647.7 / 370.1 / 473.1` | `2.38 / 1.49 / 0.98` | `41.68 / 41.75` (`See3CAM` horiz-`z` `41.45`) | `42.07` (`41.99`) | `41.69` (`41.75`) | `19.217` (`7.72 px`) | `18.680` (`6.56 px`) | `18.438` (`3.99 px`) |
| **1. Free 3D Spot-B hub ties only (no `wh(l)`)** | `663.0 / 376.0 / 487.5` | `1.34 / 0.81 / 0.97` | `41.79 / 41.87` | `42.04` | `42.49` | `19.102` (`6.91 px`) | `18.685` (`6.57 px`) | `18.687` (`4.40 px`) |
| **2. Drop occluded phone front-lip picks (`lip_y = -94`)** | `667.4 / 377.4 / 489.8` | `1.11 / 0.86 / 0.93` | `44.17 / 44.24` | `44.53` | `44.49` | `20.181` (`3.77 px`) | `19.658` (`4.09 px`) | `19.589` (`1.82 px`) |
| **3. Drop false `SHELF` orthogonality (`unforced`, `lip_y = -94`)** | `670.6 / 378.1 / 496.7` | `1.07 / 0.69 / 0.90` | `46.08 / 46.14` | `46.32` | `46.61` | `20.900` (`1.12 px`) | `20.387` (`1.94 px`) | `20.399` (`0.63 px`) |
| **4. `VictorCharger.proto` ramp projection `lip_y = -89.33` (primary unforced)** | `667.8 / 376.5 / 502.1` | `1.05 / 0.76 / 0.91` | **`47.25 / 47.28`** (`z=[14.03,12.32]`) | **`47.39`** | **`47.73`** | **`21.346`** (`2.42 px`) | **`20.808`** (`1.50 px`) | **`20.828`** (`2.34 px`) |
| **5. All front-lip constraints removed (pure 4-corner glyph + screws + free ties)** | `665.1 / 376.4 / 486.3` | `1.09 / 0.70 / 1.02` | `43.96 / 44.01` | `44.00` | `45.18` | `19.857` (`2.46 px`) | `19.518` (`3.02 px`) | `19.668` (`1.00 px`) |

Propagated `1σ` systematic + pick uncertainty around primary unforced Spot-B calibration (`Model 4`):
- Fore-to-aft hub spacing (`Bm-15` / held-out `Am-15`): **`±2.76 mm`** (`±2.56 mm` from `±1.5 mm` dock glyph boundary tolerance + `±0.59 mm` `lip_y` `-89.33` vs `-94.0` + `±0.65 mm` hub pick/bevel).
- Backpack microphone chord (`S-Am-15`): **`±1.11 mm`**.
- Independent 50 MP (`8160x6120`) convex-hull purple hubcap ellipses on `Bm-15` vertical wheel-face side-plane: `R_front = 14.57–15.01 mm`, `R_rear = 9.65–9.86 mm`, `d_3d = 44.89–46.11 mm`.

## Reproduction

```bash
python3 check_hub_scale.py --self-test
python3 check_hub_scale.py \
  --raw-dir recordings/unit 3-20261119-three-camera \
  --edge-csv evidence/heading-crossover/edge-pixels.csv \
  --survey-json evidence/three-camera-survey/survey.json \
  --fcc-pdf /private/tmp/vector-surface/vext.pdf \
  --out-dir evidence/unit3-hub-scale-audit
```
