# P2Y8 stationary head sweeps on the shelf

A head-dependent residual and a component fixed in shelf coordinates both predicted the held-out readings. A chassis offset alone missed most of the variation. The clockwise versus counterclockwise arrival difference fitted at B failed to transfer to opposite headings at A.

| Model fitted at B | B error (mg) | A opposite headings (mg) | A sideways (mg) |
|---|---:|---:|---:|
| Chassis offset | 18.35 | 17.86 | 20.15 |
| Add head position | 16.32 | 16.09 | 18.39 |
| Add shelf-frame vector to chassis offset | 9.60 | 10.74 | 8.75 |
| Chassis offset + head position + shelf-frame vector | 4.65 | **7.42** | **2.99** |
| Also add last-turn direction | 2.83 | 8.59 | 2.93 |

In the full decomposition the forward residual from the lowest to highest target changed by **23.5 mg at B and 21.2 mg at A**. The heading-dependent vector was 15.4 mg at B and 15.2 mg in an independent descriptive fit at A, equivalent to about 0.88° in each case. A shared level-head body offset was also present (B forward/left 12.3/3.8 mg). The CW/CCW contrast was +7.2 mg forward at B and −1.6 mg at A (CCW minus CW). Returning to zero head angle shifted the forward reading by as much as 6.7 mg. Repeating the same target order at both spots leaves room for mechanical hysteresis and settling. Each approach was sampled once.

All 1,219 SDK samples in the nine analyzed sweeps passed the wheel and gyro gates. The backpack template returned the same whole-pixel location at each checked rest. That constrains gross chassis displacement; it cannot resolve a subpixel shift or a small change of tread loading. The horizontal centers of the two primary spot clusters were about 72 mm apart. A setup sweep and an intermediate transit sweep are retained in the sample file and excluded from all scores.

## Measurement

P2Y8 (Vector 2.0) stood off the dock with the lift lowered. At each of two spots, we established nominal headings 0° and 180° and approached each from both directions. For each of the eight primary sweeps the final approach was a commanded 23° turn. The head targets were 0°, 25°, −15°, 40°, 10°, then 0° again. The chassis received no motion command during a sweep. After each head command completed, we allowed two seconds to settle, then sampled for 1.35 s. We kept windows separate, so multiple SDK rows from one rest do not count as independent trials.

B is the fit spot. All scores at A use the coefficients from B, including a separate check at a nominal 90° heading omitted from training. A setup sweep at B followed straight travel and is excluded from both fit and scores. The paired-approach B0 CW run was taken on returning from A. The head targets and competing models were set before viewing the A results.

We compare the equivalent horizontal specific force in milligravity (mg). The raw acceleration is in the head frame. For every sample the encoder gives the head pitch θ, so the corrected forward reading is (ax cos θ − az sin θ)/9.81 mg with accelerations in mm/s². The leftward reading is ay/9.81 mg. We require zero reported wheel speeds and gyro norm below 0.05 rad/s. Each settled window contributes one two-dimensional mean, equally weighted. The camera uses the taped See3CAM’s calibrated lens and glyph geometry. At the first level-head rest in each sweep, the backpack indicator tip and vent center supply its heading. The stationarity check tracks a patch around the backpack tip at all six rests.

## Competing explanations

Each model includes a chassis-fixed constant. The head term allows a different residual at each commanded head position relative to the zero target, after applying the real encoder angle. The shelf term rotates one horizontal vector into the body frame using the optical heading of that sweep. The arrival term changes sign with the last turn. This separates patterns in the measurements; the shelf-frame coefficient is an *equivalent* inclination. Independent leveling of the shelf was reported by the operator through CPCS. Neither the static accelerometer nor a single overhead chassis landmark measures bare-shelf slope or tiny chassis pitch directly. A head-dependent residual could include head angle calibration, sensor bias or chassis pitch as the head shifts load. The arrival test covers these short positioning turns, not the longer endurance spins.

[Measured and predicted traces](../physics-plots/head-sweep-transfer.png) · [Raw head readings, corrected readings and model comparison](../physics-plots/head-sweep-components.png) · [Fit and transfer scores](transfer.csv) · [Settled SDK samples](samples.csv) · [Optical picks](marks.csv) · [Backpack motion check](stationarity.csv)

Reproduce the fits and figures with `python analyze_head_sweep.py` from the repository root. For the optical calibration see the [See3CAM heading crossover](../heading-crossover/README.md).
