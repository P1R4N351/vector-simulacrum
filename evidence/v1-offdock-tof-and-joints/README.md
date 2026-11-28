# Unit 1 off-dock 1 mm ToF fork-occlusion scan and joint strokes

Visit B undocked `unit 1` (Vector 1.0) onto the centerline (`2536` SDK state rows, `1418/1536` ZED 2 VGA frames, battery `3.8254 V` on charger at start and end, `3.6025 V` undocked), swept the lift through `[32, 58] mm` facing both a near target (`72.5 mm`) and a far target (`200.5 mm`, after turning `-90°`), executed four off-dock `64 -> 52 mm` (`duration = 0.65 s`) lift lowers, and executed eight off-dock head moves to `+8°` (`12°` and `24°` spans).

## Key findings

1. **Multi-target optical mixing inside `is_lift_in_fov = [34, 56] mm` (`tof-near-far.csv`):**
   - `is_lift_in_fov` switched to `1` across the firmware `[34, 56] mm` band (`35.75..55.07 mm`), setting `found_object = 0`.
   - At `35.75 mm` (`near`: `73.4 mm, q = 0.193`) and `54.39–55.07 mm` (`near`: `76.1 mm, q = 0.197`; `far`: `201.0 mm, q = 0.052`), the crossbar cleared the VL53L0X optical cone completely while `ProxSensor` flatly clamped to `(30.0 mm, q = 11.7)`.
   - At `49.57–50.95 mm`, the upper edge of the crossbar partially intercepted the cone and mixed with the background target in signal-weighted photon proportion: `37.5 mm (q = 1.16)` against the `72.5 mm` near target vs `68.0 mm (q = 0.41)` against the `200.5 mm` far target.
2. **Off-dock `64 -> 52 mm` lift lowers match canonical `VectorSim` (`offdock-lift-lowers.csv`):**
   - Unlike the docked `64 -> 52 mm` stalls in `v1-docked-imu-speed-sweep`, off-dock lowers completed cleanly to `50.95–52.33 mm` (`sim 51.64 mm`) with `t90 = 767–780 ms` (`sim 720 ms`, `~300 ms` SDK transport onset included on the physical stream).
3. **Off-dock `+8°` head moves (`offdock-head-8deg.csv`):**
   - `12°` strokes (`-3° / +19° -> +8°`) reached `t90` in `353–415 ms` (`sim 360 ms`); `24°` strokes (`-16° / +32° -> +8°`) reached `t90` in `478–524 ms` (`sim 480 ms`).

## Reproduce

```bash
python3 evidence/v1-offdock-tof-and-joints/analyze.py
```

Reads [unit1-offdock-tof-joints.csv](unit1-offdock-tof-joints.csv) and [visitB-zed.jsonl](visitB-zed.jsonl) and writes [tof-near-far.csv](tof-near-far.csv), [offdock-lift-lowers.csv](offdock-lift-lowers.csv), and [offdock-head-8deg.csv](offdock-head-8deg.csv).
