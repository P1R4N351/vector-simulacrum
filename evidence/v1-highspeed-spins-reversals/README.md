# Unit 1 off-dock high-speed spins (`60–280 mm/s`) and zero-pause reversals

Visit C undocked `unit 1` (Vector 1.0) onto the flat workspace centerline (`499/837` ZED 2 VGA frames) and ran twelve `1.4 s` steady open-loop spins at `60, 100, 150, 200, 240, 280 mm/s` both directions, followed by zero-pause reversal triplets (`+100/-100/+100` and `+150/-150/+150 mm/s`) before a wheel-acceleration pulse at `acc = 100 mm/s^2` tripped the cliff sensor (`CLIFF_DETECTED`) and hardware-safe `drive_on_charger()` recovered the robot to its dock (`on_platform = True`, `charging = True`).

## Key findings

1. **Spin rate and yaw gain across `60–280 mm/s` (`steady-spins.csv`):**
   - Across `60–200 mm/s`, steady wheel speeds and yaw rates matched `VectorSim` (evaluated at reported post-undock `3.7188 V` and nominal resting `3.95 V`) within `0.3–3.8 mm/s` (`98–99%`).
   - Normalized yaw gain `|gz| / |w_mean|` stayed flat at `22.02–23.29 rad/m` (median `23.01 rad/m`, i.e., `11.50 rad/m` per side speed) across the entire `60–280 mm/s` envelope, matching canonical `VectorSim` (`22.35–23.44 rad/m`, `< 1.5%` discrepancy).
   - Both `240` and `280 mm/s` saturated cleanly at `235.5–238.7 mm/s` (`312.5–314.9 dps`), matching `VectorSim` at `3.95 V` (`234.8–235.2 mm/s`, `311.2–317.4 dps`).
2. **Cold-start onset equals zero-pause reversal latency (`reversals.csv`):**
   - Cold-start onset latency (`83–203 ms`, median `176 ms`) was statistically indistinguishable from zero-pause sign-reversal zero-crossing latency (`119–209 ms`, median `176 ms`), showing the latency beyond `command_delay_ms` reflects wireless SDK/bridge transport rather than tread static breakaway stiction.

## Reproduce

```bash
python3 evidence/v1-highspeed-spins-reversals/analyze.py
```

Reads [unit1-highspeed-spins-reversals.csv](unit1-highspeed-spins-reversals.csv) and [visitC-zed.jsonl](visitC-zed.jsonl) and writes [steady-spins.csv](steady-spins.csv) and [reversals.csv](reversals.csv).
