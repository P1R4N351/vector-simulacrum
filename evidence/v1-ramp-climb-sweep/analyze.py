"""Reproduce unit 1 controlled reverse ramp climbs vs. ZED 2 stereo and canonical MuJoCo 4.0.1."""

from __future__ import annotations

import csv
import json
import math
import sys
from pathlib import Path
from typing import Any, Dict, List, Sequence

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from simulacrum import autonomy, robot

MAX_CLIMB_TICKS = 4000


def load_sdk_rows(path: Path) -> List[Dict[str, str]]:
    """Load non-empty SDK CSV rows."""
    assert path.is_file(), f"missing SDK CSV: {path}"
    with path.open("r", encoding="utf-8", newline="") as handle:
        rows = list(csv.DictReader(handle))
    assert len(rows) > 100, f"too few SDK rows in {path}"
    return rows


def load_zed_rows(path: Path) -> List[Dict[str, Any]]:
    """Load valid ZED stereo pose frames."""
    assert path.is_file(), f"missing ZED jsonl: {path}"
    rows = [json.loads(line) for line in path.open("r", encoding="utf-8") if line.strip()]
    found = [r for r in rows if r.get("found")]
    assert len(found) > 100, f"too few found ZED frames in {path}"
    return found


def _sim_climb(spd: float, max_dur: float) -> Dict[str, Any]:
    """Run one reverse climb on canonical Vector 1.0 starting clear of the dock apron."""
    assert spd > 0.0 and max_dur > 0.0, "climb speed and duration must be positive"
    sim = autonomy.make_dock_sim(start_docked=True, head_deg=0.0, lift_mm=32.0, noise_seed=42)
    sim.control_level = robot.CONTROL_OVERRIDE
    sim.has_control = True
    sim._cancelled = False
    sim.drive_straight(95.0, 40.0)
    sim.advance(0.4)
    autonomy.sync_dock_contact(sim)
    x0 = sim.pose_mm()[0]
    p0 = math.degrees(sim._pitch())
    sim.set_wheel_motors(-spd, -spd)
    ticks = min(MAX_CLIMB_TICKS, int(round(max_dur / sim.dt)))
    for _ in range(ticks):
        sim.step(1)
        autonomy.sync_dock_contact(sim)
        if sim.charger_contact:
            break
    sim.set_wheel_motors(0.0, 0.0)
    sim.advance(0.6)
    autonomy.sync_dock_contact(sim)
    return {
        "sim_mounted": int(sim.charger_contact),
        "sim_dx_mm": sim.pose_mm()[0] - x0,
        "sim_pitch_start_deg": p0,
        "sim_pitch_end_deg": math.degrees(sim._pitch()),
    }


def analyze_climbs(rows: Sequence[Dict[str, str]],
                   zed: Sequence[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """Compare physical ramp climb attempts with ZED optical travel and canonical simulation."""
    states = [r for r in rows if r["event"] == "state"]
    cmds = [r for r in rows if r["event"] == "wheel_cmd"]
    stops = [r for r in rows if r["event"] == "wheel_stop"]
    assert len(cmds) == 4 and len(stops) == 4, "expected 4 ramp climb attempts"
    out: List[Dict[str, Any]] = []
    for cmd, stop in zip(cmds, stops):
        tc, ts = float(cmd["s"]), float(stop["s"])
        tc_w, ts_w = float(cmd["wall_s"]), float(stop["wall_s"])
        spd = abs(float(cmd["detail"].split("=")[1].split()[0]))
        max_dur = float(cmd["detail"].split("dur=")[1].split()[0])
        mounted = int("mounted=1" in stop["detail"])
        sub = [r for r in states if tc - 0.1 <= float(r["s"]) <= ts + 0.5]
        ws = [0.5 * (float(r["wl"]) + float(r["wr"])) for r in sub]
        t_s = [float(r["s"]) for r in sub]
        odo_trav = sum(0.5 * (ws[k] + ws[k + 1]) * (t_s[k + 1] - t_s[k]) for k in range(len(ws) - 1))
        zp0 = [z for z in zed if tc_w - 0.5 <= z["wall"] <= tc_w + 0.1]
        zp1 = [z for z in zed if ts_w + 0.2 <= z["wall"] <= ts_w + 0.8]
        zx0 = float(np.median([z["dock_x_m"] * 1000.0 for z in zp0])) if zp0 else float("nan")
        zx1 = float(np.median([z["dock_x_m"] * 1000.0 for z in zp1])) if zp1 else float("nan")
        p0 = math.degrees(float(sub[0]["pitch_rad"]))
        p1 = math.degrees(float(sub[-1]["pitch_rad"]))
        pmin = min(math.degrees(float(r["pitch_rad"])) for r in sub)
        sim_res = _sim_climb(spd, max_dur)
        out.append({
            "phase": cmd["phase"], "cmd_mmps": f"-{spd:.0f}", "dur_s": f"{ts - tc:.2f}",
            "phys_mounted": mounted, "phys_odo_mm": f"{odo_trav:+.1f}",
            "zed_dock_start_mm": f"{zx0:.1f}", "zed_dock_end_mm": f"{zx1:.1f}",
            "zed_dx_mm": f"{zx1 - zx0:+.1f}",
            "phys_pitch_start_deg": f"{p0:.2f}", "phys_pitch_min_deg": f"{pmin:.2f}",
            "phys_pitch_end_deg": f"{p1:.2f}",
            "phys_yaw_start_deg": f"{math.degrees(float(sub[0]['yaw_rad'])):+.1f}",
            "phys_yaw_end_deg": f"{math.degrees(float(sub[-1]['yaw_rad'])):+.1f}",
            "sim_mounted": sim_res["sim_mounted"],
            "sim_dx_mm": f"{sim_res['sim_dx_mm']:+.1f}",
            "sim_pitch_end_deg": f"{sim_res['sim_pitch_end_deg']:.2f}",
        })
    return out


def write_csv(path: Path, rows: List[Dict[str, Any]]) -> None:
    """Write uniform list of dicts to CSV."""
    assert len(rows) > 0, f"no rows for {path}"
    cols = list(rows[0].keys())
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=cols, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    """Run ramp-climb sweep comparison against ZED stereo and MuJoCo 4.0.1."""
    rows = load_sdk_rows(HERE / "unit1-ramp-climb.csv")
    zed = load_zed_rows(HERE / "visitA-zed.jsonl")
    table = analyze_climbs(rows, zed)
    write_csv(HERE / "ramp-climb-scores.csv", table)
    print(f"wrote {len(table)} ramp climb rows")


if __name__ == "__main__":
    main()
