"""Reproduce unit 1 off-dock drive speed steps (40..160 mm/s) and spins (60..200 mm/s)."""

from __future__ import annotations

import csv
import json
import math
import statistics
import sys
from pathlib import Path
from typing import Any, Dict, List, Sequence

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from simulacrum import params, robot


def load_sdk_rows(path: Path) -> List[Dict[str, str]]:
    """Load non-empty SDK CSV rows."""
    assert path.is_file(), f"missing SDK CSV: {path}"
    with path.open("r", encoding="utf-8", newline="") as handle:
        rows = list(csv.DictReader(handle))
    assert len(rows) > 100, f"too few SDK rows in {path}"
    return rows


def load_zed_rows(path: Path) -> List[Dict[str, Any]]:
    """Load ZED stereo pose frames."""
    assert path.is_file(), f"missing ZED jsonl: {path}"
    rows = [json.loads(line) for line in path.open("r", encoding="utf-8") if line.strip()]
    found = [r for r in rows if r.get("found")]
    assert len(found) > 100, f"too few found ZED frames in {path}"
    return found


def score_straight_pulses(rows: Sequence[Dict[str, str]],
                          zed: Sequence[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """Compare 40/80/120/160 mm/s forward and reverse pulses against canonical VectorSim."""
    states = [r for r in rows if r["event"] == "state"]
    cmds = [r for r in rows if r["event"] == "wheel_cmd"][:8]
    stops = [r for r in rows if r["event"] == "wheel_stop"][:8]
    assert len(cmds) == 8 and len(stops) == 8, "expected 8 straight wheel pulses"
    v1 = params.values(params.load(model="vector_1"))
    out: List[Dict[str, Any]] = []
    for cmd, stop in zip(cmds, stops):
        tc, ts = float(cmd["wall_s"]), float(stop["wall_s"])
        dur = float(stop["s"]) - float(cmd["s"])
        spd = float(cmd["detail"].split("=")[1].split()[0])
        pre = [r for r in states if tc - 0.45 <= float(r["wall_s"]) <= tc]
        post = [r for r in states if ts + 0.35 <= float(r["wall_s"]) <= ts + 0.85]
        mid = [r for r in states if tc + 0.5 * (ts - tc) <= float(r["wall_s"]) <= ts]
        odo_dist = math.hypot(np.median([float(r["x_mm"]) for r in post]) - np.median([float(r["x_mm"]) for r in pre]),
                              np.median([float(r["y_mm"]) for r in post]) - np.median([float(r["y_mm"]) for r in pre]))
        w_mid = float(np.mean([0.5 * (float(r["wl"]) + float(r["wr"])) for r in mid]))
        w_peak = max(abs(0.5 * (float(r["wl"]) + float(r["wr"]))) for r in states if tc <= float(r["wall_s"]) <= ts + 0.1)
        sim_w, sim_dist = _sim_straight_pulse(v1, spd, dur)
        zp0 = [z for z in zed if tc - 0.55 <= z["wall"] <= tc]
        zp1 = [z for z in zed if ts + 0.30 <= z["wall"] <= ts + 0.85]
        dzed = (math.hypot(np.median([z["x_m"] for z in zp1]) - np.median([z["x_m"] for z in zp0]),
                           np.median([z["y_m"] for z in zp1]) - np.median([z["y_m"] for z in zp0])) * 1000.0
                if zp0 and zp1 else float("nan"))
        out.append({
            "phase": cmd["phase"], "cmd_mmps": f"{spd:+.0f}", "dur_s": f"{dur:.3f}",
            "phys_w_mid_mmps": f"{w_mid:+.1f}", "sim_w_mid_mmps": f"{sim_w:+.1f}",
            "phys_w_peak_mmps": f"{w_peak:.1f}", "phys_odo_mm": f"{odo_dist:.1f}",
            "sim_dist_mm": f"{sim_dist:.1f}", "zed_sil_mm": f"{dzed:.1f}",
        })
    return out


def _sim_straight_pulse(v1: Dict[str, Any], spd: float, dur: float) -> tuple[float, float]:
    """Run one isolated straight pulse on canonical Vector 1.0."""
    sim = robot.VectorSim(values=v1, control_level=robot.CONTROL_OVERRIDE, noise_seed=42)
    sim.battery_volts = 3.8035
    sim.has_control = True
    sim._cancelled = False
    sim.set_wheel_motors(spd, spd)
    sim.advance(dur)
    sim.set_wheel_motors(0.0, 0.0)
    sim.advance(0.6)
    x_mm, _, _ = sim.pose_mm()
    s_mid = [s for s in sim.states if 0.5 * dur <= s.time_s <= dur]
    sw = float(np.mean([0.5 * (s.left_wheel_speed_mmps + s.right_wheel_speed_mmps) for s in s_mid]))
    return sw, abs(x_mm)


def score_spin_pulses(rows: Sequence[Dict[str, str]]) -> List[Dict[str, Any]]:
    """Compare alternating 60/100/150/200 mm/s spins against canonical VectorSim."""
    states = [r for r in rows if r["event"] == "state"]
    cmds = [r for r in rows if r["event"] == "wheel_cmd"][8:]
    stops = [r for r in rows if r["event"] == "wheel_stop"][8:]
    assert len(cmds) == 8 and len(stops) == 8, "expected 8 spin pulses"
    v1 = params.values(params.load(model="vector_1"))
    head_off = math.radians(v1["imu_pitch_offset_deg"])
    out: List[Dict[str, Any]] = []
    for cmd, stop in zip(cmds, stops):
        tc_rel, ts_rel = float(cmd["s"]), float(stop["s"])
        dur = ts_rel - tc_rel
        spd = float(cmd["detail"].split("=")[1].split()[0])
        mid = [r for r in states if tc_rel + 0.5 * dur <= float(r["s"]) <= ts_rel]
        assert len(mid) >= 3, f"too few steady spin samples for {cmd['phase']}"
        yaw_p = float(np.mean([float(r["gz"]) * math.cos(math.radians(float(r["head_deg"])) + head_off)
                               - float(r["gx"]) * math.sin(math.radians(float(r["head_deg"])) + head_off) for r in mid]))
        wd_p = float(np.mean([float(r["wr"]) - float(r["wl"]) for r in mid]))
        wd_s, yaw_s = _sim_spin_pulse(v1, spd, dur, head_off)
        out.append({
            "phase": cmd["phase"], "cmd_mmps": f"{spd:+.0f}", "dur_s": f"{dur:.3f}",
            "phys_wdiff_mmps": f"{wd_p:+.1f}", "sim_wdiff_mmps": f"{wd_s:+.1f}",
            "phys_yaw_radps": f"{yaw_p:+.3f}", "sim_yaw_radps": f"{yaw_s:+.3f}",
            "phys_gain_rad_per_m": f"{abs(yaw_p) / abs(wd_p) * 1000.0:.2f}",
            "sim_gain_rad_per_m": f"{abs(yaw_s) / abs(wd_s) * 1000.0:.2f}",
            "raw_yaw_err_pct": f"{100.0 * (abs(yaw_s) - abs(yaw_p)) / abs(yaw_p):+.2f}",
        })
    return out


def _sim_spin_pulse(v1: Dict[str, Any], spd: float, dur: float, head_off: float) -> tuple[float, float]:
    """Run one isolated alternating-wheel spin on canonical Vector 1.0."""
    sim = robot.VectorSim(values=v1, control_level=robot.CONTROL_OVERRIDE, noise_seed=42)
    sim.battery_volts = 3.8035
    sim.has_control = True
    sim._cancelled = False
    sim.set_wheel_motors(-spd, spd)
    sim.advance(dur)
    s_mid = [s for s in sim.states if 0.5 * dur <= s.time_s <= dur]
    swd = float(np.mean([s.right_wheel_speed_mmps - s.left_wheel_speed_mmps for s in s_mid]))
    syaw = float(np.mean([s.gyro_radps[2] * math.cos(s.head_angle_rad + head_off)
                          - s.gyro_radps[0] * math.sin(s.head_angle_rad + head_off) for s in s_mid]))
    return swd, syaw


def summarize_undock_and_dock(rows: Sequence[Dict[str, str]],
                              zed: Sequence[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """Summarize ZED dock_x travel and SDK return-contact metrics for Visit 2."""
    states = [r for r in rows if r["event"] == "state"]
    u_rows = [r for r in states if r["phase"] == "undock"]
    zu = [z for z in zed if float(u_rows[0]["wall_s"]) <= z["wall"] <= float(u_rows[-1]["wall_s"])
          and z["source"] == "eyes"]
    d_rows = [r for r in states if r["phase"] == "dock_return"]
    first_on = next((i for i, r in enumerate(d_rows) if int(r["status"]) & 0x1000), None)
    assert first_on is not None and len(zu) >= 10, "missing undock ZED frames or charger contact"
    last_m5 = next(i for i in range(first_on, 0, -1)
                   if math.degrees(float(d_rows[i - 1]["pitch_rad"])) > -5.0
                   >= math.degrees(float(d_rows[i]["pitch_rad"])))
    sub_ret = d_rows[last_m5:first_on + 1]
    ws = [0.5 * (float(r["wl"]) + float(r["wr"])) for r in sub_ret]
    ts = [float(r["s"]) for r in sub_ret]
    trav = sum(0.5 * (ws[k] + ws[k + 1]) * (ts[k + 1] - ts[k]) for k in range(len(ws) - 1))
    w_fin = float(np.median([0.5 * (float(r["wl"]) + float(r["wr"])) for r in d_rows[max(0, first_on - 12):first_on]]))
    z_m5 = [z["dock_x_m"] * 1000.0 for z in zed if abs(z["wall"] - float(d_rows[last_m5]["wall_s"])) < 0.25]
    z_on = [z["dock_x_m"] * 1000.0 for z in zed if abs(z["wall"] - float(d_rows[first_on]["wall_s"])) < 0.25]
    return [{
        "undock_zed_start_mm": f"{zu[0]['dock_x_m'] * 1000.0:.1f}",
        "undock_zed_end_mm": f"{zu[-1]['dock_x_m'] * 1000.0:.1f}",
        "return_dt_m5_to_on_s": f"{float(d_rows[first_on]['s']) - float(d_rows[last_m5]['s']):.2f}",
        "return_odo_travel_mm": f"{trav:.1f}",
        "return_final_med_mmps": f"{w_fin:.1f}",
        "return_pitch_at_on_deg": f"{math.degrees(float(d_rows[first_on]['pitch_rad'])):.2f}",
        "return_zed_m5_mm": f"{statistics.median(z_m5):.1f}" if z_m5 else "nan",
        "return_zed_on_mm": f"{statistics.median(z_on):.1f}" if z_on else "nan",
    }]


def write_csv(path: Path, rows: List[Dict[str, Any]]) -> None:
    """Write uniform list of dicts to CSV."""
    assert len(rows) > 0, f"empty output for {path}"
    cols = list(rows[0].keys())
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=cols, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    """Run Visit 2 drive, spin, and dock-return comparisons."""
    rows = load_sdk_rows(HERE / "unit1-visit2-drive-spins.csv")
    zed = load_zed_rows(HERE / "visit2-zed.jsonl")
    drive_rows = score_straight_pulses(rows, zed)
    spin_rows = score_spin_pulses(rows)
    dock_rows = summarize_undock_and_dock(rows, zed)
    write_csv(HERE / "drive-pulses.csv", drive_rows)
    write_csv(HERE / "spin-pulses.csv", spin_rows)
    write_csv(HERE / "dock-return.csv", dock_rows)
    print(f"wrote {len(drive_rows)} drive rows, {len(spin_rows)} spin rows, {len(dock_rows)} dock summary")


if __name__ == "__main__":
    main()
