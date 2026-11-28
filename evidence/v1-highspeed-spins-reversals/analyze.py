"""Reproduce unit 1 off-dock high-speed spins (60..280 mm/s) and zero-pause reversals."""

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

MAX_SIM_TICKS = 4000
VOLT_REST_EQUIV = 3.95


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


def _sim_steady_spin(v1: Dict[str, Any], spd: float, sgn: int, volts: float) -> tuple[float, float, float]:
    """Run one 1.4 s open-loop spin in VectorSim at the given resting voltage."""
    cfg = dict(v1, battery_rest_voltage_v=volts, accel_noise_rms_mg=0.0, gyro_noise_rms_dps=0.0)
    sim = robot.VectorSim(values=cfg, control_level=robot.CONTROL_OVERRIDE, head_deg=0.0, lift_mm=32.0, noise_seed=0)
    sim.set_wheel_motors(-spd * sgn, spd * sgn)
    for _ in range(MAX_SIM_TICKS):
        if sim.tick * sim.dt >= 1.4:
            break
        sim.step(1)
    sub = [s for s in sim.states if 0.5 <= s.time_s <= 1.35]
    assert len(sub) > 10, "insufficient steady sim samples"
    sw = float(np.mean([0.5 * (abs(s.left_wheel_speed_mmps) + abs(s.right_wheel_speed_mmps)) for s in sub]))
    sgz = abs(float(np.mean([math.degrees(s.gyro_radps[2]) for s in sub])))
    sgain = math.radians(sgz) / (0.001 * sw) if sw > 0.0 else 0.0
    return sw, sgz, sgain


def analyze_steady_spins(rows: Sequence[Dict[str, str]]) -> List[Dict[str, Any]]:
    """Score steady 60..280 mm/s spins and normalized yaw gain vs VectorSim."""
    states = [r for r in rows if r["event"] == "state"]
    cmds = [r for r in rows if r["event"] == "wheel_cmd" and r["phase"].startswith("steady_")]
    assert len(cmds) == 12, "expected 12 steady spin commands"
    v1 = params.values(params.load(model="vector_1"))
    out: List[Dict[str, Any]] = []
    for cmd in cmds:
        tc = float(cmd["s"])
        signed = float(cmd["detail"].split("=")[1].split()[0])
        spd, sgn = abs(signed), (1 if signed > 0.0 else -1)
        sub = [r for r in states if tc + 0.5 <= float(r["s"]) <= tc + 1.35]
        post = [r for r in states if float(r["s"]) >= tc]
        assert len(sub) >= 10, f"missing steady window for {cmd['phase']}"
        wl = statistics.mean(float(r["wl"]) for r in sub)
        wr = statistics.mean(float(r["wr"]) for r in sub)
        gz = abs(statistics.mean(math.degrees(float(r["gz"])) for r in sub))
        w_abs = 0.5 * (abs(wl) + abs(wr))
        gain = math.radians(gz) / (0.001 * w_abs) if w_abs > 0.0 else 0.0
        onset = next((float(r["s"]) - tc for r in post if abs(float(r["wl"])) > 5.0 or abs(float(r["wr"])) > 5.0),
                     float("nan"))
        sw_l, sgz_l, sgain_l = _sim_steady_spin(v1, spd, sgn, 3.7188)
        sw_n, sgz_n, _ = _sim_steady_spin(v1, spd, sgn, VOLT_REST_EQUIV)
        out.append({
            "phase": cmd["phase"], "cmd_mmps": f"{signed:+.0f}", "onset_ms": f"{onset * 1000.0:.0f}",
            "phys_w_mmps": f"{w_abs:.1f}", "sim37_w_mmps": f"{sw_l:.1f}", "sim395_w_mmps": f"{sw_n:.1f}",
            "phys_gz_dps": f"{gz:.1f}", "sim37_gz_dps": f"{sgz_l:.1f}", "sim395_gz_dps": f"{sgz_n:.1f}",
            "phys_gain_radpm": f"{gain:.2f}", "sim_gain_radpm": f"{sgain_l:.2f}",
        })
    return out


def analyze_reversals(rows: Sequence[Dict[str, str]]) -> List[Dict[str, Any]]:
    """Measure zero-crossing and steady speed across zero-pause +100/-100/+100 and +150/-150/+150 reversals."""
    states = [r for r in rows if r["event"] == "state"]
    cmds = [r for r in rows if r["event"] == "wheel_cmd" and r["phase"].startswith("rev_")]
    assert len(cmds) == 6, "expected 6 reversal pulse commands"
    out: List[Dict[str, Any]] = []
    for cmd in cmds:
        tc = float(cmd["s"])
        sgn = 1 if "+" in cmd["detail"] else -1
        post = [r for r in states if float(r["s"]) >= tc]
        sub = [r for r in states if tc + 0.45 <= float(r["s"]) <= tc + 0.80]
        t_cross = next((float(r["s"]) - tc for r in post if float(r["wr"]) * sgn > 5.0), float("nan"))
        wr = statistics.mean(float(r["wr"]) for r in sub)
        gz = statistics.mean(math.degrees(float(r["gz"])) for r in sub)
        out.append({
            "phase": cmd["phase"], "cmd": cmd["detail"].split()[0],
            "cross_ms": f"{t_cross * 1000.0:.0f}", "wr_steady_mmps": f"{wr:+.1f}", "gz_steady_dps": f"{gz:+.1f}",
        })
    return out


def write_csv(path: Path, rows: List[Dict[str, Any]]) -> None:
    """Write uniform list of dicts to CSV."""
    assert len(rows) > 0, f"no rows for {path}"
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def main() -> int:
    """Run Visit C spin, reversal, and ceiling analyses."""
    sdk_rows = load_sdk_rows(HERE / "unit1-highspeed-spins-reversals.csv")
    zed_rows = load_zed_rows(HERE / "visitC-zed.jsonl")
    spins = analyze_steady_spins(sdk_rows)
    revs = analyze_reversals(sdk_rows)
    write_csv(HERE / "steady-spins.csv", spins)
    write_csv(HERE / "reversals.csv", revs)
    print(f"zed_found={len(zed_rows)} steady_spins={len(spins)} reversals={len(revs)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
