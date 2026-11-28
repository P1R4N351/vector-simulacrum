"""Reproduce unit 1 off-dock 1 mm ToF scan (near & far) and off-dock joint strokes."""

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

from simulacrum import params, prox, robot

MAX_SIM_TICKS = 4000


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


def analyze_tof_scans(rows: Sequence[Dict[str, str]]) -> List[Dict[str, Any]]:
    """Compare near ascending/descending and far ascending fork-ToF scans against ProxSensor."""
    states = [r for r in rows if r["event"] == "state"]
    v1 = params.values(params.load(model="vector_1"), {"tof_range_std_fraction": 0.0, "tof_weak_distance_sd_mm": 0.0})
    n_rays = len(prox.ray_directions(v1))
    out: List[Dict[str, Any]] = []
    for phase, wall_mm in (("tof_near_asc", 72.5), ("tof_near_desc", 73.0), ("tof_far_asc", 200.5)):
        cmds = [r for r in rows if r["phase"] == phase and r["event"] == "cmd_lift"]
        assert len(cmds) >= 12, f"incomplete ToF scan {phase}"
        for idx, cmd in enumerate(cmds):
            t0 = float(cmd["s"])
            t1 = float(cmds[idx + 1]["s"]) if idx + 1 < len(cmds) else t0 + 0.7
            win = [r for r in states if r["phase"] == phase and t0 + 0.35 <= float(r["s"]) <= t1 - 0.03]
            if not win:
                continue
            lh = float(np.mean([float(r["lift_mm"]) for r in win]))
            dm = float(np.mean([float(r["prox_mm"]) for r in win]))
            qm = float(np.mean([float(r["prox_q"]) for r in win]))
            fv = float(np.mean([float(r["lift_in_fov"]) for r in win]))
            fo = float(np.mean([float(r["found_obj"]) for r in win]))
            rd = prox.ProxSensor(v1, noise_seed=0).read([wall_mm] * n_rays, lh, 0.0)
            out.append({
                "phase": phase, "cmd_mm": cmd["detail"].split("=")[1],
                "wall_mm": f"{wall_mm:.1f}", "lift_mm": f"{lh:.2f}",
                "phys_dist_mm": f"{dm:.2f}", "sim_dist_mm": f"{rd.distance_mm:.2f}",
                "phys_q": f"{qm:.4f}", "sim_q": f"{rd.signal_quality:.4f}",
                "phys_fov": f"{fv:.2f}", "sim_fov": int(rd.is_lift_in_fov),
                "phys_found": f"{fo:.2f}", "sim_found": int(rd.found_object),
            })
    return out


def analyze_lift_lowers(rows: Sequence[Dict[str, str]]) -> List[Dict[str, Any]]:
    """Score off-dock 64->52 mm lift lowers (duration=0.65 s) against VectorSim."""
    states = [r for r in rows if r["event"] == "state"]
    cmds = [r for r in rows if r["event"] == "cmd_lift_lower"]
    assert len(cmds) == 4, "expected 4 off-dock lift lower commands"
    v1 = params.values(params.load(model="vector_1"))
    out: List[Dict[str, Any]] = []
    for idx, cmd in enumerate(cmds):
        tc = float(cmd["s"])
        post = [r for r in states if tc <= float(r["s"]) <= tc + 1.5]
        assert len(post) >= 10, "too few samples after lift lower"
        h0, h1 = float(post[0]["lift_mm"]), float(post[-1]["lift_mm"])
        t90 = next((float(r["s"]) - tc for r in post if float(r["lift_mm"]) <= h0 - 0.9 * (h0 - h1)), float("nan"))
        ton = next((float(r["s"]) - tc for r in post if abs(float(r["lift_mm"]) - h0) > 0.8), float("nan"))
        sim = robot.VectorSim(values=v1, head_deg=0.0, lift_mm=h0, noise_seed=idx)
        sim.command_lift_height(52.0, duration_s=0.65)
        for _ in range(MAX_SIM_TICKS):
            if sim.tick * sim.dt >= 1.5:
                break
            sim.step(1)
        st_sim = list(sim.states)
        sim_h1 = st_sim[-1].lift_height_mm
        sim_t90 = next((s.time_s for s in st_sim if s.lift_height_mm <= h0 - 0.9 * (h0 - sim_h1)), float("nan"))
        out.append({
            "cycle": idx, "start_mm": f"{h0:.2f}", "phys_end_mm": f"{h1:.2f}",
            "sim_end_mm": f"{sim_h1:.2f}", "phys_onset_ms": f"{ton * 1000.0:.0f}",
            "phys_t90_ms": f"{t90 * 1000.0:.0f}", "sim_t90_ms": f"{sim_t90 * 1000.0:.0f}",
        })
    return out


def analyze_head_moves(rows: Sequence[Dict[str, str]]) -> List[Dict[str, Any]]:
    """Score off-dock head transitions to +8 deg against VectorSim."""
    states = [r for r in rows if r["event"] == "state"]
    cmds = [r for r in rows if r["event"] == "cmd_head_8"]
    assert len(cmds) == 8, "expected 8 off-dock head commands to +8 deg"
    v1 = params.values(params.load(model="vector_1"))
    out: List[Dict[str, Any]] = []
    for idx, cmd in enumerate(cmds):
        tc = float(cmd["s"])
        post = [r for r in states if tc <= float(r["s"]) <= tc + 1.2]
        assert len(post) >= 10, "too few samples after head move"
        a0, a1 = float(post[0]["head_deg"]), float(post[-1]["head_deg"])
        t90 = next((float(r["s"]) - tc for r in post if abs(float(r["head_deg"]) - a1) <= 0.1 * abs(a1 - a0)),
                   float("nan"))
        ton = next((float(r["s"]) - tc for r in post if abs(float(r["head_deg"]) - a0) > 0.8), float("nan"))
        sim = robot.VectorSim(values=v1, head_deg=a0, lift_mm=32.0, noise_seed=idx)
        sim.command_head_angle(8.0)
        for _ in range(MAX_SIM_TICKS):
            if sim.tick * sim.dt >= 1.2:
                break
            sim.step(1)
        st_sim = list(sim.states)
        sim_a1 = math.degrees(st_sim[-1].head_angle_rad)
        sim_t90 = next((s.time_s for s in st_sim
                        if abs(math.degrees(s.head_angle_rad) - sim_a1) <= 0.1 * abs(sim_a1 - a0)), float("nan"))
        out.append({
            "trial": idx, "start_deg": f"{a0:+.2f}", "phys_end_deg": f"{a1:+.2f}",
            "sim_end_deg": f"{sim_a1:+.2f}", "phys_onset_ms": f"{ton * 1000.0:.0f}",
            "phys_t90_ms": f"{t90 * 1000.0:.0f}", "sim_t90_ms": f"{sim_t90 * 1000.0:.0f}",
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
    """Run all Visit B off-dock ToF and joint analyses."""
    sdk_rows = load_sdk_rows(HERE / "unit1-offdock-tof-joints.csv")
    zed_rows = load_zed_rows(HERE / "visitB-zed.jsonl")
    tof_rows = analyze_tof_scans(sdk_rows)
    lift_rows = analyze_lift_lowers(sdk_rows)
    head_rows = analyze_head_moves(sdk_rows)
    write_csv(HERE / "tof-near-far.csv", tof_rows)
    write_csv(HERE / "offdock-lift-lowers.csv", lift_rows)
    write_csv(HERE / "offdock-head-8deg.csv", head_rows)
    print(f"zed_found={len(zed_rows)} tof_steps={len(tof_rows)} lift={len(lift_rows)} head={len(head_rows)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
