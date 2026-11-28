"""Reproduce unit 1 stock-fork ToF occlusion, hysteresis, and passive calm-mode comparisons."""

from __future__ import annotations

import csv
import math
import statistics
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from simulacrum import autonomy, mjcf, params, prox, robot

G_MMPS2 = 9806.65
MAX_SIM_TICKS = 40000
MAX_CALM_SAMPLES = 5000


def load_rows(path: Path) -> List[Dict[str, str]]:
    """Load non-empty CSV state/event rows from one capture."""
    assert path.is_file(), f"missing capture file: {path}"
    with path.open("r", encoding="utf-8", newline="") as handle:
        rows = list(csv.DictReader(handle))
    assert len(rows) > 0, f"empty capture file: {path}"
    return rows


def make_docked_sim(model_name: str, head_deg: float, lift_mm: float,
                    calm_mode: bool = False, cube: Optional[str] = None,
                    seed: int = 42) -> robot.VectorSim:
    """Create a docked MuJoCo VectorSim with body-clock ToF enabled."""
    v = params.values(params.load(model=model_name))
    xml = mjcf.build_mjcf(v, (), cube=cube, dock="scene")
    sim = robot.VectorSim(values=v, noise_seed=seed, calm_mode=calm_mode,
                          control_level=robot.CONTROL_OVERRIDE)
    sim.model = mujoco.MjModel.from_xml_string(xml)
    sim.data = mujoco.MjData(sim.model)
    sim._index_model()
    sim._init_commands()
    sim.control_level = robot.CONTROL_OVERRIDE
    sim.has_control = True
    sim._cancelled = False
    autonomy._place_docked(sim, head_deg, lift_mm)
    sim.body_range_clock = True
    sim._settle()
    sim.has_control = True
    sim._cancelled = False
    autonomy.sync_dock_contact(sim)
    sim._publish_state()
    return sim


def extract_fresh_tof(states: List[Dict[str, str]]) -> List[Dict[str, Any]]:
    """Select stationary SDK rows immediately following a fresh ToF update."""
    fresh: List[Dict[str, Any]] = []
    for i in range(1, len(states)):
        prev_r, cur_r = states[i - 1], states[i]
        changed = (cur_r["prox_mm"] != prev_r["prox_mm"]
                   or cur_r["prox_q"] != prev_r["prox_q"])
        if not changed:
            continue
        t_cur = float(cur_r["s"])
        near = [float(r["lift_mm"]) for r in states
                if t_cur - 0.18 <= float(r["s"]) <= t_cur + 0.02]
        if near and (max(near) - min(near) < 0.4):
            fresh.append({
                "lift_mm": float(cur_r["lift_mm"]),
                "in_fov": int(cur_r["lift_in_fov"]),
                "found": int(cur_r["found_obj"]),
                "prox_mm": float(cur_r["prox_mm"]),
                "prox_q": float(cur_r["prox_q"]),
            })
    assert len(fresh) >= 20, "too few stationary fresh ToF samples"
    return fresh


def summarize_tof_curve(rows: List[Dict[str, str]]) -> List[Dict[str, Any]]:
    """Bucket fresh ToF observations by settled encoder lift height and compare against canonical 1.0."""
    states = [r for r in rows if r["event"] == "state"]
    fresh = extract_fresh_tof(states)
    buckets: Dict[float, List[Dict[str, Any]]] = {}
    for item in fresh:
        buckets.setdefault(round(item["lift_mm"], 2), []).append(item)
    v1 = params.values(params.load(model="vector_1"))
    pitch_dock = float(states[0]["pitch_rad"])
    out: List[Dict[str, Any]] = []
    for height_mm in sorted(buckets):
        grp = buckets[height_mm]
        ps = prox.ProxSensor(v1, noise_seed=1)
        rd = ps.read([180.0] * len(ps.rays), height_mm, pitch_dock)
        qs = [x["prox_q"] for x in grp]
        ds = [x["prox_mm"] for x in grp]
        out.append({
            "lift_mm": f"{height_mm:.2f}",
            "n": len(grp),
            "phys_in_fov": f"{np.mean([x['in_fov'] for x in grp]):.3f}",
            "phys_found": f"{np.mean([x['found'] for x in grp]):.3f}",
            "phys_prox_mm_med": f"{np.median(ds):.1f}",
            "phys_prox_q_med": f"{np.median(qs):.4f}",
            "phys_prox_q_min": f"{min(qs):.4f}",
            "phys_prox_q_max": f"{max(qs):.4f}",
            "sim_in_fov": int(rd.is_lift_in_fov),
            "sim_found": int(rd.found_object),
            "sim_prox_mm": f"{rd.distance_mm:.1f}",
            "sim_prox_q": f"{rd.signal_quality:.4f}",
        })
    return out


def replay_hysteresis(rows: List[Dict[str, str]]) -> List[Dict[str, Any]]:
    """Replay the 0/25/50/75/100% duration=0.7s lift hysteresis section on docked Vector 1.0."""
    states = [r for r in rows if r["event"] == "state"]
    cmds = [r for r in rows if r["event"] == "command_lift" and r["param"].startswith("hyst_")]
    rets = [r for r in rows if r["event"] == "return_lift" and r["param"].startswith("hyst_")]
    assert len(cmds) == len(rets) and len(cmds) > 0, "missing hysteresis commands"
    t_start = float(cmds[0]["s"]) - 0.5
    sub_states = [r for r in states if float(r["s"]) >= t_start]
    sim = make_docked_sim("vector_1", float(sub_states[0]["head_deg"]),
                          float(sub_states[0]["lift_mm"]), cube=None, seed=42)
    sim_series = _step_lift_replay(sim, sub_states, cmds)
    out: List[Dict[str, Any]] = []
    for i, (cmd, ret) in enumerate(zip(cmds, rets)):
        t_ret = float(ret["s"])
        t_next = float(cmds[i + 1]["s"]) if i + 1 < len(cmds) else float(sub_states[-1]["s"])
        hi = min(t_ret + 0.95, t_next - 0.05)
        p_vals = [float(r["lift_mm"]) for r in sub_states if t_ret + 0.2 <= float(r["s"]) <= hi]
        s_vals = [float(r["lift_mm"]) for r in sim_series if t_ret + 0.2 <= float(r["s"]) <= hi]
        tgt = float(cmd["target"])
        p_med, s_med = float(np.median(p_vals)), float(np.median(s_vals))
        out.append({
            "trial": cmd["trial"], "target_mm": f"{tgt:.1f}",
            "phys_med_mm": f"{p_med:.2f}", "phys_err_mm": f"{p_med - tgt:+.2f}",
            "sim_med_mm": f"{s_med:.2f}", "sim_err_mm": f"{s_med - tgt:+.2f}",
            "sim_minus_phys_mm": f"{s_med - p_med:+.2f}",
        })
    return out


def _step_lift_replay(sim: robot.VectorSim, states: List[Dict[str, str]],
                      cmds: List[Dict[str, str]]) -> List[Dict[str, float]]:
    """Advance docked sim across the observed state timestamps."""
    t0 = float(states[0]["s"])
    next_cmd = 0
    out: List[Dict[str, float]] = []
    for row in states:
        target_t = float(row["s"]) - t0
        for _ in range(MAX_SIM_TICKS):
            if sim.tick * sim.dt >= target_t - 1e-6:
                break
            tc = t0 + sim.tick * sim.dt
            for _j in range(16):
                if next_cmd >= len(cmds) or float(cmds[next_cmd]["s"]) > tc + 1e-6:
                    break
                sim.command_lift_height(float(cmds[next_cmd]["target"]), duration_s=0.7)
                next_cmd += 1
            sim.step(1)
            autonomy.sync_dock_contact(sim)
        assert sim.charger_contact, "simulator lost charger during docked lift replay"
        out.append({"s": float(row["s"]), "lift_mm": float(sim.state.lift_height_mm)})
    return out


def _fresh_imu_mask(states: List[Dict[str, Any]], is_sim: bool) -> List[int]:
    """Indices where the accelerometer or gyro changed from the prior sample."""
    idx = [0]
    for i in range(1, len(states)):
        if is_sim:
            a0, a1 = states[i - 1].accel_mmps2, states[i].accel_mmps2
            if a1 != a0:
                idx.append(i)
        else:
            s0, s1 = states[i - 1], states[i]
            if any(s1[k] != s0[k] for k in ("ax", "ay", "az", "gx", "gy", "gz")):
                idx.append(i)
    return idx


def summarize_calm_capture(label: str, path: Path, model_name: str,
                           cube: Optional[str]) -> Dict[str, Any]:
    """Compare a passive calm-mode capture against canonical calm_mode VectorSim."""
    rows = load_rows(path)
    st = [r for r in rows if r["event"] == "state"]
    assert all(int(r["status"]) & 0x1000 for r in st), f"{label} left charger"
    assert all(float(r["wl"]) == 0.0 and float(r["wr"]) == 0.0 for r in st), f"{label} wheels moved"
    fi = _fresh_imu_mask(st, is_sim=False)
    fdt = [float(st[fi[k]]["s"]) - float(st[fi[k - 1]]["s"]) for k in range(1, len(fi))]
    ax = np.array([float(st[i]["ax"]) / G_MMPS2 for i in fi]) * 1000.0
    ay = np.array([float(st[i]["ay"]) / G_MMPS2 for i in fi]) * 1000.0
    az = np.array([float(st[i]["az"]) / G_MMPS2 for i in fi]) * 1000.0
    ut = [0] + [i for i in range(1, len(st))
                if st[i]["prox_mm"] != st[i - 1]["prox_mm"] or st[i]["prox_q"] != st[i - 1]["prox_q"]]
    tdt = [float(st[ut[k]]["s"]) - float(st[ut[k - 1]]["s"]) for k in range(1, len(ut))]
    sim_stats = _run_calm_sim(model_name, st, cube)
    return {
        "capture": label, "packets": len(st),
        "phys_fresh_imu_pct": f"{100.0 * len(fi) / len(st):.2f}",
        "phys_imu_dt_ms": f"{statistics.median(fdt) * 1000.0:.2f}",
        "phys_x_sd_mg": f"{ax.std(ddof=1):.3f}",
        "phys_y_sd_mg": f"{ay.std(ddof=1):.3f}",
        "phys_z_sd_mg": f"{az.std(ddof=1):.3f}",
        "phys_tof_updates": len(ut),
        "phys_tof_dt_ms": f"{statistics.median(tdt) * 1000.0:.1f}",
        "phys_prox_med_mm": f"{np.median([float(r['prox_mm']) for r in st]):.1f}",
        "phys_prox_sd_mm": f"{np.std([float(r['prox_mm']) for r in st], ddof=1):.2f}",
        "phys_prox_q_med": f"{np.median([float(r['prox_q']) for r in st]):.5f}",
        **sim_stats,
    }


def _run_calm_sim(model_name: str, st: List[Dict[str, str]],
                  cube: Optional[str]) -> Dict[str, Any]:
    """Step calm_mode VectorSim through passive observation timestamps."""
    sim = make_docked_sim(model_name, float(st[0]["head_deg"]),
                          float(st[0]["lift_mm"]), calm_mode=True, cube=cube, seed=42)
    t0 = float(st[0]["s"])
    sim_rows: List[robot.RobotState] = []
    assert len(st) <= MAX_CALM_SAMPLES, "passive capture too long"
    for r in st:
        target_t = float(r["s"]) - t0
        for _ in range(MAX_SIM_TICKS):
            if sim.tick * sim.dt >= target_t - 1e-6:
                break
            sim.step(1)
            autonomy.sync_dock_contact(sim)
        sim_rows.append(sim.state)
    fi_s = _fresh_imu_mask(sim_rows, is_sim=True)
    ax_s = np.array([sim_rows[i].accel_mmps2[0] / G_MMPS2 for i in fi_s]) * 1000.0
    ut_s = [0] + [i for i in range(1, len(sim_rows))
                  if sim_rows[i].prox_distance_mm != sim_rows[i - 1].prox_distance_mm
                  or sim_rows[i].prox_signal_quality != sim_rows[i - 1].prox_signal_quality]
    pm_s = np.array([s.prox_distance_mm for s in sim_rows])
    pq_s = np.array([s.prox_signal_quality for s in sim_rows])
    return {
        "sim_fresh_imu_pct": f"{100.0 * len(fi_s) / len(sim_rows):.2f}",
        "sim_x_sd_mg": f"{ax_s.std(ddof=1):.3f}",
        "sim_tof_updates": len(ut_s),
        "sim_prox_med_mm": f"{np.median(pm_s):.1f}",
        "sim_prox_sd_mm": f"{pm_s.std(ddof=1):.2f}",
        "sim_prox_q_med": f"{np.median(pq_s):.5f}",
    }


def write_csv(path: Path, rows: List[Dict[str, Any]]) -> None:
    """Write a non-empty list of uniform dictionaries to CSV."""
    assert len(rows) > 0, f"no rows to write for {path}"
    cols = list(rows[0].keys())
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=cols, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    """Run all unit 1 ToF/hysteresis and unit 1/3 passive calm-mode analyses."""
    tof_rows = load_rows(HERE / "unit1-fork-tof-sweep.csv")
    tof_curve = summarize_tof_curve(tof_rows)
    hyst_table = replay_hysteresis(tof_rows)
    calm_table = [
        summarize_calm_capture("unit1_v1_asleep_20s", HERE / "unit1-passive-asleep.csv", "vector_1", None),
        summarize_calm_capture("unit3_v2_calm_60s", HERE / "unit3-passive-calm.csv", "vector_2", None),
    ]
    write_csv(HERE / "tof-curve.csv", tof_curve)
    write_csv(HERE / "hysteresis-replay.csv", hyst_table)
    write_csv(HERE / "calm-summary.csv", calm_table)
    print(f"tof_curve={len(tof_curve)} rows, hyst={len(hyst_table)} rows, calm={len(calm_table)} rows")


if __name__ == "__main__":
    main()
