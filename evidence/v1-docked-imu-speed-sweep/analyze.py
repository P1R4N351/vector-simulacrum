"""Reproduce unit 1 docked head gravity-vector fit, Sweep 3 replay, and joint speed matrix."""

from __future__ import annotations

import csv
import math
import statistics
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from simulacrum import autonomy, mjcf, params, robot

G_MMPS2 = 9806.65
MAX_SIM_TICKS = 45000


def load_rows(path: Path) -> List[Dict[str, str]]:
    """Load non-empty CSV rows from a recording."""
    assert path.is_file(), f"missing capture: {path}"
    with path.open("r", encoding="utf-8", newline="") as handle:
        rows = list(csv.DictReader(handle))
    assert len(rows) > 0, f"empty capture: {path}"
    return rows


def make_docked_sim(head_deg: float, lift_mm: float, seed: int = 42) -> robot.VectorSim:
    """Create a docked Vector 1.0 simulator ready for override joint commands."""
    v = params.values(params.load(model="vector_1"))
    xml = mjcf.build_mjcf(v, (), cube=None, dock="scene")
    sim = robot.VectorSim(values=v, noise_seed=seed, calm_mode=False,
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


def extract_holds(rows: List[Dict[str, str]]) -> List[Dict[str, Any]]:
    """Compute settled per-hold head angle and accelerometer means."""
    states = [r for r in rows if r["event"] == "state"]
    cmds = [r for r in rows if r["event"] == "command_head"]
    rets = [r for r in rows if r["event"] == "return_head"]
    assert len(cmds) == len(rets) and len(cmds) >= 20, "incomplete head sweep"
    holds: List[Dict[str, Any]] = []
    for i, (cmd, ret) in enumerate(zip(cmds, rets)):
        t_ret = float(ret["s"])
        t_next = float(cmds[i + 1]["s"]) if i + 1 < len(cmds) else float(states[-1]["s"])
        hi = min(t_ret + 2.3, t_next - 0.1)
        win = [r for r in states if t_ret + 0.8 <= float(r["s"]) <= hi and abs(float(r["gy"])) < 0.03]
        assert len(win) >= 5, f"insufficient settled samples for {cmd['trial']}"
        holds.append({
            "block": cmd["param"], "trial": cmd["trial"], "target_deg": float(cmd["target"]),
            "head_deg": float(np.mean([float(r["head_deg"]) for r in win])),
            "pitch_deg": math.degrees(float(np.mean([float(r["pitch_rad"]) for r in win]))),
            "ax": float(np.mean([float(r["ax"]) for r in win])),
            "ay": float(np.mean([float(r["ay"]) for r in win])),
            "az": float(np.mean([float(r["az"]) for r in win])),
            "n": len(win),
        })
    return holds


def _gravity_resid(p: np.ndarray, data: Sequence[Dict[str, Any]], free_kx: bool) -> np.ndarray:
    """Predicted minus measured specific force across head holds."""
    p0, bx, by, bz = p[0], p[1], p[2], p[3]
    kx = p[4] if free_kx else 1.0
    out: List[float] = []
    for h in data:
        theta = math.radians(h["head_deg"]) + p0
        out.extend([kx * G_MMPS2 * math.sin(theta) + bx - h["ax"],
                    by - h["ay"],
                    G_MMPS2 * math.cos(theta) + bz - h["az"]])
    return np.array(out, dtype=float)


def fit_gravity_model(train: Sequence[Dict[str, Any]], free_kx: bool) -> np.ndarray:
    """Gauss-Newton least squares for (dock+IMU pitch offset, bias_xyz, optional scale_x)."""
    p = np.array([-0.165, -400.0, 100.0, 200.0, 1.0], dtype=float)
    steps = [1e-6, 1e-3, 1e-3, 1e-3] + ([1e-6] if free_kx else [])
    ncols = len(steps)
    for _ in range(40):
        r0 = _gravity_resid(p, train, free_kx)
        jac = np.zeros((len(r0), ncols), dtype=float)
        for k, stp in enumerate(steps):
            q = p.copy()
            q[k] += stp
            jac[:, k] = (_gravity_resid(q, train, free_kx) - r0) / stp
        delta = np.linalg.lstsq(jac, -r0, rcond=None)[0]
        p[:ncols] += delta
        if float(np.max(np.abs(delta))) < 1e-9:
            break
    return p


def score_gravity_holds(holds: Sequence[Dict[str, Any]]) -> Tuple[List[Dict[str, Any]], Dict[str, Any]]:
    """Fit on Sweeps 1+2 and evaluate held-out residuals on Sweep 3."""
    train = [h for h in holds if h["block"] in ("sweep1_asc", "sweep2_desc")]
    test = [h for h in holds if h["block"] == "sweep3_holdout"]
    p_nom = fit_gravity_model(train, free_kx=False)
    p_kx = fit_gravity_model(train, free_kx=True)
    r_tr_kx = _gravity_resid(p_kx, train, True) * 1000.0 / G_MMPS2
    r_te_kx = _gravity_resid(p_kx, test, True) * 1000.0 / G_MMPS2
    r_tr_nom = _gravity_resid(p_nom, train, False) * 1000.0 / G_MMPS2
    r_te_nom = _gravity_resid(p_nom, test, False) * 1000.0 / G_MMPS2
    rows: List[Dict[str, Any]] = []
    for h in holds:
        th = math.radians(h["head_deg"]) + p_kx[0]
        rx = (p_kx[4] * G_MMPS2 * math.sin(th) + p_kx[1] - h["ax"]) * 1000.0 / G_MMPS2
        ry = (p_kx[2] - h["ay"]) * 1000.0 / G_MMPS2
        rz = (G_MMPS2 * math.cos(th) + p_kx[3] - h["az"]) * 1000.0 / G_MMPS2
        rows.append({
            "block": h["block"], "trial": h["trial"], "target_deg": f"{h['target_deg']:.1f}",
            "head_deg": f"{h['head_deg']:.2f}", "pitch_deg": f"{h['pitch_deg']:.2f}",
            "ax_mg": f"{h['ax'] * 1000.0 / G_MMPS2:.2f}",
            "ay_mg": f"{h['ay'] * 1000.0 / G_MMPS2:.2f}",
            "az_mg": f"{h['az'] * 1000.0 / G_MMPS2:.2f}",
            "res_x_mg": f"{rx:+.2f}", "res_y_mg": f"{ry:+.2f}", "res_z_mg": f"{rz:+.2f}",
        })
    summary = {
        "nom_train_rmse_mg": math.sqrt(float(np.mean(r_tr_nom ** 2))),
        "nom_test_rmse_mg": math.sqrt(float(np.mean(r_te_nom ** 2))),
        "kx_train_rmse_mg": math.sqrt(float(np.mean(r_tr_kx ** 2))),
        "kx_test_rmse_mg": math.sqrt(float(np.mean(r_te_kx ** 2))),
        "scale_x": float(p_kx[4]),
        "pitch0_deg": math.degrees(float(p_kx[0])),
        "bias_mg": [float(v) * 1000.0 / G_MMPS2 for v in p_kx[1:4]],
    }
    return rows, summary


def replay_head_sweep(rows: List[Dict[str, str]]) -> List[Dict[str, Any]]:
    """Replay Sweep 3 head moves through canonical docked MuJoCo Vector 1.0."""
    states = [r for r in rows if r["event"] == "state"]
    cmds = [r for r in rows if r["event"] == "command_head"]
    sim = make_docked_sim(float(states[0]["head_deg"]), float(states[0]["lift_mm"]))
    t0, nc = float(states[0]["s"]), 0
    sim_series: List[Dict[str, float]] = []
    for r in states:
        target_t = float(r["s"]) - t0
        for _ in range(MAX_SIM_TICKS):
            if sim.tick * sim.dt >= target_t - 1e-6:
                break
            tc = t0 + sim.tick * sim.dt
            for _j in range(8):
                if nc >= len(cmds) or float(cmds[nc]["s"]) > tc + 1e-6:
                    break
                sim.command_head_angle(float(cmds[nc]["target"]))
                nc += 1
            sim.step(1)
            autonomy.sync_dock_contact(sim)
        assert sim.charger_contact, "lost charger in head sweep replay"
        st = sim.state
        sim_series.append({"s": float(r["s"]), "head_deg": math.degrees(st.head_angle_rad),
                           "gy": float(st.gyro_radps[1])})
    return _score_sweep3_moves(states, cmds, sim_series)


def _score_sweep3_moves(states: List[Dict[str, str]], cmds: List[Dict[str, str]],
                        sim_series: List[Dict[str, float]]) -> List[Dict[str, Any]]:
    """Compute per-move head angle and gyro Y RMSE for held-out Sweep 3."""
    out: List[Dict[str, Any]] = []
    for i, cmd in enumerate(cmds):
        if cmd["param"] != "sweep3_holdout":
            continue
        tc = float(cmd["s"])
        tn = float(cmds[i + 1]["s"]) if i + 1 < len(cmds) else float(states[-1]["s"])
        hi = min(tc + 1.5, tn - 0.05)
        pairs = [(float(p["head_deg"]), float(p["gy"]), q["head_deg"], q["gy"])
                 for p, q in zip(states, sim_series) if tc <= float(p["s"]) <= hi]
        e_ang = math.sqrt(float(np.mean([(a - b) ** 2 for a, _, b, _ in pairs])))
        e_gy = math.sqrt(float(np.mean([(g - sg) ** 2 for _, g, _, sg in pairs])))
        out.append({"trial": cmd["trial"], "target_deg": f"{float(cmd['target']):.1f}",
                    "head_rmse_deg": f"{e_ang:.2f}", "gyro_y_rmse_radps": f"{e_gy:.3f}"})
    return out


def _crossing_ms(ts: Sequence[float], ys: Sequence[float], y0: float, exc: float, frac: float) -> float:
    """Linear-interpolated threshold crossing time in ms after command."""
    target_y = y0 + frac * exc
    for k in range(1, len(ys)):
        if (exc > 0.0 and ys[k] >= target_y > ys[k - 1]) or (exc < 0.0 and ys[k] <= target_y < ys[k - 1]):
            t_hit = ts[k - 1] + (target_y - ys[k - 1]) / (ys[k] - ys[k - 1]) * (ts[k] - ts[k - 1])
            return t_hit * 1000.0
    return float("nan")


def replay_speed_matrix(rows: List[Dict[str, str]]) -> List[Dict[str, Any]]:
    """Replay the multi-speed head and lift matrix and score 10-90% transit times and RMSE."""
    states = [r for r in rows if r["event"] == "state"]
    cmds = [r for r in rows if r["event"].startswith("command_")]
    sim = make_docked_sim(float(states[0]["head_deg"]), float(states[0]["lift_mm"]))
    t0, nc = float(states[0]["s"]), 0
    sim_series: List[Dict[str, float]] = []
    for r in states:
        target_t = float(r["s"]) - t0
        for _ in range(MAX_SIM_TICKS):
            if sim.tick * sim.dt >= target_t - 1e-6:
                break
            tc = t0 + sim.tick * sim.dt
            for _j in range(8):
                if nc >= len(cmds) or float(cmds[nc]["s"]) > tc + 1e-6:
                    break
                c = cmds[nc]
                nc += 1
                spd = float(c["param"].split("=")[1]) if "max_speed=" in c["param"] else None
                tgt = float(c["target"])
                if c["event"] == "command_head":
                    sim.command_head_angle(tgt) if spd is None else sim._start_head(tgt, spd, 0.0)
                else:
                    sim.command_lift_height(tgt) if spd is None else sim._start_lift(tgt, spd, 0.0)
            sim.step(1)
            autonomy.sync_dock_contact(sim)
        assert sim.charger_contact, "lost charger in speed matrix replay"
        st = sim.state
        sim_series.append({"s": float(r["s"]), "head_deg": math.degrees(st.head_angle_rad),
                           "lift_mm": float(st.lift_height_mm)})
    return _score_speed_cmds(states, cmds, sim_series)


def _score_speed_cmds(states: List[Dict[str, str]], cmds: List[Dict[str, str]],
                      sim_series: List[Dict[str, float]]) -> List[Dict[str, Any]]:
    """Tabulate physical vs simulated 10-90% duration, onset, and RMSE per speed trial."""
    out: List[Dict[str, Any]] = []
    for i, cmd in enumerate(cmds):
        if cmd["param"] == "prep":
            continue
        tc = float(cmd["s"])
        tn = float(cmds[i + 1]["s"]) if i + 1 < len(cmds) else float(states[-1]["s"])
        field = "head_deg" if cmd["event"] == "command_head" else "lift_mm"
        p10, p90, pend = _transit_stats(states, tc, tn, field)
        s10, s90, send = _transit_stats(sim_series, tc, tn, field)
        pairs = [(float(p[field]), float(q[field])) for p, q in zip(states, sim_series)
                 if tc <= float(p["s"]) <= min(tc + 1.5, tn - 0.05)]
        rmse = math.sqrt(float(np.mean([(a - b) ** 2 for a, b in pairs])))
        out.append({
            "trial": cmd["trial"], "param": cmd["param"], "target": cmd["target"],
            "phys_onset_ms": f"{p10:.0f}", "phys_10_90_ms": f"{p90 - p10:.0f}", "phys_end": f"{pend:.2f}",
            "sim_onset_ms": f"{s10:.0f}", "sim_10_90_ms": f"{s90 - s10:.0f}", "sim_end": f"{send:.2f}",
            "rmse_1p5s": f"{rmse:.2f}",
        })
    return out


def _transit_stats(series: Sequence[Dict[str, Any]], tc: float, tn: float, field: str) -> Tuple[float, float, float]:
    """Compute pre-move baseline, 10% and 90% crossing times (ms), and end settle value."""
    pre = [float(x[field]) for x in series if tc - 0.5 <= float(x["s"]) <= tc]
    win = [x for x in series if tc <= float(x["s"]) <= min(tc + 2.5, tn - 0.05)]
    y0 = float(np.median(pre))
    y1 = float(np.median([float(x[field]) for x in win[-6:]]))
    exc = y1 - y0
    ts = [float(x["s"]) - tc for x in win]
    ys = [float(x[field]) for x in win]
    return _crossing_ms(ts, ys, y0, exc, 0.1), _crossing_ms(ts, ys, y0, exc, 0.9), y1


def write_csv(path: Path, rows: List[Dict[str, Any]]) -> None:
    """Write uniform list of dicts to CSV."""
    assert len(rows) > 0, f"no rows to write for {path}"
    cols = list(rows[0].keys())
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=cols, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    """Run unit 1 head gravity-vector fit, Sweep 3 replay, and joint speed matrix scoring."""
    head_rows = load_rows(HERE / "unit1-head-imu-sweep.csv")
    holds = extract_holds(head_rows)
    grav_rows, grav_sum = score_gravity_holds(holds)
    sweep3_rows = replay_head_sweep(head_rows)
    speed_rows = replay_speed_matrix(load_rows(HERE / "unit1-joint-speed-matrix.csv"))
    write_csv(HERE / "gravity-holds.csv", grav_rows)
    write_csv(HERE / "sweep3-replay.csv", sweep3_rows)
    write_csv(HERE / "speed-matrix-scores.csv", speed_rows)
    print("grav_summary:", grav_sum)
    print(f"wrote {len(grav_rows)} grav holds, {len(sweep3_rows)} sweep3 moves, {len(speed_rows)} speed trials")


if __name__ == "__main__":
    main()
