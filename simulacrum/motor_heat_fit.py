"""How a tread's top speed falls under sustained full power: a first-order heat state, per tread..

    python3 -m simulacrum.motor_heat_fit evidence/hw-rec-20261118/run-f-endurance-unit1.jsonl

Fitted from the recording alone, without the MuJoCo model, the way imu_fit.py fits the accelerometer.
In an endurance run every spin is commanded above the treads' top speed, so the duty is full and the
speed each tread reaches is its ceiling at that moment. Per tread:

    dh/dt = (u - h) / tau        u = 1 while a spin is commanded, 0 at rest; h starts at 0
    ceiling = c_dir - a * h      c_dir: the cold ceiling for that tread turning forward or back

tau is found on a fixed grid; c_forward, c_back and a by least squares at each tau. The fit uses the
first FIT_CYCLES cycles and predicts the rest, against a constant-ceiling baseline. The voltage
cannot be split out: the SDK's battery reading did not move during the run (REC2 run F), so this
is the drop of one tread relative to its own cold value, whatever its cause.

No robot is touched; it reads a file.
"""

from __future__ import annotations

import json
import math
import statistics
import sys
from pathlib import Path
from typing import Any, Dict, List, Sequence, Tuple

from simulacrum import recording

SETTLE_S = 1.5
STEP_S = 0.05
TAU_GRID_S = tuple(float(t) for t in range(5, 605, 5))
FIT_CYCLES = 10
MIN_ROWS = 8


def spins(path: Path) -> List[Dict[str, Any]]:
    """Each endurance spin: its window, and each tread's mean |speed| after SETTLE_S, with its direction."""
    states, commands = recording.load(path)
    out = []
    for index, row in enumerate(commands):
        if row["name"] != "wheels" or not str(row.get("label", "")).startswith("endurance_"):
            continue
        stops = [later for later in commands[index + 1:] if later["name"] == "stop"]
        assert stops, f"no stop after {row['label']}"
        if stops[0].get("why") != "segment_end":
            continue  # ended by a hazard or the robot itself: not a full-power window
        rows = [s for s in states if row["t"] + SETTLE_S <= s["t"] <= stops[0]["t"]]
        if len(rows) < MIN_ROWS:
            continue
        cycle = int(str(row["label"]).rsplit("_", 1)[1])
        out.append({"label": row["label"], "cycle": cycle, "t_on": row["t"], "t_off": stops[0]["t"],
                    "t_from": row["t"] + SETTLE_S,
                    "left": statistics.fmean(abs(s["wl"]) for s in rows), "right": statistics.fmean(abs(s["wr"]) for s in rows),
                    "left_forward": float(row["left"]) > 0.0, "right_forward": float(row["right"]) > 0.0})
    assert len(out) >= 4, "too few endurance spins to fit"
    return out


def heat_means(windows: Sequence[Dict[str, Any]], tau: float) -> List[float]:
    """Mean heat state over each spin's measurement window, integrated in STEP_S steps from the first spin."""
    assert tau > 0.0, "time constant must be positive"
    t, h, out = windows[0]["t_on"], 0.0, []
    decay = math.exp(-STEP_S / tau)
    end = max(w["t_off"] for w in windows)
    assert (end - t) / STEP_S < 1.0e6, "run too long to integrate"
    samples: List[Tuple[float, float]] = []
    while t <= end:
        on = any(w["t_on"] <= t < w["t_off"] for w in windows)
        h = (1.0 if on else 0.0) + (h - (1.0 if on else 0.0)) * decay
        t += STEP_S
        samples.append((t, h))
    for w in windows:
        inside = [hh for tt, hh in samples if w["t_from"] <= tt <= w["t_off"]]
        assert inside, "no heat samples in a window"
        out.append(statistics.fmean(inside))
    return out


def _solve3(rows: Sequence[Tuple[float, float, float]], ys: Sequence[float]) -> Tuple[float, float, float]:
    """Least squares for y = p0*x0 + p1*x1 + p2*x2 through the normal equations (3x3, Cramer's rule)."""
    m = [[math.fsum(r[i] * r[j] for r in rows) for j in range(3)] for i in range(3)]
    b = [math.fsum(r[i] * y for r, y in zip(rows, ys)) for i in range(3)]
    det = lambda a: (a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1]) - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
                     + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]))
    d = det(m)
    assert abs(d) > 1e-12, "the fit is degenerate: both directions and some heat are needed"
    out = []
    for k in range(3):
        mk = [[b[i] if j == k else m[i][j] for j in range(3)] for i in range(3)]
        out.append(det(mk) / d)
    return out[0], out[1], out[2]


def _design(points: Sequence[Dict[str, Any]], heats: Sequence[float], side: str) -> List[Tuple[float, float, float]]:
    return [(1.0 if p[side + "_forward"] else 0.0, 0.0 if p[side + "_forward"] else 1.0, -h) for p, h in zip(points, heats)]


def fit_tread(points: Sequence[Dict[str, Any]], side: str) -> Dict[str, Any]:
    """Best tau on the grid and c_forward, c_back, a, on the cycles below FIT_CYCLES; scored on the rest."""
    assert side in ("left", "right"), "side is left or right"
    best: Dict[str, Any] = {}
    fit_idx = [i for i, p in enumerate(points) if p["cycle"] < FIT_CYCLES]
    hold_idx = [i for i, p in enumerate(points) if p["cycle"] >= FIT_CYCLES]
    assert fit_idx, "no cycles to fit"
    for tau in TAU_GRID_S:
        heats = heat_means(points, tau)
        rows = _design(points, heats, side)
        ys = [p[side] for p in points]
        cf, cb, a = _solve3([rows[i] for i in fit_idx], [ys[i] for i in fit_idx])
        pred = [cf * r[0] + cb * r[1] + a * r[2] for r in rows]
        sse = math.fsum((pred[i] - ys[i]) ** 2 for i in fit_idx)
        if not best or sse < best["sse"]:
            best = {"tau_s": tau, "c_forward_mmps": cf, "c_back_mmps": cb, "a_mmps": a, "sse": sse, "pred": pred,
                    "ys": ys}
    rmse = lambda idx, pred: math.sqrt(statistics.fmean((pred[i] - best["ys"][i]) ** 2 for i in idx)) if idx else None
    ys = best["ys"]
    cold = {True: statistics.fmean(ys[i] for i in fit_idx if points[i][side + "_forward"]),
            False: statistics.fmean(ys[i] for i in fit_idx if not points[i][side + "_forward"])}
    flat = [cold[p[side + "_forward"]] for p in points]
    mean_cold = 0.5 * (best["c_forward_mmps"] + best["c_back_mmps"])
    return {"side": side, "tau_s": best["tau_s"], "c_forward_mmps": round(best["c_forward_mmps"], 2),
            "c_back_mmps": round(best["c_back_mmps"], 2), "a_mmps": round(best["a_mmps"], 2),
            "loss_at_full_heat": round(best["a_mmps"] / mean_cold, 4),
            "n_fit": len(fit_idx), "n_held_out": len(hold_idx),
            "rmse_fit_mmps": round(rmse(fit_idx, best["pred"]), 2),
            "rmse_held_out_mmps": round(rmse(hold_idx, best["pred"]), 2) if hold_idx else None,
            "rmse_held_out_constant_mmps": round(rmse(hold_idx, flat), 2) if hold_idx else None}


def main(argv: Sequence[str]) -> int:
    if len(argv) != 1:
        print("usage: python3 -m simulacrum.motor_heat_fit <endurance recording>", file=sys.stderr)
        return 2
    points = spins(Path(argv[0]))
    out = {"recording": argv[0], "spins": len(points), "fit_cycles": FIT_CYCLES,
           "left": fit_tread(points, "left"), "right": fit_tread(points, "right")}
    print("HEAT " + json.dumps(out, sort_keys=True))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
