"""Fit the accelerometer's steady offsets from the recording of the real robot..

    python3 -m simulacrum.imu_fit
    python3 -m simulacrum.imu_fit --head-sweep evidence/head-sweep-unit3-20261120

Needs numpy and no MuJoCo; it reads a file and never talks to a robot.

Whenever the robot stands still the accelerometer reads gravity. It sits in
the head, so the direction it reports turns with the head angle; and the
table it stood on was not level, so the direction also turns with the
robot's heading. Fitting every still stretch on the table at once separates
six numbers:

  tilt_x, tilt_y   the table's tilt, in the robot's odometry frame (rad)
  pitch            pitch of the accelerometer's frame at head angle 0 (rad)
  bias x, y, z     a constant offset on each axis (mm/s^2)

Limits. One robot, one run, 26 still stretches at three clusters of head
angle. Gravity is taken as 9806.65 mm/s^2; the true local value differs by
up to about 0.3 percent, which moves the z offset by up to about 3 mg. The
y offset cannot be told from a roll of the accelerometer's frame. Heading
is the robot's own odometry. The standard errors printed are those of the
least-squares fit and do not include any of this.

Pitch and the x offset trade against each other: near level a small pitch
moves the x reading by an almost constant amount, which an offset can
absorb. Their correlation in the fit is about -0.98, so neither is known as
well as its own standard error suggests; what is well fixed is their sum,
the x reading on a level surface. describe() reports the correlation and a
bootstrap over the still stretches. Leaving out a cluster of head angles
does not test this, because the 22 degree cluster also faces the other way
from most of the rest (an external review lane's point, conferral of 2026-11-15).

A seventh number, scale_x, multiplies the x axis's reading of gravity
before the offset is added (the part's sensitivity error on that axis).
fit(..., free_scale=True) fits it; otherwise it is 1. y and z keep a scale
of 1: across the head angles recorded, the z axis's share of gravity
changes too little for its scale to be told from its offset, and nothing
here turns the robot onto its side to show y's.

head_sweep() reads unit 3's stationary head sweeps (the another substrate's
analysis, the lab's coordination room turn 166, of SDK samples taken by
a lane): fit on spot B, score at spot A, which the fit never saw.
Each settled window is one equally weighted mean; heading is the robot's
odometry, which is continuous across both spots.
"""

from __future__ import annotations

import collections
import csv
import json
import math
import random
import statistics
import sys
from pathlib import Path
from typing import Any, Dict, List, Sequence, Tuple

import numpy as np

from . import recording

GRAVITY_MMPS2 = 9806.65
OFF_CHARGER_AFTER_S = 25.0
NAMES = ("tilt_x_rad", "tilt_y_rad", "pitch_rad", "bias_x_mmps2", "bias_y_mmps2", "bias_z_mmps2")
STEPS = (1e-6, 1e-6, 1e-6, 1e-3, 1e-3, 1e-3)
SCALE_NAME, SCALE_STEP = "scale_x", 1e-6
SWEEP_LEFT_OUT = ("B0setup", "Transit0cw")  # followed straight travel, or a transit; excluded by the sweep's authors
SWEEP_GYRO_STILL_RADPS = 0.05
SWEEP_COLUMNS = ("block", "visit", "head_deg", "yaw_deg", "wl_mm_s", "wr_mm_s", "gx_rad_s", "gy_rad_s", "gz_rad_s",
                 "ax_mm_s2", "ay_mm_s2", "az_mm_s2")
MAX_ITERATIONS = 60
Stretch = Tuple[float, float, Tuple[float, float, float], int]  # head rad, yaw rad, mean accel, rows


def stretches(states: Sequence[Dict[str, Any]], after_s: float = OFF_CHARGER_AFTER_S) -> List[Stretch]:
    """One (head angle, heading, mean reading, row count) per still stretch on the table."""
    out: List[Stretch] = []
    for run in recording.still_runs(states):
        if run[0]["t"] <= after_s:
            continue
        mean = tuple(statistics.mean(row["accel"][i] for row in run) for i in range(3))
        out.append((statistics.mean(row["head"] for row in run), statistics.mean(row["yaw"] for row in run),
                    (mean[0], mean[1], mean[2]), len(run)))
    assert len(out) >= 6, "too few still stretches to fit six numbers"
    return out


def predicted(p: Sequence[float], head: float, yaw: float) -> np.ndarray:
    """What the accelerometer reads at rest, in its own frame, for one head angle and heading.
    A seventh number, if given, is the x axis's scale."""
    assert len(p) in (len(NAMES), len(NAMES) + 1), "six numbers, or seven with the x scale"
    kx = p[len(NAMES)] if len(p) > len(NAMES) else 1.0
    ux = math.cos(yaw) * p[0] + math.sin(yaw) * p[1]
    uy = -math.sin(yaw) * p[0] + math.cos(yaw) * p[1]
    uz = math.sqrt(max(0.0, 1.0 - ux * ux - uy * uy))
    angle = head + p[2]
    g = GRAVITY_MMPS2
    return np.array([kx * g * (ux * math.cos(angle) + uz * math.sin(angle)) + p[3], g * uy + p[4],
                     g * (-ux * math.sin(angle) + uz * math.cos(angle)) + p[5]])


def _residuals(p: Sequence[float], data: Sequence[Stretch]) -> np.ndarray:
    assert len(data) > 0, "nothing to fit"
    return np.concatenate([math.sqrt(n) * (predicted(p, head, yaw) - np.array(mean)) for head, yaw, mean, n in data])


def fit(data: Sequence[Stretch], free_scale: bool = False) -> Dict[str, Any]:
    """Gauss-Newton least squares, each stretch weighted by its row count; the x scale too if free_scale."""
    names = NAMES + ((SCALE_NAME,) if free_scale else ())
    steps = STEPS + ((SCALE_STEP,) if free_scale else ())
    p = np.zeros(len(names))
    if free_scale:
        p[len(NAMES)] = 1.0
    jac = np.zeros((3 * len(data), len(names)))
    for _ in range(MAX_ITERATIONS):
        r = _residuals(p, data)
        for k, step in enumerate(steps):
            moved = p.copy()
            moved[k] += step
            jac[:, k] = (_residuals(moved, data) - r) / step
        delta = np.linalg.lstsq(jac, -r, rcond=None)[0]
        p = p + delta
        if float(np.max(np.abs(delta))) < 1e-10:
            break
    r = _residuals(p, data)
    rows = sum(n for _, _, _, n in data)
    assert int(np.linalg.matrix_rank(jac)) == len(names), "a fitted number has no effect on the readings"
    cov = np.linalg.inv(jac.T @ jac) * float(r @ r) / max(1, len(r) - len(names))
    assert float(np.max(np.abs(delta))) < 1e-6, "the fit did not converge"
    spread = np.sqrt(np.diag(cov))
    return {"values": {name: float(v) for name, v in zip(names, p)},
            "standard_errors": {name: float(v) for name, v in zip(names, spread)},
            "pitch_bias_x_correlation": float(cov[2, 3] / (spread[2] * spread[3])),
            "rms_mmps2": math.sqrt(float(r @ r) / (3 * rows)), "stretches": len(data), "rows": rows}


def bootstrap(data: Sequence[Stretch], draws: int = 200, seed: int = 1, free_scale: bool = True) -> Dict[str, Any]:
    """Refit on still stretches drawn with replacement: the middle 95 percent of each fitted number."""
    assert 20 <= draws <= 2000, "draw count out of range"
    rng = random.Random(seed)
    to_mg = 1000.0 / GRAVITY_MMPS2
    rows: List[List[float]] = []
    for _ in range(draws):
        v = fit([data[rng.randrange(len(data))] for _ in data], free_scale=free_scale)["values"]
        kx = v.get(SCALE_NAME, 1.0)
        level_x = v["bias_x_mmps2"] * to_mg + 1000.0 * kx * math.sin(v["pitch_rad"])
        rows.append([math.degrees(v["pitch_rad"]), v["bias_x_mmps2"] * to_mg, v["bias_y_mmps2"] * to_mg,
                     v["bias_z_mmps2"] * to_mg, level_x, kx])
    table = np.array(rows)
    names = ("pitch_deg", "bias_x_mg", "bias_y_mg", "bias_z_mg", "level_x_mg", SCALE_NAME)
    out: Dict[str, Any] = {name: [float(q) for q in np.percentile(table[:, i], [2.5, 97.5])] for i, name in enumerate(names)}
    out["draws"] = draws
    assert len(out) == len(names) + 1, "a column is missing"
    return out


def describe() -> Dict[str, Any]:
    """The fit over every still stretch with x scale freed, and again with each cluster of head angle left out."""
    states, _ = recording.load()
    data = stretches(states)
    to_mg = 1000.0 / GRAVITY_MMPS2
    out = fit(data, free_scale=True)
    v = out["values"]
    out["scale_x"] = v[SCALE_NAME]
    out["bias_mg"] = [v["bias_x_mmps2"] * to_mg, v["bias_y_mmps2"] * to_mg, v["bias_z_mmps2"] * to_mg]
    out["pitch_deg"] = math.degrees(v["pitch_rad"])
    out["table_tilt_deg"] = math.degrees(math.hypot(v["tilt_x_rad"], v["tilt_y_rad"]))
    out["bootstrap_95"] = bootstrap(data, free_scale=True)
    out["without"] = {}
    for label, low, high in (("head near 22 deg", 0.3, 1.0), ("head near 7 deg", 0.09, 0.2), ("head near 0 deg", -0.1, 0.05)):
        rest = [s for s in data if not low <= s[0] <= high]
        part = fit(rest, free_scale=True)["values"]
        out["without"][label] = {"stretches": len(rest), "scale_x": part[SCALE_NAME], "pitch_deg": math.degrees(part["pitch_rad"]),
                                 "bias_mg": [part[name] * to_mg for name in NAMES[3:]]}
    assert len(out["without"]) == 3 and len(out["bias_mg"]) == 3, "a part of the report is missing"
    return out


def sweep_windows(folder: Path) -> Dict[str, List[Stretch]]:
    """One (head, odometry heading, mean reading, 1) per settled window of the head sweeps, by spot:
    "B" (fit), "A" (headings 0 and 180, held out) and "A90" (the sideways check, held out)."""
    groups: Dict[Tuple[str, int], List[Dict[str, str]]] = collections.defaultdict(list)
    with open(Path(folder) / "samples.csv", newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        assert set(SWEEP_COLUMNS) <= set(reader.fieldnames or ()), "samples.csv lacks a column the fit reads"
        for row in reader:
            gyro = math.sqrt(sum(float(row[k]) ** 2 for k in ("gx_rad_s", "gy_rad_s", "gz_rad_s")))
            moving = float(row["wl_mm_s"]) != 0.0 or float(row["wr_mm_s"]) != 0.0 or gyro >= SWEEP_GYRO_STILL_RADPS
            if row["block"] not in SWEEP_LEFT_OUT and not moving:
                groups[(row["block"], int(row["visit"]))].append(row)
    out: Dict[str, List[Stretch]] = {"B": [], "A": [], "A90": []}
    for (block, _), rows in sorted(groups.items()):
        spot = "A90" if block.startswith("A90") else block[0]
        mean = tuple(statistics.mean(float(r[k]) for r in rows) for k in ("ax_mm_s2", "ay_mm_s2", "az_mm_s2"))
        out[spot].append((math.radians(statistics.mean(float(r["head_deg"]) for r in rows)),
                          math.radians(statistics.mean(float(r["yaw_deg"]) for r in rows)),
                          (mean[0], mean[1], mean[2]), 1))
    assert all(out[spot] for spot in out), "a spot has no settled windows"
    return out


def vector_rms_mg(values: Dict[str, float], data: Sequence[Stretch]) -> float:
    """Root mean square length of the per-window error, in mg, for fitted values applied to some windows."""
    p = [values[name] for name in NAMES] + ([values[SCALE_NAME]] if SCALE_NAME in values else [])
    errors = [predicted(p, head, yaw) - np.array(mean) for head, yaw, mean, _ in data]
    assert errors, "no windows to score"
    return math.sqrt(statistics.mean(float(e @ e) for e in errors)) * 1000.0 / GRAVITY_MMPS2


def head_sweep(folder: Path, draws: int = 200, seed: int = 1) -> Dict[str, Any]:
    """Fit on spot B with and without the x scale; score both on spot A; bootstrap the scale over B's windows."""
    assert 20 <= draws <= 2000, "draw count out of range"
    spots = sweep_windows(folder)
    to_mg = 1000.0 / GRAVITY_MMPS2
    out: Dict[str, Any] = {"windows": {spot: len(rows) for spot, rows in spots.items()}}
    for label, free in (("scale_1", False), ("scale_x_free", True)):
        v = fit(spots["B"], free_scale=free)["values"]
        out[label] = {"bias_mg": [v[name] * to_mg for name in NAMES[3:]], "pitch_deg": math.degrees(v["pitch_rad"]),
                      "scale_x": v.get(SCALE_NAME, 1.0),
                      "rms_mg": {spot: vector_rms_mg(v, rows) for spot, rows in spots.items()}}
    rng = random.Random(seed)
    b = spots["B"]
    rows = []
    for _ in range(draws):
        v = fit([b[rng.randrange(len(b))] for _ in b], free_scale=True)["values"]
        rows.append([v[SCALE_NAME], math.degrees(v["pitch_rad"])] + [v[name] * to_mg for name in NAMES[3:]])
    table = np.array(rows)
    names = ("scale_x", "pitch_deg", "bias_x_mg", "bias_y_mg", "bias_z_mg")
    out["bootstrap_95"] = {name: [float(q) for q in np.percentile(table[:, i], [2.5, 97.5])] for i, name in enumerate(names)}
    out["draws"] = draws
    assert len(out["bootstrap_95"]) == len(names), "a column is missing"
    return out


def level_reading_mg(result: Dict[str, Any], head_rad: float) -> List[float]:
    """What the fitted robot reads beyond a level, unpitched accelerometer at one head angle, in mg."""
    v = result["values"]
    level = [0.0, 0.0, v["pitch_rad"], v["bias_x_mmps2"], v["bias_y_mmps2"], v["bias_z_mmps2"]]
    if SCALE_NAME in v:
        level.append(v[SCALE_NAME])
    ideal = [0.0] * len(NAMES)
    diff = predicted(level, head_rad, 0.0) - predicted(ideal, head_rad, 0.0)
    assert diff.shape == (3,), "three axes"
    return [float(d) * 1000.0 / GRAVITY_MMPS2 for d in diff]


if __name__ == "__main__":
    if len(sys.argv) == 3 and sys.argv[1] == "--head-sweep":
        print(json.dumps(head_sweep(Path(sys.argv[2])), indent=1, sort_keys=True))
        sys.exit(0)
    if len(sys.argv) != 1:
        print("\n".join(__doc__.splitlines()[2:4]))
        sys.exit(2)
    report = describe()
    report["level_reading_mg"] = {str(deg): level_reading_mg(report, math.radians(deg)) for deg in (-22, 0, 22, 45)}
    report["with_scale_x"] = fit(stretches(recording.load()[0]), free_scale=True)
    print(json.dumps(report, indent=1, sort_keys=True))
    sys.exit(0)
