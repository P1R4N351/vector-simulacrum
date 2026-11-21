"""Fit the accelerometer's steady offsets from the recording of the real robot..

    python3 -m simulacrum.imu_fit

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
"""

from __future__ import annotations

import json
import math
import random
import statistics
import sys
from typing import Any, Dict, List, Sequence, Tuple

import numpy as np

from . import recording

GRAVITY_MMPS2 = 9806.65
OFF_CHARGER_AFTER_S = 25.0
NAMES = ("tilt_x_rad", "tilt_y_rad", "pitch_rad", "bias_x_mmps2", "bias_y_mmps2", "bias_z_mmps2")
STEPS = (1e-6, 1e-6, 1e-6, 1e-3, 1e-3, 1e-3)
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
    """What the accelerometer reads at rest, in its own frame, for one head angle and heading."""
    assert len(p) == len(NAMES), "six numbers are fitted"
    ux = math.cos(yaw) * p[0] + math.sin(yaw) * p[1]
    uy = -math.sin(yaw) * p[0] + math.cos(yaw) * p[1]
    uz = math.sqrt(max(0.0, 1.0 - ux * ux - uy * uy))
    angle = head + p[2]
    g = GRAVITY_MMPS2
    return np.array([g * (ux * math.cos(angle) + uz * math.sin(angle)) + p[3], g * uy + p[4],
                     g * (-ux * math.sin(angle) + uz * math.cos(angle)) + p[5]])


def _residuals(p: Sequence[float], data: Sequence[Stretch]) -> np.ndarray:
    assert len(data) > 0, "nothing to fit"
    return np.concatenate([math.sqrt(n) * (predicted(p, head, yaw) - np.array(mean)) for head, yaw, mean, n in data])


def fit(data: Sequence[Stretch]) -> Dict[str, Any]:
    """Gauss-Newton least squares, each stretch weighted by its row count."""
    p = np.zeros(len(NAMES))
    jac = np.zeros((3 * len(data), len(NAMES)))
    for _ in range(MAX_ITERATIONS):
        r = _residuals(p, data)
        for k, step in enumerate(STEPS):
            moved = p.copy()
            moved[k] += step
            jac[:, k] = (_residuals(moved, data) - r) / step
        delta = np.linalg.lstsq(jac, -r, rcond=None)[0]
        p = p + delta
        if float(np.max(np.abs(delta))) < 1e-10:
            break
    r = _residuals(p, data)
    rows = sum(n for _, _, _, n in data)
    cov = np.linalg.inv(jac.T @ jac) * float(r @ r) / max(1, len(r) - len(NAMES))
    assert float(np.max(np.abs(delta))) < 1e-6, "the fit did not converge"
    spread = np.sqrt(np.diag(cov))
    return {"values": {name: float(v) for name, v in zip(NAMES, p)},
            "standard_errors": {name: float(v) for name, v in zip(NAMES, spread)},
            "pitch_bias_x_correlation": float(cov[2, 3] / (spread[2] * spread[3])),
            "rms_mmps2": math.sqrt(float(r @ r) / (3 * rows)), "stretches": len(data), "rows": rows}


def bootstrap(data: Sequence[Stretch], draws: int = 200, seed: int = 1) -> Dict[str, Any]:
    """Refit on still stretches drawn with replacement: the middle 95 percent of each fitted number."""
    assert 20 <= draws <= 2000, "draw count out of range"
    rng = random.Random(seed)
    to_mg = 1000.0 / GRAVITY_MMPS2
    rows: List[List[float]] = []
    for _ in range(draws):
        v = fit([data[rng.randrange(len(data))] for _ in data])["values"]
        level_x = v["bias_x_mmps2"] * to_mg + 1000.0 * math.sin(v["pitch_rad"])
        rows.append([math.degrees(v["pitch_rad"]), v["bias_x_mmps2"] * to_mg, v["bias_y_mmps2"] * to_mg,
                     v["bias_z_mmps2"] * to_mg, level_x])
    table = np.array(rows)
    names = ("pitch_deg", "bias_x_mg", "bias_y_mg", "bias_z_mg", "level_x_mg")
    out: Dict[str, Any] = {name: [float(q) for q in np.percentile(table[:, i], [2.5, 97.5])] for i, name in enumerate(names)}
    out["draws"] = draws
    assert len(out) == len(names) + 1, "a column is missing"
    return out


def describe() -> Dict[str, Any]:
    """The fit over every still stretch, and again with each cluster of head angle left out."""
    states, _ = recording.load()
    data = stretches(states)
    to_mg = 1000.0 / GRAVITY_MMPS2
    out = fit(data)
    v = out["values"]
    out["bias_mg"] = [v["bias_x_mmps2"] * to_mg, v["bias_y_mmps2"] * to_mg, v["bias_z_mmps2"] * to_mg]
    out["pitch_deg"] = math.degrees(v["pitch_rad"])
    out["table_tilt_deg"] = math.degrees(math.hypot(v["tilt_x_rad"], v["tilt_y_rad"]))
    out["bootstrap_95"] = bootstrap(data)
    out["without"] = {}
    for label, low, high in (("head near 22 deg", 0.3, 1.0), ("head near 7 deg", 0.09, 0.2), ("head near 0 deg", -0.1, 0.05)):
        rest = [s for s in data if not low <= s[0] <= high]
        part = fit(rest)["values"]
        out["without"][label] = {"stretches": len(rest), "pitch_deg": math.degrees(part["pitch_rad"]),
                                 "bias_mg": [part[name] * to_mg for name in NAMES[3:]]}
    assert len(out["without"]) == 3 and len(out["bias_mg"]) == 3, "a part of the report is missing"
    return out


def level_reading_mg(result: Dict[str, Any], head_rad: float) -> List[float]:
    """What the fitted robot reads beyond a level, unpitched accelerometer at one head angle, in mg."""
    v = result["values"]
    level = [0.0, 0.0, v["pitch_rad"], v["bias_x_mmps2"], v["bias_y_mmps2"], v["bias_z_mmps2"]]
    ideal = [0.0] * len(NAMES)
    diff = predicted(level, head_rad, 0.0) - predicted(ideal, head_rad, 0.0)
    assert diff.shape == (3,), "three axes"
    return [float(d) * 1000.0 / GRAVITY_MMPS2 for d in diff]


if __name__ == "__main__":
    report = describe()
    report["level_reading_mg"] = {str(deg): level_reading_mg(report, math.radians(deg)) for deg in (-22, 0, 22, 45)}
    print(json.dumps(report, indent=1, sort_keys=True))
    sys.exit(0)
