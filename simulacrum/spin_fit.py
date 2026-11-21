"""Where the treads meet the table, fitted from the robots' recorded spins. Authored-by: CPCS.

    cauldron python fit_spins.py

Needs MuJoCo. It replays recordings and never talks to a robot.

The model spun too slowly: on unit 1's run E at 0.85 to 0.90 of the robot's
rate, on unit 3's camera spins at 0.78. A centre of mass further back fixes
the rate but makes the model turn about a point 10 mm behind where unit 3
turns (README, "The spins of 2026-11-18"). What is left is the contact:
the model's treads touch the table only under the axles, 48 mm apart,
and the further apart the contacts, the more the treads' sideways
friction resists a spin. tread_contact_inset_mm moves both contacts
inward (simulacrum/mjcf.py, wheel_x_mm).

fit() replays unit 1's four spins of run E (150 and 220 mm/s, both ways) at
each inset of INSET_GRID_MM and finds where the model's mean steady turn
rate equals the robot's. held_out() then replays unit 3's four camera spins
(150 mm/s, the 2.0 table) at no inset and at the fitted one: turn rate,
the turn centre (simulacrum/camera_spins.py measures unit 3's from the
camera), and how much the treads rock (roll and pitch rate in the spin).

Steady means the second half of the commanded time, as in
replay_recording.py. Roll is gyro x less tan(head angle) times gyro z,
since the gyro sits in the pitched head; pitch is gyro y. Both gyros are
low-passed by the IMU chip: the robot's in the part, the model's by
simulacrum/imu_filter.py at every physics step (README, "The IMU chip's
own filter").
"""

from __future__ import annotations

import json
import math
import statistics
import sys
from pathlib import Path
from typing import Any, Dict, List, Sequence, Tuple

import numpy as np

from . import params, recording, replay
from .robot import VectorSim

ROOT = Path(__file__).resolve().parent.parent
FIT_RECORDING = ROOT / "evidence" / "hw-rec-20261118" / "run-e-spins.jsonl"
HELD_OUT_RECORDING = ROOT / "evidence" / "camera-spins-unit3-20261120" / "robot.jsonl"
INSET_GRID_MM = (0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0)
LENS_X_MM = -36.3  # the backpack lens the camera followed: 25 mm drive centre plus that lane's 11.3 mm lever
STEADY_FROM = 0.5


def spins(path: Path, values: Dict[str, Any]) -> Tuple[List[Dict[str, Any]], List[recording.Segment]]:
    """The recording's state rows and its spins (wheel segments with the treads turning opposite ways)."""
    states, commands = recording.load(path)
    found = [s for s in recording.segments(commands, values).values() if s.kind == "wheels" and s.left * s.right < 0.0]
    assert found, "the recording has no spins"
    assert all(s.t_stop > s.t_cmd for s in found), "a spin has no duration"
    return states, found


def _roll_pitch(gyro: Sequence[float], head_rad: float) -> Tuple[float, float]:
    """Roll and pitch rate of the body from the head's gyro (rad/s)."""
    assert len(gyro) == 3, "the gyro has three axes"
    assert abs(head_rad) < 1.2, "head angle out of range"
    return gyro[0] - gyro[2] * math.tan(head_rad), gyro[1]


def _centre_x(points: np.ndarray, yaw: np.ndarray) -> float:
    """x of the turn centre in the body frame, from the lens point's path over part of a spin."""
    assert points.shape[0] >= 4 and points.shape == (len(yaw), 2), "too few points for a circle"
    x, y = points[:, 0], points[:, 1]
    a = np.column_stack((2.0 * x, 2.0 * y, np.ones(len(x))))
    cx, cy, _ = np.linalg.lstsq(a, x * x + y * y, rcond=None)[0]
    ahead = (cx - x) * np.cos(yaw) + (cy - y) * np.sin(yaw)
    assert np.all(np.isfinite(ahead)), "the circle fit failed"
    return LENS_X_MM + float(np.median(ahead))


def _model_spin(seg: recording.Segment, values: Dict[str, Any], start: Dict[str, Any]) -> Dict[str, float]:
    """Drive the model through one spin from the robot's recorded head and lift; steady-state figures."""
    sim = VectorSim(values=dict(values, **replay.OPEN_TABLE), head_deg=math.degrees(start["head"]),
                    lift_mm=start["lift"], noise_seed=0)
    duration = seg.t_stop - seg.t_cmd
    assert duration > 2.0 * sim.dt, "the spin is shorter than two control ticks"
    ticks_pre, ticks_on = int(round(replay.PRE_S / sim.dt)), int(round(duration / sim.dt))
    rows = []
    for tick in range(ticks_pre + ticks_on):
        if tick == ticks_pre:
            sim.set_wheel_motors(seg.left, seg.right)
        state = sim.step(1)
        if STEADY_FROM * duration <= (tick + 1 - ticks_pre) * sim.dt <= duration:
            rows.append(state)
    assert len(rows) >= 4, "no steady part in the spin"
    rock = [_roll_pitch(s.gyro_radps, s.head_angle_rad) for s in rows]
    yaw = np.array([s.pose_angle_rad for s in rows])
    lens = np.array([(s.x_mm + LENS_X_MM * math.cos(s.pose_angle_rad), s.y_mm + LENS_X_MM * math.sin(s.pose_angle_rad))
                     for s in rows])
    return {"rate": statistics.mean(abs(s.gyro_radps[2]) for s in rows),
            "roll": statistics.mean(abs(r) for r, _ in rock), "pitch": statistics.mean(abs(p) for _, p in rock),
            "centre_x_mm": _centre_x(lens, yaw)}


def _robot_spin(seg: recording.Segment, states: Sequence[Dict[str, Any]]) -> Dict[str, float]:
    """The robot's steady turn rate and rocking over the same part of the spin."""
    duration = seg.t_stop - seg.t_cmd
    rows = [r for r in states if seg.t_cmd + STEADY_FROM * duration <= r["t"] <= seg.t_stop]
    assert len(rows) >= 4, "too few recorded rows in the steady part"
    rock = [_roll_pitch(r["gyro"], r["head"]) for r in rows]
    assert all(len(r["gyro"]) == 3 for r in rows), "a gyro row is malformed"
    return {"rate": statistics.mean(abs(r["gyro"][2]) for r in rows), "rows": len(rows),
            "roll": statistics.mean(abs(r) for r, _ in rock), "pitch": statistics.mean(abs(p) for _, p in rock)}


def compare(path: Path, values: Dict[str, Any]) -> List[Dict[str, Any]]:
    """Every spin of a recording, model against robot."""
    states, found = spins(path, values)
    out = []
    for seg in found:
        robot = _robot_spin(seg, states)
        model = _model_spin(seg, values, replay._state_at(states, seg.t_cmd))
        out.append({"label": seg.label, "robot": robot, "model": model, "ratio": model["rate"] / robot["rate"]})
    assert len(out) == len(found), "a spin was not compared"
    return out


def _crossing(grid: Sequence[float], ratios: Sequence[float]) -> float:
    """The inset at which the ratio reaches 1, by linear interpolation on the grid; nan if it never does."""
    assert len(grid) == len(ratios) >= 2, "grid and ratios differ"
    for k in range(len(grid) - 1):
        lo, hi = ratios[k], ratios[k + 1]
        if (lo - 1.0) * (hi - 1.0) <= 0.0 and lo != hi:
            return grid[k] + (1.0 - lo) * (grid[k + 1] - grid[k]) / (hi - lo)
    return float("nan")


def fit(values: Dict[str, Any], grid: Sequence[float] = INSET_GRID_MM, path: Path = FIT_RECORDING) -> Dict[str, Any]:
    """The inset at which the model's mean turn rate over the recording's spins equals the robot's."""
    assert all(b > a for a, b in zip(grid, grid[1:])), "grid must increase"
    table: List[List[float]] = []
    labels: List[str] = []
    for inset in grid:
        rows = compare(path, dict(values, tread_contact_inset_mm=inset))
        labels = [r["label"] for r in rows]
        table.append([r["ratio"] for r in rows])
    means = [statistics.mean(row) for row in table]
    per_spin = {label: _crossing(grid, [row[k] for row in table]) for k, label in enumerate(labels)}
    best = _crossing(grid, means)
    assert math.isfinite(best), "the mean ratio never reaches 1 on the grid"
    return {"inset_mm": best, "grid_mm": list(grid), "mean_ratio": means, "ratios": table, "labels": labels,
            "per_spin_inset_mm": per_spin, "at_fit": compare(path, dict(values, tread_contact_inset_mm=best))}


def held_out(values: Dict[str, Any], inset: float, path: Path = HELD_OUT_RECORDING) -> Dict[str, Any]:
    """Unit 3's camera spins on the 2.0 table, at no inset and at the fitted one."""
    assert 0.0 <= inset, "inset must not be negative"
    out = {"no_inset": compare(path, dict(values, tread_contact_inset_mm=0.0)),
           "fitted": compare(path, dict(values, tread_contact_inset_mm=inset))}
    assert len(out["no_inset"]) == len(out["fitted"]), "the two runs saw different spins"
    return out


def main() -> int:
    base = params.values(params.load(model="vector_1"))
    report: Dict[str, Any] = {"fit": fit(base)}
    inset = report["fit"]["inset_mm"]
    report["held_out"] = held_out(params.values(params.load(model="vector_2")), inset)
    print(json.dumps(report, indent=1, sort_keys=True, default=float))
    return 0


if __name__ == "__main__":
    sys.exit(main())
