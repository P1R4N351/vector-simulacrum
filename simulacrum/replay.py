"""Replay a recording of the real robot through the model and compare..

The recording (evidence/hw-rec-20261115) holds the robot's state messages
and the commands that were sent, with host receive times. Each commanded
segment is replayed on its own from rest: the same command at the same
moment, the same stop, and the model's reported signals read at the
recorded sample times. This module never talks to a robot.

Compared signals are the ones the robot measures itself: reported wheel
speeds (encoder based), gyro z, head angle and lift height. The robot's
pose and yaw are its own odometry and are not used as truth.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

from . import params
from .recording import Segment, load, segments
from .robot import VectorSim

PRE_S = 0.3
POST_S = 0.7
OPEN_TABLE = {"table_length_mm": 1500.0, "table_width_mm": 1500.0}
FIT_WHEELS = ("spin_ccw_30", "spin_cw_30", "spin_ccw_100", "spin_cw_100", "back_40", "forward_40")
HOLD_WHEELS = ("spin_ccw_60", "spin_cw_60", "back_80", "forward_80")
FIT_SERVOS = ("head_2", "head_4", "lift_1")
HOLD_SERVOS = ("head_3", "lift_2")


def _state_at(states: Sequence[Dict[str, Any]], t: float) -> Dict[str, Any]:
    """The last state row at or before t."""
    earlier = [row for row in states if row["t"] <= t]
    assert earlier, "no state row before the segment"
    return earlier[-1]


def _run(sim: VectorSim, seg: Segment, pre_s: float) -> List[Dict[str, float]]:
    """Step the model through one segment; one record of reported signals per control tick."""
    ticks_pre = int(round(pre_s / sim.dt))
    ticks_on = int(round((seg.t_stop - seg.t_cmd) / sim.dt))
    ticks_post = int(round(POST_S / sim.dt)) if seg.kind == "wheels" else 0
    out: List[Dict[str, float]] = []
    for tick in range(ticks_pre + ticks_on + ticks_post):
        if tick == ticks_pre and seg.kind == "wheels":
            sim.set_wheel_motors(seg.left, seg.right)
        elif tick == ticks_pre and seg.kind == "head":
            sim.command_head_angle(seg.target)
        elif tick == ticks_pre:
            sim.command_lift_height(seg.target)
        if tick == ticks_pre + ticks_on and seg.kind == "wheels":
            sim.stop_all_motors()
        sim.step(1)
        out.append(sim.reported_now())
    assert len(out) == ticks_pre + ticks_on + ticks_post, "a tick left no record"
    return out


def _rmse(pairs: Sequence[Tuple[float, float]]) -> float:
    assert len(pairs) > 0, "nothing to compare"
    return math.sqrt(sum((a - b) ** 2 for a, b in pairs) / len(pairs))


def replay(seg: Segment, values: Dict[str, Any], states: Sequence[Dict[str, Any]]) -> Dict[str, Any]:
    """Replay one segment and return its errors against the recording, with the sample count."""
    start = _state_at(states, seg.t_cmd)
    sim = VectorSim(values=dict(values, **OPEN_TABLE), head_deg=math.degrees(start["head"]),
                    lift_mm=start["lift"], noise_seed=0)
    pre = PRE_S if seg.kind == "wheels" else 0.2
    trace = _run(sim, seg, pre)
    window_end = seg.t_stop + (POST_S - 0.05 if seg.kind == "wheels" else 0.0)
    rows = [row for row in states if seg.t_cmd - pre + 0.05 <= row["t"] <= window_end]
    model = [trace[min(len(trace) - 1, max(0, int(round((row["t"] - seg.t_cmd + pre) / sim.dt)) - 1))] for row in rows]
    out: Dict[str, Any] = {"label": seg.label, "kind": seg.kind, "n": len(rows)}
    if seg.kind == "wheels":
        out["wheel_rmse_mmps"] = _rmse([(m["wl"], r["wl"]) for m, r in zip(model, rows)]
                                       + [(m["wr"], r["wr"]) for m, r in zip(model, rows)])
        out["gyro_rmse_dps"] = _rmse([(m["gyro_z_dps"], math.degrees(r["gyro"][2])) for m, r in zip(model, rows)])
        out["command_mmps"] = max(abs(seg.left), abs(seg.right))
        out["is_spin"] = seg.left * seg.right < 0.0
    elif seg.kind == "head":
        out["angle_rmse_deg"] = _rmse([(m["head_deg"], math.degrees(r["head"])) for m, r in zip(model, rows)])
    else:
        out["height_rmse_mm"] = _rmse([(m["lift_mm"], r["lift"]) for m, r in zip(model, rows)])
    out["trace"] = [(round(r["t"] - seg.t_cmd, 3), m, r) for m, r in zip(model, rows)]
    return out


def compare(values: Dict[str, Any], labels: Sequence[str], path: Optional[Path] = None) -> List[Dict[str, Any]]:
    """Replay the named segments. One result per segment, without the per-sample traces."""
    states, commands = load(path)
    segs = segments(commands, values)
    out = []
    for label in labels:
        assert label in segs, f"no segment {label!r} in the recording"
        result = replay(segs[label], values, states)
        result.pop("trace")
        out.append(result)
    return out


def wheel_score(results: Sequence[Dict[str, Any]], nominal: Dict[str, Any]) -> float:
    """One number for a set of wheel segments: each error as a fraction of the segment's own scale."""
    assert len(results) > 0, "no segments to score"
    total = 0.0
    for result in results:
        total += result["wheel_rmse_mmps"] / result["command_mmps"]
        if result["is_spin"]:
            total += result["gyro_rmse_dps"] / math.degrees(2.0 * result["command_mmps"] / nominal["tread_spacing_mm"])
    return total / len(results)


def servo_score(results: Sequence[Dict[str, Any]], nominal: Dict[str, Any]) -> float:
    """One number for a set of head and lift segments: error over the joint's range."""
    assert len(results) > 0, "no segments to score"
    head_span = nominal["head_angle_max_deg"] - nominal["head_angle_min_deg"]
    lift_span = nominal["lift_height_max_mm"] - nominal["lift_height_min_mm"]
    total = sum(r["angle_rmse_deg"] / head_span if r["kind"] == "head" else r["height_rmse_mm"] / lift_span
                for r in results)
    return total / len(results)


def nominal_values() -> Dict[str, Any]:
    return params.values(params.load())
