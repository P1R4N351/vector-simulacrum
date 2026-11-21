"""Read the recording of the real robot and measure it. Authored-by: CPCS.

No MuJoCo, and nothing here talks to a robot: it reads two files in
evidence/hw-rec-20261115. The figures filed as "measured" in the parameter
table are recomputed from those files by the functions below, so the table
and the recording cannot drift apart unnoticed.

The robot's pose and yaw in the recording are its own odometry. They are
used only for distances the robot drove in a straight line, never as truth
about a turn.

The gyro and the accelerometer sit in the head (firmware imuFilter.cpp
subtracts the head angle to get pitch; gyro_frame() below shows it in the
data). A turn about the body's vertical axis therefore appears on gyro z
times the cosine of the head's pitch and on gyro x times its sine. The head
was 22.4 degrees up in every recorded spin, so gyro z alone understates
those turns by about 7 percent; turn_rate_dps() gives the whole rate.
"""

from __future__ import annotations

import hashlib
import json
import math
import statistics
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

from . import params

HW_DIR = Path(__file__).resolve().parent.parent / "evidence" / "hw-rec-20261115"
RECORDING = HW_DIR / "table-run1.jsonl"
REST = HW_DIR / "rest-on-charger.jsonl"
STATUS_CLIFF = 0x4000
SERVO_S = 1.1
MAX_ROWS = 100000


@dataclass(frozen=True)
class Segment:
    label: str
    kind: str  # "wheels", "head" or "lift"
    t_cmd: float
    t_stop: float
    left: float = 0.0
    right: float = 0.0
    target: float = 0.0


def sha256(path: Path) -> str:
    assert path.is_file(), f"missing {path}"
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load(path: Optional[Path] = None) -> Tuple[List[Dict[str, Any]], List[Dict[str, Any]]]:
    """Read the table run. Returns (state rows, command rows), in time order."""
    with open(path if path is not None else RECORDING, encoding="utf-8") as handle:
        rows = [json.loads(line) for line in handle]
    assert 0 < len(rows) <= MAX_ROWS, "recording is empty or too long"
    states = [row for row in rows if row.get("k") == "s"]
    commands = [row for row in rows if row.get("k") == "c"]
    assert states and commands, "recording has no state rows or no command rows"
    return states, commands


def load_rest(path: Optional[Path] = None) -> List[Dict[str, Any]]:
    """Read the rows recorded with the robot asleep on its charger."""
    with open(path if path is not None else REST, encoding="utf-8") as handle:
        rows = [json.loads(line) for line in handle]
    assert 0 < len(rows) <= MAX_ROWS, "rest recording is empty or too long"
    return rows


def _wheel_segments(commands: Sequence[Dict[str, Any]]) -> List[Segment]:
    out = []
    for index, row in enumerate(commands):
        if row["name"] != "wheels":
            continue
        # Earlier recorder versions only stamped the successful RPC return.
        stops = [later.get("issued_t", later["t"]) for later in commands[index + 1:] if later["name"] == "stop"]
        assert stops, f"no stop after {row['label']}"
        out.append(Segment(row["label"], "wheels", row["t"], stops[0], float(row["left"]), float(row["right"])))
    return out


def _servo_segments(commands: Sequence[Dict[str, Any]], lift_range: Tuple[float, float]) -> List[Segment]:
    out, counts = [], {"head": 0, "lift": 0}
    for row in commands:
        if row["name"] not in counts:
            continue
        counts[row["name"]] += 1
        target = float(row["deg"]) if row["name"] == "head" else lift_range[0] + float(row["norm"]) * (
            lift_range[1] - lift_range[0])
        out.append(Segment(f"{row['name']}_{counts[row['name']]}", row["name"], row["t"], row["t"] + SERVO_S,
                           target=target))
    return out


def segments(commands: Sequence[Dict[str, Any]], nominal: Dict[str, Any]) -> Dict[str, Segment]:
    """Every commanded segment of the recording, by label."""
    lift_range = (nominal["lift_height_min_mm"], nominal["lift_height_max_mm"])
    found = _wheel_segments(commands) + _servo_segments(commands, lift_range)
    assert len({seg.label for seg in found}) == len(found), "segment labels repeat"
    return {seg.label: seg for seg in found}


def state_interval_ms(states: Sequence[Dict[str, Any]]) -> Dict[str, float]:
    """Time between state messages as the host received them."""
    gaps = [1000.0 * (b["t"] - a["t"]) for a, b in zip(states, states[1:])]
    assert len(gaps) > 10, "too few state rows"
    return {"n": len(gaps), "median": statistics.median(gaps), "mean": statistics.mean(gaps)}


def rest_noise(rows: Sequence[Dict[str, Any]]) -> Dict[str, Any]:
    """Spread of the gyro, accelerometer and forward range with the robot asleep on its charger."""
    assert len(rows) > 10, "too few rest rows"
    gyro = [statistics.pstdev([math.degrees(r["gyro"][i]) for r in rows]) for i in range(3)]
    gyro_mean = [statistics.mean([math.degrees(r["gyro"][i]) for r in rows]) for i in range(3)]
    accel = [statistics.pstdev([r["accel"][i] for r in rows]) for i in range(3)]
    magnitude = statistics.mean(math.sqrt(sum(a * a for a in r["accel"])) for r in rows)
    prox = [float(r["prox_mm"]) for r in rows]
    return {"n": len(rows), "distinct": len({tuple(r["gyro"]) for r in rows}),
            "gyro_sd_dps": gyro, "gyro_mean_dps": gyro_mean, "accel_sd_mmps2": accel,
            "accel_magnitude_mmps2": magnitude, "prox_mean_mm": statistics.mean(prox),
            "prox_sd_mm": statistics.pstdev(prox)}


def turn_rate_dps(row: Dict[str, Any]) -> float:
    """Turn rate about the body's vertical axis from one state row, whatever the head angle."""
    gx, gz = float(row["gyro"][0]), float(row["gyro"][2])
    rate = math.degrees(math.copysign(math.hypot(gx, gz), gz))
    assert math.isfinite(rate), "gyro reading is not a number"
    return rate


def gyro_frame(states: Sequence[Dict[str, Any]], at_least_dps: float = 20.0) -> Dict[str, float]:
    """Is the gyro in the head? Line fit of gyro x over gyro z against tan(head angle), turning samples only.

    A slope of 1 means yes. The intercept is the pitch of the gyro's frame at head angle 0.
    """
    rows = [r for r in states if r["wl"] * r["wr"] < 0.0 and abs(math.degrees(r["gyro"][2])) > at_least_dps]
    assert len(rows) > 30, "too few turning samples"
    slope, intercept = line_fit([math.tan(r["head"]) for r in rows], [r["gyro"][0] / r["gyro"][2] for r in rows])
    heads = [math.degrees(r["head"]) for r in rows]
    return {"n": len(rows), "slope": slope, "offset_deg": math.degrees(math.atan(intercept)),
            "head_min_deg": min(heads), "head_max_deg": max(heads)}


def line_fit(xs: Sequence[float], ys: Sequence[float]) -> Tuple[float, float]:
    """Least-squares slope and intercept. statistics.linear_regression does this from Python 3.10; this runs on 3.9."""
    assert len(xs) == len(ys) >= 2, "need two or more points"
    mx, my = statistics.fmean(xs), statistics.fmean(ys)
    sxx = math.fsum((x - mx) ** 2 for x in xs)
    assert sxx > 0.0, "every x is the same: no line through them"
    slope = math.fsum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx
    return slope, my - slope * mx


def spin_ratios(states: Sequence[Dict[str, Any]], commands: Sequence[Dict[str, Any]],
                tread_spacing_mm: float) -> Dict[str, Dict[str, float]]:
    """Each recorded spin's turn rate over the no-slip rate, in the second half of the segment.

    "gyro_z" is the figure read from gyro z alone (what the model's head-mounted gyro is compared
    with in the replay); "whole" uses turn_rate_dps and is the body's turn rate.
    """
    assert tread_spacing_mm > 0.0, "tread spacing must be positive"
    out: Dict[str, Dict[str, float]] = {}
    for seg in _wheel_segments(commands):
        if seg.left * seg.right >= 0.0:
            continue
        rows = [r for r in states if 0.5 * (seg.t_cmd + seg.t_stop) <= r["t"] <= seg.t_stop]
        ideal = math.degrees((abs(seg.left) + abs(seg.right)) / tread_spacing_mm)
        out[seg.label] = {
            "n": len(rows), "head_deg": statistics.mean(math.degrees(r["head"]) for r in rows),
            "gyro_z": statistics.mean(abs(math.degrees(r["gyro"][2])) for r in rows) / ideal,
            "whole": statistics.mean(abs(turn_rate_dps(r)) for r in rows) / ideal}
    assert len(out) > 0, "no spin in the recording"
    return out


def moving_scatter(states: Sequence[Dict[str, Any]], commands: Sequence[Dict[str, Any]],
                   gravity_mmps2: float) -> Dict[str, Dict[str, float]]:
    """Scatter of the gyro (deg/s) and accelerometer (mg) in the second half of each recorded spin.

    What the sensors carry while the robot moves, beyond the motion itself: the largest
    standard deviation over the three axes. For checking the model's own vibration against.
    """
    assert gravity_mmps2 > 0.0, "gravity must be positive"
    out: Dict[str, Dict[str, float]] = {}
    for seg in _wheel_segments(commands):
        rows = [r for r in states if 0.5 * (seg.t_cmd + seg.t_stop) <= r["t"] <= seg.t_stop]
        if seg.left * seg.right >= 0.0 or len(rows) < 10:
            continue
        gyro = max(statistics.pstdev(math.degrees(r["gyro"][i]) for r in rows) for i in range(3))
        accel = max(statistics.pstdev(r["accel"][i] for r in rows) for i in range(3)) * 1000.0 / gravity_mmps2
        out[seg.label] = {"n": len(rows), "gyro_sd_dps": gyro, "accel_sd_mg": accel}
    assert len(out) > 0, "no spin long enough to measure"
    return out


def still_runs(states: Sequence[Dict[str, Any]], min_rows: int = 16, trim: int = 3,
               gyro_limit_radps: float = 0.01) -> List[List[Dict[str, Any]]]:
    """Stretches where the treads report zero and the gyro is quiet, with the ends trimmed off."""
    assert min_rows > 2 * trim >= 0, "the trim would leave nothing"
    runs: List[List[Dict[str, Any]]] = []
    current: List[Dict[str, Any]] = []
    for row in list(states) + [None]:
        still = row is not None and row["wl"] == 0.0 and row["wr"] == 0.0 and max(
            abs(g) for g in row["gyro"]) < gyro_limit_radps
        if still:
            current.append(row)
            continue
        if len(current) >= min_rows:
            runs.append(current[trim:len(current) - trim])
        current = []
    assert len(runs) > 0, "the robot never stood still"
    return runs


def gyro_at_rest(states: Sequence[Dict[str, Any]]) -> Dict[str, Any]:
    """Mean gyro reading over every still stretch: what is left after the firmware's own bias removal."""
    runs = still_runs(states)
    rows = [row for run in runs for row in run]
    means = [statistics.mean(math.degrees(r["gyro"][i]) for r in rows) for i in range(3)]
    run_means = [[statistics.mean(math.degrees(r["gyro"][i]) for r in run) for i in range(3)] for run in runs]
    worst = max(abs(m) for one in run_means for m in one)
    assert len(means) == 3, "gyro has three axes"
    return {"n": len(rows), "runs": len(runs), "mean_dps": means, "worst_run_mean_dps": worst}


def cliff_event(states: Sequence[Dict[str, Any]]) -> Dict[str, float]:
    """The one cliff reaction in the recording: how far it rolled on, how long it waited, how far it moved away."""
    flagged = [row for row in states if row["status"] & STATUS_CLIFF]
    assert flagged, "no cliff reaction in the recording"
    first = flagged[0]
    after = [row for row in states if row["t"] > first["t"]]
    still = [row for row in after if row["wl"] == 0.0 and row["wr"] == 0.0]
    moving = [row for row in after if row["t"] > still[0]["t"] and (row["wl"] != 0.0 or row["wr"] != 0.0)]
    last_still = [row for row in after if row["t"] < moving[0]["t"]][-1]
    burst = [row for row in moving if row["t"] <= moving[0]["t"] + 0.1 and row["wl"] > 0.0]
    rest = [row for row in after if row["t"] > moving[0]["t"] and row["wl"] == 0.0 and row["wr"] == 0.0][0]
    stop_xy, rest_xy = still[0]["pose"], rest["pose"]
    return {
        "cliff_rows": len(flagged), "t_first": first["t"], "speed_at_cliff_mmps": abs(first["wl"]),
        "rolled_on_mm": math.hypot(stop_xy[0] - first["pose"][0], stop_xy[1] - first["pose"][1]),
        "pause_min_s": last_still["t"] - first["t"], "pause_max_s": moving[0]["t"] - first["t"],
        "moved_away_mm": math.hypot(rest_xy[0] - stop_xy[0], rest_xy[1] - stop_xy[1]),
        "move_speed_mean_mmps": statistics.mean([row[key] for row in burst for key in ("wl", "wr")]),
    }


def fast_turn_after_cliff(states: Sequence[Dict[str, Any]], tread_spacing_mm: float,
                          at_least_mmps: float = 200.0) -> Dict[str, Any]:
    """The robot's own turn after its cliff reaction: the samples where both treads report at least a given speed.

    The only place in the recording where the treads run near their limit. It is a short
    transient the robot commanded itself, not a held spin, and the commands are not in the file.
    """
    first = cliff_event(states)["t_first"]
    rows = [row for row in states if row["t"] > first and row["wl"] * row["wr"] < 0.0
            and min(abs(row["wl"]), abs(row["wr"])) >= at_least_mmps]
    assert rows, "no fast turn after the cliff reaction"
    gyro = [abs(turn_rate_dps(row)) for row in rows]
    ratio = [g / math.degrees((abs(row["wl"]) + abs(row["wr"])) / tread_spacing_mm) for g, row in zip(gyro, rows)]
    return {"samples": len(rows), "seconds": rows[-1]["t"] - rows[0]["t"],
            "wheel_mmps_min": min(min(abs(row["wl"]), abs(row["wr"])) for row in rows),
            "wheel_mmps_max": max(max(abs(row["wl"]), abs(row["wr"])) for row in rows),
            "head_deg": statistics.mean(math.degrees(row["head"]) for row in rows),
            "gyro_dps_min": min(gyro), "gyro_dps_max": max(gyro), "ratio_min": min(ratio), "ratio_max": max(ratio)}


def describe() -> Dict[str, Any]:
    """Everything measured directly from the recording, for the README and the tests."""
    states, commands = load()
    spacing = params.values(params.load())["tread_spacing_mm"]
    out = {"table_run_sha256": sha256(RECORDING), "rest_sha256": sha256(REST),
           "state_rows": len(states), "command_rows": len(commands),
           "span_s": states[-1]["t"] - states[0]["t"], "state_interval_ms": state_interval_ms(states),
           "rest_state_interval_ms": state_interval_ms(load_rest()), "rest": rest_noise(load_rest()),
           "cliff": cliff_event(states), "fast_turn_after_cliff": fast_turn_after_cliff(states, spacing),
           "gyro_frame": gyro_frame(states), "spin_ratios": spin_ratios(states, commands, spacing),
           "gyro_at_rest": gyro_at_rest(states),
           "moving_scatter": moving_scatter(states, commands, params.values(params.load())["gravity_mmps2"])}
    assert out["state_rows"] > 0 and out["command_rows"] > 0, "empty recording"
    return out


if __name__ == "__main__":
    print(json.dumps(describe(), indent=1, sort_keys=True))
