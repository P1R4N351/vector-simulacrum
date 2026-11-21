"""Centre of mass from wheelstands, read by the robot's own accelerometer..

    python3 -m simulacrum.wheelstand <recording.jsonl> [outer tread width mm]   the report, as JSON
    python3 -m simulacrum.wheelstand --expect              the balance angles the table predicts

No MuJoCo, and nothing here talks to a robot: it reads a recording made by
the recorder's listen-only "wheelstand" phase while a person tips the robot.

THE IDEA. A body resting on a round wheel balances when its centre of mass
is straight above the wheel's axle: on flat ground the contact point of a
circle is always under its centre. Tipped back onto its rear wheels by a
pitch theta, the robot balances when

    tan(theta_rear) = (com_x + wheel_base) / (com_z - rear_wheel_radius)

Tipped sideways onto one tread's outer edge (a fixed point: the edge of the
tread at tread_spacing / 2 + tread_width / 2 from the centre line, on the
ground) it balances at a roll psi where

    tan(psi_side) = edge_y / com_z

so the side stand gives com_z and the rear stand then gives com_x. Robot
frame as in mjcf.py: origin on the ground under the front axle, x forward,
z up. The table's com_x and com_z are for the lift down and the head level,
so every trial is checked against that pose and set aside if it is off.

READING THE ANGLE. While the robot is held still, the accelerometer reads
gravity. It sits in the head, pitched by the head angle and by the fitted
imu_pitch_offset_deg; the fitted offsets are subtracted first. The tilt is
then the direction of "up" in the body frame. A common scale error of the
accelerometer (it read 1.039 g at rest) cancels in the ratio.

TWO WAYS TO FIND THE BALANCE ANGLE, both from one recording:
- held: the person finds the point where the robot feels weightless and
  holds it still; the angle of each such hold is a trial.
- released: the person holds the robot at an angle, lets go, and it falls
  one way or the other. Released trials that fell each way bracket the
  balance angle; the bracket is the better estimate and is used when both
  sides are present.

Limits, stated: the real tread is a belt, not a cylinder, so the rear
contact radius and the side edge are known only to the table's ranges
(wheel_rear_radius_mm 10.25 to 11.25, tread spacing and width); the
interval reported carries those ranges. A finger on the robot during a
"held" trial pushes it, which is why released trials are preferred.
"""

from __future__ import annotations

import itertools
import json
import math
import statistics
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

from . import params, recording

STILL_GYRO_DPS = 4.0          # a hand-held robot is "still" below this on every gyro axis
STILL_MIN_ROWS = 8            # about half a second of state rows
ACCEL_G_TOLERANCE = 0.12      # a still reading must be within this fraction of 1 g
LEVEL_DEG = 6.0               # standing on all four wheels
TILT_MIN_DEG = 20.0           # a trial is tilted at least this far
CROSS_MAX_DEG = 8.0           # and no further than this about the other axis
ON_ITS_BACK_DEG = 80.0        # past this, a rear trial has fallen onto its back
ON_ITS_SIDE_DEG = 75.0        # and a side trial onto its side
FALL_WITHIN_S = 2.0           # a release reaches its next rest this soon ...
FALL_RATE_DPS = 60.0          # ... and turns at least this fast on the way
LIFT_TOL_MM = 3.0
STRETCH_DRIFT_DEG = 2.0       # a still stretch ends if the tilt drifts further than this
OUTER_WIDTH_TOL_MM = 1.0     # a ruler read to the half millimetre at each tread edge
MAX_ROWS = 100000
KINDS = ("rear", "side")


def geometry(v: Dict[str, Any], outer_width_mm: Optional[float] = None) -> Dict[str, float]:
    """The three lengths the balance angles depend on, mm.

    outer_width_mm: the width across the outside of both treads, by ruler. Given, it sets the
    side edge directly; otherwise the edge comes from the table's tread spacing and width.
    """
    edge = 0.5 * outer_width_mm if outer_width_mm is not None else 0.5 * (v["tread_spacing_mm"] + v["tread_width_mm"])
    assert edge > 0.0 and v["wheel_base_mm"] > 0.0 and v["wheel_rear_radius_mm"] > 0.0, "bad geometry"
    return {"wheel_base": v["wheel_base_mm"], "rear_radius": v["wheel_rear_radius_mm"], "edge_y": edge}


def expected_deg(com_x: float, com_z: float, g: Dict[str, float]) -> Dict[str, float]:
    """The balance angles of a robot whose centre of mass is (com_x, com_z), lift down and head level."""
    assert com_z > g["rear_radius"], "centre of mass below the rear axle: no rear balance point"
    return {"rear": math.degrees(math.atan2(com_x + g["wheel_base"], com_z - g["rear_radius"])),
            "side": math.degrees(math.atan2(g["edge_y"], com_z))}


def solve(rear_deg: float, side_deg: float, g: Dict[str, float]) -> Tuple[float, float]:
    """(com_x, com_z) in mm from the two balance angles."""
    assert 0.0 < side_deg < 90.0 and 0.0 < rear_deg < 90.0, "balance angles must be between 0 and 90 degrees"
    com_z = g["edge_y"] / math.tan(math.radians(side_deg))
    com_x = math.tan(math.radians(rear_deg)) * (com_z - g["rear_radius"]) - g["wheel_base"]
    return com_x, com_z


def edge_range(v: Dict[str, Any], outer_width_mm: Optional[float]) -> Tuple[float, float]:
    """Smallest and largest side edge distance: the ruler reading +- OUTER_WIDTH_TOL_MM / 2, or the table's ranges."""
    if outer_width_mm is not None:
        return 0.5 * (outer_width_mm - OUTER_WIDTH_TOL_MM), 0.5 * (outer_width_mm + OUTER_WIDTH_TOL_MM)
    e = params.load()["parameters"]
    return (0.5 * (e["tread_spacing_mm"]["low"] + e["tread_width_mm"]["low"]),
            0.5 * (e["tread_spacing_mm"]["high"] + e["tread_width_mm"]["high"]))


def solve_interval(rear: Tuple[float, float], side: Tuple[float, float], v: Dict[str, Any],
                   outer_width_mm: Optional[float] = None, angles_only: bool = False) -> Dict[str, List[float]]:
    """Smallest and largest (com_x, com_z) over the angle ranges and the geometry ranges.

    rear and side are (estimate, half-width) in degrees. Geometry: wheel base and rear radius over
    the table's ranges, the side edge over edge_range(). angles_only holds the geometry at its value.
    Each solution is monotonic in each input, so the extremes are at the corners of the box.
    """
    g0 = geometry(v, outer_width_mm)
    e = params.load()["parameters"]
    bases = (g0["wheel_base"],) if angles_only else (e["wheel_base_mm"]["low"], e["wheel_base_mm"]["high"])
    radii = (g0["rear_radius"],) if angles_only else (e["wheel_rear_radius_mm"]["low"], e["wheel_rear_radius_mm"]["high"])
    edges = (g0["edge_y"],) if angles_only else edge_range(v, outer_width_mm)
    xs, zs = [], []
    for base, radius, edge, r, s in itertools.product(bases, radii, edges, (rear[0] - rear[1], rear[0] + rear[1]),
                                                      (side[0] - side[1], side[0] + side[1])):
        x, z = solve(r, s, {"wheel_base": base, "rear_radius": radius, "edge_y": edge})
        xs.append(x)
        zs.append(z)
    assert len(xs) in (4, 32), "the corner count changed"
    return {"com_x_mm": [min(xs), max(xs)], "com_z_mm": [min(zs), max(zs)]}


def up_in_body(row: Dict[str, Any], v: Dict[str, Any]) -> Tuple[float, float, float]:
    """Unit vector pointing up, in the robot's body frame, from one still accelerometer reading."""
    mg = 0.001 * v["gravity_mmps2"]
    ax, ay, az = (float(row["accel"][i]) - mg * v["accel_bias_mg"][i] for i in range(3))
    angle = float(row["head"]) + math.radians(v["imu_pitch_offset_deg"])
    ux = ax * math.cos(angle) - az * math.sin(angle)
    uz = ax * math.sin(angle) + az * math.cos(angle)
    norm = math.sqrt(ux * ux + ay * ay + uz * uz)
    assert norm > 0.0, "accelerometer reads zero"
    return ux / norm, ay / norm, uz / norm


def tilt_deg(row: Dict[str, Any], v: Dict[str, Any]) -> Tuple[float, float]:
    """(pitch, roll) in degrees: pitch nose up positive, roll left side down positive. Either side may be used."""
    ux, uy, uz = up_in_body(row, v)
    # Pitch may pass 90 (a robot on its back); roll is taken against the whole of the rest of "up",
    # so a robot on its back reads roll 0, not 180.
    return math.degrees(math.atan2(ux, uz)), math.degrees(math.atan2(-uy, math.hypot(ux, uz)))


def _is_still(row: Dict[str, Any], v: Dict[str, Any]) -> bool:
    gyro_still = max(abs(math.degrees(float(g))) for g in row["gyro"]) < STILL_GYRO_DPS
    magnitude = math.sqrt(sum(float(a) ** 2 for a in row["accel"])) / v["gravity_mmps2"]
    return gyro_still and abs(magnitude - 1.0) < ACCEL_G_TOLERANCE


def still_stretches(states: Sequence[Dict[str, Any]], v: Dict[str, Any]) -> List[Tuple[int, int]]:
    """(first, last) row index of every run of at least STILL_MIN_ROWS still rows.

    A run also ends when the tilt drifts more than STRETCH_DRIFT_DEG from where it began: a hand
    tipping the robot slowly can stay under the gyro threshold while the angle changes.
    """
    out: List[Tuple[int, int]] = []
    start: Optional[int] = None
    first: Tuple[float, float] = (0.0, 0.0)
    for i in range(len(states) + 1):
        still = i < len(states) and _is_still(states[i], v)
        tilt = tilt_deg(states[i], v) if still else first
        drifted = start is not None and max(abs(tilt[0] - first[0]), abs(tilt[1] - first[1])) > STRETCH_DRIFT_DEG
        if start is not None and (not still or drifted):
            if i - start >= STILL_MIN_ROWS:
                out.append((start, i - 1))
            start = None
        if still and start is None:
            start, first = i, tilt
    return out


def _kind(pitch: float, roll: float) -> str:
    if abs(pitch) < LEVEL_DEG and abs(roll) < LEVEL_DEG:
        return "level"
    if pitch >= ON_ITS_BACK_DEG and abs(roll) < CROSS_MAX_DEG:
        return "on_its_back"
    if abs(roll) >= ON_ITS_SIDE_DEG and abs(pitch) < CROSS_MAX_DEG:
        return "on_its_side"
    if TILT_MIN_DEG <= pitch < ON_ITS_BACK_DEG and abs(roll) < CROSS_MAX_DEG:
        return "rear"
    if TILT_MIN_DEG <= abs(roll) < ON_ITS_SIDE_DEG and abs(pitch) < CROSS_MAX_DEG:
        return "side"
    return "other"


def _stretch_summary(states: Sequence[Dict[str, Any]], span: Tuple[int, int], v: Dict[str, Any]) -> Dict[str, Any]:
    rows = states[span[0]:span[1] + 1]
    tilts = [tilt_deg(r, v) for r in rows]
    pitch, roll = statistics.median(t[0] for t in tilts), statistics.median(t[1] for t in tilts)
    head = statistics.median(math.degrees(float(r["head"])) for r in rows)
    lift = statistics.median(float(r["lift"]) for r in rows)
    pose_ok = abs(head) <= v["head_angle_tol_deg"] and abs(lift - v["lift_height_min_mm"]) <= LIFT_TOL_MM
    return {"t0": rows[0]["t"], "t1": rows[-1]["t"], "rows": len(rows), "pitch_deg": pitch, "roll_deg": roll,
            "kind": _kind(pitch, roll), "head_deg": head, "lift_mm": lift, "pose_ok": pose_ok}


def _outcome(states: Sequence[Dict[str, Any]], here: Dict[str, Any], after: Optional[Dict[str, Any]]) -> str:
    """What happened when a tilted hold ended: a release falling one way or the other, or a hand moving it."""
    if after is None or after["t0"] - here["t1"] > FALL_WITHIN_S:
        return "moved_by_hand"
    peak = max((max(abs(math.degrees(float(g))) for g in r["gyro"]) for r in states
                if here["t1"] < r["t"] < after["t0"]), default=0.0)
    if peak < FALL_RATE_DPS:
        return "moved_by_hand"
    toward = {"rear": "on_its_back", "side": "on_its_side"}[here["kind"]]
    if after["kind"] == toward:
        return "fell_over"
    if after["kind"] == "level":
        return "fell_back_onto_treads"
    return "moved_by_hand"


def trials(states: Sequence[Dict[str, Any]], v: Dict[str, Any]) -> List[Dict[str, Any]]:
    """Every still stretch, its tilt and pose, and for tilted ones what followed."""
    spans = [_stretch_summary(states, s, v) for s in still_stretches(states, v)]
    for here, after in zip(spans, spans[1:] + [None]):
        here["outcome"] = _outcome(states, here, after) if here["kind"] in KINDS else None
    return spans


def balance(spans: Sequence[Dict[str, Any]], kind: str) -> Dict[str, Any]:
    """The balance angle for one kind of stand: bracketed by releases when possible, else the held angles."""
    assert kind in KINDS, "unknown stand"
    angle = "pitch_deg" if kind == "rear" else "roll_deg"
    usable = [s for s in spans if s["kind"] == kind and s["pose_ok"]]
    back = [abs(s[angle]) for s in usable if s["outcome"] == "fell_back_onto_treads"]
    over = [abs(s[angle]) for s in usable if s["outcome"] == "fell_over"]
    held = [abs(s[angle]) for s in usable if s["outcome"] == "moved_by_hand"]
    out: Dict[str, Any] = {"kind": kind, "trials": len(usable), "off_pose": sum(1 for s in spans if s["kind"] == kind
                                                                                and not s["pose_ok"]),
                           "fell_back_onto_treads": len(back), "fell_over": len(over), "held": len(held)}
    if back and over:
        low, high = max(back), min(over)
        out["method"] = "bracket" if low < high else "bracket_overlapping"
        out["estimate_deg"], out["half_width_deg"] = 0.5 * (low + high), 0.5 * abs(high - low)
    elif held:
        out["method"] = "held"
        spread = statistics.pstdev(held) if len(held) > 1 else 0.0
        out["estimate_deg"], out["half_width_deg"] = statistics.median(held), max(spread, 1.0)
    else:
        out["method"] = "none"
    return out


def report(states: Sequence[Dict[str, Any]], v: Dict[str, Any], outer_width_mm: Optional[float] = None) -> Dict[str, Any]:
    """Trials, both balance angles, and the centre of mass they give, beside the table's values."""
    assert outer_width_mm is None or 40.0 <= outer_width_mm <= 80.0, "outer tread width out of range"
    spans = trials(states, v)
    g = geometry(v, outer_width_mm)
    stands = {kind: balance(spans, kind) for kind in KINDS}
    out: Dict[str, Any] = {"trials": spans, "stands": stands, "geometry_mm": g,
                           "table": {"com_x_mm": v["com_x_mm"], "com_z_mm": v["com_z_mm"],
                                     "expected_deg": expected_deg(v["com_x_mm"], v["com_z_mm"], g)}}
    if all(stands[k]["method"] != "none" for k in KINDS):
        rear, side = stands["rear"], stands["side"]
        com_x, com_z = solve(rear["estimate_deg"], side["estimate_deg"], g)
        out["com_mm"] = {"com_x_mm": com_x, "com_z_mm": com_z,
                         "interval": solve_interval((rear["estimate_deg"], rear["half_width_deg"]),
                                                    (side["estimate_deg"], side["half_width_deg"]), v, outer_width_mm),
                         "interval_from_angles_alone": solve_interval(
                             (rear["estimate_deg"], rear["half_width_deg"]), (side["estimate_deg"], side["half_width_deg"]),
                             v, outer_width_mm, angles_only=True),
                         "side_edge_from": "ruler" if outer_width_mm is not None else "table ranges"}
    else:
        out["com_mm"] = {"not_computable": "need at least one usable rear and one usable side trial"}
    return out


def describe(path: Path, outer_width_mm: Optional[float] = None) -> Dict[str, Any]:
    states, _ = recording.load(path)
    return report(states, params.values(params.load()), outer_width_mm)


def main(argv: List[str]) -> int:
    v = params.values(params.load())
    if argv == ["--expect"]:
        print(json.dumps(expected_deg(v["com_x_mm"], v["com_z_mm"], geometry(v)), indent=1))
        return 0
    if len(argv) not in (1, 2):
        print("usage: python3 -m simulacrum.wheelstand <recording.jsonl> [outer tread width mm] | --expect")
        return 2
    width = float(argv[1]) if len(argv) == 2 else None
    print(json.dumps(describe(Path(argv[0]), width), indent=1, default=str))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
