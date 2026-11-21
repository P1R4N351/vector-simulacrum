"""Compare the model with the recording of the real robot..

    the GPU host python replay_recording.py
    the GPU host python replay_recording.py --set wheel_stall_torque_mNm=25 --set command_delay_ms=30
    the GPU host python replay_recording.py --sensitivity

Replays every commanded segment of evidence/hw-rec-20261115/table-run1.jsonl
through VectorSim with the parameter table as it stands (or with overrides)
and prints the error of each, with its sample count, split into the
segments the fit used and the ones it never saw. With --sensitivity it also changes one value at
a time and prints the two tread scores, which shows what the recording can
and cannot tell apart. It reads a file; it never talks to a robot. The last
line is one JSON object.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence

sys.path.insert(0, str(Path(__file__).resolve().parent))

from simulacrum import params, recording, replay  # noqa: E402

EXIT_OK, EXIT_USAGE = 0, 2
TORQUE, INERTIA = "wheel_stall_torque_mNm", "wheel_armature_gcm2"
# (what is changed, {parameter: (how, number)}); "x" multiplies the table's value, "=" replaces it.
SENSITIVITY = (
    ("torque x0.5 and inertia x0.5 (same ratio)", {TORQUE: ("x", 0.5), INERTIA: ("x", 0.5)}),
    ("torque x2 and inertia x2 (same ratio)", {TORQUE: ("x", 2.0), INERTIA: ("x", 2.0)}),
    ("torque x4 and inertia x4 (same ratio)", {TORQUE: ("x", 4.0), INERTIA: ("x", 4.0)}),
    ("torque x0.5 alone", {TORQUE: ("x", 0.5)}),
    ("torque x2 alone", {TORQUE: ("x", 2.0)}),
    ("inertia x0.5 alone", {INERTIA: ("x", 0.5)}),
    ("inertia x2 alone", {INERTIA: ("x", 2.0)}),
    ("no-load speed 230", {"tread_noload_speed_mmps": ("=", 230.0)}),
    ("no-load speed 320", {"tread_noload_speed_mmps": ("=", 320.0)}),
    ("friction share 0.06", {"tread_friction_power_fraction": ("=", 0.06)}),
    ("friction share 0.15", {"tread_friction_power_fraction": ("=", 0.15)}),
    ("command delay 40", {"command_delay_ms": ("=", 40.0)}),
    ("command delay 80", {"command_delay_ms": ("=", 80.0)}),
    ("tread friction 0.6", {"tread_friction": ("=", 0.6)}),
    ("tread friction 1.2", {"tread_friction": ("=", 1.2)}),
    ("mass 150 g", {"total_mass_g": ("=", 150.0)}),
    ("mass 185 g", {"total_mass_g": ("=", 185.0)}),
    ("com_x -18", {"com_x_mm": ("=", -18.0)}),
    ("com_x -30", {"com_x_mm": ("=", -30.0)}),
)


def print_wheels(title: str, results: Sequence[Dict[str, Any]], nominal: Dict[str, Any]) -> None:
    assert len(results) > 0, "no wheel segments"
    print(f"{title}: score {replay.wheel_score(results, nominal):.3f} (mean error as a fraction of each segment's scale)")
    print("  segment        samples  wheel speed rmse mm/s  (of command)  gyro z rmse deg/s")
    for r in results:
        gyro = f"{r['gyro_rmse_dps']:17.1f}" if r["is_spin"] else "                -"
        print(f"  {r['label']:13s}  {r['n']:7d}  {r['wheel_rmse_mmps']:21.1f}  {r['wheel_rmse_mmps'] / r['command_mmps']:12.2f}  {gyro}")


def print_servos(title: str, results: Sequence[Dict[str, Any]], nominal: Dict[str, Any]) -> None:
    assert len(results) > 0, "no head or lift segments"
    print(f"{title}: score {replay.servo_score(results, nominal):.3f} (mean error as a fraction of the joint's range)")
    for r in results:
        error = f"{r['angle_rmse_deg']:.1f} deg" if r["kind"] == "head" else f"{r['height_rmse_mm']:.1f} mm"
        print(f"  {r['label']:13s}  {r['n']:7d} samples  rmse {error}")


def run(values: Dict[str, Any]) -> Dict[str, Any]:
    """All four groups of segments, compared. Returns scores and per-segment results."""
    groups = {"fit_wheels": replay.FIT_WHEELS, "hold_wheels": replay.HOLD_WHEELS,
              "fit_servos": replay.FIT_SERVOS, "hold_servos": replay.HOLD_SERVOS}
    out: Dict[str, Any] = {}
    for name, labels in groups.items():
        results = replay.compare(values, labels)
        score = replay.wheel_score(results, values) if "wheels" in name else replay.servo_score(results, values)
        out[name] = {"score": round(score, 4), "segments": results}
    assert set(out) == set(groups), "a group was not compared"
    return out


def tread_scores(values: Dict[str, Any]) -> List[float]:
    """The two tread scores for one set of values: the fit's segments, then the held-out ones."""
    scores = [round(replay.wheel_score(replay.compare(values, labels), values), 3)
              for labels in (replay.FIT_WHEELS, replay.HOLD_WHEELS)]
    assert len(scores) == 2, "need a fit score and a held-out score"
    return scores


def sensitivity(values: Dict[str, Any]) -> Dict[str, List[float]]:
    """Change one thing at a time and score the treads. Nothing here is adopted."""
    out = {"table values": tread_scores(values)}
    for title, changes in SENSITIVITY:
        changed = dict(values)
        for name, (how, number) in changes.items():
            assert name in values and how in ("x", "="), "bad sensitivity entry"
            changed[name] = values[name] * number if how == "x" else number
        out[title] = tread_scores(changed)
    print("tread scores with one thing changed (fit set, held out); lower is better; none of these is adopted:")
    for title, scores in out.items():
        print(f"  {title:44s} {scores[0]:.3f}, {scores[1]:.3f}")
    return out


def _inside(name: str) -> Optional[Path]:
    """A recording inside this skill, or None."""
    root = Path(__file__).resolve().parent
    path = (root / name).resolve()
    if root not in path.parents or not path.is_file():
        print(f"error: the recording must be a file inside {root}, got {name!r}", file=sys.stderr)
        return None
    return path


def _steady(trace: Sequence[Any], duration_s: float) -> Dict[str, float]:
    """Mean |gyro z| and mean |wheel speed| over the second half of the commanded time, model and robot."""
    rows = [(m, r) for t, m, r in trace if 0.5 * duration_s <= t <= duration_s]
    assert rows, "no samples in the second half of the segment"
    mean = lambda xs: sum(xs) / len(xs)
    return {"n": len(rows),
            "robot_turn_dps": round(mean([abs(math.degrees(r["gyro"][2])) for _, r in rows]), 1),
            "model_turn_dps": round(mean([abs(m["gyro_z_dps"]) for m, _ in rows]), 1),
            "robot_wheel_mmps": round(mean([0.5 * (abs(r["wl"]) + abs(r["wr"])) for _, r in rows]), 1),
            "model_wheel_mmps": round(mean([0.5 * (abs(m["wl"]) + abs(m["wr"])) for m, _ in rows]), 1)}


def other_wheels(values: Dict[str, Any], name: str) -> int:
    """Every wheel segment of another recording through the model, scored as one held-out set."""
    path = _inside(name)
    if path is None:
        return EXIT_USAGE
    states, commands = recording.load(path)
    segs = {label: seg for label, seg in recording.segments(commands, values).items() if seg.kind == "wheels"}
    if not segs:
        print(f"error: no wheel segments in {name}", file=sys.stderr)
        return EXIT_USAGE
    results = []
    for label, seg in segs.items():
        result = replay.replay(seg, values, states)
        result.update(_steady(result.pop("trace"), seg.t_stop - seg.t_cmd))
        results.append(result)
    print_wheels(f"wheel segments of {name}", results, values)
    print("  segment        turn deg/s robot  model   wheel mm/s robot  model   (second half, gyro z)")
    for r in results:
        print(f"  {r['label']:13s}  {r['robot_turn_dps']:16.1f}  {r['model_turn_dps']:5.1f}  "
              f"{r['robot_wheel_mmps']:16.1f}  {r['model_wheel_mmps']:5.1f}")
    print("REPLAY " + json.dumps({"recording": name, "score": round(replay.wheel_score(results, values), 4),
                                  "segments": results}, sort_keys=True))
    return EXIT_OK


def other_servos(values: Dict[str, Any], name: str) -> int:
    """Every head and lift move of another recording through the model, scored as one held-out set."""
    path = _inside(name)
    if path is None:
        return EXIT_USAGE
    _, commands = recording.load(path)
    labels = [label for label, seg in recording.segments(commands, values).items() if seg.kind in ("head", "lift")]
    if not labels:
        print(f"error: no head or lift moves in {name}", file=sys.stderr)
        return EXIT_USAGE
    results = replay.compare(values, labels, path)
    print_servos(f"head and lift moves of {name}", results, values)
    by_kind = {kind: round(replay.servo_score([r for r in results if r["kind"] == kind], values), 4)
               for kind in ("head", "lift") if any(r["kind"] == kind for r in results)}
    print("REPLAY " + json.dumps({"recording": name, "score": round(replay.servo_score(results, values), 4),
                                  "by_kind": by_kind, "segments": results}, sort_keys=True))
    return EXIT_OK


def main(argv: List[str]) -> int:
    parser = argparse.ArgumentParser(description="Replay the real-robot recording through the model.")
    parser.add_argument("--set", action="append", default=[], metavar="NAME=NUMBER", help="override one parameter")
    parser.add_argument("--sensitivity", action="store_true", help="also score the treads with one value changed at a time")
    parser.add_argument("--servos-of", metavar="RECORDING", help="instead, replay every head and lift move of another "
                        "recording (a file inside this skill), which the fit never saw")
    parser.add_argument("--wheels-of", metavar="RECORDING", help="instead, replay every wheel segment of another "
                        "recording, with the steady turn rate and wheel speed of model and robot side by side")
    args = parser.parse_args(argv)
    values = params.values(params.load())
    for pair in args.set:
        name, sep, text = pair.partition("=")
        if not sep or name not in values:
            print(f"error: --set wants a known name=number, got {pair!r}", file=sys.stderr)
            return EXIT_USAGE
        values[name] = float(text)
    if args.servos_of is not None:
        return other_servos(values, args.servos_of)
    if args.wheels_of is not None:
        return other_wheels(values, args.wheels_of)
    out = run(values)
    print_wheels("wheel segments the fit used", out["fit_wheels"]["segments"], values)
    print_wheels("wheel segments held out", out["hold_wheels"]["segments"], values)
    print_servos("head and lift segments the fit used", out["fit_servos"]["segments"], values)
    print_servos("head and lift segments held out", out["hold_servos"]["segments"], values)
    if args.sensitivity:
        out["sensitivity"] = sensitivity(values)
    print("REPLAY " + json.dumps(out, sort_keys=True))
    return EXIT_OK


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
