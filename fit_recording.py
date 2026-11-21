"""Fit model parameters to the recording of the real robot..

    the GPU host python fit_recording.py
    the GPU host python fit_recording.py --servos     (the head and lift only; the treads are left as they are)
    the GPU host python fit_recording.py --servo-delay   (the head and lift delay only)
    the GPU host python fit_recording.py --wheels-held-noload   (the treads, the table's no-load speed held)

Searches the tread drive and timing parameters so that the replayed model
matches the recording on the fit segments only (the 30 and 100 mm/s spins
and the 40 mm/s steps; one head move each way and the lift going up), then
scores the held-out segments (the 60 mm/s spins, the 80 mm/s steps, another
head move and the lift going down) with the fitted values. It changes no
file: it prints the values, and the table is edited by hand from them.

The search is coordinate descent on a fixed grid of steps, a fixed number
of sweeps, in at most 4 worker processes. It reads a file; it never talks
to a robot.
"""

from __future__ import annotations

import json
import multiprocessing
import sys
from pathlib import Path
from typing import Any, Dict, List, Sequence, Tuple

sys.path.insert(0, str(Path(__file__).resolve().parent))

from simulacrum import params, replay  # noqa: E402

WORKERS = 4
SWEEPS = 4
# name, kind of step, steps, low bound, high bound
WHEEL_SEARCH = (
    ("command_delay_ms", "add", (-40.0, -20.0, -10.0, -5.0, 5.0, 10.0, 20.0, 40.0), 0.0, 300.0),
    ("tread_friction_power_fraction", "mul", (0.6, 0.8, 0.9, 1.1, 1.3, 1.7), 0.02, 0.3),
    ("tread_noload_speed_mmps", "mul", (0.85, 0.93, 0.97, 1.03, 1.08, 1.17), 250.0, 450.0),
    ("wheel_stall_torque_mNm", "mul", (0.6, 0.8, 1.25, 1.6, 2.5, 4.0), 12.0, 1000.0),
    ("wheel_armature_gcm2", "mul", (0.5, 0.7, 0.85, 1.2, 1.4, 2.0), 300.0, 5000.0),
)
SERVO_SEARCH = (
    ("servo_command_delay_ms", "add", (-40.0, -20.0, -10.0, -5.0, 5.0, 10.0, 20.0, 40.0, 80.0), 0.0, 400.0),
    ("head_speed_max_radps", "mul", (0.5, 0.7, 0.85, 0.93, 1.08, 1.2, 1.4, 2.0, 3.0), 1.0, 10.0),
    ("lift_speed_max_radps", "mul", (0.5, 0.7, 0.85, 0.93, 1.08, 1.2, 1.4, 2.0, 3.0), 1.0, 10.0),
)

# The treads with the no-load speed held where the saturated spins of REC2 run F put it.
WHEELS_HELD_NOLOAD = tuple(spec for spec in WHEEL_SEARCH if spec[0] != "tread_noload_speed_mmps")
# The delay alone, the speed caps left as the table has them (the ramp's own acceleration sets the speeds).
DELAY_SEARCH = SERVO_SEARCH[:1]


def _score(job: Tuple[Dict[str, Any], Tuple[str, ...], bool]) -> float:
    values, labels, wheels = job
    results = replay.compare(values, labels)
    return replay.wheel_score(results, values) if wheels else replay.servo_score(results, values)


def _candidates(values: Dict[str, Any], spec: Tuple[Any, ...]) -> List[float]:
    name, kind, steps, low, high = spec
    here = float(values[name])
    raw = [here + s if kind == "add" else here * s for s in steps]
    if name.endswith("_ms"):
        raw = [5.0 * round(x / 5.0) for x in raw]
    out = sorted({min(max(x, low), high) for x in raw} - {here})
    assert all(low <= x <= high for x in out), "candidate outside its bounds"
    return out


def descend(pool: Any, values: Dict[str, Any], search: Sequence[Tuple[Any, ...]], labels: Tuple[str, ...],
            wheels: bool) -> Tuple[Dict[str, Any], float, int]:
    """Coordinate descent. Returns the best values found, their score and the number of replays scored."""
    best = dict(values)
    best_score = _score((best, labels, wheels))
    evaluations = 1
    for _ in range(SWEEPS):
        improved = False
        for spec in search:
            options = _candidates(best, spec)
            scores = pool.map(_score, [(dict(best, **{spec[0]: x}), labels, wheels) for x in options])
            evaluations += len(options)
            if options and min(scores) < best_score:
                best_score = min(scores)
                best[spec[0]] = options[scores.index(best_score)]
                improved = True
        if not improved:
            break
    assert best_score <= _score((values, labels, wheels)) + 1e-12, "the search made the fit worse"
    return best, best_score, evaluations


def report(title: str, before: float, after: float) -> None:
    assert before >= 0.0 and after >= 0.0, "scores are not negative"
    print(f"{title}: {before:.3f} before, {after:.3f} after")


def main(argv: Sequence[str]) -> int:
    modes = {(): WHEEL_SEARCH + SERVO_SEARCH, ("--servos",): SERVO_SEARCH, ("--servo-delay",): DELAY_SEARCH,
             ("--wheels-held-noload",): WHEELS_HELD_NOLOAD}
    if tuple(argv) not in modes:
        print("usage: fit_recording.py [--servos | --servo-delay | --wheels-held-noload]", file=sys.stderr)
        return 2
    servo_search = DELAY_SEARCH if tuple(argv) == ("--servo-delay",) else SERVO_SEARCH
    wheels_only = tuple(argv) == ("--wheels-held-noload",)
    servos_only = bool(argv) and not wheels_only
    start = params.values(params.load())
    with multiprocessing.get_context("spawn").Pool(WORKERS) as pool:
        if servos_only:
            fitted, wheel_fit, n_wheels = dict(start), _score((start, replay.FIT_WHEELS, True)), 0
        else:
            wheel_search = WHEELS_HELD_NOLOAD if wheels_only else WHEEL_SEARCH
            fitted, wheel_fit, n_wheels = descend(pool, start, wheel_search, replay.FIT_WHEELS, True)
        if wheels_only:
            servo_fit, n_servos = _score((fitted, replay.FIT_SERVOS, False)), 0
        else:
            fitted, servo_fit, n_servos = descend(pool, fitted, servo_search, replay.FIT_SERVOS, False)
    changed = {spec[0]: fitted[spec[0]] for spec in modes[tuple(argv)]}
    print(f"replays scored: {n_wheels} for the treads, {n_servos} for head and lift")
    for name, value in changed.items():
        print(f"  {name}: {start[name]} -> {round(value, 5)}")
    report("wheel segments the fit used", _score((start, replay.FIT_WHEELS, True)), wheel_fit)
    report("wheel segments held out", _score((start, replay.HOLD_WHEELS, True)),
           _score((fitted, replay.HOLD_WHEELS, True)))
    report("head and lift segments the fit used", _score((start, replay.FIT_SERVOS, False)), servo_fit)
    report("head and lift segments held out", _score((start, replay.HOLD_SERVOS, False)),
           _score((fitted, replay.HOLD_SERVOS, False)))
    print("FIT " + json.dumps({name: round(value, 5) for name, value in changed.items()}, sort_keys=True))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
