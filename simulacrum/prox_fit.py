"""What the recordings say about the time-of-flight sensor..

    python3 -m simulacrum.prox_fit

No MuJoCo. Reads every recording under evidence/ that carries the sensor's
distance and signal quality, and recomputes the table's fitted entries:

* tof_quality_at_100mm, tof_quality_distance_exponent: a straight line through
  log quality against log distance, over rows with a usable signal, a
  distance inside the reporting limits, and the lift out of the beam;
* tof_quality_floor, tof_weak_distance_mm, tof_weak_distance_sd_mm: the median
  quality, the median distance and the spread (from the quartiles) of rows
  with no usable signal;
* tof_lift_quality: the median quality of rows with the lift in the beam that
  read the minimum distance.

One robot and whatever happened to be in front of it: the targets' colour and
size are not known, so the signal law is a law for "the things this robot
saw", with a scatter of more than a factor of ten around it.
"""

from __future__ import annotations

import json
import math
import statistics
import sys
from pathlib import Path
from typing import Any, Dict, List, Sequence, Tuple

EVIDENCE = Path(__file__).resolve().parent.parent / "evidence"
RECORDING_GLOBS = ("hw-rec-20261115/*.jsonl", "hw-rec-20261118/*.jsonl")
Row = Tuple[float, float, float]  # distance mm, signal quality, lift height mm


def load_rows(evidence: Path = EVIDENCE) -> List[Row]:
    """Every state row with a distance, a quality and a lift height."""
    rows: List[Row] = []
    paths = sorted(p for pattern in RECORDING_GLOBS for p in evidence.glob(pattern))
    assert paths, "no recordings found"
    for path in paths:
        with open(path, "r", encoding="utf-8") as handle:
            for line in handle:
                if not line.strip():
                    continue
                rec = json.loads(line)
                distance = rec.get("prox", rec.get("prox_mm"))
                quality = rec.get("proxq", rec.get("prox_q"))
                if distance is None or quality is None or rec.get("lift") is None:
                    continue
                rows.append((float(distance), float(quality), float(rec["lift"])))
    assert len(rows) > 1000, "too few sensor rows to fit"
    return rows


def _quantile(values: Sequence[float], q: float) -> float:
    assert values and 0.0 <= q <= 1.0, "quantile of nothing"
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(q * len(ordered)))]


def fit(rows: Sequence[Row], v: Dict[str, Any]) -> Dict[str, float]:
    """The six fitted values, with the row counts they rest on."""
    low, high, q_min = v["tof_report_min_mm"], v["tof_report_max_mm"], v["tof_quality_min"]
    in_beam = lambda lift: v["tof_lift_occluding_min_mm"] <= lift <= v["tof_lift_occluding_max_mm"]
    usable = [(d, q) for d, q, lift in rows if q > q_min and low < d < high and not in_beam(lift)]
    weak = [(d, q) for d, q, _ in rows if q <= q_min]
    blocked = [q for d, q, lift in rows if in_beam(lift) and d == low]
    assert len(usable) > 100 and len(weak) > 100 and len(blocked) > 10, "a class of rows is nearly empty"
    xs = [math.log(d / 100.0) for d, _ in usable]
    ys = [math.log(q) for _, q in usable]
    mean_x, mean_y = statistics.mean(xs), statistics.mean(ys)
    slope = sum((x - mean_x) * (y - mean_y) for x, y in zip(xs, ys)) / sum((x - mean_x) ** 2 for x in xs)
    weak_d = [d for d, _ in weak]
    spread = (_quantile(weak_d, 0.75) - _quantile(weak_d, 0.25)) / 1.349  # a normal's quartiles are 1.349 sd apart
    return {
        "rows": len(rows), "usable_rows": len(usable), "weak_rows": len(weak), "blocked_rows": len(blocked),
        "tof_quality_at_100mm": math.exp(mean_y - slope * mean_x), "tof_quality_distance_exponent": slope,
        "tof_quality_floor": statistics.median(q for _, q in weak), "tof_weak_distance_mm": statistics.median(weak_d),
        "tof_weak_distance_sd_mm": spread, "tof_lift_quality": statistics.median(blocked),
        "lowest_distance_mm": min(d for d, _, _ in rows), "highest_distance_mm": max(d for d, _, _ in rows),
    }


def main() -> int:
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
    from simulacrum import params  # noqa: E402  (path set above)

    result = fit(load_rows(), params.values(params.load()))
    for name, value in result.items():
        print(f"{name}: {value:.6g}" if isinstance(value, float) else f"{name}: {value}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
