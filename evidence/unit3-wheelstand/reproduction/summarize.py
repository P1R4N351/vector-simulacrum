"""Collect the reproduction runs in runs/ into ablation.csv, lhs.csv and placement.csv."""
import csv
import json
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
KEYS = ("peak_pitch_deg", "t_peak_s", "hold_pitch_deg", "pitch_rmse_deg", "travel_mm", "yaw_deg", "peak_g", "taps")


def load(path):
    rows = [json.loads(line) for line in path.read_text().splitlines() if line.startswith("{")]
    assert len(rows) == 5, f"{path.name}: expected a header and four cases, got {len(rows)} rows"
    return rows[0], rows[1:]


def flat(cases):
    out = {}
    for k in KEYS:
        values = [float(c[k]) for c in cases]
        out[k + "_median"] = float(np.median(values))
        out[k + "_min"], out[k + "_max"] = min(values), max(values)
    out["flipped_cases"] = sum(float(c["peak_pitch_deg"]) > 100.0 for c in cases)
    return out


def write(name, rows):
    assert rows, f"no rows for {name}"
    with (HERE / name).open("w") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]), lineterminator="\n")
        w.writeheader()
        w.writerows(rows)


def main():
    runs = HERE / "runs"
    ablation, lhs, placement = [], [], []
    physical = {}
    for path in sorted(runs.glob("*.jsonl")):
        head, cases = load(path)
        physical[head["trial"]] = head["physical"]
        parts = path.stem.split("_")
        if parts[0] == "ablation":
            ablation.append(dict(trial=parts[1], tip_supports=parts[2], lift=parts[3], **flat(cases)))
        elif parts[0] == "lhs":
            lhs.append(dict(trial=parts[1], stall_mNm=float(parts[2]), noload_radps=float(parts[3]),
                            com_x_mm=float(parts[4]), com_z_mm=float(parts[5]), **flat(cases)))
        elif parts[0] == "placement":
            placement.append(dict(trial=parts[1], cube_forward_mm=float(parts[2]), **flat(cases)))
    write("ablation.csv", ablation)
    write("lhs.csv", sorted(lhs, key=lambda r: r["pitch_rmse_deg_median"]))
    write("placement.csv", placement)
    write("physical.csv", [dict(trial=t, **p) for t, p in sorted(physical.items())])
    print(len(ablation), len(lhs), len(placement))


if __name__ == "__main__":
    main()
