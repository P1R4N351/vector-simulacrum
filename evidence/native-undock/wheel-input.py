"""Export SDK wheel samples during each recorded undock in receipt time."""
import csv
import gzip
import json
from pathlib import Path

runs = {}
for session in "BCDEF":
    root = Path("../unit3-physical-repeats") / session
    with open(root / "actions.csv") as f:
        end = next(float(r["receipt_s"]) for r in csv.DictReader(f)
                   if r["event"] == "end" and r["action"] == "undock")
    with gzip.open(root / "robot.csv.gz", "rt") as f:
        rows = [r for r in csv.DictReader(f) if 0 <= float(r["receipt_s"]) <= end]
    runs[session] = [[float(row[k]) for k in ("receipt_s","left_mmps","right_mmps","head_rad","lift_mm","gz")]
                     for row in rows]
print(json.dumps(runs, allow_nan=False))
