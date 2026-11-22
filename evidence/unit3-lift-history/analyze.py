"""Extract paired settled heights from the docked posture and history experiment."""
import csv
import gzip
import json
from math import degrees
from pathlib import Path
from statistics import median

import matplotlib.pyplot as plt

folder = Path(__file__).resolve().parent
with gzip.open(folder / "physical.jsonl.gz", "rt") as stream:
    events = [json.loads(line) for line in stream]
states = [r for r in events if r["kind"] == "state"]
assert any(r["kind"] == "end" and r["docked"] for r in events)
assert all(r["left"] == r["right"] == 0 and r["status"] & 0x1000 for r in states)
rows = []
for event in events:
    if event["kind"] != "arrived" or event["phase"] != "measure":
        continue
    def window(phase):
        success = next(e["t"] for e in events if e["kind"] == "arrived" and
                       e["block"] == event["block"] and e["prep"] == event["prep"] and e["phase"] == phase)
        return [r for r in states if .12 <= r["t"] - success <= .6]
    end = window("measure")
    rows.append([event["block"], event["angle"], event["prep"], median(r["lift"] for r in window("middle")),
                 degrees(median(r["head"] for r in end)), median(r["lift"] for r in end), len(end)])
assert len(rows) == 8
with (folder / "holds.csv").open("w") as stream:
    writer = csv.writer(stream)
    writer.writerow(["block","head_target_deg","prep_fraction","middle_mm","head_observed_deg","full_mm","full_samples"])
    writer.writerows(rows)
fig, axes = plt.subplots(1, 2, sharey=True, figsize=(8,4), constrained_layout=True)
for axis, angle in zip(axes, [-22, 35]):
    for block in sorted({r[0] for r in rows if r[1] == angle}):
        pair = sorted([r for r in rows if r[0] == block], key=lambda r:r[2])
        axis.plot([0,1], [r[5] for r in pair], 'o-', label=f"Block {block+1}")
    axis.set(xticks=[0,1], xticklabels=["From 50%", "From 100%"], title=f"Head target {angle}°",
             xlabel="Approach to 75% before final raise", xlim=(-.2,1.2))
    axis.legend()
axes[0].set_ylabel("Full-raise encoder hold (mm)")
fig.savefig(folder/"paired.png", dpi=160)
