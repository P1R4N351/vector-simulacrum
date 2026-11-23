"""Report physical lift holds over a common window after SDK action success."""
import csv
import gzip
import json
from pathlib import Path
from statistics import median

import matplotlib.pyplot as plt

folder = Path(__file__).resolve().parent
with gzip.open(folder / "paired.jsonl.gz", "rt") as stream:
    records = [json.loads(line) for line in stream]
commands = [r for r in records if r["kind"] == "command"]
done = {r["n"]: r for r in records if r["kind"] == "arrived"}
states = [r for r in records if r["kind"] == "state"]
assert len(commands) == len(done) == 17 and all(r["result"] == 0 for r in done.values())
assert all(r["left"] == r["right"] == 0 and r["status"] & 0x1000 for r in states)
colors = {0: "#2467a4", .75: "#c4511a"}
fig, axes = plt.subplots(1, 2, figsize=(10, 4), constrained_layout=True)
rows = []
for index, cmd in enumerate(commands):
    if cmd["pos"] != 1:
        continue
    start = commands[index-1]["pos"]
    success = done[cmd["n"]]["t"]
    series = [r for r in states if cmd["t"] <= r["t"] < success+.75]
    hold = [r["lift"] for r in series if .12 <= r["t"]-success <= .6]
    rows.append((len(rows)+1, start, median(hold), len(hold)))
    axes[0].plot([r["t"]-cmd["t"] for r in series], [r["lift"] for r in series],
                 color=colors[start], alpha=.65)
    axes[1].scatter(len(rows), median(hold), color=colors[start])
for start, color in colors.items():
    axes[0].plot([], [], color=color, label=f"From {start:.0%}")
axes[0].legend()
axes[0].set(xlabel="Seconds since raise command", ylabel="SDK lift height (mm)")
axes[1].set(xlabel="Raise in execution order", ylabel="Settled encoder height (mm)", xticks=range(1,9))
fig.savefig(folder / "paired.png", dpi=160)
with (folder / "holds.csv").open("w") as stream:
    writer = csv.writer(stream)
    writer.writerow(["raise", "start_fraction", "median_mm", "samples"])
    writer.writerows(rows)
