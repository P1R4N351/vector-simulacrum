"""Compare eight-packet cube force excursions within recorded action intervals."""
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

root = Path(__file__).resolve().parent
actions = {"pickup for inward transport", "reverse inward", "set cube down",
           "roll cube", "pop_a_wheelie"}
with (root / "hold-specific-force.csv").open() as stream:
    reference = next(float(row["median_magnitude_step_mg"]) for row in csv.DictReader(stream)
                     if row["window"] == "Raised hold")

rows = []
fig, axes = plt.subplots(3, 2, figsize=(10, 8), sharey=True, layout="constrained")
for ax, run in zip(axes.flat, ["R5", "W4", "W8", "R8", "R9", "R11"]):
    directory = root.parent / "unit3-wheelstand" / run
    with (directory / "actions.csv").open() as stream:
        log = list(csv.DictReader(stream))
    packets = np.loadtxt(directory / "cube.csv.gz", delimiter=",", skiprows=1).reshape(-1, 3, 7)
    assert np.all(packets[:, :, 2] == [0, 1, 2])
    assert np.all(np.ptp(packets[:, :, :2], axis=1) == 0)
    receipt, uptime = packets[:, 0, :2].T
    force = np.linalg.norm(packets[:, :, 4:], axis=2) * (1000 / 8192)
    steps = abs(np.diff(force, axis=1))
    origin = None
    for index, action in enumerate(log[:-1]):
        if action["kind"] != "begin" or action["action"] not in actions:
            continue
        start, end = float(action["sec"]), float(log[index + 1]["sec"])
        if origin is None:
            origin = start
        points = []
        for i in np.flatnonzero((receipt >= start) & (receipt < end)):
            sl = slice(i, i + 8)
            times = uptime[sl]
            if len(times) < 8 or np.any((receipt[sl] < start) | (receipt[sl] >= end)):
                continue
            gaps = np.diff(times)
            if min(gaps) <= 0 or max(gaps) > .051 or not .16 <= np.ptp(times) <= .30:
                continue
            points.append((receipt[i + 7] - origin, np.median(steps[sl])))
        peak = max(points, key=lambda p: p[1]) if points else (np.nan, np.nan)
        rows.append([run, action["action"], start, end, len(points), peak[1],
                     peak[0] + origin, sum(v >= reference for _, v in points)])
        if points:
            ax.scatter(*zip(*points), s=6, label=action["action"])
    ax.axhline(reference, color="darkred", ls="--", lw=1)
    ax.set_title(run)
    handles, labels = ax.get_legend_handles_labels()
    unique = dict(zip(labels, handles))
    if unique:
        ax.legend(unique.values(), unique.keys(), fontsize=7)
    ax.set_yscale("log")
fig.supxlabel("Seconds from first handling command in each recording")
fig.supylabel("Median change in specific-force magnitude over eight packets (mg)")
fig.suptitle("Cube handling compared with the docked damaged-fork hold (dashed)")
fig.savefig(root / "handling-vibration.png", dpi=160)
with (root / "handling-vibration.csv").open("w") as stream:
    writer = csv.writer(stream)
    writer.writerow("run action start_s end_s windows peak_mg peak_end_s windows_at_reference".split())
    writer.writerows(rows)
