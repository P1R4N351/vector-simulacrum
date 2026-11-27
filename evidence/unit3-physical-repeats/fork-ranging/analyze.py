"""Score the effective lift return across independent recording sessions."""
import csv
import gzip
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

HERE = Path(__file__).resolve().parent


def records(session, name):
    with gzip.open(HERE.parent / session / (name + ".csv.gz"), "rt") as f:
        return list(csv.DictReader(f))


def stable_returns(session):
    states = records(session, "robot")
    times = np.array([float(s["receipt_s"]) for s in states])
    heights = np.array([float(s["lift_mm"]) for s in states])
    previous = None
    rows = []
    for r in records(session, "range"):
        if not r["distance_mm"]:
            continue
        sample = (float(r["distance_mm"]), float(r["signal_quality"]))
        if sample == previous:
            continue
        previous = sample
        t = float(r["receipt_s"])
        window = t + np.array([-.15, -.10, -.05, 0, .05])
        if window[0] < times[0] or window[-1] > times[-1]:
            continue
        h = np.interp(window, times, heights)
        if 34 <= h[3] <= 56 and np.max(np.abs(h - h[3])) <= .8:
            rows.append((h[3], *sample))
    return np.array(rows)


sessions = {s: stable_returns(s) for s in "ABCDEF"}
old = 11.7
fit = np.median([np.median(sessions[s][:, 2]) for s in "ABC"])
fig, axes = plt.subplots(1, 3, sharey=True, figsize=(10, 3.7), layout="constrained")
with (HERE / "scores.csv").open("w", newline="") as f:
    writer = csv.writer(f, lineterminator="\n")
    writer.writerow(["session", "split", "n", "median_quality", "old_mae", "new_mae", "distance_mae_mm"])
    for s, (height, distance, quality) in ((s, a.T) for s, a in sessions.items()):
        writer.writerow([s, "fit" if s in "ABC" else "held-out", len(quality), np.median(quality),
                         np.mean(abs(quality-old)), np.mean(abs(quality-fit)), np.mean(abs(distance-30))])
        if s in "DEF":
            ax = axes["DEF".index(s)]
            ax.scatter(height, quality, alpha=.55, s=13, color="#167596", label="Physical returns")
            ax.axhline(old, ls="--", color="#776c63", label="Previous model")
            ax.axhline(fit, color="#bc4c26", label="Printed fork fit")
            ax.set(title=f"Held-out session {s}", xlabel="Lift height (mm)", xlim=(33,57))
axes[0].set_ylabel("SDK signal quality")
axes[0].legend(fontsize=8, loc="center left")
fig.savefig(HERE / "held-out.png", dpi=170)
print("Fitted signal quality:", fit)
