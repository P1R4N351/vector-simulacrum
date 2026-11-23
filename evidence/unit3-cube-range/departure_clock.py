"""Compare observed pair changes with possible acquisitions before dock release."""
import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
fig, axes = plt.subplots(2, 4, figsize=(12, 6), layout="constrained", sharey=True)
rows = []
for run, ax in zip("aefgjoq", axes.flat):
    path, = here.parent.glob(f"hw-rec-*/run-{run}-*.jsonl")
    records = [json.loads(line) for line in path.read_text().splitlines()]
    states = [r for r in records if r.get("k") == "s" and "proxq" in r]
    t = np.array([r["t"] for r in states])
    contact = np.array([bool(r["status"] & 4096) for r in states])
    speed = np.array([max(abs(r["wl"]), abs(r["wr"])) for r in states])
    pair = np.array([[r["prox"], r["proxq"]] for r in states])
    release, = 1 + np.flatnonzero(contact[:-1] & ~contact[1:])
    # Last contiguous reports of rolling while the charger flag is still set.
    start = release
    while start > 0 and contact[start-1] and speed[start-1] > 0:
        start -= 1
    assert release - start >= 2
    ix = slice(start, release)
    elapsed = t[ix] - t[start]
    change = np.any(pair[ix][1:] != pair[ix][:-1], axis=1)
    observed = np.r_[0, np.cumsum(change)]
    # Max acquisitions over this window for unknown initial clock phase.
    # Multiple acquisitions inside one SDK interval can yield only one pair change.
    def envelope(period):
        # Phase predictions change only at these sample-boundary crossings.
        phases = np.unique(np.r_[0, np.mod(-elapsed, period)])
        candidates = (phases + np.r_[phases[1:], period]) / 2
        indices = np.floor((elapsed + candidates[:, None]) / period)
        return np.cumsum(np.column_stack((np.zeros(len(candidates)),
                                         np.diff(indices, axis=1) > 0)), axis=1).max(axis=0)
    slow, fast = envelope(1.024), envelope(.03072)
    rows.append([run, release-start, 1000*elapsed[-1], observed[-1], int(slow[-1]), int(fast[-1]),
                 1000*(t[release]-t[release-1])])
    for data, label, color, style in [(fast,"Driven ceiling","#319b76","--"),
                                      (slow,"Dock-only ceiling","#bb683b",":"),
                                      (observed,"Recorded changes","#253761","-")]:
        ax.step(1000*(t[ix]-t[release]), data, where="post", label=label, c=color, ls=style)
    ax.set_title(f"Run {run.upper()}")
    ax.set_xlabel("ms before dock flag clears")
for ax in axes[:, 0]:
    ax.set_ylabel("Cumulative pair changes")
axes.flat[-1].axis("off")
axes.flat[-1].legend(*axes.flat[0].get_legend_handles_labels(), loc="center")
fig.savefig(here/"departure-clock.png", dpi=160)
with (here/"departure-clock.csv").open("w", newline="") as stream:
    writer = csv.writer(stream, lineterminator="\n")
    writer.writerow(["run","rolling_docked_rows","window_ms","observed_changes","slow_max",
                     "fast_max","last_on_to_first_off_ms"])
    writer.writerows(rows)
