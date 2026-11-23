"""Compare both link-contact phases to the measured change of top-face normal."""
import csv
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from pickup_onset import action_time, read

here = Path(__file__).resolve().parent
start = action_time(here / "actions.csv", "pickup for inward transport", "sec")
optical = read(here / "glyph-raise.csv")
lights = read(here / "raise-lights.csv")
times = np.interp(optical["frame"], lights["frame"], lights["receipt_s"]) - start
measured = optical["normal_change_deg"]
names = ["Full contact", "Muted on approach", "Muted during raise", "Muted throughout"]
fig, axes = plt.subplots(2, 1, figsize=(8, 6), sharex=True, sharey=True, layout="constrained")
rows = []
for k, ax in enumerate(axes):
    for label, directory in zip(names, sys.argv[1+k*4:5+k*4], strict=True):
        folder = Path(directory)
        a = np.loadtxt(folder / "top-normal.csv", delimiter=",", skiprows=1)
        pose = read(folder / "pickup-replay.csv")
        def at(t):
            v = np.stack([np.interp(t, a[:, 0], col) for col in a[:, 1:].T], axis=-1)
            return v / np.linalg.norm(v, axis=-1, keepdims=True)
        ref = at(times[0])
        def angle(t):
            return np.rad2deg(np.arccos(np.clip(at(t) @ ref, -1, 1)))
        step = (0.25, 0.125)[k]
        window = (a[:, 0] >= times[0]-.1) & (a[:, 0] <= times[-1]+.05)
        ax.plot(a[window, 0], angle(a[window, 0]), label=label, lw=1.4)
        for frame, t, obs, sim in zip(optical["frame"], times, measured, angle(times), strict=True):
            rows.append([step, label, frame, t, obs, sim,
                         np.interp(t, pose["sec"], pose["center_height_mm"])])
    ax.axvline(2.559, ls=":", color=".5")
    ax.errorbar(times, measured, yerr=[measured-optical["jitter_p05_deg"],
                optical["jitter_p95_deg"]-measured], fmt="ko", ms=3, capsize=3, label="Recorded glyph")
    ax.set_title(f"{step:g} ms step")
    ax.set_ylabel("Normal change from frame 85 (°)")
axes[0].legend(ncol=3, fontsize=8)
axes[1].set_xlabel("Seconds after pickup command · dotted line marks lift onset")
fig.savefig(here / "approach-links.png", dpi=160)
with (here / "approach-links.csv").open("w") as stream:
    w = csv.writer(stream, lineterminator="\n")
    w.writerow("step_ms link_condition frame receipt_s_since_cmd glyph_change_deg sim_change_deg sim_center_height_mm".split())
    w.writerows(rows)
