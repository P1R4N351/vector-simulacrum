"""Check how much of the printed-fork endpoint residual follows its predicted direction."""
import csv
import math
from pathlib import Path

import matplotlib.pyplot as plt

here = Path(__file__).resolve().parent


def read(name):
    with (here / name).open() as stream:
        return list(csv.DictReader(stream))


sim = {int(r["trial"]): (float(r["forward_mm"]), float(r["left_mm"]))
       for r in read("range-contact-transfer.csv") if r["method"] == "camera"}
physical = read("../unit3-paired-cube-push/comparison.csv")
fit = [r for r in physical if int(r["trial"]) in (2, 3)]
axes = ("forward", "left")
gain = sum(sum(v * float(r["over_" + k]) for k, v in zip(axes, sim[int(r["trial"])]))
           for r in fit) / sum(math.hypot(*sim[int(r["trial"])]) ** 2 for r in fit)

fig, panels = plt.subplots(1, 2, figsize=(9, 3.7), layout="constrained")
rows = []
for panel, subset, title in zip(panels, [[1], [2, 3, 4, 5]], ["Sideways sweep", "Repeated pushes"]):
    for r in physical:
        trial = int(r["trial"])
        if trial not in subset:
            continue
        x = sim[trial]
        scaled = tuple(gain * v for v in x)
        endpoints = [tuple(float(r[view + "_" + k]) for k in axes) for view in ("over", "on")]
        for series, xy, symbol, color in zip(
                ["Replay", "Scaled replay", "Overhead", "Onboard + odometry"],
                [x, scaled, *endpoints], ["x", "+", "o", "^"], ["#333333", "#8c42a8", "#167496", "#db741d"]):
            panel.scatter(*xy, marker=symbol, color=color, label=series if trial == subset[0] else None)
        panel.plot([x[0], scaled[0]], [x[1], scaled[1]], color="gray", linewidth=.8)
        panel.annotate(str(trial), endpoints[0], xytext=(4, 4), textcoords="offset points")
        for view, xy in zip(("overhead", "onboard"), endpoints):
            rows.append((trial, view, gain, math.dist(x, xy), math.dist(scaled, xy)))
    panel.set(title=title, xlabel="Dock-forward displacement (mm)", ylabel="Dock-left displacement (mm)", aspect="equal")
    panel.margins(.18)
panels[1].legend(fontsize=8, loc="upper left", bbox_to_anchor=(1, 1))
fig.savefig(here / "endpoint-gain.png", dpi=160)
with (here / "endpoint-gain.csv").open("w", newline="") as stream:
    writer = csv.writer(stream)
    writer.writerow(["trial", "reference", "gain_fit_overhead_2_3", "replay_error_mm", "scaled_error_mm"])
    writer.writerows(rows)

# Score identical overhead frame times, excluding the 1.5 s replay lead-in.
import gzip
import numpy as np

with gzip.open(here.parent / "unit3-printed-fork-contact/trajectories.csv.gz", "rt") as stream:
    paths = [r for r in csv.DictReader(stream)
             if (r["fork"], r["heading"], r["drive"]) == ("stl", "overhead", "weld")
             and all(float(r[k]) == 0 for k in ("depth_offset", "lateral_offset", "height_offset"))]
frames = read("../unit3-paired-cube-push/traces.csv")
fig, panels = plt.subplots(2, 3, figsize=(10, 5), layout="constrained", sharex=True)
for col, trial in enumerate((1, 4, 5)):
    path = np.array([[float(r[k]) for k in ("time", "forward_mm", "left_mm")]
                     for r in paths if int(r["trial"]) == trial])
    obs = np.array([[float(r[k]) for k in ("seconds", "forward_mm", "left_mm")]
                    for r in frames if int(r["trial"]) == trial and r["camera"] == "frames"])
    t = path[:, 0] - 1.5
    obs = obs[(obs[:, 0] >= 0) & (obs[:, 0] <= t[-1])]
    sampled = np.column_stack([np.interp(obs[:, 0], t, path[:, k]) for k in (1, 2)])
    rms = [np.sqrt(np.mean(np.sum((obs[:, 1:] - g * sampled) ** 2, axis=1))) for g in (1, gain)]
    panels[0, col].set_title(f"Trial {trial} · {len(obs)} frames\nRMS {rms[0]:.2f} → {rms[1]:.2f} mm")
    for k, axis in enumerate(axes, 1):
        ax = panels[k-1, col]
        ax.scatter(obs[:, 0], obs[:, k], s=10, label="Overhead", color="#167496")
        ax.plot(t, path[:, k], label="Replay", color="#333333")
        ax.plot(t, gain * path[:, k], "--", label="Scaled", color="#8c42a8")
        ax.set(xlim=(0, t[-1]), ylabel=axis.capitalize() + " (mm)")
    panels[1, col].set_xlabel("Time after approach command (s)")
panels[1, 0].legend(fontsize=8)
fig.savefig(here / "travel-gain.png", dpi=160)
