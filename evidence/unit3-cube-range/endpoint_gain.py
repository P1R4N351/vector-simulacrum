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
