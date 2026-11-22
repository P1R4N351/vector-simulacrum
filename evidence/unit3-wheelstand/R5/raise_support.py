"""Resolve the simulated load transfer by recorded wrist height on each pickup."""
import csv
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
earlier = here.parents[1] / "unit3-cube-accelerometer"
prior = np.genfromtxt(earlier/"pickup-support.csv", delimiter=",", names=True, dtype=None, encoding="utf8")
prior = prior[prior["initialization"] == "Before command"]
current = np.loadtxt(Path(sys.argv[1])/"contact-load.csv", delimiter=",", skiprows=1)
dt = np.median(np.diff(current[:, 0]))
width = round(.01/dt)
windows = current[:len(current)//width*width].reshape(-1, width, current.shape[1])
r5 = windows.mean(axis=1)
with (here/"raise-angle.csv").open() as stream:
    measured = list(csv.DictReader(stream))
fig, axes = plt.subplots(2, 2, figsize=(9, 6), sharex=True, layout="constrained")
rows = []
for col, (label, height, shelf, lift) in enumerate([
    ("Earlier", prior["lift_mm"], prior["shelf_z"], prior["fork_shell_z"]+prior["fork_rubber_z"]+prior["link_z"]),
    ("R5", windows[:, -1, 1], r5[:, 2], r5[:, 3:6].sum(axis=1)),
]):
    keep = (height >= 32) & (height < 90)
    ax = axes[0, col]
    ax.plot(height[keep], shelf[keep], label="Shelf")
    ax.plot(height[keep], lift[keep], label="Fork and links")
    ax.axhline(1, color=".5", ls=":")
    ax.set(title=label, ylabel="Mean vertical force / cube weight", yscale="symlog")
    ax.legend(fontsize=8)
    for r in measured:
        if r["run"] != label:
            continue
        low, high = float(r["low_mm"]), float(r["high_mm"])
        use = (height >= low) & (height < high)
        rows.append([label, low, high, use.sum(), shelf[use].mean(), lift[use].mean(),
                     np.mean(lift[use] >= .5), r["packets"], r["apparent_deg"]])
        if r["apparent_deg"]:
            axes[1,col].hlines(float(r["apparent_deg"]), low, high, lw=3, color="tab:blue")
    axes[1,col].set(ylabel="Physical median apparent tilt (°)", xlabel="Recorded wrist height (mm)", ylim=(0, 40))
fig.savefig(here/"raise-support.png", dpi=155)
with (here/"raise-support.csv").open("w") as stream:
    writer = csv.writer(stream)
    writer.writerow("run low_mm high_mm windows mean_shelf_weight mean_lift_weight fraction_windows_lift_ge_half packets apparent_deg".split())
    writer.writerows(rows)
