"""Score the measured R5 replay and the three chassis-motion controls."""
import csv
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
fig, axes = plt.subplots(2, 1, figsize=(8, 6), sharex=True, layout="constrained")
rows = []
labels = ["Measured motion", "Fixed planar pose", "Fixed pitch", "Fixed planar pose and pitch"]
for label, directory in zip(labels, sys.argv[1:], strict=True):
    root = Path(directory)
    loads = np.loadtxt(root/"contact-load.csv", delimiter=",", skiprows=1)
    cube = np.loadtxt(root/"pickup-replay.csv", delimiter=",", skiprows=1)
    if label == labels[0]:
        original = loads
        onset = loads[loads[:, 1] > loads[0, 1]+1, 0][0]
    pre = loads[:, 0] < onset
    np.testing.assert_array_equal(loads[pre], original[pre])
    force = loads[:, 3:6].sum(axis=1)
    lifting = (loads[:, 0] >= onset) & (loads[:, 1] < 90) & (loads[:, 0] < 3.3)
    supported = lifting & (force >= .5)
    hold = (cube[:, 0] >= 5.5) & (cube[:, 0] <= 7.9)
    rows.append([label, np.mean(force[lifting]), np.max(loads[supported, 1]) if supported.any() else "",
                 np.median(cube[hold, 1]), np.median(cube[hold, 2]), np.mean(cube[hold, 3])])
    dt = np.median(np.diff(loads[:, 0]))
    width = round(.01/dt)
    n = len(force)//width*width
    axes[0].plot(loads[:n, 0].reshape(-1,width).mean(axis=1)-onset,
                 force[:n].reshape(-1,width).mean(axis=1), label=label)
    axes[1].plot(cube[:, 0]-onset, cube[:, 2])
axes[0].legend(fontsize=8)
axes[0].set(ylabel="Vertical lift force / cube weight", yscale="symlog")
axes[1].axhline(31.78, color=".4", ls=":", label="Physical steady hold")
axes[1].legend(fontsize=8)
axes[1].set(xlabel="Seconds from first 1 mm wrist rise", ylabel="Cube inclination (°)", xlim=(-.15,1.1))
fig.savefig(here/"chassis-hold.png", dpi=155)
with (here/"chassis-hold.csv").open("w") as stream:
    writer = csv.writer(stream)
    writer.writerow("run mean_raise_force_weight last_half_weight_height_mm hold_center_mm hold_tilt_deg hold_floor_fraction".split())
    writer.writerows(rows)
