"""Compare a fresh seated baseline with the final W34 approach."""

import csv
import gzip
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

root = Path(__file__).resolve().parent


def series(filename):
    with gzip.open(root / filename, "rt") as input_file:
        rows = list(csv.DictReader(input_file))
    return {
        name: np.asarray([float(row[name]) for row in rows])
        for name in "receipt_s x_mm y_mm left_mmps right_mmps status".split()
    }


fig, axes = plt.subplots(3, 2, figsize=(9, 5), sharex="col", layout="constrained")
for col, filename in enumerate(["states.csv.gz", "rest.csv.gz"]):
    values = series(filename)
    on = (values["status"].astype(int) & 4096) != 0
    i = np.where(np.diff(on.astype(int)) == 1)[0][-1] + 1 if col == 0 else 0
    t = values["receipt_s"] - values["receipt_s"][i]
    keep = (t >= -1) & (t <= 1) if col == 0 else np.ones(len(t), dtype=bool)
    xy = np.hypot(
        values["x_mm"] - values["x_mm"][i], values["y_mm"] - values["y_mm"][i]
    )
    for field, label in [("left_mmps", "Left"), ("right_mmps", "Right")]:
        axes[1, col].plot(t[keep], values[field][keep], label=label)
    axes[0, col].step(t[keep], on[keep], where="post")
    axes[2, col].plot(t[keep], xy[keep])
    axes[0, col].set_ylim(-0.1, 1.1)
    axes[1, col].legend(fontsize=8)
    print(
        filename,
        len(t),
        t[-1] - t[0],
        int(on.sum()),
        xy[keep].max(),
        np.max(abs(values["left_mmps"][keep])),
        np.max(abs(values["right_mmps"][keep])),
    )
axes[0, 0].set_title("W34 return")
axes[0, 1].set_title("Fresh seated baseline")
axes[2, 0].set_xlabel("Seconds from first on-charger report")
axes[2, 1].set_xlabel("Seconds from first report")
for ax, label in zip(
    axes[:, 0], ["On charger", "Wheel speed (mm/s)", "SDK position change (mm)"]
):
    ax.set_ylabel(label)
fig.savefig(root / "rest-check.png", dpi=160)
