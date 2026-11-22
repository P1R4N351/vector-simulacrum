"""Show initial interference for each direction of the observed carry tilt."""
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import Normalize

here = Path(__file__).resolve().parent
data = np.genfromtxt(here/"static-depth.csv", names=True, delimiter=",", dtype=None, encoding="utf8")
fig, axes = plt.subplots(2, 4, sharex=True, sharey=True, layout="constrained", figsize=(10, 5))
norm = Normalize(0, max(data["initial_overlap_mm"]))
for ax, yaw in zip(axes.flat, np.unique(data["yaw_deg"])):
    rows = data[data["yaw_deg"] == yaw]
    depths = rows["initial_overlap_mm"]
    dots = ax.scatter(rows["forward_mm"], rows["vertical_mm"], c=depths, norm=norm,
                      s=45, marker="s", cmap="magma")
    clear = rows[depths == 0]
    ax.scatter(clear["forward_mm"], clear["vertical_mm"], marker="x", c="cyan", s=32)
    ax.set_title(f"Yaw {yaw}°")
fig.supxlabel("Forward offset from nominal carry center (mm)")
fig.supylabel("Vertical offset (mm)")
fig.suptitle("Printed fork at full lift · crosses mark clear starts")
fig.colorbar(dots, ax=axes, label="Deepest initial penetration (mm)")
fig.savefig(here/"static-depth.png", dpi=150)
