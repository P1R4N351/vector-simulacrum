"""Compare the two reconstructions of the same seated W34 frame."""
import csv
from pathlib import Path

import matplotlib.pyplot as plt

source = Path(__file__).resolve().parents[1] / "dock-pad-registration" / "geometry.csv"
rows = {}
with source.open() as stream:
    for row in csv.DictReader(stream):
        if row["frame"] == "W34":
            rows[row["anchors"], row["choice"], row["view"]] = row

fig, axes = plt.subplots(1, 2, figsize=(9, 4), layout="constrained")
with open("cross-view.csv", "w") as stream:
    writer = csv.writer(stream, lineterminator="\n")
    writer.writerow(["anchors", "plane", "component", "A_mm", "B_mm", "A_minus_B_mm"])
    for ax, component, label in zip(axes, ["outward_error_mm", "up_error_mm"], ["Outward", "Upward"]):
        for anchors, name, marker in [("4", "Glyph", "o"), ("5", "Glyph and lip", "s")]:
            differences = []
            for plane in ["27", "30", "33", "span"]:
                a, b = (float(rows[anchors, plane, view][component]) for view in "AB")
                differences.append(a-b)
                writer.writerow([anchors, plane, component, a, b, a-b])
            ax.plot(range(4), differences, marker=marker, label=name)
        ax.axhline(0, color="gray", linewidth=1)
        ax.set(xticks=range(4), xticklabels=["27", "30", "33", "Axle fit"],
               xlabel="Assumed hub-face plane (mm) or axle-span fit",
               ylabel=f"{label} difference, camera A minus B (mm)")
axes[0].legend()
fig.suptitle("W34 seated pad center inferred from opposite sides")
fig.savefig("cross-view.png", dpi=160)
