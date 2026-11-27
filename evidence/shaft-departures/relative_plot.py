import csv
import matplotlib.pyplot as plt
import numpy as np

with open("relative.csv") as stream:
    rows = list(csv.DictReader(stream))
fig, axes = plt.subplots(2, 1, figsize=(8, 5), layout="constrained", sharex=True)
for ax, key, label in zip(axes, ["shift_mm","signed_miss_ms"],
                          ["Outward seat shift (mm)","Release bracket miss (ms)"]):
    for offset, kind, color in [(-.09, "fixed", "#315d87"), (.09, "relative", "#ae5823")]:
        values = [float(row[key]) for row in rows if row["seat"] == kind]
        ax.scatter(np.arange(len(values))+offset, values, label=kind.title(), color=color)
    ax.axvline(2.5, color="grey", ls="--", lw=1)
    ax.set_ylabel(label)
axes[0].legend()
axes[0].set_title("Pre-motion depth corrections on A–C and held-out D–F")
axes[1].axhline(0, color="grey", lw=.8)
axes[1].set(xticks=range(6), xticklabels=list("ABCDEF"), xlabel="Physical departure")
fig.savefig("relative.png", dpi=160)
