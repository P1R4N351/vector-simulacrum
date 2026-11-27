"""Plot each docked recording and both simulated contact solvers."""
import matplotlib.pyplot as plt
import numpy as np


def read(path):
    return np.genfromtxt(path, delimiter=",", names=True)


fig, axes = plt.subplots(3, 3, figsize=(11, 7), sharex=True, layout="constrained")
for row, unit in enumerate("ABC"):
    physical = read(f"unit-{unit}.csv")
    time = physical["s"] - physical["s"][0]
    accel = physical["ax"] / 9.80665
    axes[row, 2].plot(time, accel - np.median(accel[(time >= 3) & (time < 5)]),
                      label="Physical", color="black", alpha=0.55, lw=0.8)
    for mode, label in [("baseline", "Canonical"), ("noslip", "4 noslip iterations")]:
        trial = read(f"settling-{unit}-{mode}.csv")
        time = trial["s"]
        accel = trial["ax"] / 9.80665
        axes[row, 0].plot(time, trial["x_mm"] - trial["x_mm"][0], label=label)
        axes[row, 1].plot(time, trial["pitch_deg"], label=label)
        axes[row, 2].plot(time, accel - np.median(accel[(time >= 3) & (time < 5)]),
                          label=label, lw=0.8)
    axes[row, 0].set_ylabel(f"Unit {unit}")
for ax, title in zip(axes[0], ["Simulated downhill travel (mm)", "Simulated chassis pitch (°)",
                               "Forward acceleration change (mg)"]):
    ax.set_title(title)
axes[0, 2].legend(fontsize=8)
for ax in axes[-1]:
    ax.set_xlabel("Elapsed time (s)")
fig.savefig("settling.png", dpi=155)
