"""Compare 100 three-slot reports at rest; the time axes are independent."""
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
live = np.genfromtxt(here/"received.csv", names=True, delimiter=",")
real = np.genfromtxt(here.parent/"resting.csv", names=True, delimiter=",")[:len(live)]
fig, axs = plt.subplots(2, 1, figsize=(9,6))
for label, data, keys, times, color in [
    ("Unit 3",real,["x_counts","y_counts","z_counts"],real["robot_rx_uptime_s"][::3],"tab:blue"),
    ("WireOS UDP receiver",live,["x","y","z"],live["wall_s"][2::3],"tab:orange"),
]:
    words = np.stack([data[k] for k in keys], axis=1)
    g = np.linalg.norm(words/8192,axis=1)
    interval = np.diff(times)*1000
    print(label, "magnitude (g) median, SD", np.median(g), g.std(), "median interval (ms)",np.median(interval),
          "elapsed (s)", times[-1]-times[0], "gaps >150 ms",sum(interval>150),
          "word residues",np.unique(words.astype(int)%4))
    axs[0].plot(g, label=label, color=color, lw=1)
    axs[1].plot(np.arange(1,len(times)),interval, label=label,color=color,lw=1)
axs[0].set(xlabel="Acceleration slot",ylabel="Specific force magnitude (g)")
axs[1].set(xlabel="Three-slot report",ylabel="Receiver interval (ms)")
for ax in axs:
    ax.grid(alpha=.25)
    ax.legend()
fig.tight_layout()
fig.savefig(here/"rest.png",dpi=150)
