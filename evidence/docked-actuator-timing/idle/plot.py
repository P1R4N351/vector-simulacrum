"""Plot measured SDK outputs at host receipt times during a passive docked capture."""
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np

path = Path(sys.argv[1])
data = np.genfromtxt(path, names=True, delimiter=",", dtype=None, encoding="utf8")
t = data["elapsed_s"]-data["elapsed_s"][0]
fig, axes = plt.subplots(3, 1, figsize=(8, 6), sharex=True, layout="constrained")
axes[0].plot(t, data["prox_mm"], ".", ms=3)
axes[0].set_ylabel("Range (mm)")
axes[1].plot(t, data["prox_quality"], ".", ms=3)
axes[1].set_ylabel("Signal quality")
for axis in "xyz":
    axes[2].plot(t, np.rad2deg(data["gyro_"+axis]), label=axis.upper(), lw=.8)
axes[2].set(ylabel="Head gyro (°/s)", xlabel="Seconds since first state")
axes[2].legend(ncol=3)
fig.savefig(path.with_suffix(".png"), dpi=155)
print(len(t), t[-1])
for key in ("prox_mm","prox_quality","found","head_rad","lift_mm","cliff_detected","left_mmps","right_mmps"):
    print(key, np.percentile(data[key], [0, 50, 100]))
