"""Plot stationary Bluetooth slots. Pass the CSV from web/tools/cube-rest.mjs."""
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
fig, ax = plt.subplots(figsize=(6.8, 3.8), layout="constrained")
for label, path in [("Shelf", here.parent/"stationary.csv.gz"),
                    ("Shelf repeat", here.parent/"stationary-repeat.csv.gz"),
                    ("Browser default cube", sys.argv[1])]:
    slots = np.genfromtxt(path, delimiter=",", names=True)
    magnitude = np.sqrt(sum(slots[key]**2 for key in "xyz")) / 8192
    print(label, len(magnitude), np.percentile(magnitude, [5, 50, 95]))
    ax.step(np.sort(magnitude), np.arange(1, len(slots)+1)/len(slots), where="post", label=label)
ax.set(xlabel="Cube accelerometer magnitude (g)", ylabel="Fraction of stationary slots")
ax.legend()
fig.savefig(here/"browser-rest.png", dpi=150)
