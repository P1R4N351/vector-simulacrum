"""Physical SDK event codes and a separate controlled 90-degree sensor input."""
import json
import sys
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

here = Path(__file__).resolve().parent
sys.path.insert(0, str(here.parents[1]))
from simulacrum import cube, params

record = here.parent / "hw-rec-20261121/run-s-hold-cube.jsonl"
with record.open() as f:
    events = [r for r in map(json.loads, f) if r.get("event") == "object_up_axis_changed"]
sensor = cube.CubeAccelerometer(params.values(params.load()), .001)
for step in range(1, 751):
    sensor.step(step / 1000, np.array([0,0,1] if step <= 300 else [-1,0,0]))
fig, axes = plt.subplots(2, 1, figsize=(8, 5))
axes[0].scatter([r["t"] for r in events], [r["up_axis"] for r in events], s=17)
axes[0].set(title="Physical run S · SDK notifications", xlabel="Recording time (s)")
axes[1].axvline(.3, color="gray", ls="--", label="Idealized quarter turn")
axes[1].scatter(*zip(*sensor.up_axis.events), label="Simulated notification")
axes[1].set(xlim=(0,.75), title="Independent simulated step from +Z to −X", xlabel="Simulated time (s)")
axes[1].legend()
for ax in axes:
    ax.set(yticks=range(1,7), yticklabels=["−X","+X","−Y","+Y","−Z","+Z"], ylim=(.5,6.5))
fig.tight_layout()
fig.savefig(here / "up-axis.png",dpi=150)
print(len(events), sorted({r["up_axis"] for r in events}),sensor.up_axis.events)
