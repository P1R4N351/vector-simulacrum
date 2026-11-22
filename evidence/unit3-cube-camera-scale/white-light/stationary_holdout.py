"""Compare stationary cube measurements to the unchanged Vector 2 sensor model."""
import csv
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
sys.path.insert(0, str(here.parents[2]))
from simulacrum import params
from simulacrum.cube import CubeAccelerometer


def measure(packets):
    assert all(len(p) == 20 and p[0] == 2 for _, p in packets)
    words = np.stack([np.frombuffer(p[2:], "<i2").reshape(3, 3) for _, p in packets])
    mg = words * (1000.0 / 8192)
    noise = np.sqrt(np.mean(np.diff(mg, axis=1)**2, axis=(0, 1))/2)
    times = np.array([t for t, _ in packets])
    print("word residues", np.unique(words % 4), "tap counters", sorted({p[1] for _, p in packets}))
    return mg.mean(axis=(0, 1)), noise, times-times[0]


with (here / "events.csv").open() as f:
    packets = [(float(r["seconds"]), bytes.fromhex(r["value"])) for r in csv.DictReader(f)
               if r["kind"] == "accel"]
v = params.values(params.load(model="vector_2"))
dt = v["sim_timestep_ms"] / 1000
chip = CubeAccelerometer(v, dt, seed=0)
up = np.array([0., 0., 1.])
for i in range(1, int(30/dt)+1):
    chip.step(i*dt, up)
assert len(chip.messages) >= len(packets)
fig, axes = plt.subplots(1, 3, figsize=(12, 3.6))
for idx, (name, stream) in enumerate([("Physical", packets), ("Frozen sim", chip.messages[:len(packets)])]):
    mean, noise, time = measure(stream)
    print(name, mean, noise, time[-1], "max gap", np.diff(time).max())
    x = np.arange(3) + (idx-.5)*.4
    axes[0].bar(x, mean - [0, 0, 970], .4, label=name)
    axes[1].bar(x, noise, .4)
    axes[2].plot(time, np.arange(1, len(time)+1), ["-", "--"][idx], label=["Host receipt", "Sim emission"][idx])
axes[0].set(xticks=range(3), xticklabels=["X", "Y", "Z − 970"], ylabel="mg", title="Resting mean")
axes[0].legend()
axes[1].set(xticks=range(3), xticklabels=list("XYZ"), ylabel="mg RMS", title="Within-packet noise")
axes[2].set(xlabel="Seconds since first packet", ylabel="Cumulative packets", title="Packet timing")
axes[2].legend()
fig.tight_layout()
fig.savefig(here / "stationary-holdout.png", dpi=150)
