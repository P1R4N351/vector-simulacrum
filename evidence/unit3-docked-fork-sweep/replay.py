"""Replay SDK command receipt times on the frozen Vector 2 model."""
import csv
import gzip
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from simulacrum import params
from simulacrum.robot import VectorSim

folder = Path(__file__).resolve().parent
with gzip.open(folder / "state.csv.gz", "rt") as stream:
    samples = np.genfromtxt(stream, names=True, delimiter=",")
moves = np.genfromtxt(folder / "holds.csv", names=True, delimiter=",")
sim = VectorSim(params.values(params.load(model="vector_2")), lift_mm=32,
                head_deg=np.degrees(samples["head_rad"][0]), charger_contact=True)
times, heights = [], []
i = 0
while len(times) * sim.dt <= samples["seconds"][-1]:
    t = len(times) * sim.dt
    if i < len(moves) and t >= moves[i]["command_s"]:
        assert sim.command_lift_height(32 + 60 * moves[i]["command_fraction"], duration_s=0.7)
        i += 1
    heights.append(sim.step().lift_height_mm)
    times.append(t + sim.dt)

fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharey=True, constrained_layout=True)
with (folder / "replay-errors.csv").open("w") as stream:
    writer = csv.writer(stream)
    writer.writerow(["move", "cycle", "target_fraction", "hold_error_sim_minus_real_mm",
                     "command_aligned_rmse_mm"])
    for cycle, ax in enumerate(axes):
        start = moves[1 + 8 * cycle]["command_s"]
        end = moves[9]["command_s"] if cycle == 0 else samples["seconds"][-1]
        for x, y, label in [(times, heights, "Frozen simulation"),
                              (samples["seconds"], samples["lift_mm"], "Unit 3 encoder")]:
            x = np.asarray(x)
            mask = (x >= start) & (x < end)
            ax.plot(x[mask] - start, np.asarray(y)[mask], label=label)
        ax.set(title=["First sweep", "Held-out second sweep"][cycle], ylabel="Lift height (mm)")
        for j in range(1 + 8 * cycle, 9 + 8 * cycle):
            move = moves[j]
            stop = moves[j+1]["command_s"] if j+1 < len(moves) else end
            x = samples["seconds"]
            mask = (x >= move["command_s"]) & (x < stop)
            error = np.interp(x[mask], times, heights) - samples["lift_mm"][mask]
            hold = (x >= move["success_s"] + .12) & (x <= move["success_s"] + .6)
            hold_error = np.median(np.interp(x[hold], times, heights)) - move["lift_mm"]
            writer.writerow([j, cycle+1, move["command_fraction"], round(hold_error,3),
                             round(np.sqrt(np.mean(error**2)),3)])
axes[0].legend()
axes[1].set_xlabel("Seconds since first raise command in sweep")
fig.savefig(folder / "physical-vs-replay.png", dpi=160)
