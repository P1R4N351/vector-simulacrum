"""Compare travel between actual frames spanning charger acquisition."""
import csv
import io
import subprocess
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from release_optics import states

ROOT = Path(__file__).resolve().parent


def table(name):
    return np.genfromtxt(ROOT / name, delimiter=",", names=True, dtype=None, encoding="utf8")


def wheel_travel(times, speeds, start, end):
    interior = times[(start < times) & (times < end)]
    grid = np.r_[start, interior, end]
    return -np.trapezoid(np.interp(grid, times, speeds), grid)


def main():
    frames = table("return-frames.csv")
    sim = table("return-replay.csv")
    offsets = np.arange(-150, 151) / 1000
    trials = table("return-scores.csv")
    approaches = {}
    for trial in trials:
        name = trial["session"]
        rows, _ = states(name)
        approach_samples = [(float(r["left_mmps"]) + float(r["right_mmps"])) / 2 for r in rows
                            if -1 <= float(r["receipt_s"]) - trial["first_on_s"] <= -.15]
        approaches[name] = -np.median(approach_samples)
    replay = subprocess.check_output(["node", str(ROOT / "return_replay.mjs"),
                                      *map(str, sorted(set(approaches.values())))], text=True)
    matched_runs = np.genfromtxt(io.StringIO(replay), delimiter=",", names=True, dtype=None, encoding="utf8")
    scores = []
    curves = []
    for trial in trials:
        name = trial["session"]
        rows, _ = states(name)
        times = np.array([float(row["receipt_s"]) for row in rows]) - trial["first_on_s"]
        speeds = np.array([(float(r["left_mmps"]) + float(r["right_mmps"])) / 2 for r in rows])
        frames_now = frames[frames["session"] == name]
        before = frames_now[frames_now["rel_s"] < 0][-1]
        after = frames_now[frames_now["rel_s"] >= 0][0]
        start, end = before["rel_s"], after["rel_s"]
        optical = before["forward_mm"] - after["forward_mm"]
        estimates = np.array([wheel_travel(times, speeds, start + lag, end + lag) for lag in offsets])
        curves.append(estimates - optical)
        simulated = []
        for dataset, speed in [(sim, 25), (sim, 35), (sim, 45), (matched_runs, approaches[name])]:
            cycle = dataset[(dataset["speed"] == speed) & np.isin(dataset["phase"], ["return", "seat"])]
            tick = cycle["t_s"] - cycle["t_s"][np.flatnonzero(cycle["on"])[0]]
            x = np.interp([start, end], tick, cycle["x_mm"])
            simulated.append(x[0] - x[1])
        scores.append([name, start, end, optical, estimates[len(offsets)//2], *simulated[:3],
                       approaches[name], simulated[3]])
    fig, axes = plt.subplots(1, 2, figsize=(10, 4), layout="constrained")
    residuals = np.stack(curves)
    for index, row in enumerate(scores):
        peers = np.delete(residuals, index, axis=0)
        best = np.argmin(np.mean(peers**2, axis=0))
        row.extend([offsets[best], row[3] + residuals[index, best]])
        axes[1].plot(1000 * offsets, residuals[index], label=row[0])
        axes[1].scatter(1000 * offsets[best], residuals[index, best], s=18)
    for col, label, marker in [(3, "Camera", "o"), (4, "Integrated wheels", "x"),
                               (9, "Sim at approach speed", "+"), (11, "Wheels, held-out offset", "s")]:
        axes[0].scatter(range(len(scores)), [row[col] for row in scores], marker=marker, label=label)
    axes[0].set(xticks=range(len(scores)), xticklabels=[r[0] for r in scores],
                xlabel="Recorded return", ylabel="Backing between frames (mm)")
    axes[1].axhline(0, color="gray", linewidth=.8)
    axes[1].set(xlabel="Offset added to camera times (ms)", ylabel="Integrated wheels minus camera (mm)")
    for ax in axes:
        ax.legend(fontsize=8)
    fig.savefig(ROOT / "return-braking.png", dpi=160)
    with (ROOT / "return-braking.csv").open("w") as output:
        writer = csv.writer(output, lineterminator="\n")
        writer.writerow("session before_s after_s camera_mm wheels_mm sim25_mm sim35_mm sim45_mm approach_mmps matched_sim_mm heldout_offset_s heldout_wheels_mm".split())
        writer.writerows(scores)


if __name__ == "__main__":
    main()
