"""Evaluate the frozen clip and dispatch offset at each held-out SDK sample."""
import csv
import gzip
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

sim = np.genfromtxt("turns.csv", delimiter=",", names=True, dtype=None, encoding="utf8")
fig, axes = plt.subplots(2, 3, figsize=(12, 6), layout="constrained", sharex=True)
with open("turn-scores.csv", "w") as out:
    writer = csv.writer(out, lineterminator="\n")
    writer.writerow(["session", "mode", "samples", "wheel_diff_rmse_mmps", "gyro_z_rmse_dps"])
    for col, name in enumerate("DEF"):
        root = Path("../unit3-physical-repeats") / name
        with open(root / "actions.csv") as f:
            end = next(float(r["receipt_s"]) for r in csv.DictReader(f)
                       if r["event"] == "end" and r["action"] == "undock")
        with gzip.open(root / "robot.csv.gz", "rt") as f:
            rows = [r for r in csv.DictReader(f) if 13.7 <= float(r["receipt_s"]) <= min(end,16)]
        def series(key):
            return np.array([float(r[key]) for r in rows])
        t = series("receipt_s")
        obs = [(series("right_mmps") - series("left_mmps")) / 2, np.degrees(series("gz"))]
        for ax, y in zip(axes[:, col], obs):
            ax.plot(t, y, "ko-", ms=3, lw=1.5, label="Physical SDK")
        for mode in ["open", "ramped", "feedback"]:
            s = sim[sim["clip"] == mode]
            scores = []
            for ax, measured, model in zip(axes[:, col], obs, [(s["right"]-s["left"])/2, s["gyro"]]):
                pred = np.interp(t-.4, s["t"], model)
                scores.append(np.mean((measured-pred)**2)**.5)
                ax.plot(t, pred, lw=1, label=mode)
            writer.writerow([name, mode, len(t), *scores])
        axes[0,col].set_title("Held-out session " + name)
        axes[1,col].set_xlabel("Seconds from undock request")
axes[0,0].set_ylabel("Half wheel difference (mm/s)")
axes[1,0].set_ylabel("Head gyro Z (°/s)")
axes[0,0].legend(fontsize=8)
fig.savefig("turns.png", dpi=150)
