"""Compare shelf support mechanisms on frozen late turns and the earlier release window."""
import csv
import gzip
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np

root = Path(sys.argv[1])
variants = ("stock", "box", "plane", "rear-free", "belly-free", "clearance", "driven", "driven-feedback")
fig, axs = plt.subplots(2, 3, figsize=(12, 6), layout="constrained", sharex=True)
with open("support-scores.csv", "w") as output:
    writer = csv.writer(output, lineterminator="\n")
    writer.writerow("trial variant gyro_rmse_dps wheel_diff_rmse_mmps release_wheel_rmse_mmps release_error_s wheel_support_fraction".split())
    for col, trial in enumerate("DEF"):
        folder = Path("../unit3-physical-repeats") / trial
        with open(folder / "actions.csv") as f:
            end = next(float(r["receipt_s"]) for r in csv.DictReader(f) if r["event"]=="end" and r["action"]=="undock")
        with gzip.open(folder / "robot.csv.gz", "rt") as f:
            rows = [r for r in csv.DictReader(f) if 0 <= float(r["receipt_s"]) <= end]
        def series(k):
            return np.array([float(r[k]) for r in rows])
        t = series("receipt_s")
        late = (t>=13.7) & (t<=16)
        edge = next(float(r["receipt_s"]) for r in rows if int(r["status"]) & 4096 == 0)
        release = abs(t-edge)<=1
        wl, wr, gyro = series("left_mmps"), series("right_mmps"), np.degrees(series("gz"))
        signals = (gyro, (wr-wl)/2)
        for ax, signal in zip(axs[:, col], signals):
            ax.plot(t[late], signal[late], "k.-", label="Physical", lw=1.4)
        for name in variants:
            sim = np.genfromtxt(root / (name+".csv"), names=True, delimiter=",", dtype=None, encoding="utf8")
            def pred(k):
                return np.interp(t-.4, sim["t"], sim[k])
            sl, sr = pred("left"), pred("right")
            estimates = (pred("gyro"), (sr-sl)/2)
            rms = [np.mean((a[late]-b[late])**2)**.5 for a,b in zip(estimates,signals)]
            near = np.mean(((sl-wl)[release]**2+(sr-wr)[release]**2)/2)**.5
            with open(root / (name+".contacts")) as f:
                support = ["wheel_" in line for line in f if 13.7 <= float(line.split()[0])+.4 <= min(16,end)]
            writer.writerow([trial,name,*rms,near,sim["t"][sim["on"]==0][0]+.4-edge,np.mean(support)])
            if name in ("stock","driven","driven-feedback"):
                for ax, est in zip(axs[:, col], estimates):
                    ax.plot(t[late],est[late],label=name,lw=1.2)
        axs[0,col].set_title("Session "+trial)
        axs[1,col].set_xlabel("Seconds after undock request")
axs[0,0].set_ylabel("Head gyro Z (°/s)")
axs[1,0].set_ylabel("Half wheel difference (mm/s)")
axs[0,0].legend(fontsize=8)
fig.savefig("support.png",dpi=150)
