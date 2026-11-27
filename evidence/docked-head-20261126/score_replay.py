"""Compare recorded SDK packets with the latest simulated report at each receipt time."""
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).parent
rows = list(csv.DictReader((here / "unit2.csv").open()))
sim = np.genfromtxt(here / "canonical.csv", names=True, delimiter=",")
assert np.all(sim["charger"] == 1)
states = [r for r in rows if r["event"] == "state" and float(r["t_s"]) <= sim["t_s"][-1]]
t, angle, gyro = np.array([[float(r[k]) for k in ("t_s", "head_deg", "gyro_y_rad_s")] for r in states]).T
j = np.searchsorted(sim["t_s"], t, side="right") - 1
assert min(j) >= 0
pred = sim["head_deg"][j]
commands = [r for r in rows if r["event"] == "command"]
fig, axes = plt.subplots(2, 1, figsize=(11, 5.8), sharex=True, layout="constrained")
for ax, actual, model, label in zip(axes, (angle, gyro), (pred, sim["gyro_y_rad_s"][j]), ("Head encoder (°)", "Gyro Y (rad/s)")):
    ax.plot(t, model, color="#d07817", label="Canonical sim")
    ax.plot(t, actual, ".-", ms=3, color="#235980", label="Physical SDK")
    for r in commands:
        ax.axvline(float(r["t_s"]), lw=.8, color="grey", alpha=.4)
    ax.set_ylabel(label)
    ax.legend(loc="upper right", ncol=2)
axes[-1].set_xlabel("Capture time (s) · vertical lines mark commands")
fig.suptitle("Docked head motion · recorded command schedule")
fig.savefig(here / "canonical-overlay.png", dpi=170)
with (here / "canonical-scores.csv").open("w") as f:
    w = csv.writer(f)
    w.writerow(["cue","target_deg","physical_pre_deg","sim_pre_deg","physical_first_2deg_ms","sim_first_2deg_ms","angle_rmse_deg"])
    for i,r in enumerate(commands):
        start = float(r["t_s"])
        end = float(commands[i+1]["t_s"]) if i+1<len(commands) else t[-1]+.001
        window = (t >= start) & (t < end)
        idx = np.where(t < start)[0][-1]
        stable = np.ptp(angle[(t >= start-.22) & (t < start)]) < 1
        def crossing(a):
            hits = np.flatnonzero(window & (np.abs(a-a[idx]) >= 2))
            return round(1000*(t[hits[0]]-start)) if hits.size else ""
        w.writerow([i,r["target_deg"],round(angle[idx],2),round(pred[idx],2),crossing(angle) if stable else "",crossing(pred),round(np.mean((angle[window]-pred[window])**2)**.5,2)])

# Estimate one time offset on the first clean lower. Score the next lower once.
def errors(cue, delay):
    start = float(commands[cue]["t_s"])
    mask = (t >= start) & (t < start + 1)
    shifted = np.searchsorted(sim["t_s"], t[mask] - delay, side="right") - 1
    return [np.sqrt(np.mean((v[mask]-sim[k][shifted])**2))
            for v,k in [(angle,"head_deg"),(gyro,"gyro_y_rad_s")]]

grid = np.arange(-.120, .121, .005)
delay = min(grid, key=lambda sec: errors(4, sec)[0])
with (here / "lower-transfer.csv").open("w") as f:
    w = csv.writer(f)
    w.writerow(["cue","split","delay_ms","angle_rmse_deg","gyro_y_rmse_rad_s"])
    for cue, split in [(4,"fit"),(6,"held-out")]:
        for sec in [0,delay]:
            w.writerow([cue,split,round(sec*1000),*np.round(errors(cue,sec),3)])

fig, axes = plt.subplots(2, 2, figsize=(9, 6), sharex=True, layout="constrained")
for col, cue in enumerate([4,6]):
    start = float(commands[cue]["t_s"])
    mask = (t >= start-.1) & (t < start+1)
    x = t[mask]-start
    for ax, observed, field, label in zip(axes[:,col], (angle, gyro), ("head_deg","gyro_y_rad_s"), ("Encoder (°)","Gyro Y (rad/s)")):
        ax.plot(x, observed[mask], "k.", label="Measured")
        for sec,color,name in [(0,"#ce7624","Frozen"),(delay,"#216282","Shifted")]:
            k = np.searchsorted(sim["t_s"],t[mask]-sec,side="right")-1
            ax.plot(x,sim[field][k],color=color,label=name)
        ax.set_ylabel(label)
    axes[0,col].set_title(["Fit lower","Held-out lower"][col])
    axes[1,col].set_xlabel("Time from lower command (s)")
axes[0,0].legend()
fig.savefig(here/"lower-transfer.png",dpi=160)
