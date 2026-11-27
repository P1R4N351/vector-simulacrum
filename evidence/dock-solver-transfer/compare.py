"""Paired per-turn gyro errors and time-series overlays."""
from collections import defaultdict
import csv
import numpy as np
import matplotlib.pyplot as plt

trials = defaultdict(list)
with open("predictions.csv") as src:
    for r in csv.DictReader(src):
        trials[r["unit"], r["turn"], r["scene"], int(r["iterations"])].append(r)

with open("scores.csv", "w") as out:
    w = csv.writer(out, lineterminator="\n")
    w.writerow(["unit","turn","scene","iterations","phase","samples","rmse_dps","bias_dps"])
    for key, rows in trials.items():
        for phase in ["steady", "stop"]:
            err = np.array([float(r["predicted"])-float(r["measured"]) for r in rows if r["phase"] == phase])
            w.writerow([*key,phase,len(err),np.sqrt(np.mean(err**2)),err.mean()])

for scene in ["vector-free-dock","vector-contact42-free-dock"]:
    fig, panels = plt.subplots(3, 4, figsize=(13, 8.5), layout="constrained")
    keys = list(dict.fromkeys((unit,turn) for unit,turn,_,_ in trials))
    for ax, (unit,turn) in zip(panels.flat,keys):
        for it in [0,4]:
            rows = trials[unit,turn,scene,it]
            t = [float(r["t"]) for r in rows]
            if it == 0:
                ax.plot(t,[float(r["measured"]) for r in rows],color="black",label="Hardware")
            ax.plot(t,[float(r["predicted"]) for r in rows],label=f"{it} iterations",lw=1.2)
        ax.axvline(0,color="grey",ls=":",lw=1)
        ax.set_title(f"{unit}  {turn}")
        ax.set_xlabel("Time (s)")
    for ax in panels[:,0]:
        ax.set_ylabel("Gyro Z (°/s)")
    panels[0,0].legend(fontsize=8)
    fig.savefig(scene+".png",dpi=145)
