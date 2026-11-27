"""Condition the existing lift gate on the physical encoder trajectory."""
import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
def read(path):
    with path.open() as f:
        return list(csv.DictReader(f))

values = json.loads((here.parents[4]/"web/public/robot/model.json").read_text())["values"]
arm = values["lift_arm_length_mm"]
base = values["lift_base_position_mm"][2]
tol = np.deg2rad(values["lift_angle_tol_deg"])
target = np.arcsin((52-base)/arm)
raw = read(here.parent/"recording.csv")
canonical = read(here.parent/"replay.csv")
commands = [r for r in raw if r["event"]=="command"]
physical = [r for r in raw if r["event"]=="state"]
times = np.array([float(r["s"]) for r in physical])
assert np.array_equal(times,[float(r["s"]) for r in canonical])
ready = read(here/"ready.csv")
scores = []
fig,axes = plt.subplots(2,4,figsize=(12,5.2),layout="constrained",sharex=True,sharey=True)
for j in range(1,16,2):
    trial=j//2+1
    cue=float(commands[j]["s"])
    assert cue==float(ready[j]["command_s"])
    active=float(ready[j]["ready_s"])-cue
    keep=(times>=cue-.1)&(times<float(commands[j+1]["s"]))
    x=times[keep]-cue
    # The grid is anchored to the tick at which the final command reached the servo.
    grid=np.arange(int((x[-1]-active)/.005)+1)*.005+active
    ax=axes.flat[trial-1]
    for label,rows,color in [("Physical",physical,"C0"),("Canonical",canonical,"C1")]:
        height=np.array([float(r["lift_mm"]) for r in rows])[keep]
        flags=np.array([int(r["in_position"]) for r in rows])[keep]
        first_clear=np.flatnonzero((x>=0)&(flags==0))[0]
        observed=x[first_clear+np.flatnonzero(flags[first_clear:])[0]]
        ax.plot(x,height,".-",lw=1,ms=2,color=color,label=label)
        ax.axvline(observed,color=color,lw=1)
        estimates=[]
        for h in [np.interp(grid,x,height),height[np.searchsorted(x,grid,side="right")-1]]:
            within=np.abs(np.arcsin((h-base)/arm)-target)<tol
            # First eligible tick plus 21 consecutive 5 ms intervals (strictly over 100 ms).
            ends=np.flatnonzero(np.convolve(within.astype(int),np.ones(22,dtype=int),"valid")==22)
            if not len(ends):
                estimates.append(np.nan)
                continue
            first_tick=grid[ends[0]+21]
            estimates.append(x[np.searchsorted(x,first_tick)])
        if label=="Physical":
            ax.axvline(estimates[0],color="black",ls="--",label="Gate on encoder")
        scores.append([trial,"raise" if float(commands[j-1]["target_mm"])<52 else "lower",
                       label,*[1000*v for v in (active,observed,*estimates)]])
    lo,hi=base+arm*np.sin(target+np.array([-tol,tol]))
    ax.axhspan(lo,hi,color="gray",alpha=.12)
    ax.set(xlim=(.45,1.5),ylim=(47,57),title=f"Trial {trial} · {scores[-1][1]}")
axes.flat[0].legend(fontsize=7)
fig.supxlabel("Seconds after command · Vertical lines mark first in-position report")
fig.supylabel("Lift height (mm)")
fig.savefig(here/"gate.png",dpi=155)
with (here/"scores.csv").open("w") as f:
    w=csv.writer(f,lineterminator="\n")
    w.writerow("trial direction trace profile_ready_ms observed_flag_ms encoder_linear_ms encoder_hold_ms".split())
    w.writerows(scores)
a=np.array([r[4:] for r in scores if r[2]=="Physical"])
b=np.array([r[4:] for r in scores if r[2]=="Canonical"])
print("trial: observed / sim / encoder linear / held (ms)")
print(np.round(np.column_stack((a[:,0],b[:,0],a[:,1:]))))
print("MAE",np.mean(abs(np.column_stack((b[:,0],a[:,1:]))-a[:,[0]]),axis=0))
