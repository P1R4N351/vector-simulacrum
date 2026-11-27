"""Intermediate encoder dwells and residual height on equal-travel docked lowers."""
import csv
from itertools import groupby
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
def read(name):
    with (here/name).open() as f:
        return list(csv.DictReader(f))

events = read("recording.csv")
states = [r for r in events if r["event"] == "state"]
model = read("replay.csv")
assert all(int(r["on_dock"]) for r in states+model)
t = np.array([float(r["s"]) for r in states])
assert np.array_equal(t, [float(r["s"]) for r in model])
assert all(r["code"] == "0" for r in events if r["event"]=="return")
lowers = [r for r in events if r["event"]=="command" and float(r["duration_s"])==.6]
assert len(lowers)==8
fig, axes = plt.subplots(2,2,sharex=True,sharey="row",layout="constrained",figsize=(9,6))
results=[]
for i, cmd in enumerate(lowers,1):
    target = int(cmd["target_mm"])
    elapsed = t-float(cmd["s"])
    x = elapsed[(elapsed>=-.2)&(elapsed<=1.1)]
    for col, (name,rows) in enumerate([("Physical",states),("Canonical",model)]):
        y = np.array([float(r["lift_mm"]) for r,dt in zip(rows,elapsed) if -.2<=dt<=1.1])
        start = np.median(y[x<0])
        end = np.median(y[x>=.9])
        # Runs of exactly equal reported values, with descent on either side.
        spans=[]
        for value,indices in groupby(range(len(x)),lambda k:y[k]):
            k=list(indices)
            if x[k[0]]<0 or not target+1<=value<=start-1:
                continue
            if y[k[0]-1]>value and k[-1]+1<len(y) and y[k[-1]+1]<value:
                spans.append((x[k[-1]]-x[k[0]],value))
        duration,height=max(spans,default=(0,np.nan))
        results.append((i,target,name,1000*duration,height,end-target))
        ax=axes[int(target==40),col]
        ax.step(x,y,where="post",alpha=.75,linewidth=1.3,label=f"Trial {i}")
        ax.axhline(target,color="grey",alpha=.15,ls=":")
        ax.set_title(f"{name} · {target+12} → {target} mm")
for ax in axes[-1]:
    ax.set_xlabel("Seconds after SDK command")
for ax in axes[:,0]:
    ax.set_ylabel("Reported lift height (mm)")
for ax in axes.flat:
    ax.legend(fontsize=7)
fig.savefig(here/"dwells.png",dpi=160)
with (here/"scores.csv").open("w") as f:
    w=csv.writer(f,lineterminator="\n")
    w.writerow("trial target_mm trace longest_dwell_ms dwell_height_mm settled_minus_target_mm".split())
    w.writerows(results)
for target in (32,40):
    for name in ("Physical","Canonical"):
        sub=np.array([r[3:] for r in results if r[1:3]==(target,name)])
        print(target,name,"dwell ms",np.round(sub[:,0]),"height",np.round(sub[:,1],1),"end error",np.round(sub[:,2],1))
