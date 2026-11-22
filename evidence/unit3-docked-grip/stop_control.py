"""Compare the SDK lift bit around interleaved stationary stop and sham events."""
import csv
import gzip
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
with gzip.open(here/"stop-control.jsonl.gz","rt") as stream:
    records = [json.loads(line) for line in stream]
events = [r for r in records if r["kind"]=="command"]
states = [r for r in records if r["kind"]=="state"]
assert len(events)==8 and any(r.get("docked") for r in records if r["kind"]=="end")
assert all(r["status"]&4096 and r["left"]==0 and r["right"]==0 for r in states)
fig,axes = plt.subplots(2,1,figsize=(8,5),sharex=True,layout="constrained")
with (here/"stop-control.csv").open("w") as stream:
    writer=csv.writer(stream)
    writer.writerow(["trial","action","samples","pre_set","first_clear_s","first_return_s","height_span_mm"])
    for e in events:
        rows=[r for r in states if -.2<=r["t"]-e["t"]<=1.15]
        t=np.array([r["t"]-e["t"] for r in rows])
        y=np.array([bool(r["status"]&256) for r in rows])
        h=np.array([r["lift"] for r in rows])
        clears=t[(t>=0)&~y]
        clear=clears[0] if len(clears) else None
        rises=t[(t>clear)&y] if clear is not None else []
        ret=rises[0] if len(rises) else None
        writer.writerow([e["n"],e["action"],len(rows),int(y[t<0][-1]),clear,ret,np.ptp(h)])
        ax=axes[0 if e["action"]=="stop" else 1]
        ax.step(t,y.astype(float)+.018*e["n"],where="post",label=f"Trial {e['n']+1}")
for ax,name in zip(axes,["Stop all motors","No-command control"]):
    ax.set(yticks=[0,1],yticklabels=["Clear","Set"],ylim=(-.08,1.25),title=name)
    ax.axvline(0,color="gray",ls=":");ax.legend(ncol=4,loc="lower right")
axes[1].set_xlabel("Seconds from event")
fig.savefig(here/"stop-control.png",dpi=160)

# Replay both command histories; the original capture called stop immediately
# after the failed SDK response but did not timestamp the stop call separately.
from simulacrum import params
from simulacrum.robot import VectorSim
failed=np.genfromtxt(here/"dock-status.csv",names=True,delimiter=",")
t=failed["seconds"]
actual=(failed["status"].astype(int)&256)!=0
fig,ax=plt.subplots(figsize=(8,3),layout="constrained")
ax.step(t,actual.astype(float),where="post",label="Physical")
for with_stop in (False,True):
    sim=VectorSim(params.values(params.load(model="vector_2")),lift_mm=33.02,head_deg=21.5)
    sim.command_lift_height(47,duration_s=1)
    bits=[sim.state.lift_in_pos]
    sent=False
    while (len(bits)-1)*sim.dt<t[-1]:
        if with_stop and not sent and (len(bits)-1)*sim.dt>=1.518:
            sim.stop_all_motors()
            sent=True
        bits.append(sim.step().lift_in_pos)
    clock=np.arange(len(bits))*sim.dt
    pred=np.array(bits)[np.maximum(0,np.searchsorted(clock,t-.11552706003188984,side="right")-1)]
    ax.step(t,pred.astype(float)+(.09 if with_stop else .045),where="post",label="Raise + stop" if with_stop else "Raise only")
    print(with_stop,np.mean(actual[t>=0]==pred[t>=0]),t[1:][pred[1:]!=pred[:-1]])
ax.legend();ax.set(yticks=[0,1],yticklabels=["Clear","Set"],xlabel="Seconds from failed raise command")
fig.savefig(here/"stop-replay.png",dpi=160)
