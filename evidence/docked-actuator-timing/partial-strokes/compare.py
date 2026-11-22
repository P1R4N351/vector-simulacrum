"""Compare short SDK moves with the frozen model and prior receipt-delay estimate."""
import csv
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np

here=Path(__file__).resolve().parent
sys.path.insert(0,str(here.parent))
from analyze import GyroReplay, params, recording, replay

states=np.genfromtxt(here/"states.csv",delimiter=",",names=True)
commands=list(csv.DictReader((here/"commands.csv").read_text().splitlines()))
values=dict(params.values(params.load(model="vector_2")),**replay.OPEN_TABLE)
fig,axes=plt.subplots(3,6,figsize=(15,6.5),layout="constrained",sharex=True)
scores=[]
for i,c in enumerate(commands):
    joint=c["joint"]
    target=float(c["target"])
    t=states["receipt_s"]-float(c["start_s"])
    pre=(t<0)&(t>-.24)
    window=(t>=0)&(t<=1)
    head=np.degrees(states["head_rad"])
    lift=states["lift_mm"]
    measured=(head if joint=="head" else lift)[window]
    sim=GyroReplay(values=values,head_deg=np.median(head[pre]),lift_mm=np.median(lift[pre]),noise_seed=0)
    desired=target if joint=="head" else 32+60*target
    segment=recording.Segment(joint,joint,0,1.2,target=desired)
    path=replay._run(sim,segment,.32)
    ts=(1+np.arange(len(path)))*sim.dt-.32
    position=[x["head_deg" if joint=="head" else "lift_mm"] for x in path]
    ax=axes.flat[i]
    ax.plot(t[window],measured,"k.",label="Physical")
    ax.plot(ts,position,color="#d27613",ls="--",label="Frozen")
    rmse=lambda delay: float(np.mean((np.interp(t[window]-delay,ts,position)-measured)**2)**.5)
    delayed=None
    if joint=="head":
        delayed=rmse(.0916)
        ax.plot(ts+.0916,position,color="#197699",label="Prior 91.6 ms shift")
    scores.append((i+1,c["cycle"],joint,target,measured[0],np.median(measured[-3:]),rmse(0),delayed))
    ax.set(xlim=(-.03,1.0),title=f"{joint} {desired:g}"+("°" if joint=="head" else " mm"))
    if i%6==0:ax.set_ylabel(f"Cycle {c['cycle']}")
    if i>=12:ax.set_xlabel("Seconds")
fig.legend(*axes.flat[0].get_legend_handles_labels(),loc="outside lower center",ncol=3)
fig.savefig(here/"profiles.png",dpi=150)
with (here/"scores.csv").open("w") as f:
    w=csv.writer(f,lineterminator="\n")
    w.writerow("move cycle joint target initial settled frozen_rms shifted_rms".split())
    w.writerows(scores)
for joint in ["head","lift"]:
    r=[s for s in scores if s[2]==joint]
    print(joint,len(r),np.median([s[-2] for s in r]),[round(float(s[5]),1) for s in r])
    if joint=="head":print("shifted",np.median([s[-1] for s in r]),sum(s[-1]<s[-2] for s in r))
print("wheel max",max(np.max(np.abs(states[k])) for k in ["left_mmps","right_mmps"]))
