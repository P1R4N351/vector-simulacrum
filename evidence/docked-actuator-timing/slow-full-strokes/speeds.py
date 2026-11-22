"""Compare the physical and simulated shoulder speeds at the same wrist heights."""
import csv
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np

here=Path(__file__).resolve().parent
sys.path.insert(0,str(here.parent))
from analyze import GyroReplay, params, recording, replay

values=dict(params.values(params.load(model="vector_2")),**replay.OPEN_TABLE)
states=np.genfromtxt(here/"states.csv",names=True,delimiter=",")
fig,axes=plt.subplots(2,2,figsize=(9,6),layout="constrained")
scores=[]


def profile(t,h):
    # A fixed central height band excludes acceleration and terminal settling.
    mask=(h>=45)&(h<=78)
    angles=np.arcsin((h[mask]-values["lift_base_position_mm"][2])/values["lift_arm_length_mm"])
    times=t[mask]
    slope,intercept=np.polyfit(times,angles,1)
    return slope,-intercept/slope,len(times)


for command in csv.DictReader((here/"commands.csv").read_text().splitlines()):
    t=states["receipt_s"]-float(command["start_s"])
    speed=float(command["speed_radps"])
    target=32+60*float(command["target"])
    pre=(t<0)&(t>-.3)
    stop=float(command["end_s"])-float(command["start_s"])+.35
    active=(t>=0)&(t<=stop)
    sim=GyroReplay(values=dict(values,lift_speed_default_radps=speed),
                   head_deg=np.degrees(np.median(states["head_rad"][pre])),
                   lift_mm=np.median(states["lift_mm"][pre]),noise_seed=0)
    run=replay._run(sim,recording.Segment("lift","lift",0,stop,target=target),.32)
    ts=(1+np.arange(len(run)))*sim.dt-.32
    model=np.array([s["lift_mm"] for s in run])
    hw,anchor,n=profile(t[active],states["lift_mm"][active])
    predicted,model_anchor,_=profile(ts,model)
    scores.append((command["cycle"],target,speed,hw,predicted,n))
    ax=axes[int(speed>.6),int(target<40)]
    labels=int(command["cycle"]) in (1,2)
    ax.plot(t[active]-anchor,states["lift_mm"][active],"k.-",ms=3,label="Physical" if labels else None)
    ax.plot(ts-model_anchor,model,"C1--",label="Simulated" if labels else None)
    ax.axhspan(45,78,color="gray",alpha=.06)
    ax.set(title=f"{speed} rad/s · {'lower' if target<40 else 'raise'}",ylabel="Wrist height (mm)")
for ax in axes[-1]:ax.set_xlabel("Seconds relative to fitted 45 mm crossing")
axes[0,0].legend()
fig.savefig(here/"speeds.png",dpi=160)
with (here/"speeds.csv").open("w") as f:
    w=csv.writer(f,lineterminator="\n")
    w.writerow("cycle target_mm cap_radps physical_radps simulated_radps physical_samples".split())
    w.writerows(scores)
for r in scores:print(r)

# Fit a cap multiplier on the first pair, then score actual model reruns.
import json
factor=np.mean([abs(s[3]/s[4]) for s in scores[:2]])
checks=[]
for session,group in [("New",scores[2:]),("Archive",[])]:
    if session=="Archive":
        events=list(map(json.loads,(here.parent/"repeat/slow3.jsonl").read_text().splitlines()))
        for c in events:
            if c.get("name")!="lift" or c.get("norm")!=1:
                continue
            q=[r for r in events if r["k"]=="s" and 0<=r["t"]-c["t"]<1.45]
            if max(r["lift"] for r in q)-min(r["lift"] for r in q)<40:
                continue
            hw,_,n=profile(np.array([r["t"]-c["t"] for r in q]),np.array([r["lift"] for r in q]))
            group.append((len(group)+1,32+60*c["norm"],1.5,hw,None,n))
    for cycle,target,speed,hw,base,n in group:
        result=[]
        for multiplier in (1.,factor):
            sim=GyroReplay(values=dict(values,lift_speed_default_radps=speed*multiplier),
                           lift_mm=92 if target==32 else 32,noise_seed=0)
            path=replay._run(sim,recording.Segment("lift","lift",0,3,target=target),.32)
            pred,_,_=profile(np.arange(len(path))*sim.dt,np.array([v["lift_mm"] for v in path]))
            result.append(pred)
        checks.append([session,cycle,speed,target,hw,*result])
for session in ("New","Archive"):
    r=np.array([row[4:] for row in checks if row[0]==session])
    print(session,len(r),"factor",factor,"RMSE",np.mean((r[:,1:]-r[:,0,None])**2,axis=0)**.5)
with (here/"transfer.csv").open("w") as f:
    w=csv.writer(f,lineterminator="\n")
    w.writerow("session cycle cap_radps target_mm physical_radps nominal_radps scaled_radps".split())
    w.writerows(checks)
