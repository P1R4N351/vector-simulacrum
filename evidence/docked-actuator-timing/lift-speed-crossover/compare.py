"""Test a frozen command-clock shift on held-out short lift strokes at two speeds."""
import csv
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np

here=Path(__file__).resolve().parent
sys.path.insert(0,str(here.parent))
from analyze import GyroReplay, params, recording, replay

strokes=[]
values=dict(params.values(params.load(model="vector_2")),**replay.OPEN_TABLE)
for session,folder in [("Original",here.parent/"partial-strokes"),("Crossover",here)]:
    states=np.genfromtxt(folder/"states.csv",names=True,delimiter=",")
    for c in csv.DictReader((folder/"commands.csv").read_text().splitlines()):
        if c["joint"]!="lift":
            continue
        target=32+60*float(c["target"])
        speed=float(c.get("speed_radps",10))
        t=states["receipt_s"]-float(c["start_s"])
        pre=(t<0)&(t>-.3)
        window=(t>=0)&(t<=1.2)
        sim=GyroReplay(values=dict(values,lift_speed_default_radps=speed),
                       head_deg=np.degrees(np.median(states["head_rad"][pre])),
                       lift_mm=np.median(states["lift_mm"][pre]),noise_seed=0)
        seg=recording.Segment("lift","lift",0,1.3,target=target)
        model=replay._run(sim,seg,.32)
        ts=(1+np.arange(len(model)))*sim.dt-.32
        pos=np.array([r["lift_mm"] for r in model])
        strokes.append((session,int(c["cycle"]),speed,target,t[window],states["lift_mm"][window],ts,pos))


def rms(stroke,delay):
    _,_,_,_,t,h,ts,pos=stroke
    return np.mean((np.interp(t-delay,ts,pos)-h)**2)**.5


def crossings(t,h):
    a=np.median(h[:2]);b=np.median(h[-3:])
    u=(h-a)/(b-a)
    def at(f):
        i=np.flatnonzero(u>=f)[0]
        return np.interp(f,u[i-1:i+1],t[i-1:i+1])
    return at(.1),at(.9)


delay=min(np.arange(0,.251,.001),key=lambda d:sum(rms(s,d)**2 for s in strokes[:2]))
rows=[]
fig,axes=plt.subplots(2,2,figsize=(9,6),layout="constrained",sharex=True,sharey=True)
for s in strokes:
    session,cycle,speed,target,t,h,ts,pos=s
    p,q=crossings(t,h)
    a,b=crossings(ts,pos)
    rows.append([session,cycle,speed,target,rms(s,0),rms(s,delay),1000*(p-a),1000*(q-p),1000*(b-a)])
    if session=="Crossover":
        ax=axes[int(speed==1.5),int(target==32)]
        ax.plot(t-p,h,"k.-",ms=3,label="Recorded" if cycle in (1,2) else None)
        ax.plot(ts-a,pos,"C1--",label="Frozen" if cycle in (1,2) else None)
        ax.set(title=f"{speed:g} rad/s · {'raise' if target>32 else 'lower'}",xlim=(-.15,.85))
for ax in axes[-1]:ax.set_xlabel("Seconds from 10% travel")
for ax in axes[:,0]:ax.set_ylabel("Lift height (mm)")
axes[0,0].legend()
fig.savefig(here/"profiles.png",dpi=160)
with (here/"scores.csv").open("w") as f:
    w=csv.writer(f,lineterminator="\n")
    w.writerow("session cycle speed_radps target_mm frozen_rms_mm shifted_rms_mm delay_at_10pct_ms real_10_90_ms sim_10_90_ms".split())
    w.writerows(rows)
print("fit delay",delay)
for r in rows:print(r[:4],np.round(r[4:],2))
