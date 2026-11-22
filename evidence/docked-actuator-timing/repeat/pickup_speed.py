"""Compare unloaded docked raises at the native pickup speed to both cube pickups."""
import csv
import json
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from pickup_transfer import crossing

here = Path(__file__).resolve().parent
repo = here.parents[2]
sys.path.insert(0,str(repo))
from simulacrum import params, replay
from simulacrum.robot import VectorSim


def model_trace(unit, speed):
    v = dict(params.values(params.load(model="vector_2" if unit==3 else "vector_1")),**replay.OPEN_TABLE)
    v["lift_speed_default_radps"] = speed
    sim = VectorSim(values=v,lift_mm=32)
    sim.step(round(.4/sim.dt))
    sim.command_lift_height(92)
    h = [sim.step().lift_height_mm for _ in range(round(1.5/sim.dt))]
    return sim.dt*np.arange(1,len(h)+1),np.array(h)


def main():
    fig,axes = plt.subplots(1,2,figsize=(10,4.5),sharey=True,layout="constrained")
    with (here/"pickup-speed.csv").open("w") as f:
        w = csv.writer(f,lineterminator="\n")
        w.writerow(["unit","source","trial","speed_cap_radps","40_to_80_ms","bracket_low_ms","bracket_high_ms"])
        for unit,ax in zip((1,3),axes):
            def add(label,t,h,speed,trial,color,style):
                a,b = [crossing(t,h,v) for v in (40,80)]
                w.writerow([unit,label,trial,speed,*np.round(1000*np.array([b[0]-a[0],b[1]-a[2],b[2]-a[1]]),1)])
                ax.plot(t-a[0],h,style,color=color,lw=1.2,ms=3,label=label if trial==1 else None)
            for speed,prefix in [(10,"unit"),(1.5,"slow")]:
                rows = list(map(json.loads,(here/f"{prefix}{unit}.jsonl").read_text().splitlines()))
                cmds = [r for r in rows if r.get("name")=="lift" and r.get("norm")==1]
                for j,c in enumerate(cmds,1):
                    states = [s for s in rows if s["k"]=="s" and 0<=s["t"]-c["t"]<1.45]
                    t,h = np.array([(s["t"]-c["t"],s["lift"]) for s in states]).T
                    add(f"Empty docked, {speed:g} rad/s",t,h,speed,j,"C0" if speed==10 else "C1",".-")
                add(f"Empty replay, {speed:g} rad/s",*model_trace(unit,speed),speed,1,"C0" if speed==10 else "C1","--")
            if unit==3:
                for name,path,clock,height in [
                    ("Earlier pickup","unit3-cube-accelerometer/pickup-path.csv","time_s","lift_mm"),
                    ("R5 pickup","unit3-wheelstand/R5/approach.csv","sec","lift")]:
                    a=np.genfromtxt(repo/"evidence"/path,names=True,delimiter=",")
                    mask=(a[clock]>=1.8)&(a[clock]<5.5)
                    add(name,a[clock][mask],a[height][mask],1.5,1,"C2" if name=="Earlier pickup" else "C3",".-")
            ax.set(title=f"Unit {unit}",xlim=(-.15,.75),ylim=(28,96),xlabel="Seconds from 40 mm")
            ax.legend(fontsize=8)
    axes[0].set_ylabel("Wrist height (mm)")
    fig.savefig(here/"pickup-speed.png",dpi=150)


if __name__=="__main__":
    main()
