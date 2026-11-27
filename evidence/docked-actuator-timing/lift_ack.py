"""Score SDK lift completion and in-position status against the recorded docked moves."""
import csv
import gzip
import json
from math import degrees, radians
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from simulacrum import params
from simulacrum.robot import VectorSim

here=Path(__file__).parent.resolve()


def measure(events,unit,history):
    states=[r for r in events if r.get("k")=="s" or r.get("kind")=="state"]
    values=params.values(params.load(model="vector_2" if unit==3 else "vector_1"))
    rows=[]
    for i,cue in enumerate(events):
        if cue.get("name")!="lift" and cue.get("kind")!="command":continue
        done=next(r for r in events[i+1:] if r.get("name")=="lift_done" or r.get("kind")=="arrived")
        assert done.get("code",0)==0
        t=cue["t"]
        pose=next(r for r in reversed(states) if r["t"]<t)
        target=32+60*(cue["pos"] if history else cue["norm"])
        same=abs(target-pose["lift"])<2
        response=done["t"]-t
        # Distinguish a new flag edge from the preceding command's latched status.
        nextcue=next((r["t"] for r in events[i+1:] if r.get("name")=="lift" or r.get("kind")=="command"),done["t"]+.5)
        window=[r for r in states if t<=r["t"]<min(nextcue,done["t"]+.5)]
        clear=next((r for r in window if not r["status"]&256),None)
        edge=next((r for r in window if clear and r["t"]>clear["t"] and r["status"]&256),None)
        sim=VectorSim(values,head_deg=degrees(pose["head"]),lift_mm=pose["lift"],charger_contact=True)
        duration=.7 if history else 0
        sim.command_lift_height(target,duration_s=duration)
        tol=radians(values["lift_angle_tol_deg"])
        while abs(float(sim.data.qpos[sim._q["lift"]])-sim._lift_target)>tol and sim.tick*sim.dt<3:sim.step()
        current=sim.tick*sim.dt
        while not sim._lift_in_pos and sim.tick*sim.dt<3:sim.step()
        settled=sim.tick*sim.dt if sim._lift_in_pos else np.nan
        action_sim=VectorSim(values,head_deg=degrees(pose["head"]),lift_mm=pose["lift"],charger_contact=True)
        action=action_sim.set_lift_height(target,duration_s=duration)
        assert action.result=="completed"
        rows.append([unit,"history" if history else "full",cue.get("block",""),target,pose["lift"],same,
                     response,edge["t"]-t if edge else np.nan,current,settled,action.ticks*action_sim.dt])
    return rows


def main():
    rows=[]
    for unit,path in [(1,"repeat/unit1.jsonl"),(2,"unit2.jsonl"),(3,"repeat/unit3.jsonl")]:
        rows.extend(measure([json.loads(x) for x in (here/path).read_text().splitlines()],unit,False))
    with gzip.open(here.parent/"unit3-lift-history/physical.jsonl.gz","rt") as f:
        rows.extend(measure([json.loads(x) for x in f],3,True))
    with (here/"lift-ack.csv").open("w") as f:
        w=csv.writer(f,lineterminator="\n")
        w.writerow("unit protocol block target_mm start_mm near_target sdk_success_s physical_flag_edge_s sim_success_s sim_settled_s revised_success_s".split())
        w.writerows(rows)
    fig,axes=plt.subplots(1,2,figsize=(10,4),layout="constrained")
    for i,(protocol,name) in enumerate([("full","Endpoint cycles"),("history","0.7 s partial and full moves")]):
        group=[r for r in rows if r[1]==protocol and not r[5]]
        for j,label,color in [(7,"Measured flag","black"),(8,"Earlier completion","C1"),(10,"Revised completion","C0")]:
            axes[i].scatter([r[6] for r in group],[r[j] for r in group],alpha=.7,label=label,s=20,color=color)
        axes[i].plot([.3,1.7],[.3,1.7],":",color=".5")
        axes[i].set(title=name,xlabel="SDK success (s)",ylabel="Seconds from command")
    axes[0].legend(fontsize=8)
    fig.savefig(here/"lift-ack.png",dpi=160)


if __name__=="__main__": main()
