"""Separate encoder, reported in-position transition and SDK return on one clock."""
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

here=Path(__file__).resolve().parent
def read(name):
    with (here/name).open() as f:
        return list(csv.DictReader(f))

raw=read("recording.csv")
states=[r for r in raw if r["event"]=="state"]
model=read("replay.csv")
t=np.array([float(r["s"]) for r in states])
assert np.array_equal(t,[float(r["s"]) for r in model])
assert all(int(r["on_dock"]) for r in states+model)
commands=[r for r in raw if r["event"]=="command"]
acks=[r for r in raw if r["event"]=="return"]
simacks=read("returns.csv")
assert len(commands)==len(acks)==len(simacks)==17
assert all(r["code"]=="0" for r in acks) and all(r["result"]=="completed" for r in simacks)
fig,axes=plt.subplots(4,2,figsize=(10,8),sharex=True,layout="constrained")
scores=[]
for i in range(1,16,2):
    trial=i//2+1
    ax=axes.flat[trial-1]
    cue=float(commands[i]["s"])
    up=int(commands[i-1]["target_mm"])<52
    select=(t>=cue-.1)&(t<float(commands[i+1]["s"]))
    x=t[select]-cue
    for name,rows,ack,col in [("Physical",states,float(acks[i]["s"])-cue,"C0"),
                               ("Canonical",model,float(simacks[i]["sim_elapsed_s"]),"C1")]:
        flags=np.array([int(r["in_position"]) for r in rows])[select]
        y=np.array([float(r["lift_mm"]) for r in rows])[select]
        first_clear=np.flatnonzero((x>=0)&(flags==0))[0]
        on=first_clear+np.flatnonzero(flags[first_clear:])[0]
        scores.append([trial,"up" if up else "down",name,1000*x[on-1],1000*x[on],1000*ack,1000*(ack-x[on]),y[on]])
        ax.step(x,y,where="post",color=col,lw=1.4,label=name)
        ax.scatter(x[on],y[on],color=col,s=35)
        ax.axvline(ack,color=col,ls=":",lw=1.4)
    ax.set_title(f"Trial {trial} · {'raise' if up else 'lower'} to 52 mm",fontsize=10)
    ax.set_xlim(-.08,1.65)
    ax.grid(alpha=.2)
axes[0,0].legend()
fig.supxlabel("Seconds after command · Dot = first in-position packet after clear · Dotted line = action return")
fig.supylabel("Lift height (mm)")
fig.savefig(here/"flags.png",dpi=160)
with (here/"scores.csv").open("w") as f:
    w=csv.writer(f,lineterminator="\n")
    w.writerow("trial direction trace last_clear_ms first_set_ms return_ms return_minus_first_set_ms first_set_height_mm".split())
    w.writerows(scores)
for direction in ("up","down"):
    for name in ("Physical","Canonical"):
        a=np.array([r[3:] for r in scores if r[1:3]==[direction,name]])
        print(direction,name,"median ms last_clear,first_set,return,tail; height mm",np.round(np.median(a,axis=0),1),"tails",np.round(a[:,3]))
print(len(states),"physical packets")
