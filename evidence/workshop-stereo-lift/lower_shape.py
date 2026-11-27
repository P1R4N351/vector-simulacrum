"""Predict first in-position packet from earlier encoder samples on matched lowers."""
import csv
from pathlib import Path
import sys
import numpy as np
import matplotlib.pyplot as plt

root=Path(sys.argv[1]) if len(sys.argv)>1 else Path("../workshop-joints-20261127/lift-duration")
runs=[]
for session in ["status-52","history-52"]:
    with (root/session/"recording.csv").open() as f:
        rows=list(csv.DictReader(f))
    sim=np.genfromtxt(root/session/"replay.csv",names=True,delimiter=",")
    commands=[(i,row) for i,row in enumerate(rows) if row["event"]=="command"]
    for n,(i,cmd) in enumerate(commands):
        if n==0 or float(cmd["target_mm"])!=52 or float(cmd["duration_s"])!=.6 or float(commands[n-1][1]["target_mm"])!=64:
            continue
        end=commands[n+1][0] if n+1<len(commands) else len(rows)
        samples=[r for r in rows[i+1:end] if r["event"]=="state"]
        t=np.array([float(r["s"])-float(cmd["s"]) for r in samples])
        h=np.array([float(r["lift_mm"]) for r in samples])
        flags=np.array([int(r["in_position"]) for r in samples])
        off=np.flatnonzero(flags==0)[0]
        k=off+np.flatnonzero(flags[off:]==1)[0]
        sim_rows=sim[(sim["s"]>=float(cmd["s"]))&(sim["s"]<=float(samples[-1]["s"]))]
        sim_off=np.flatnonzero(sim_rows["in_position"]==0)[0]
        sim_k=sim_off+np.flatnonzero(sim_rows["in_position"][sim_off:]==1)[0]
        sim_min=min(sim_rows["lift_mm"][:sim_k+1])
        runs.append(dict(sim_min=sim_min,sim_flag=sim_rows["s"][sim_k]-float(cmd["s"]),session=session,index=n+1,t=t,h=h,ready=t[k],minimum=min(h[:k+1]),recovery=h[k]-min(h[:k+1])))
def feature(run,cut):
    grid=np.arange(.2,cut+.001,.05)
    # Zero-order hold: each feature uses only packets received by its grid time.
    indices=np.searchsorted(run["t"],grid,side="right")-1
    assert np.all(indices>=0)
    return run["h"][indices]
fig,(ax,b)=plt.subplots(1,2,figsize=(11,4.5),layout="constrained")
out=[]
for r in runs:
    train=[s for s in runs if s["session"]!=r["session"]]
    predicted=[np.median([s["ready"] for s in train])]
    for cut in [.6,.8]:
        best=min(train,key=lambda s:np.mean(abs(feature(s,cut)-feature(r,cut))))
        predicted.append(best["ready"])
    out.append([r["session"],r["index"],r["minimum"],r["recovery"],r["ready"],*predicted])
    late=r["ready"]>1.0
    color="tab:orange" if late else "tab:blue"
    m=r["t"]<=r["ready"]
    ax.step(r["t"][m],r["h"][m],where="post",color=color,alpha=.55)
    ax.scatter(r["ready"],r["h"][m][-1],color=color,s=15)
    b.scatter(r["minimum"],r["ready"],color=color,marker="x" if r["session"]=="status-52" else "o")
ax.axhline(52,color="black",ls="--",lw=1)
ax.set(xlabel="Time since command (s)",ylabel="Reported height (mm)",title="Encoder traces through first in-position packet")
b.set(xlabel="Minimum reported height through first flag (mm)",ylabel="First in-position packet (s)",title="Points represent individual physical lowers")
print("sim minimum and first flag ranges",[(min(r[k] for r in runs),max(r[k] for r in runs)) for k in ("sim_min","sim_flag")])
fig.savefig("lower-shape.png",dpi=150)
with open("lower-shape.csv","w") as f:
    w=csv.writer(f,lineterminator="\n");w.writerow(["session","command_index","minimum_mm","recovery_mm","first_flag_s","other_session_median_s","neighbor_600ms_s","neighbor_800ms_s"]);w.writerows(out)
for group in ["status-52","history-52"]:
    selected=np.array([r[2:] for r in out if r[0]==group])
    print(group,len(selected),np.mean(abs(selected[:,3:]-selected[:,2,None]),axis=0))
for late in [False,True]:
    group=[r for r in runs if (r["ready"]>1)==late]
    print(late,len(group),[(k,min(r[k] for r in group),max(r[k] for r in group)) for k in ("minimum","recovery","ready")])
