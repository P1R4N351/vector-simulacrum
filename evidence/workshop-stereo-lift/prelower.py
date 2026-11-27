"""Forecast recovery from the last 200 ms before a matched lowering command."""
import csv
from pathlib import Path
import sys
import numpy as np
import matplotlib.pyplot as plt

here=Path(__file__).resolve().parent
base=Path(sys.argv[1]) if len(sys.argv)>1 else here.parent/"workshop-joints-20261127/lift-duration"
held=Path(sys.argv[2]) if len(sys.argv)>2 else here/"recovery/sdk.csv"
def examples(path):
    with path.open() as f:
        rows=list(csv.DictReader(f))
    clock="s" if "s" in rows[0] else "relative_s"
    target="target_mm" if clock=="s" else "target"
    commands=[r for r in rows if r["event"]=="command"]
    states=[r for r in rows if r["event"]=="state"]
    time=np.array([float(r[clock]) for r in states])
    h=np.array([float(r["lift_mm"]) for r in states])
    flag=np.array([int(r["in_position"]) for r in states])
    result=[]
    for i,c in enumerate(commands):
        if i==0 or float(c[target])!=52 or float(commands[i-1][target])!=64:continue
        if clock=="s" and float(c["duration_s"])!=.6:continue
        start=float(c[clock]);stop=float(commands[i+1][clock]) if i+1<len(commands) else time[-1]+.01
        pre=h[(time>=start-.2)&(time<start)]
        assert len(pre)>=2
        off=np.flatnonzero((time>=start)&(time<stop)&(flag==0))[0]
        k=off+np.flatnonzero(flag[off:]==1)[0]
        assert time[k]<stop
        result.append([np.median(pre),h[k]-min(h[off:k+1]),time[k]-start])
    return np.array(result)

train=np.vstack([examples(base/s/"recording.csv") for s in ["status-52","history-52"]])
test=examples(held)
assert (len(train),len(test))==(16,6)
def crps(draws,actual):
    return np.mean(abs(draws-actual))-.5*np.mean(abs(draws[:,None]-draws))
sim_start=np.genfromtxt(here/"prelower-sim.csv",names=True,delimiter=",")["sim_initial_mm"]
assert len(sim_start)==len(test)
fig,axes=plt.subplots(1,2,figsize=(10,4),layout="constrained")
with open("prelower-scores.csv","w") as f:
    w=csv.writer(f,lineterminator="\n");w.writerow(["move","initial_mm","outcome","observed","pool_mean","conditional_mean","pool_crps","conditional_crps","pool_probability_positive","conditional_probability_positive","neighbors","sim_initial_mm","sim_conditional_mean"])
    for j,(ax,label) in enumerate(zip(axes,["Recovery (mm)","Time to first in-position packet (s)"]),1):
        ax.scatter(train[:,0],train[:,j],label="Earlier sessions",alpha=.7)
        for i,row in enumerate(test):
            distance=abs(np.rint(100*train[:,0])-round(100*row[0]))
            # Include all ties at the fourth-nearest distance.
            near=train[distance<=np.sort(distance)[3],j]
            sim_distance=abs(np.rint(100*train[:,0])-round(100*sim_start[i]))
            sim_near=train[sim_distance<=np.sort(sim_distance)[3],j]
            ax.scatter(row[0],row[j],color="black",marker="x",label="Stereo repeat" if not i else None)
            ax.scatter(row[0],np.mean(near),color="tab:orange",marker="+",label="Height-based mean" if not i else None)
            w.writerow([i+1,row[0],label,row[j],train[:,j].mean(),near.mean(),crps(train[:,j],row[j]),crps(near,row[j]),np.mean(train[:,j]>0),np.mean(near>0),len(near),sim_start[i],sim_near.mean()])
        ax.set(xlabel="Median measured height before command (mm)",ylabel=label)
        ax.legend(fontsize=8)
fig.savefig("prelower.png",dpi=160)
print("train",train.tolist(),"test",test.tolist())
