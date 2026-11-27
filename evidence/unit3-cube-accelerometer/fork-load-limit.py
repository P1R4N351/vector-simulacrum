"""Vertical impulse available from the recorded fork contacts with fixed normals and loads."""
from pathlib import Path
import csv
import numpy as np
import matplotlib.pyplot as plt

here=Path(__file__).resolve().parent
data=np.genfromtxt(here/"pickup-patch.csv",delimiter=",",names=True,dtype=None,encoding="utf8")
weight=.05*9.81
def loads(lo,hi):
    r=data[(data["time_s"]>=lo)&(data["time_s"]<hi)]
    rubber=np.char.startswith(r["cube_geom"],"cube_rubber")
    tangent=r["normal_impulse_Ns"]*np.sqrt(np.maximum(0,1-r["normal_z"]**2))
    normal=np.sum(r["normal_up_Ns"])
    actual=normal+np.sum(r["friction_up_Ns"])
    limit=normal+.5*np.sum(tangent[~rubber])+.9*np.sum(tangent[rubber])
    return np.array([actual,limit,limit+.9*np.sum(tangent[rubber])])/(weight*(hi-lo))

with (here/"fork-load-limit.csv").open("w") as f:
    w=csv.writer(f,lineterminator="\n")
    w.writerow(["start_s","end_s","actual_weight_fraction","upper_bound_nominal","upper_bound_double_pad_mu"])
    for a,b in [(2.23,2.44),(2.953,3.01),(3.01,3.065),(3.065,3.165),(3.165,3.36),(3.01,3.36)]:
        w.writerow([a,b,*loads(a,b)])
edges=np.arange(2170,3371,20)/1000
vals=np.array([loads(a,b) for a,b in zip(edges,edges[1:])])
fig,(ax,bx)=plt.subplots(2,1,figsize=(9,7),layout="constrained")
for i,label in enumerate(["Applied vertical force","Nominal friction bound","Double pad friction bound"]):
    ax.stairs(vals[:,i],edges,label=label)
ax.axhline(1,color="k",ls=":",label="Cube weight")
ax.axvspan(2.953,3.010,alpha=.16,color="gray",label="Physical wrist onset bracket")
ax.set(xlabel="Seconds from pickup command",ylabel="Fork vertical force / cube weight")
ax.legend(fontsize=8)
table=np.genfromtxt(here/"fork-rake.csv",names=True,delimiter=",")
for step in np.unique(table["dt_ms"]):
    r=table[table["dt_ms"]==step]
    r=r[np.argsort(r["rake_deg"])]
    bx.plot(r["rake_deg"],r["slide_2p86_mm"],"o-",label=f"Planar travel, {step:g} ms")
    bx.plot(r["rake_deg"],r["held_rise_mm"],"x--",label=f"Held rise, {step:g} ms")
bx.axhline(27.199,color="k",ls=":",label="Camera planar travel at 2.86 s")
bx.set(xlabel="Fork pitch offset about wrist (°)",ylabel="Distance (mm)")
bx.legend(fontsize=8)
fig.savefig(here/"fork-load-limit.png",dpi=150)
