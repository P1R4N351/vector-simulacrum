"""Evaluate each initial seating against later camera frames and electrical contact."""
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

here=Path(__file__).resolve().parent
def read(path):
    return np.genfromtxt(path,delimiter=",",names=True)
cam=read(here.parent/"unit3-w34-return/departure-hubs.csv")
t=cam["t_s"]
scale=48/(cam["rear_x"][0]-cam["front_x"][0])
hubs=np.array([(cam[k][0]-cam[k]+cam["static_x"]-cam["static_x"][0])*scale for k in ("rear_x","front_x")])
fig,axes=plt.subplots(2,2,figsize=(10,7),sharex=True,sharey=True,layout="constrained")
with (here/"scores.csv").open("w") as f:
    w=csv.writer(f,lineterminator="\n")
    w.writerow("constraints drive registration shift_mm initial_pad_mm initial_contact first_off_ms x_at_physical_off_mm x_at_sim_off_mm rear_track_rmse_mm front_track_rmse_mm".split())
    for i,c in enumerate(("base","both")):
        for j,d in enumerate(("command","shaft")):
            ax=axes[i,j]
            for k,h in enumerate(hubs):
                ax.plot(t,h,"k." if k==0 else "k+",alpha=.6,label=["Rear hub","Front hub"][k])
            for reg in ("nominal","glyph","glyph-lip"):
                r=read(here/f"{c}-{d}-{reg}.csv")
                s=r["s"]
                x=r["chassis_x_mm"]-np.interp(t[0],s,r["chassis_x_mm"])
                start=np.searchsorted(s,t[0])
                off=np.flatnonzero((r["contact"][:-1]==1)&(r["contact"][1:]==0))+1
                off=off[off>=start]
                at=s[off[0]] if len(off) and r["contact"][start] else np.nan
                line,=ax.plot(s,x,label=reg)
                if np.isfinite(at):
                    ax.scatter(at,np.interp(at,s,x),marker="x",color=line.get_color())
                usable=(t>t[0])&(t<=s[-1])
                errors=np.sqrt(np.mean((np.interp(t[usable],s,x)-hubs[:,usable])**2,axis=1))
                w.writerow([c,d,reg,r["shift_mm"][start],r["pad_outward_mm"][start],r["contact"][start],
                            1000*at,np.interp(0,s,x),np.interp(at,s,x),*errors])
            ax.axvspan(-.081,0,color="gray",alpha=.16)
            ax.set(title=f"{c} constraints · {d} drive",xlim=(-.49,.55),ylim=(-1,24))
axes[0,0].legend(fontsize=7)
fig.supylabel("Outward travel from first camera frame (mm)")
fig.supxlabel("Seconds from physical first off · × simulated release")
fig.savefig(here/"release.png",dpi=160)
