"""Compare signed contact moments over the two photographed encoder frames."""
import csv
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np

from pickup_onset import action_time, read

here = Path(__file__).resolve().parent
start = action_time(here/"actions.csv","pickup for inward transport","sec")
light = read(here/"raise-lights.csv")
lo, hi = light["receipt_s"][np.isin(light["frame"],[91,94])]-start
fig,axes = plt.subplots(2,1,figsize=(8,5),sharex=True,layout="constrained")
rows=[]
for ax,folder in zip(axes,sys.argv[1:],strict=True):
    a=np.loadtxt(Path(folder)/"tipping-moment.csv",delimiter=",",skiprows=1)
    dt=np.median(np.diff(a[:,0]))
    n=round(.005/dt)
    smooth=a[:len(a)//n*n].reshape(-1,n,5).mean(axis=1)
    for i,label in enumerate(["Shelf","Printed fork","Lift links"]):
        ax.plot(smooth[:,0]-lo,smooth[:,2+i],label=label)
    ax.axvspan(0,hi-lo,alpha=.12,color="black")
    ax.axhline(0,color=".5",lw=.6)
    ax.set(title=f"{dt*1000:.3g} ms solver step",ylabel="Tipping moment (mN·m)")
    use=(a[:,0]>=lo)&(a[:,0]<hi)
    means=a[use,2:].mean(axis=0)
    pose=read(Path(folder)/"pickup-replay.csv")
    prior=np.interp([lo-.01,lo],pose["sec"],pose["tilt_deg"])
    rate=(prior[1]-prior[0])/.01
    observed=(pose["sec"]>=lo)&(pose["sec"]<hi)
    rows.append([dt*1000,lo,hi,*means,means.sum(),rate,np.mean(pose["floor_contact"][observed])])
axes[0].legend(ncol=3,fontsize=8)
axes[1].set_xlabel("Seconds from physical 38.6 mm wrist frame")
fig.savefig(here/"moment-overlay.png",dpi=150)
with (here/"moment-overlay.csv").open("w") as stream:
    writer=csv.writer(stream)
    writer.writerow("step_ms start_s end_s mean_shelf_mNm mean_fork_mNm mean_links_mNm mean_total_mNm incoming_tilt_rate_dps floor_fraction_10ms".split())
    writer.writerows(rows)
