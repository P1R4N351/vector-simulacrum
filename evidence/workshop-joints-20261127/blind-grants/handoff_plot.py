"""Overlay the forked head trajectories near each SDK raise."""
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

root=Path(__file__).parent
with (root/"handoff-traces.csv").open() as f:
    traces=list(csv.DictReader(f))

fig,axes=plt.subplots(2,3,figsize=(13,6),layout="constrained",sharex=True,sharey="row")
for col,cue in enumerate((1.5,6,1.5)):
    for mode,color,label in (("immediate",".5","Immediate"),("in-position","C1","Motor in position"),
                             ("first-run-delay","C0","Delay from run 1")):
        selected=[r for r in traces if int(r["run"])==col+1 and r["mode"]==mode]
        t=np.array([float(r["t_s"]) for r in selected])-cue
        for k,key in enumerate(("deg","gyro")):
            ax=axes[k,col]
            if mode=="immediate":
                ax.plot(t,[float(r["real_"+key]) for r in selected],"k.-",ms=3,label="Recorded")
            ax.plot(t,[float(r["sim_"+key]) for r in selected],color=color,ls="--",label=label)
            ax.set_xlim(-.08,1.85)
    axes[0,col].set_title(["Early · fit","Late · control","Early · held out"][col])
    axes[1,col].set_xlabel("Seconds after head command")
axes[0,0].set_ylabel("Head angle (°)")
axes[1,0].set_ylabel("Gyro Y (rad/s)")
axes[0,0].legend(fontsize=7)
fig.savefig(root/"handoff.png",dpi=160)
