"""Summarize joint predictions and plot SDK-time head and gyro from held-out runs."""
import csv
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

root=Path(__file__).resolve().parent
with (root/"motor-traces.csv").open() as f: rows=list(csv.DictReader(f))
runs=list(dict.fromkeys(r["run"] for r in rows))
scores=[]
fig,axes=plt.subplots(2,4,figsize=(14,6),sharex=True,sharey="row",layout="constrained")
panels=["calm-cross-1","calm-cross-2","wake-raise-1","wake-raise-2"]
for run in runs:
    for mode in ("baseline","sdk-stream","motor","compensated"):
        a=np.array([[float(r[k]) for k in ("t_s","real_deg","real_gyro","sim_deg","sim_gyro")] for r in rows if r["run"]==run and r["mode"]==mode])
        rms=np.sqrt(np.mean((a[:,1:3]-a[:,3:5])**2,axis=0))
        scores.append([run,mode,len(a),*rms])
        if run in panels and mode!="sdk-stream":
            j=panels.index(run)
            for k in (0,1):
                ax=axes[k,j]
                if mode=="baseline":ax.plot(a[:,0],a[:,k+1],color="black",label="Recorded",lw=1.5)
                ax.plot(a[:,0],a[:,k+3],label=mode,color={"baseline":"0.65","motor":"C1","compensated":"C0"}[mode],lw=1)
            axes[0,j].set_title(["Calm grant 1","Calm grant 2","Raise at 1.5 s","Raise at 6 s"][j])
axes[0,0].set_ylabel("Head angle (°)")
axes[1,0].set_ylabel("Gyro Y (rad/s)")
for ax in axes[-1]:ax.set_xlabel("Seconds after grant")
axes[0,0].legend(fontsize=7)
fig.savefig(root/"motor-overlay.png",dpi=160)
with (root/"motor-scores.csv").open("w") as f:
    w=csv.writer(f,lineterminator="\n")
    w.writerow("run mode reports angle_rmse_deg gyro_rmse_rad_s".split())
    w.writerows(scores)
