"""Score the native SDK packet times; compare calibrated position and gyro without shifting the traces."""
import csv
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
runs = {}
with (here/"motor-traces.csv").open() as f:
    for row in csv.DictReader(f):
        group = runs.setdefault((row["run"],row["mode"]),[])
        group.append([float(row[k]) for k in ("t_s","real_deg","real_gyro","sim_deg","sim_gyro")])
with (here/"motor-scores.csv").open("w") as f:
    w = csv.writer(f,lineterminator="\n")
    w.writerow(["run","mode","window_s","n","encoder_rmse_deg","gyro_rmse_radps"])
    for (run,mode),rows in runs.items():
        a = np.array(rows)
        for end in (5.2,8):
            b = a[a[:,0]<end]
            w.writerow([run,mode,end,len(b),*np.sqrt(np.mean((b[:,1:3]-b[:,3:5])**2,axis=0))])

fig,axes = plt.subplots(2,3,figsize=(12,6),sharex=True,sharey="row",layout="constrained")
for col,(run,title) in enumerate([("calm-cross-2","Second unit · calm wake"),("wake-raise-3","Early raise · repeat"),("wake-raise-2","Late raise")]):
    for key,label in (("untimed","Default motor speed"),("timed","Keyframe duration")):
        a = np.array(runs[run,key])
        for row in (0,1):
            ax = axes[row,col]
            if key == "untimed": ax.plot(a[:,0],a[:,row+1],color="black",lw=1.4,label="Measured")
            ax.plot(a[:,0],a[:,row+3],lw=1,ls="--" if key=="untimed" else "-",label=label)
    axes[0,col].set_title(title)
    axes[1,col].set_xlabel("Seconds after SDK grant")
axes[0,0].set_ylabel("Head angle (°)")
axes[1,0].set_ylabel("Gyro Y (rad/s)")
axes[0,0].legend(fontsize=8)
fig.savefig(here/"motor.png",dpi=160)

# Diagnostic only: compare on an identical interior interval for every offset.
with (here/"gyro-phase.csv").open("w") as f:
    w = csv.writer(f,lineterminator="\n")
    w.writerow(["run","sim_delay_ms","gyro_rmse_radps"])
    for (run,mode),rows in runs.items():
        if mode != "timed" or run.startswith("blind-"): continue
        a = np.array(rows)
        interior = (a[:,0]>=.3)&(a[:,0]<4.9)
        t = a[interior,0]
        for delay_ms in range(-180,181,10):
            y = np.interp(t-delay_ms*.001,a[:,0],a[:,4])
            w.writerow([run,delay_ms,np.sqrt(np.mean((a[interior,2]-y)**2))])
