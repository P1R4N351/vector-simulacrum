"""Score docked head strokes on the physical SDK receipt clock."""
from pathlib import Path
import csv
import matplotlib.pyplot as plt
import numpy as np
here=Path(__file__).resolve().parent
data=np.genfromtxt(here/"replay.csv",delimiter=",",names=True)
deg_per_count=180/(4*366.211)
rmse=lambda e: np.sqrt(np.mean(e**2))
fig,axes=plt.subplots(2,3,figsize=(10,5.5),layout="constrained",sharex=True)
out=[]
for u in (1,2,3):
    group=data[data["unit"]==u]
    for j in np.unique(group["trial"]):
        s=group[group["trial"]==j]
        t=s["t"]; gt=s["physical_head_deg"]; direction="raise" if gt[-1]>gt[0] else "lower"
        p=s["physical_gyro_y"]; q=s["sim_gyro_y"]
        old=s["legacy_counts"]*deg_per_count; new=s["native_counts"]*deg_per_count
        # Compare relative excursions; absolute encoder zero is boot/calibration dependent.
        zero=lambda a:a-a[0]
        out.append([u,int(j),direction,rmse(zero(old)-zero(gt)),rmse(zero(new)-zero(gt)),rmse(q-p),
                    zero(new)[-1]-zero(gt)[-1],np.degrees(np.trapezoid(-q,t)-np.trapezoid(-p,t))])
        ax=axes[int(direction=="lower"),u-1]
        ax.plot(t,p,color="k",alpha=.55,lw=1)
        ax.plot(t,q,color="tab:blue",alpha=.55,lw=1)
        ax.set_title(f"Unit {u} · {direction}")
        ax.set(xlabel="Time from command (s)",ylabel="Head gyro Y (rad/s)")
axes[0,0].plot([],[],"k-",label="Physical SDK");axes[0,0].plot([],[],color="tab:blue",label="Canonical sim")
axes[0,0].legend(fontsize=8)
fig.savefig(here/"gyro.png",dpi=145)
with (here/"scores.csv").open("w") as f:
    w=csv.writer(f,lineterminator="\n")
    w.writerow("unit trial direction legacy_travel_rmse_deg native_travel_rmse_deg gyro_rmse_radps endpoint_travel_error_deg integrated_gyro_difference_deg".split());w.writerows(out)
for u in (1,2,3):
    a=np.array([r[3:] for r in out if r[0]==u]);print(u,len(a),np.median(a,axis=0).round(2))
