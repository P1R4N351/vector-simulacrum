"""Summarize only cases reproducing the two near-zero packet slots."""
import csv
from pathlib import Path
import matplotlib.pyplot as plt

root=Path(__file__).resolve().parent
rows=list(csv.DictReader((root/"browser-impact-slip.csv").open()))
fig,axes=plt.subplots(1,2,figsize=(9,4),layout="constrained")
for gap in (1,2,3):
    valid=[r for r in rows if int(r["gap_mm"])==gap and int(r["low_pairs"])]
    for ax,field in zip(axes,("nearest_peak_x_g","xyz_rmse_g")):
        ax.scatter([float(r["velocity_mmps"])+(gap-2)*4 for r in valid],
                   [float(r[field]) for r in valid],s=20,alpha=.7,label=f"{gap} mm")
axes[0].axhline(15057/8192,ls="--",color="black",label="Recorded X")
axes[0].set_ylabel("Signed X in the recorded peak slot (g)")
pickup=list(csv.DictReader((root/"browser-impact-slip-pickup.csv").open()))
for dataset,label in ((rows,"Shelf return"),(pickup,"Pickup release")):
    xs,ys=[],[]
    for vel in (-150,-75,0,75,150):
        subset=[float(r["xyz_rmse_g"]) for r in dataset if r["low_pairs"]!="0" and int(r["velocity_mmps"])==vel]
        if subset:
            xs.append(vel)
            ys.append(min(subset))
    axes[1].plot(xs,ys,"o--",label=f"{label} best case")
axes[1].legend(fontsize=8)
axes[1].set_ylabel("Three post-drop slots, XYZ RMS error (g)")
for ax in axes:
    ax.set_xlabel("Initial horizontal velocity (mm/s)")
axes[0].legend(fontsize=8)
fig.savefig(root/"browser-impact-slip.png",dpi=155)
for vel in (-150,-75,0,75,150):
    cases=[r for r in rows if int(r["velocity_mmps"])==vel and int(r["low_pairs"])]
    print(vel,len(cases),max((float(r["nearest_peak_x_g"]) for r in cases),default=None),
          min((float(r["xyz_rmse_g"]) for r in cases),default=None))
