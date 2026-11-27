"""Compare reconstructed net horizontal travel and the simulated releases."""
import csv
from pathlib import Path
import matplotlib.pyplot as plt

root=Path(__file__).resolve().parent
def read(name):
    with (root/name).open() as stream:
        return list(csv.DictReader(stream))

camera=read("release-translation.csv")
runs=read("browser-release-travel.csv")
fig,ax=plt.subplots(figsize=(7.5,4.2),layout="constrained")
for branch,color in (("0","C0"),("1","C1")):
    distances=[float(r["shelf_norm_mm"]) for r in camera if r["branch"]==branch]
    ax.axhspan(min(distances),max(distances),color=color,alpha=.2,label=f"Camera pose branch {int(branch)+1}")
for key,marker,label in (("0","x","Other release"),("1","o","Two low-force slots match")):
    cases=[r for r in runs if r["matches_low_pair"]==key]
    ax.scatter([float(r["velocity_mmps"])+(int(r["gap_mm"])-2)*4 for r in cases],
               [float(r["horizontal_mm"]) for r in cases],marker=marker,label=label,alpha=.65,s=28)
ax.set(xlabel="Simulated horizontal release speed (mm/s)",ylabel="Net horizontal travel (mm)")
ax.legend(fontsize=8)
fig.savefig(root/"browser-release-travel.png",dpi=155)
for v in (-150,-75,0,75,150):
    selected=[float(r["horizontal_mm"]) for r in runs if r["matches_low_pair"]=="1" and float(r["velocity_mmps"])==v]
    print(v,min(selected),max(selected))
