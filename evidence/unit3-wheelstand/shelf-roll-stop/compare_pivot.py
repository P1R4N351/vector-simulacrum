"""Score simulated and observed net material-point displacement using the same envelope."""
import csv
from itertools import groupby
import numpy as np
import matplotlib.pyplot as plt
from material_pivot import closest, here

with (here/"browser-pivot-corners.csv").open() as f:
    corners=list(csv.DictReader(f))
rows=[]
for key,items in groupby(corners,key=lambda r:tuple(int(r[k]) for k in ("velocity_mmps","gap_mm","phase_ms","matches_low_pair"))):
    group=list(items)
    assert [int(r["corner"]) for r in group]==list(range(4))
    displacements=np.array([[float(r[k]) for k in ("dx_mm","dy_mm")] for r in group])
    lowest=np.argmin([float(r["initial_z_mm"]) for r in group])
    rows.append([*key,closest(displacements),np.linalg.norm(displacements[lowest])])
with (here/"browser-pivot.csv").open("w") as f:
    w=csv.writer(f,lineterminator="\n")
    w.writerow("speed gap phase low_pair min_face_mm lowest_corner_mm".split())
    w.writerows(rows)
with (here/"material-pivot.csv").open() as f:
    camera=[r for r in csv.DictReader(f) if r["threshold"]=="80"]
fig,ax=plt.subplots(figsize=(7,4.2),layout="constrained")
for b in ("0","1"):
    ys=[float(r["min_face_mm"]) for r in camera if r["branch"]==b]
    ax.axhspan(min(ys),max(ys),color=f"C{b}",alpha=.22,label=f"Camera branch {int(b)+1}")
for i in (1,2,3):
    subset=[r for r in rows if r[1]==i and r[3]]
    ax.scatter([r[0]+3*(i-2) for r in subset],[r[4] for r in subset],s=27,label=f"{i} mm gap")
ax.set(xlabel="Initial horizontal speed (mm/s)",ylabel="Least bottom-face point travel (mm)")
ax.legend(fontsize=8,ncols=2)
fig.savefig(here/"browser-pivot.png",dpi=155)
for speed in (-150,-75,0,75,150):
    selected=[r[4] for r in rows if r[0]==speed and r[3]]
    print(speed,min(selected),max(selected))
