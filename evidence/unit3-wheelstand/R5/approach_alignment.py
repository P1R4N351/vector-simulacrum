"""Compare approach displacement along the two initial cube axes and signed yaw."""
import csv
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np

from pickup_onset import read

here = Path(__file__).resolve().parent
g = read(here/"approach-glyph-axes.csv")
fig, axes = plt.subplots(3, 1, figsize=(8,7), sharex=True, layout="constrained")
rows = []
def record(label,t,x,y,yaw):
    # Sorting unsigned axis displacements removes the unknown glyph-to-SDK quarter turn.
    cross, along = np.sort(np.abs([x,y]),axis=0)
    for ax, values in zip(axes,(along,cross,yaw),strict=True):
        ax.plot(t,values,label=label)
    rows.append([label,t[-1],along[-1],cross[-1],np.rad2deg(np.arctan2(cross[-1],along[-1])),yaw[-1]])

for width in (20,23):
    obs = g[g["width_mm"]==width]
    t = obs["sec"]
    record(f"Shelf · {width} mm glyph",t,obs["along_edge_mm"],obs["across_edge_mm"],obs["yaw_change_deg"])

for directory, step, mode in zip(sys.argv[1:],(.25,.25,.125,.125),
                                 ("Pose only","Velocity","Pose only","Velocity"),strict=True):
    d = read(Path(directory)/"pickup-replay.csv")
    angles = read(Path(directory)/"cube-heading.csv")
    yaw = np.interp(t,angles["sec"],np.unwrap(angles["yaw_rad"]))
    x,y = [np.interp(t,d["sec"],d[k]) for k in ("dx_mm","dy_mm")]
    init = np.median(yaw[t<0])
    c,s = np.cos(init),np.sin(init)
    record(f"{mode} · {step} ms",t,c*x+s*y,-s*x+c*y,np.rad2deg(yaw-init))

for ax,label in zip(axes,("Larger axis component (mm)","Smaller axis component (mm)","Yaw change (°)"),strict=True):
    ax.set_ylabel(label)
axes[0].legend(fontsize=7,ncol=3)
axes[-1].set_xlabel("Seconds from pickup command")
fig.savefig(here/"approach-alignment.png",dpi=160)
with (here/"approach-alignment.csv").open("w") as f:
    w=csv.writer(f,lineterminator="\n")
    w.writerow("source sec major_mm minor_mm off_axis_deg yaw_change_deg".split())
    w.writerows(rows)
