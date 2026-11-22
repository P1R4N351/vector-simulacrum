"""Overlay R5 fork contacts on forks7.stl and score independent lower-lip cuts."""
import csv
from pathlib import Path
import sys

import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
import numpy as np

from pickup_onset import read

here = Path(__file__).resolve().parent
sys.path.insert(0,str(here.parents[1]/"unit3-cube-accelerometer"))
from fork_envelope import registered_fork

tri = registered_fork(Path(sys.argv[1]))
fig, axes = plt.subplots(1, 2, figsize=(10,4), layout="constrained")
base = read(Path(sys.argv[3])/"approach-yaw-contacts.csv")
ax = axes[0]
ax.add_collection(PolyCollection(tri[:,:,[1,2]], facecolors=".77",edgecolors="none"))
ax.autoscale()
ax.axhline(-12,color=".35",ls=":",lw=1,label="Top of candidate cuts")
for early, label in ((False,"Later approach contacts"),(True,"First 178 ms")):
    f=base[(base["sec"]<2.159)==early]
    ax.scatter(1000*f["fork_y_m"],1000*f["fork_z_m"],s=4,label=label)
ax.set(aspect="equal",xlabel="Across fork (mm)",ylabel="Above wrist origin (mm)",title="Intact STL · front view")
ax.legend(fontsize=7)
times=read(here/"approach-links.csv")
times=times[(times["step_ms"]==.125)&(times["link_condition"]=="Full contact")]
t80=read(here/"approach-glyph-axes.csv")
t80=t80["sec"][t80["frame"]==80][0]
t85,t106=[times["receipt_s_since_cmd"][times["frame"]==i].item() for i in (85,106)]
rows=[]
variants=["Intact","−y · 1 mm","−y · 3 mm","+y · 1 mm","+y · 3 mm"]
for j,folder in enumerate(sys.argv[2:]):
    idx=j//2
    path=Path(folder)
    yaw=read(path/"cube-heading.csv")
    pose=read(path/"pickup-replay.csv")
    angle=np.rad2deg(np.unwrap(yaw["yaw_rad"])-yaw["yaw_rad"][0])
    early,end=np.interp([t80,t85],yaw["sec"],angle)
    rise=np.diff(np.interp([t85,t106],pose["sec"],pose["center_height_mm"])).item()
    rows.append([(.25,.125)[j%2],variants[idx],early,end,rise])
    axes[1].scatter(idx+(j%2-.5)*.1,end,c=f"C{j%2}",label=f"{(.25,.125)[j%2]} ms" if idx==0 else None)
jitter=read(here/"approach-yaw-jitter.csv")
axes[1].axhspan(jitter["yaw_p05_deg"].item(),jitter["yaw_p95_deg"].item(),alpha=.2,color="gray",label="Filmed · corner jitter")
axes[1].axhline(-1.89,color="k",ls=":")
axes[1].set(xticks=range(5),xticklabels=variants,ylabel="Frame 85 yaw change (°)",title="One-sided lower lip removal")
axes[1].tick_params(axis="x",labelrotation=25)
axes[1].legend(fontsize=7)
fig.savefig(here/"lip-transfer.png",dpi=160)
with (here/"lip-transfer.csv").open("w") as stream:
    w=csv.writer(stream,lineterminator="\n")
    w.writerow("step_ms variant frame80_yaw_deg frame85_yaw_deg frame85_to106_rise_mm".split())
    w.writerows(rows)
