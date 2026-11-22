"""Isolate geometric pushing from the existing chassis and front wheel colliders."""
import csv
import numpy as np
import matplotlib.pyplot as plt
from free_departure import here, replay, poses

fig,axes=plt.subplots(1,2,figsize=(10,4),layout="constrained",sharey=True)
with (here/"front-push.csv").open("w") as f:
    w=csv.writer(f,lineterminator="\n")
    w.writerow(["pose","added_parts","clearance_mm","step_ms","forward_at_5p4_mm","first_center_below_shelf_s"])
    for ax,name,pose in zip(axes,("Penultimate","Final"),poses[poses["run"]=="W20"][-2:]):
        for parts in ((),("body",),("wheels",),("body","wheels")):
            for clearance in (0,22):
                for dt in (.00025,.000125):
                    trace=replay(pose,clearance,dt,parts)
                    label="+".join(parts) or "fork only"
                    below=trace[trace[:,3]<0]
                    w.writerow([name,label,clearance,dt*1000,np.interp(5.4,trace[:,0],trace[:,1]),
                                below[0,0] if len(below) else ""])
                    if clearance==0 and dt==.000125:
                        ax.plot(trace[:,0],trace[:,1],label=label)
        ax.set(title=name+" cube pose",xlabel="Seconds from request",xlim=(5.1,5.85))
axes[0].set_ylabel("Forward travel of cube center (mm)")
axes[1].legend()
fig.savefig(here/"front-push.png",dpi=160)
