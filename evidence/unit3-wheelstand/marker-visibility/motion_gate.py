"""Compare stock marker motion limits with SDK gyro samples near reported image receipts."""
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
vis = np.genfromtxt(here/"visibility.csv", delimiter=",", names=True, dtype=None, encoding="utf8")
fig, axes = plt.subplots(1,3,figsize=(12,3.8),layout="constrained")
with (here/"motion-gate.csv").open("w") as output:
    writer = csv.writer(output,lineterminator="\n")
    writer.writerow(["trial","sec","image_ms","reported","visible","sdk_y_deg_s","sdk_z_deg_s","over_limit","all_shifts_over","any_shift_over"])
    for ax,trial in zip(axes,("W4","W8","W12")):
        path=here.parent/trial
        state=np.genfromtxt(path/"state.csv.gz",delimiter=",",names=True)
        with (path/"actions.csv").open() as f:
            zero=next(float(row["sec"]) for row in csv.DictReader(f) if row["action"]=="pop_a_wheelie" and row["kind"]=="begin")
        time=state["sec"]-zero
        rates=np.abs(np.rad2deg(np.column_stack([state["gy"],state["gz"]])))
        mask=(time>=-.65)&(time<=5)
        ax.plot(time[mask],rates[mask,0],label="Head pitch |Y|")
        ax.plot(time[mask],rates[mask,1],label="Body yaw |Z|")
        for limit,color in [(10,"C0"),(30,"C1")]:
            ax.axhline(limit,ls="--",color=color,alpha=.6)
        for row in vis[(vis["trial"]==trial)&(vis["size_mm"]==23)]:
            t=row["sec"]
            # Bracket reception using successive SDK states. Offset probes are
            # sensitivity cases; exposure-to-state latency is unknown.
            indices=np.searchsorted(time,t+np.array([-.12,-.06,0,.06,.12]))
            indices=np.clip(indices,1,len(time)-1)
            peaks=np.maximum(rates[indices],rates[indices-1])
            gate=(peaks>np.array([10,30])).any(axis=1)
            if row["seen"]:
                ax.scatter(t,peaks[2,0],color="black",s=18,zorder=3)
            writer.writerow([trial,t,row["stamp"],row["seen"],row["visible_fraction"],*peaks[2],int(gate[2]),int(all(gate)),int(any(gate))])
        ax.set(title=trial,xlabel="Seconds from wheelstand request",yscale="symlog",ylim=(0,350))
axes[0].set_ylabel("Absolute SDK gyro rate (°/s)")
axes[0].legend(fontsize=8)
fig.savefig(here/"motion-gate.png",dpi=170)
