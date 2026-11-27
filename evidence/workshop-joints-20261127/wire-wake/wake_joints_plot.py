"""Compare reported joints on the original SDK receipt clock."""
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
with (here/"wake-joints.csv").open() as f:
    rows = list(csv.DictReader(f))
names = list(dict.fromkeys(r["run"] for r in rows))
fig, axes = plt.subplots(3,1,figsize=(9,7),sharex=True,layout="constrained")
metrics = []
for i,name in enumerate(names):
    group = [r for r in rows if r["run"]==name]
    def series(field):
        return np.array([float(r[field]) for r in group])
    t = series("s")
    real, sim = series("real_lift_mm"), series("sim_lift_mm")
    wheels = np.array([series(k) for k in ("real_left_mmps","real_right_mmps")])
    simulated = np.array([series(k) for k in ("sim_left_mmps","sim_right_mmps")])
    assert min(series("real_contact")) == min(series("sim_contact")) == 1
    metrics.append([name,len(t),real.min(),real.max(),np.sqrt(np.mean((sim-real)**2)),
                    np.max(np.abs(wheels)),np.max(np.abs(simulated)),np.count_nonzero(np.any(wheels,axis=0))])
    for ax,values in zip(axes,(real,sim, wheels[0])):
        ax.plot(t,values,lw=1,alpha=.85,label=name)
axes[0].set_ylabel("Real lift (mm)")
axes[1].set_ylabel("Sim lift (mm)")
axes[2].set_ylabel("Real left wheel (mm/s)")
axes[2].set_xlabel("Seconds after SDK control grant")
axes[0].legend(ncol=4,fontsize=8)
for ax in axes:
    ax.grid(alpha=.2)
    ax.set_xlim(0,8)
for ax in axes[:2]:
    ax.set_ylim(30,33)
fig.savefig(here/"wake-joints.png",dpi=160)
with (here/"wake-joint-scores.csv").open("w") as f:
    w = csv.writer(f,lineterminator="\n")
    w.writerow(["run","n","real_min_mm","real_max_mm","lift_rms_mm","real_max_abs_wheel_mmps",
                "sim_max_abs_wheel_mmps","nonzero_physical_wheel_reports"])
    w.writerows(metrics)
