"""Replay the updated model at the same SDK times as the archived comparison."""
import csv
import sys
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
sys.path.insert(0, str(here.parents[2]))
from simulacrum import params
from simulacrum.robot import VectorSim

v = params.values(params.load(model="vector_2"))
data = np.genfromtxt(here/"Unit 3.csv", names=True, delimiter=",", dtype=None, encoding="utf8")
sim = VectorSim(values=v, head_deg=np.rad2deg(data["head_rad"][0]), lift_mm=data["lift_mm"][0],
                charger_contact=True, body_range_clock=True)
sim.advance(1.)
origin = sim.tick
samples = []
for sec in data["elapsed_s"]-data["elapsed_s"][0]:
    sim.step(max(0,round(sec/sim.dt)+origin-sim.tick))
    samples.append(sim.state.gyro_radps)
physical = np.column_stack([data["gyro_"+axis] for axis in "xyz"])
fig,axes = plt.subplots(1,2,figsize=(9,4),layout="constrained")
with (here/"gyro-update.csv").open("w") as f:
    w = csv.writer(f,lineterminator="\n")
    w.writerow(["source","axis","sd_dps","median_step_error_dps"])
    for i,(label,series) in enumerate([("Unit 3",physical),("Updated sim",np.array(samples))]):
        g = np.rad2deg(series[np.r_[True,np.any(np.diff(series,axis=0)!=0,axis=1)]])
        for j,axis in enumerate("XYZ"):
            step = v["gyro_range_dps"]/32768*(.989 if j==2 else 1)
            delta = np.diff(g[:,j])
            delta = delta[abs(delta)>step/2]
            err = abs(delta-step*np.rint(delta/step))
            w.writerow([label,axis,g[:,j].std(ddof=1),np.median(err)])
            axes[0].bar(j+(i-.5)*.35,g[:,j].std(ddof=1),.35,label=label if j==0 else None)
            if j==2:
                axes[1].plot(np.sort(err),np.arange(1,len(err)+1)/len(err),label=label)
axes[0].set(xticks=range(3),xticklabels=list("XYZ"),ylabel="Resting standard deviation (°/s)")
axes[1].set(xlabel="Z increment residual (°/s)",ylabel="Cumulative fraction")
for ax in axes: ax.legend()
fig.savefig(here/"gyro-update.png",dpi=150)
