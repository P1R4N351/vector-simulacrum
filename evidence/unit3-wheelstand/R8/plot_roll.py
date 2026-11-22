"""Evaluate a native roll attempt using cube and robot telemetry on the receiver clock."""
from pathlib import Path
import csv
import sys
import matplotlib.pyplot as plt
import numpy as np

p=Path(sys.argv[1])
with (p/"actions.csv").open() as f:
    rows=list(csv.DictReader(f))
r=np.genfromtxt(p/"state.csv.gz",names=True,delimiter=",")
c=np.genfromtxt(p/"cube.csv.gz",names=True,delimiter=",")
v=np.array([c[k] for k in "xyz"]).T/8192
fig,axes=plt.subplots(3,1,figsize=(9,6),sharex=True,layout="constrained")
for i,k in enumerate("XYZ"):
    axes[0].plot(c["sec"],v[:,i],".",ms=1.6,label=k)
axes[0].legend(ncol=3)
axes[0].set_ylabel("Cube (g)")
axes[1].plot(r["sec"],r["lift"],label="Lift (mm)")
axes[1].plot(r["sec"],np.degrees(r["pitch"]),label="Chassis pitch (°)")
axes[1].legend()
axes[2].plot(r["sec"],r["left"],label="Left")
axes[2].plot(r["sec"],r["right"],label="Right")
axes[2].legend()
axes[2].set(xlabel="Seconds since undock command",ylabel="Wheel speed (mm/s)")
for event in rows:
    if event["action"]=="roll cube":
        for ax in axes: ax.axvline(float(event["sec"]),color=".4",ls=":" if event["kind"]=="end" else "--")
axes[2].set_xlim(45.9,52.0)
for ax in axes: ax.axvline(51.286772,color="tab:red",ls=":")
axes[0].set_title("Unit 3 autonomous cube roll   |   dashed: roll command   ·   red: first cliff flag")
fig.savefig(p/"roll-traces.png",dpi=150)
