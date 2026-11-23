"""Ablate lift servo holding torque at the recorded docked in-position transition."""
import csv
import json
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from simulacrum import params
from simulacrum.robot import VectorSim

here = Path(__file__).resolve().parent
raw = [json.loads(s) for s in Path(sys.argv[1]).read_text().splitlines() if s.startswith("{")]
t0 = next(r["t"] for r in raw if r.get("action") == "lift 0.25")
state = [(r["t"]-t0,r["data"]["lift_height_mm"],int(r["data"]["status"])) for r in raw
         if r["kind"] == "robot_state" and -.3 < r["t"]-t0 < 2.45]
edges = [b for a,b in zip(state,state[1:]) if (a[2]^b[2]) & 256]
restore = next(r for r in edges if r[2] & 256)
idx = state.index(restore)
lo,hi = state[idx-1][0], restore[0]
with (here/"dock-status.csv").open("w") as f:
    w=csv.writer(f); w.writerow(["seconds","encoder_mm","status"]);w.writerows(state)
fig, ax = plt.subplots(figsize=(8,4),layout="constrained")
ax.plot([r[0] for r in state],[r[1] for r in state],".-",label="Physical cube contact",color="black")
for label,cut in [("Powered hold",float("inf")),("Early cutoff",lo),("Late cutoff",hi)]:
    sim = VectorSim(params.values(params.load(model="vector_2")),head_deg=21.5,lift_mm=33.02,charger_contact=True)
    assert sim.command_lift_height(47,duration_s=1)
    aid = sim.model.actuator("lift").id
    trace=[]
    for tick in range(round(2.45/sim.dt)):
        if tick*sim.dt>=cut:
            sim.model.actuator_gainprm[aid,:]=0
            sim.model.actuator_biasprm[aid,:]=0
        trace.append(((tick+1)*sim.dt,sim.step().lift_height_mm))
    x,y=np.array(trace).T
    ax.plot(x,y,label=label)
    print(label,"final",y[-1],"first below 34",next((t for t,h in trace if t>1.2 and h<34),None))
ax.axvspan(lo,hi,alpha=.12,color="gray",label="SDK lift-in-position bit returned")
ax.set(xlabel="Seconds after lift request",ylabel="Lift encoder (mm)")
ax.legend();fig.savefig(here/"dock-power.png",dpi=160)
print("status bracket",lo,hi)
