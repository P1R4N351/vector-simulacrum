"""Sweep unpowered lift friction and equivalent downward loads on the level fork."""
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from unittest.mock import patch
from simulacrum import mjcf, params
from simulacrum.robot import VectorSim

here = Path(__file__).resolve().parent
values = params.values(params.load(model="vector_2"))
def release(height, friction, load, solver="stiff"):
    constraint = 'solreffriction=".005 1" solimpfriction=".99 .999 .001"' if solver == "stiff" else ""
    build = mjcf.build_mjcf
    def with_friction(*args):
        xml = build(*args)
        joint = '<joint name="lift_joint"'
        assert xml.count(joint) == 1
        return xml.replace(joint, f'{joint} {constraint} frictionloss="{friction * .001}"')
    with patch.object(mjcf, "build_mjcf", with_friction):
        sim = VectorSim(values, lift_mm=height, head_deg=21.5, charger_contact=True)
    actuator = sim.model.actuator("lift").id
    sim.model.actuator_gainprm[actuator] = 0
    sim.model.actuator_biasprm[actuator] = 0
    dof = int(sim.model.joint("lift_joint").dofadr[0])
    assert np.isclose(sim.model.dof_frictionloss[dof], friction * .001)
    body = sim.model.body("lift_fork").id
    sim.data.xfrc_applied[body,2] = -load * values["cube_mass_g"] * .001 * 9.81
    start = sim.lift_height_mm()
    trace = [sim.step().lift_height_mm for _ in range(round(.6/sim.dt))]
    crossings = np.flatnonzero(np.array(trace) < 34)
    return start, trace[-1], (crossings[0]+1)*sim.dt if len(crossings) else np.nan

frictions = np.arange(0,32,2)
loads = [0,.25,.5,1.]
rows=[]
for load in loads:
    for friction in frictions:
        rows.append((45.42, friction,load,*release(45.42,friction,load)))
# First grid value that keeps the empty fork within 1 mm at the recorded peak.
candidate = next(r[1] for r in rows if r[2]==0 and abs(r[4]-r[3])<=1)
for h in [62,77,89.5]:
    rows.append((h,candidate,0,*release(h,candidate,0)))
with (here/"passive-load.csv").open("w") as stream:
    writer=csv.writer(stream)
    writer.writerow(["initial_target_mm","friction_mNm","weight_fraction","start_mm","end_mm","below34_s"])
    writer.writerows(rows)
fig,axes=plt.subplots(1,2,figsize=(10,4),layout="constrained")
for load in loads:
    block=np.array([r for r in rows if r[0]==45.42 and r[2]==load])
    axes[0].plot(block[:,1],block[:,3]-block[:,4],".-",label=f"{load:g} cube weight")
    axes[1].plot(block[:,1],1000*block[:,5],".-")
for ax in axes: ax.set_xlabel("Passive lift friction (mN·m)")
axes[0].set_ylabel("Encoder drop after 0.6 s (mm)")
axes[1].set_ylabel("Time to cross 34 mm (ms)")
axes[0].legend()
fig.savefig(here/"passive-load.png",dpi=160)
print(candidate, [r for r in rows if r[1]==candidate])

with (here/"passive-sensitivity.csv").open("w") as stream:
    w=csv.writer(stream)
    w.writerow(["solver","friction_mNm","start_mm","end_mm"])
    for solver in ["default","stiff"]:
        for friction in [4,6,10,30]:
            start,end,_=release(45.42,friction,0,solver)
            w.writerow([solver,friction,start,end])
