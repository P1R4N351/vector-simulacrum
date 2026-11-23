"""Check the passive-hold candidate on two measured powered staircases, without refitting."""
import csv
from pathlib import Path
from unittest.mock import patch

import matplotlib.pyplot as plt
import numpy as np
from simulacrum import mjcf, params
from simulacrum.robot import VectorSim

here = Path(__file__).resolve().parent
source = here.parent / "unit3-docked-fork-sweep"
state = np.genfromtxt(source / "state.csv.gz", names=True, delimiter=",")
moves = np.genfromtxt(source / "holds.csv", names=True, delimiter=",")
x = state["seconds"]
fig, axes = plt.subplots(2,1,sharex=True,figsize=(10,5),layout="constrained")
axes[0].plot(x,state["lift_mm"],color="black",label="Physical encoder")
rows = []
for friction in (0,6):
    original = mjcf.build_mjcf
    def build(*args):
        xml = original(*args)
        name = '<joint name="lift_joint"'
        assert xml.count(name) == 1
        return xml.replace(name, name + ' frictionloss=".006" solreffriction=".005 1" solimpfriction=".99 .999 .001"') if friction else xml
    with patch.object(mjcf,"build_mjcf",build):
        sim = VectorSim(params.values(params.load(model="vector_2")),lift_mm=32,
                        head_deg=np.degrees(state["head_rad"][0]),charger_contact=True)
    ys = []
    i = 0
    while len(ys)*sim.dt <= x[-1]:
        if i < len(moves) and len(ys)*sim.dt >= moves[i]["command_s"]:
            assert sim.command_lift_height(32+60*moves[i]["command_fraction"],duration_s=.7)
            i += 1
        ys.append(sim.step().lift_height_mm)
    predicted = np.interp(x,sim.dt*np.arange(1,len(ys)+1),ys)
    error = predicted-state["lift_mm"]
    label = "Frozen" if friction==0 else "6 mN·m"
    axes[0].plot(x,predicted,label=label)
    axes[1].plot(x,error,label=label)
    for j,move in enumerate(moves[1:],1):
        stop = moves[j+1]["command_s"] if j+1<len(moves) else x[-1]
        span = (x>=move["command_s"]) & (x<stop)
        hold = (x>=move["success_s"]+.12) & (x<=move["success_s"]+.6)
        rows.append([friction,1+(j-1)//8,j,move["command_fraction"],
                     np.sqrt(np.mean(error[span]**2)),np.median(predicted[hold])-move["lift_mm"]])
for ax in axes: ax.legend()
axes[0].set_ylabel("Lift height (mm)")
axes[1].set(xlabel="Seconds from initial command",ylabel="Sim minus encoder (mm)")
fig.savefig(here/"powered-transfer.png",dpi=160)
with (here/"powered-transfer.csv").open("w") as stream:
    writer=csv.writer(stream)
    writer.writerow(["friction_mNm","cycle","move","target","move_rmse_mm","hold_error_mm"])
    writer.writerows(rows)
for friction in (0,6):
    for cycle in (1,2):
        values=np.array([r[-2:] for r in rows if r[:2]==[friction,cycle]])
        print(friction,cycle,np.abs(values).mean(axis=0))
