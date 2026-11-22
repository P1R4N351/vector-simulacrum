"""Compare simulated and measured SDK lift-in-position flags on both physical sweeps."""
import csv
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np
from simulacrum import params
from simulacrum.robot import VectorSim

here = Path(__file__).resolve().parent
source = here.parent / "unit3-docked-fork-sweep"
state = np.genfromtxt(source/"state.csv.gz",names=True,delimiter=",")
moves = np.genfromtxt(source/"holds.csv",names=True,delimiter=",")
t = state["seconds"]
actual = (state["status"].astype(int) & 256) != 0
sim=VectorSim(params.values(params.load(model="vector_2")),lift_mm=32,
              head_deg=np.degrees(state["head_rad"][0]),charger_contact=True)
bits=[sim.state.lift_in_pos]
i=0
while (len(bits)-1)*sim.dt <= t[-1]:
    if i<len(moves) and (len(bits)-1)*sim.dt >= moves[i]["command_s"]:
        assert sim.command_lift_height(32+60*moves[i]["command_fraction"],duration_s=.7)
        i+=1
    bits.append(sim.step().lift_in_pos)
estimated=np.array(bits)[np.maximum(0,np.floor(t/sim.dt).astype(int))]
fig,axes=plt.subplots(2,1,figsize=(10,4),layout="constrained")
with (here/"status-transfer.csv").open("w") as stream:
    w=csv.writer(stream)
    w.writerow(["sweep","states","agreement_fraction","always_set_fraction","physical_clear_recall","physical_set_recall"])
    for cycle,ax in enumerate(axes):
        start=moves[1+8*cycle]["command_s"]
        stop=moves[9]["command_s"] if cycle==0 else t[-1]+.001
        mask=(t>=start)&(t<stop)
        observed=actual[mask]; predicted=estimated[mask]
        metrics=[int(mask.sum()),np.mean(predicted==observed),np.mean(observed),np.mean(~predicted[~observed]),np.mean(predicted[observed])]
        w.writerow([cycle+1,*metrics])
        print(cycle+1,metrics)
        ax.step(t[mask]-start,observed.astype(int),where="post",label="Unit 3")
        ax.step(t[mask]-start,predicted.astype(int)+.06,where="post",label="Simulation")
        ax.set(yticks=[0,1],yticklabels=["Clear","Set"],title=f"Sweep {cycle+1}")
axes[0].legend();axes[1].set_xlabel("Seconds since first raise")
fig.savefig(here/"status-transfer.png",dpi=160)

# Independent contact failure: nominal simulation receives the command, never the recorded outcome.
failed=np.genfromtxt(here/"dock-status.csv",names=True,delimiter=",")
nominal=VectorSim(params.values(params.load(model="vector_2")),lift_mm=33.02,
                  head_deg=21.5,charger_contact=True)
assert nominal.command_lift_height(47,duration_s=1)
bits=[nominal.state.lift_in_pos]
while (len(bits)-1)*nominal.dt < failed["seconds"][-1]:
    bits.append(nominal.step().lift_in_pos)
ft=failed["seconds"]
for name,v in [("physical",(failed["status"].astype(int)&256)!=0),
               ("simulated",np.array(bits)[np.maximum(0,np.floor(ft/nominal.dt).astype(int))])]:
    print(name,"SDK sample times at flag transitions",ft[1:][v[1:]!=v[:-1]])
