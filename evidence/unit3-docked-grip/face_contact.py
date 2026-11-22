"""Replay the physical command with a free, face-square cube just ahead of the fork."""
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import mujoco
import numpy as np

from simulacrum import params
from simulacrum.robot import VectorSim

here = Path(__file__).resolve().parent
physical = np.genfromtxt(here / "encoder.csv", delimiter=",", names=True)
rows = []
fig, axes = plt.subplots(2, 1, sharex=True, figsize=(8,5), layout="constrained")
axes[0].plot(physical["seconds"], physical["lift_mm"], color="black", label="Unit 3 encoder")
for gap in (0, 1, 2):
    sim = VectorSim(params.values(params.load(model="vector_2")), lift_mm=33.02,
                    head_deg=21.5, charger_contact=True, cube="free")
    fork = [sim.model.geom(name).id for name in ("fork_side_left","fork_side_right","lift_fork_geom")]
    # World X extent of each oriented box. Face and shelf are initially axis-aligned.
    front = max(sim.data.geom_xpos[g,0] + abs(sim.data.geom_xmat[g].reshape(3,3)[0]) @ sim.model.geom_size[g] for g in fork)
    root = sim.data.joint("cube_root")
    root.qpos[:3] = [front+(sim.v["cube_size_mm"]/2+gap)/1000,0,sim.v["cube_size_mm"]/2000]
    root.qpos[3:] = [1,0,0,0]
    root.qvel[:] = 0
    mujoco.mj_forward(sim.model,sim.data)
    sim.step(round(.5/sim.dt))
    base = sim.data.body("cube").xpos[2]
    assert sim.command_lift_height(47,duration_s=1)
    series = []
    for tick in range(round(2.45/sim.dt)):
        state = sim.step()
        series.append(((tick+1)*sim.dt,state.lift_height_mm,1000*(sim.data.body("cube").xpos[2]-base)))
    s = np.array(series)
    axes[0].plot(s[:,0], s[:,1],label=f"{gap} mm gap")
    axes[1].plot(s[:,0], s[:,2],label=f"{gap} mm gap")
    rows.extend((gap,*row) for row in series)
    print(gap, s[:,1].max(),s[-1,1],s[:,2].max(),s[-1,2])
axes[0].legend(ncol=2);axes[0].set_ylabel("Lift encoder (mm)")
axes[1].set(xlabel="Seconds from lift command",ylabel="Simulated cube\ncenter rise (mm)")
fig.savefig(here/"face-replay.png",dpi=160)
with (here/"face-replay.csv").open("w") as stream:
    writer=csv.writer(stream);writer.writerow(["gap_mm","seconds","lift_mm","cube_center_rise_mm"]);writer.writerows(rows)
