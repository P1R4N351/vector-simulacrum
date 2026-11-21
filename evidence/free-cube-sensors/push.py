"""A one-robot simulated push, with packet timing and ground-truth cube displacement."""
import sys
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
sys.path.insert(0, str(here.parents[1]))
from simulacrum import cube, params
from simulacrum.robot import VectorSim

v = params.values(params.load(model="vector_2"))
sim = VectorSim(values=v, cube="free", cube_app="mean", noise_seed=5)
start = sim.data.body("cube").xpos.copy()
trace = []
for speed, ticks in [(0,10),(40,46),(0,10)]:
    sim.set_wheel_motors(speed,speed)
    for _ in range(ticks):
        sim.advance(.05)
        trace.append([sim.data.time, 1000*(sim.data.body("cube").xpos-start)[0]])
rows = cube.rows(sim.cube.messages)
accel = cube.samples_g(rows,v["cube_accel_counts_per_g"],app="mean")
times = np.array([row["t"] + offset for row in rows for offset in (-.02,-.01,0)])
trace = np.array(trace)
fig, axes = plt.subplots(2,1,sharex=True,figsize=(8,5))
axes[0].plot(*trace.T)
axes[0].set_ylabel("Cube travel (mm)")
axes[1].plot(times,accel[:,0],label="Forward")
axes[1].plot(times,accel[:,1],label="Left")
axes[1].set(xlabel="Simulated time (s)",ylabel="Cube reading (g)")
axes[1].legend()
fig.suptitle("Free cube pushed by Vector · simulated mean-firmware packets")
fig.tight_layout()
fig.savefig(here/"push.png",dpi=150)
print(len(rows),trace[-1,1], np.max(np.abs(accel[:,:2])))
