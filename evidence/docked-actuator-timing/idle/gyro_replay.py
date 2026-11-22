"""Check SDK gyro noise and the putative 16-bit gyro output step at rest."""
import csv
import json
import sys
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
sys.path.insert(0, str(here.parents[2]))
from simulacrum import params
from simulacrum.robot import VectorSim

v = params.values(params.load(model="vector_2"))
d = np.genfromtxt(here/"Unit 3.csv", names=True, delimiter=",", dtype=None, encoding="utf8")
t = d["elapsed_s"] - d["elapsed_s"][0]
real = np.rad2deg(np.column_stack([d["gyro_"+axis] for axis in "xyz"]))
archived = here.parents[1]/"hw-rec-20261115/rest-on-charger.jsonl"
prior = np.rad2deg([json.loads(line)["gyro"] for line in archived.read_text().splitlines()])
sim = VectorSim(values=v, head_deg=np.rad2deg(d["head_rad"][0]), lift_mm=d["lift_mm"][0],
                charger_contact=True, body_range_clock=True)
sim.advance(1.)
# Sample published states at distinct ticks nearest the observed receipt times.
values = []
origin = sim.tick
for seconds in t:
    sim.step(max(0, round(seconds/sim.dt)+origin-sim.tick))
    values.append(sim.state.gyro_radps)
model = np.rad2deg(values)
quantum = v["gyro_range_dps"]/32768
fig, axes = plt.subplots(1, 2, figsize=(10, 4), layout="constrained")
with (here/"gyro-replay.csv").open("w") as f:
    writer = csv.writer(f, lineterminator="\n")
    writer.writerow(["source","changed_vectors","axis","mean_dps","sd_dps","median_step_residual_dps","median_absolute_grid_residual_dps"])
    for i,(name, samples) in enumerate([("Unit 3",real),("Earlier dock",prior),("Frozen sim",model),
                                        ("Rounded sim",quantum*np.rint(model/quantum))]):
        fresh = samples[np.r_[True, np.any(np.diff(samples,axis=0)!=0,axis=1)]]
        delta = np.diff(fresh,axis=0)
        residual = abs(delta-quantum*np.rint(delta/quantum))
        sd = fresh.std(axis=0,ddof=1)
        for j,axis in enumerate("XYZ"):
            writer.writerow([name,len(fresh),axis,fresh.mean(axis=0)[j],sd[j],np.median(residual[:,j]),
                             np.median(abs(fresh[:,j]-quantum*np.rint(fresh[:,j]/quantum)))])
        axes[0].bar(np.arange(3)+(i-1.5)*.2,sd,.2,label=name)
        axes[1].plot(np.sort(residual.ravel()),np.arange(1,residual.size+1)/residual.size,label=name)
axes[0].set(xticks=np.arange(3),xticklabels=list("XYZ"),ylabel="Resting gyro SD (°/s)")
axes[0].legend()
axes[1].set(xlabel="Distance to nearest 0.01526 °/s step",ylabel="Cumulative fraction of axis differences")
fig.savefig(here/"gyro-replay.png",dpi=155)
