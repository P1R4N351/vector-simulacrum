"""Compare conditional range noise and docked acquisition timing on held-out SDK states."""
from pathlib import Path
import sys
import csv
import numpy as np
import matplotlib.pyplot as plt

here = Path(__file__).resolve().parent
sys.path.insert(0, str(here.parents[2]))
from simulacrum import params
from simulacrum.prox import ProxSensor

v = params.values(params.load(model="vector_2"))
d = np.genfromtxt(here/"Unit 3.csv", names=True, delimiter=",", dtype=None, encoding="utf8")
t = d["elapsed_s"]
idx = 1 + np.flatnonzero((np.diff(d["prox_mm"]) != 0) | (np.diff(d["prox_quality"]) != 0))
actual = d["prox_mm"][idx]
distance = np.median(actual)
sds = []
for seed in range(500):
    prox = ProxSensor(v, seed)
    hits = [distance]*len(prox.rays)
    simulated = [prox.read(hits, d["lift_mm"][0], 0.).distance_mm for _ in actual]
    sds.append(np.std(simulated, ddof=1))
fig, axes = plt.subplots(1, 2, figsize=(10, 4), layout="constrained")
axes[0].hist(sds, bins=24, color="steelblue", label="500 simulated runs")
axes[0].axvline(np.std(actual, ddof=1), color="darkorange", label="Unit 3")
axes[0].set(xlabel="Range SD over 19 acquisitions (mm)", ylabel="Simulated runs")
axes[0].legend()
n = np.arange(1, len(idx))
elapsed = t[idx[1:]]-t[idx[0]]
archive = np.genfromtxt(here.parents[1]/"unit3-cube-range/dock-clock.csv", names=True, delimiter=",", dtype=None, encoding="utf8")
with (here/"range-replay.csv").open("w") as f:
    w = csv.writer(f, lineterminator="\n")
    w.writerow(["period_s","idle_mean_absolute_error_ms","idle_last_error_ms","archive_mean_absolute_error_ms"])
    for period in (1., 1.024, 1.016):
        residual = 1000*(elapsed-n*period)
        archived = 1000*(archive["observed_elapsed_s"]-archive["cycle"]*period)
        w.writerow([period,np.mean(abs(residual)),residual[-1],np.mean(abs(archived))])
        axes[1].plot(n,residual,".-",label=f"{period:.3f} s")
axes[1].axhline(0, lw=.7, color=".5")
axes[1].set(xlabel="Updates after first changed report", ylabel="Arrival − prediction (ms)")
axes[1].legend()
fig.savefig(here/"range-replay.png", dpi=155)
print(len(actual), distance, np.std(actual, ddof=1), np.percentile(sds,[2.5,50,97.5]))
