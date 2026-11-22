"""Challenge a fixed cube-frame transverse offset on upright stationary windows."""
import csv
import gzip
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
evidence = here.parents[1]
rows = []


def record(name, times, vectors):
    if not len(times):
        return
    times = np.asarray(times)
    first = np.floor(times - times[0]).astype(int)
    for second in np.unique(first):
        window = np.asarray(vectors)[first == second]
        if len(window) < 60:
            continue
        mean = window.mean(axis=0)
        rms = np.linalg.norm(window.std(axis=0))
        if rms <= 8 and mean[2] > 950 and np.linalg.norm(mean[:2]) < 100:
            rows.append((name, second, len(window), rms, *mean))
            return


for path in sorted((evidence / "unit3-wheelstand").glob("*/cube.csv.gz")):
    with gzip.open(path, "rt") as f:
        samples = list(csv.DictReader(f))
    record(path.parent.name, [float(r["sec"]) for r in samples],
           [[float(r[a])*1000/8192 for a in "xyz"] for r in samples])

with (here / "events.csv").open() as f:
    packets = [(float(r["seconds"]), np.frombuffer(bytes.fromhex(r["value"])[2:], "<i2"))
               for r in csv.DictReader(f) if r["kind"] == "accel"]
record("LED holdout", np.repeat([t for t, _ in packets], 3),
       np.stack([w for _, w in packets]).reshape(-1,3)* (1000/8192))
with (here / "resting-direction.csv").open("w") as f:
    writer = csv.writer(f, lineterminator="\n")
    writer.writerow(["run","second","samples","scatter_mg","x_mg","y_mg","z_mg"])
    writer.writerows(rows)
with (evidence / "unit3-cube-accelerometer/resting.tsv").open() as f:
    words = [np.frombuffer(bytes.fromhex(line.split()[2])[2:], "<i2")
             for line in f if line.split()[2].startswith("02") and len(line.split()[2])==40]
reference = np.stack(words).reshape(-1,3).mean(axis=0)[:2]*1000/8192
xy = np.array([row[4:6] for row in rows])
baseline = np.linalg.norm(xy, axis=1)
adjusted = np.linalg.norm(xy-reference, axis=1)
fig, axes = plt.subplots(1,2,figsize=(11,4),layout="constrained")
axes[0].scatter(*xy.T, label="Quiet window")
axes[0].scatter(*reference, marker="x", label="Earlier resting capture")
axes[0].scatter(*xy[-1], marker="*", s=100,label="LED holdout")
axes[0].set(xlabel="Cube X (mg)", ylabel="Cube Y (mg)", aspect="equal")
axes[0].legend(fontsize=8)
x = np.arange(len(rows))
axes[1].plot(x,baseline,"o",label="Level, zero offset")
axes[1].plot(x,adjusted,"x",label="Earlier resting offset")
axes[1].set(xticks=x,xticklabels=[r[0] for r in rows],ylabel="Transverse prediction error (mg)")
axes[1].tick_params(axis="x",labelrotation=70)
axes[1].legend(fontsize=8)
fig.savefig(here/"resting-direction.png", dpi=160)
print(len(rows), reference, np.median(baseline), np.median(adjusted),
      sum(adjusted<baseline), baseline[-1], adjusted[-1])
