"""Compare adjacent SDK rate increments with the stock Z-only sensitivity correction."""
import csv
import json
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
timing = here.parent
d = np.genfromtxt(here/"Unit 3.csv", names=True, delimiter=",", dtype=None, encoding="utf8")
captures = {"Unit 3 idle": np.column_stack([d["gyro_"+axis] for axis in "xyz"])}
for label, path in [("Unit 1",timing/"repeat/unit1.jsonl"),("Unit 2",timing/"unit2.jsonl"),
                    ("Unit 3",timing/"repeat/unit3.jsonl")]:
    records = [json.loads(line) for line in path.read_text().splitlines()]
    captures[label] = np.array([r["gyro"] for r in records if r.get("k")=="s" and "gyro" in r])

q = 500/32768
fig, ax = plt.subplots(figsize=(7,4),layout="constrained")
with (here/"gyro-scale.csv").open("w") as f:
    w = csv.writer(f,lineterminator="\n")
    w.writerow(["capture","axis","nonzero_increments","nominal_median_residual_dps","scaled_median_residual_dps"])
    for i,(label,data) in enumerate(captures.items()):
        for j,axis in enumerate("XYZ"):
            delta = np.diff(np.rad2deg(data[:,j]))
            delta = delta[abs(delta)>q/2]
            def residual(step):
                return np.median(abs(delta - step*np.rint(delta/step)))
            nominal, scaled = residual(q), residual(q*.989)
            w.writerow([label,axis,len(delta),nominal,scaled])
            if j==2:
                ax.bar(i-.2,nominal,.4,label="Nominal step" if i==0 else None)
                ax.bar(i+.2,scaled,.4,label="Firmware Z step" if i==0 else None)
ax.set(xticks=np.arange(len(captures)),xticklabels=list(captures),
       ylabel="Median Z increment residual (°/s)",title="Stock gyro Z sensitivity across docked recordings")
ax.legend()
fig.savefig(here/"gyro-scale.png",dpi=155)
