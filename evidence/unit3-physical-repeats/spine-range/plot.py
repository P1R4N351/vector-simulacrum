from pathlib import Path
import csv
import gzip
import matplotlib.pyplot as plt
import numpy as np
here=Path(__file__).resolve().parent
rows=list(csv.DictReader((here/"scores.csv").open()))
fig,(ax,bx)=plt.subplots(1,2,figsize=(9.5,4),layout="constrained")
for r in rows:
    with gzip.open(here.parent/r["run"]/"range.csv.gz","rt") as f:
        d=list(csv.DictReader(f))
    q=np.array([float(x["signal_quality"]) for x in d])
    ax.plot(np.linspace(0,100,len(q)),np.sort(q),label=r["run"])
ax.axhline(65535/(90*128),color="k",ls=":",label="Former bridge limit")
ax.axhline(.01,color="gray",ls="--",label="Detection threshold")
ax.set(yscale="log",xlabel="Percentile within capture",ylabel="Physical SDK signal quality")
ax.legend(fontsize=7,ncol=2)
bx.bar([r["run"] for r in rows],[float(r["max_new_error"]) for r in rows],color="tab:blue")
bx.set(xlabel="Physical capture",ylabel="Maximum decoded signal-quality error")
bx.ticklabel_format(style="sci",axis="y",scilimits=(0,0))
fig.savefig(here/"quality.png",dpi=155)
