"""Align a physical camera recording with the first SDK state carrying each image stamp."""
import csv
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

here=Path(__file__).resolve().parent
records=list(csv.DictReader((here/"docked-clocks.csv").open()))
def first(kind):
    times={}
    for row in records:
        if row["kind"]==kind:
            times.setdefault(int(row["stamp"]),float(row["receipt_s"]))
    return times
cam,state,objects=map(first,("camera","state","object"))
stamps=sorted(cam.keys() & state.keys())
t=np.array([cam[k] for k in stamps])
lag=np.array([1000*(state[k]-cam[k]) for k in stamps])
print("camera/state matches",len(stamps),"object matches",len(objects.keys() & cam.keys()),
      "first-state minus JPEG ms (min/median/max)",np.round(np.percentile(lag,[0,50,100]),2))
fig,ax=plt.subplots(figsize=(8,3.4),layout="constrained")
ax.axhline(0,color="gray",lw=.8)
ax.scatter(t,lag,label="Camera frame and state share stamp")
subset=[i for i,k in enumerate(stamps) if k in objects]
ax.scatter(t[subset],lag[subset],facecolors="none",edgecolors="darkorange",s=95,label="Also has object report")
ax.set(xlabel="Camera packet receipt (seconds into capture)",ylabel="First state minus JPEG receipt (ms)")
ax.legend(fontsize=8)
fig.savefig(here/"docked-clocks.png",dpi=170)
