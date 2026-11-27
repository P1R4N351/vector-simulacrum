"""Estimate net displacement across the unfilmed interval from the two archived camera tracks."""
import csv
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

root=Path("../unit3-w34-return")
with (root/"dense-hubs.csv").open() as source:
    last=[r for r in csv.DictReader(source) if r["direction"]=="backward"][-1]
with (root/"departure-hubs.csv").open() as source:
    first=next(csv.DictReader(source))
scale=48/(float(first["rear_x"])-float(first["front_x"]))
t0=-59.702+float(last["t_s"])
t1=float(first["t_s"])
values={}
for hub in ["front","rear"]:
    corrected_first=float(first[hub+"_x"])-float(first["static_x"])
    corrected_last=float(last[hub+"_x"])-float(last["dock_x"])
    values[hub+" hub"]=(corrected_last-corrected_first)*scale
for mode in ["full","shaft","shaft-linear","noslip","shaft-noslip"]:
    r=np.genfromtxt(mode+".csv",names=True,delimiter=",")
    values[mode]=np.diff(np.interp([t0,t1],r["s"],r["x_mm"]))[0]
with open("endpoints.csv","w") as f:
    w=csv.writer(f,lineterminator="\n")
    w.writerow(["source","start_s","end_s","outward_mm"])
    for label,x in values.items():
        w.writerow([label,t0,t1,x])
fig, ax=plt.subplots(figsize=(8,3.8),layout="constrained")
ax.barh(list(values),list(values.values()),color=["#444444"]*2+["#4376a5"]*5)
ax.axvline(0,color="black",lw=.8)
ax.invert_yaxis()
ax.set(xlabel="Net outward travel between camera endpoints (mm)", title="W34 settled frame to W35 predeparture frame")
fig.savefig("endpoints.png",dpi=150)
