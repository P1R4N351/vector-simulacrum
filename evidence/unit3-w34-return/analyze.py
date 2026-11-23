"""Run in evidence/unit3-w34-return with the repository web dependencies installed."""
import csv
import gzip
import io
import json
import subprocess
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
runner = here.parent / "unit3-cube-range/recorded_mount.mjs"
with gzip.open(here / "states.csv.gz", "rt") as f:
    rows = list(csv.DictReader(f))
def field(name):
    return np.array([float(r[name]) for r in rows])

on = (field("status").astype(int) & 4096) != 0
contact = np.flatnonzero(on)[0]
t = field("receipt_s") - field("receipt_s")[contact]
wheel = np.column_stack([field("left_mmps"), field("right_mmps")])
start = np.flatnonzero((t < -1) & (t > -6) & (abs(wheel).max(axis=1) == 0))[-1]
end = np.searchsorted(t, 1.51)
sel = slice(start, end)
xy = np.column_stack([field("x_mm"), field("y_mm")])
heading = field("yaw_rad")[start]
travel = -(xy - xy[start]) @ np.array([np.cos(heading), np.sin(heading)])
cases = []
for depth in (72.3, 84, 95.7):
    cases.append(dict(name=str(depth), out=depth, lat=0, yaw=0,
                      head=np.rad2deg(field("head_rad")[start]), lift=field("lift_mm")[start],
                      wheels=np.column_stack([t[sel], wheel[sel]]).tolist(), duration_s=t[end-1]-t[start]))
xml = runner.parents[2] / "web/public/robot/vector2/vector-free-dock.xml"
output = subprocess.check_output(["node", str(runner), str(xml)], input=json.dumps(cases), text=True)
sim = np.genfromtxt(io.StringIO(output), names=True, delimiter=",", dtype=None, encoding="utf8")
fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True, layout="constrained")
axes[0].plot(t[sel], travel[sel], color="black", label="SDK position along initial heading")
for c in cases:
    run = sim[sim["session"] == float(c["name"])]
    ds = run["out_mm"][0] - run["out_mm"]
    axes[0].plot(run["t_s"], ds, lw=1.3, label=f"Canonical, start {c['out']} mm")
    print("depth, travel at physical flag, any sim contact",c["out"],np.interp(0,run["t_s"],ds),run["on"].any())
photo = np.genfromtxt(here/"hubs.csv", names=True, delimiter=",")
mid = (photo["front_x"] + photo["rear_x"]) / 2
scale = 48 / np.median(photo["rear_x"]-photo["front_x"])
axes[0].scatter(photo["t_s"],(mid-mid[0])*scale,marker="D",color="tab:red",s=28,label="Side camera, axle midpoint (approx.)",zorder=5)
axes[1].plot(t[sel],wheel[sel,0],label="Left"); axes[1].plot(t[sel],wheel[sel,1],label="Right")
axes[2].plot(t[sel],np.rad2deg(field("pitch_rad")[sel]),label="SDK body pitch",color="tab:purple")
for ax,unit in zip(axes,["Reverse travel (mm)","Wheel speed (mm/s)","Pitch (degrees)"]):
    ax.axvline(0,ls=":",color="black"); ax.grid(alpha=.2); ax.set_ylabel(unit); ax.legend(fontsize=8,loc="best")
axes[-1].set_xlabel("Seconds from first SDK on-charger report")
axes[0].set_title("W34 physical return to dock • independent side view and canonical replay")
fig.savefig(here/"return.png",dpi=160)
print("physical travel last off and first on; camera travel",travel[contact-1:contact+1],(mid-mid[0])*scale)

with gzip.open(here/"continuation.csv.gz", "rt") as f:
    rows = list(csv.DictReader(f))
off = np.flatnonzero((field("status").astype(int) & 4096) == 0)[0]
t = field("receipt_s")-field("receipt_s")[off]
window = (t>=-1.8)&(t<.55)
wheels = np.column_stack([t,field("left_mmps"),field("right_mmps")])[window]
first = np.flatnonzero(window)[0]
args = dict(head=np.rad2deg(field("head_rad")[first]),lift=field("lift_mm")[first],wheels=wheels.tolist())
out = subprocess.check_output(["node",str(here/"depart.mjs")],input=json.dumps(args),text=True)
sim = np.genfromtxt(io.StringIO(out),names=True,delimiter=",")
np.savetxt(here/"departure-replay.csv",np.column_stack([sim[k] for k in sim.dtype.names]),delimiter=",",header=",".join(sim.dtype.names),comments="",fmt="%.5f")
h = np.genfromtxt(here/"departure-hubs.csv",names=True,delimiter=",")
px_mm = 48/(h["rear_x"][0]-h["front_x"][0])
fig,ax=plt.subplots(figsize=(8.4,4.7),layout="constrained")
for key,label in [("front_x","Front hub"),("rear_x","Rear hub")]:
    dist=(h[key][0]-h[key]+h["static_x"]-h["static_x"][0])*px_mm
    ax.plot(h["t_s"],dist,".-",label=f"Camera • {label}")
    print(label,"last-on / first-off mm",np.interp(t[[off-1,off]],h["t_s"],dist))
angle=field("yaw_rad")[first]
travel=(field("x_mm")-field("x_mm")[first])*np.cos(angle)+(field("y_mm")-field("y_mm")[first])*np.sin(angle)
ax.plot(t[window],travel[window],"k.-",label="SDK position")
ax.plot(sim["t_s"],sim["travel_mm"],label="Canonical • reported speeds as targets")
release=np.flatnonzero(sim["on"]==0)[0]
print("sim release",sim[release])
ax.scatter(sim["t_s"][release],sim["travel_mm"][release],marker="*",s=120,label="Sim first off")
ax.axvspan(t[off-1],0,color="gray",alpha=.22,label="SDK release bracket")
ax.set(xlim=(-.49,.49),xlabel="Seconds from first SDK off-charger report",ylabel="Forward travel (mm)",title="W35 departure • immediately following the W34 return")
ax.legend(fontsize=8);ax.grid(alpha=.2)
fig.savefig(here/"departure.png",dpi=160)
