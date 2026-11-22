"""Compare wheelstand telemetry at the lift descent and show W20 camera frames."""
import csv
import gzip
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
fig = plt.figure(figsize=(12, 7), layout="constrained")
grid = fig.add_gridspec(3, 2, width_ratios=[1, 1.3])
axes = [fig.add_subplot(grid[i, 0]) for i in range(3)]
with (here/"ejection.csv").open("w") as f:
    writer = csv.writer(f, lineterminator="\n")
    writer.writerow(["run","descent_s","peak_pitch_next_400ms_deg","peak_mean_wheel_next_400ms_mmps",
                     "lowest_lift_next_400ms_mm"])
    for run in ["W4","W8","W12","W20"]:
        with gzip.open(here.parent/run/"state.csv.gz", "rt") as stream:
            d = np.genfromtxt(stream, delimiter=",", names=True)
        fields = ("pitch_rad","lift_mm","left_mmps","right_mmps") if run=="W20" else ("pitch","lift","left","right")
        p,h,l,r = [d[k] for k in fields]
        t = d["sec"]
        if run == "W20":
            with (here/"trial.csv").open() as stream:
                request = next(row for row in csv.DictReader(stream) if row["event"]=="pop_a_wheelie")
        else:
            with (here.parent/run/"actions.csv").open() as stream:
                request = next(row for row in csv.DictReader(stream) if row["action"]=="pop_a_wheelie" and row["kind"]=="begin")
        t = t - float(request["sec"])
        anchor = np.flatnonzero((t[1:] > 0) & (h[:-1]>80) & (np.diff(h)<-3))[0]+1
        elapsed = t-t[anchor]
        pitch = np.rad2deg(p)
        speed = (l+r)/2
        score = (elapsed>=0)&(elapsed<=.4)
        writer.writerow([run,t[anchor],pitch[score].max(),speed[score].max(),h[score].min()])
        for ax,values in zip(axes,[h,speed,pitch]):
            keep=(elapsed>=-.45)&(elapsed<=.65)
            ax.plot(elapsed[keep],values[keep],".-",ms=3,label=run)
for ax,label in zip(axes,["Lift (mm)","Mean wheel speed (mm/s)","Reported pitch (°)"]):
    ax.set_ylabel(label)
    ax.axvline(0,color=".55",lw=.7)
axes[0].legend(ncol=4,fontsize=8)
axes[-1].set_xlabel("Seconds from first recorded lift drop >3 mm")
cam=cv2.VideoCapture(str(here/"dock-request.mp4"))
for i,(frame,label) in enumerate([(65,"Cube upright, lift raised"),(67,"Cube rotating as lift drops"),(68,"Cube moving away")]):
    cam.set(cv2.CAP_PROP_POS_FRAMES,frame)
    ok,img=cam.read()
    assert ok
    ax=fig.add_subplot(grid[i,1])
    ax.imshow(cv2.cvtColor(img,cv2.COLOR_BGR2RGB))
    ax.set_title(f"W20 · {frame/8-3.097:.2f} s after request · {label}",fontsize=10)
    ax.axis("off")
fig.savefig(here/"ejection.png",dpi=165)
