"""Track the visible W8 cube rim and stationary background throughout chassis ascent."""
import csv
import sys

import cv2 as cv
import matplotlib.pyplot as plt
import numpy as np

from acceleration_receipts import here, replay
from early_cube_track import match

PATCHES = {"left rim": (298,341), "top rim": (322,328), "wall screw": (467,289), "dock outline": (437,279)}


def main(path):
    cap = cv.VideoCapture(path)
    frames, pts = [], []
    for i in range(112):
        ok, frame = cap.read()
        assert ok
        frames.append(frame)
        pts.append(cap.get(cv.CAP_PROP_POS_MSEC)*.001)
    cap.release()
    tr = replay.Trial("W8")
    t = np.array(pts)-pts[86]+3.065-tr.late
    ref = np.flatnonzero(t < -.13)[-1]
    use = np.flatnonzero((t >= -.24) & (t <= .75))
    rows = []
    fig, ax = plt.subplots(2,1,figsize=(8,5),layout="constrained",sharex=True)
    for ch in (1,2):
        series = {}
        for key,xy in PATCHES.items():
            samples = [match(frames[i][:,:,ch], frames[ref][:,:,ch], xy) for i in use]
            series[key] = np.array([xy for xy,_ in samples])
            rows.extend((i,t[i],key,ch,*d,q) for i,(d,q) in zip(use,samples))
        drift = (series["wall screw"]+series["dock outline"])/2
        for key in ("left rim","top rim"):
            delta = series[key]-drift
            for k in (0,1):
                ax[k].plot(t[use],delta[:,k],".-",label=f"{key}, "+("green" if ch==1 else "red"))
        print(ch,"reference disagreement",np.max(np.linalg.norm(series["wall screw"]-series["dock outline"],axis=1)))
    for k in (0,1):
        ax[k].axvline(0,color=".65",lw=1)
        ax[k].set_ylabel(("Horizontal" if k==0 else "Vertical")+" change (px)")
    ax[0].legend(fontsize=8,ncol=2)
    ax[1].set_xlabel("Seconds from last raised-lift sample")
    fig.savefig(here/"ascent-cube.png",dpi=150)
    out,panels=plt.subplots(1,3,figsize=(10,3),layout="constrained")
    for panel,sec in zip(panels,(-.13,.16,.42)):
        i=np.argmin(abs(t-sec))
        panel.imshow(cv.cvtColor(frames[i],cv.COLOR_BGR2RGB))
        for x,y in PATCHES.values():
            panel.add_patch(plt.Rectangle((x-6,y-6),12,12,fill=False,color="orange",lw=.8))
        panel.set(xlim=(283,478),ylim=(418,265),title=f"Frame {i} · {t[i]:+.3f} s")
        panel.axis("off")
    out.savefig(here/"ascent-views.png",dpi=170)
    with (here/"ascent-cube.csv").open("w") as f:
        w=csv.writer(f,lineterminator="\n")
        w.writerow("frame seconds patch channel dx_px dy_px correlation".split())
        w.writerows(rows)


if __name__=="__main__": main(sys.argv[1])
