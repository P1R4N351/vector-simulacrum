"""Measure the fork's apparent vertical translation between paired 75% holds."""
import csv
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np

folder = Path(__file__).resolve().parent
with (folder / "holds.csv").open() as stream:
    holds = list(csv.DictReader(stream))
capture = cv2.VideoCapture(str(folder / "physical-sweeps.mp4"))
fps = capture.get(cv2.CAP_PROP_FPS)
def frame(t):
    capture.set(cv2.CAP_PROP_POS_FRAMES, round(t * fps))
    ok, im = capture.read()
    assert ok
    return cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)

def locate(template, image, box):
    x,y,w,h = box
    margin = 14
    area = image[y-margin:y+h+margin,x-margin:x+w+margin]
    scores = cv2.matchTemplate(area, template, cv2.TM_CCOEFF_NORMED)
    _,score,_,(px,py) = cv2.minMaxLoc(scores)
    assert 0 < py < 2*margin and 0 < px < 2*margin
    a,b,c = scores[py-1:py+2,px]
    sub = .5*(a-c)/(a-2*b+c) if a-2*b+c else 0
    return py+sub-margin, score

boxes = {"A": [(250,121,72,76),(179,278,56,49)], "B": [(618,52,73,78),(706,213,51,44)]}
rows = []
fig, axes = plt.subplots(2,2,figsize=(9,6),constrained_layout=True)
for cycle,(first,second) in enumerate([(3,5),(11,13)],1):
    t = float(holds[first]["success_s"])
    ref = frame(t+.35)
    for col,(view,(fork,hub)) in enumerate(boxes.items()):
        def template(box):
            x,y,w,h = box
            return ref[y:y+h,x:x+w]
        measured = []
        for index in (first,second):
            values = []
            for offset in (.15,.25,.35,.45,.55):
                im = frame(float(holds[index]["success_s"])+offset)
                fy,fc = locate(template(fork),im,fork)
                hy,hc = locate(template(hub),im,hub)
                values.append(fy-hy)
                rows.append([cycle,view,index,offset,fy,hy,fy-hy,fc,hc])
            measured.append(np.median(values))
        ax = axes[cycle-1,col]
        x,y,w,h = fork
        crop = np.s_[y-12:y+h+12,x-12:x+w+12]
        a,b = ref[crop],frame(float(holds[second]["success_s"])+.35)[crop]
        ax.imshow(np.dstack([a,b,b]))
        ax.set(title=f"{view}, sweep {cycle}   {measured[1]-measured[0]:+.2f} px down")
        ax.axis("off")
fig.suptitle("Fork at 75% after raising (red) and lowering (cyan)")
fig.savefig(folder/"fork-hold-overlay.png",dpi=160)
capture.release()
with (folder/"fork-hold-optical.csv").open("w") as stream:
    writer=csv.writer(stream)
    writer.writerow(["sweep","view","hold","after_success_s","fork_dy_px","hub_dy_px","relative_dy_px","fork_score","hub_score"])
    writer.writerows(rows)
for cycle in (1,2):
    for view in boxes:
        a = np.array([r[6:] for r in rows if r[0]==cycle and r[1]==view]).reshape(2,5,3)
        print(cycle,view,"shift",np.diff(np.median(a[:,:,0],axis=1)), "within-hold spans",np.ptp(a[:,:,0],axis=1),"scores",a[:,:,1:].min())
