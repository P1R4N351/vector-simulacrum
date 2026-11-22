"""Track the two printed wheel rims in each corrected view around charger dropout."""
import csv
import gzip
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np

HERE = Path(__file__).resolve().parent
with gzip.open(HERE.parent/"unit3-wheelstand/R11/state.csv.gz","rt") as f:
    state = list(csv.DictReader(f))
def col(key):
    return np.array([float(q[key]) for q in state])
t = col("sec")
contact = col("status").astype(int) & 0x1000 != 0
cap = cv2.VideoCapture(str(HERE/"r11-undock.mp4"))
frames = []
while True:
    ok, frame = cap.read()
    if not ok: break
    frames.append(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
cap.release()
vt = np.arange(len(frames)) / 15
ref = frames[45]  # 3 s
# The two hub centers and a fixed section of the charger base in each view.
marks = [[(142,183,15),(74,182,9),(159,218,8)],[(1067,145,14),(1139,138,9),(1165,172,10)]]
def displacements(mark):
    x,y,k = mark
    template = ref[y-k:y+k+1,x-k:x+k+1]
    result = []
    for frame in frames:
        n = 7
        region = frame[y-k-n:y+k+n+1,x-k-n:x+k+n+1]
        score = cv2.matchTemplate(region,template,cv2.TM_CCOEFF_NORMED)
        _, peak, _, (u,v) = cv2.minMaxLoc(score)
        if peak < .8 or min(u,v)<=0 or max(u,v)>=2*n:
            result.append([np.nan,np.nan])
            continue
        def refine(a,b,c):
            denom = a-2*b+c
            return .5*(a-c)/denom if denom else 0.
        result.append([u-n+refine(*score[v,u-1:u+2]),v-n+refine(*score[v-1:v+2,u])])
    return np.array(result)

fig, axs = plt.subplots(4,1,figsize=(9,8),sharex=True)
records=[]
for side,marks_here in enumerate(marks):
    front,rear,dock = [displacements(m) for m in marks_here]
    center = (front+rear)/2-dock
    baseline = np.array(marks_here[0][:2])-marks_here[1][:2]
    axis = front-rear+baseline
    angle = np.rad2deg(np.arctan2(baseline[0]*axis[:,1]-baseline[1]*axis[:,0],axis@baseline))
    direction = 1 if side==0 else -1
    axs[0].plot(vt,center[:,0]*direction,label=f"camera {'AB'[side]} toward lip")
    axs[1].plot(vt,-angle*direction,label=f"camera {'AB'[side]} nose up")
    records.extend(zip([side]*len(vt),vt,center[:,0],center[:,1],angle,dock[:,0],dock[:,1]))
    for lo,hi in [(2.7,3.3),(3.9,4.3)]:
        m=(vt>=lo)&(vt<=hi)
        print(side,lo, "center, angle",np.nanmedian(center[m],axis=0),np.nanmedian(angle[m]),
              "relative median absolute deviation",np.nanmedian(abs(center[m]-np.nanmedian(center[m],axis=0)),axis=0))
axs[0].set_ylabel("Rim midpoint\nshift (pixels)")
axs[1].set_ylabel("Projected hub-axis\nrotation (°)")
axs[2].plot(t,col("lift"),label="lift (mm)")
axs[2].plot(t,np.rad2deg(col("head")),label="head (°)")
axs[2].set_ylabel("Lift and head")
axs[3].step(t,contact.astype(int),where="post",label="on charger")
axs[3].plot(t,(abs(col("left"))+abs(col("right")))>0,label="either wheel moving",alpha=.7)
axs[3].set_ylabel("SDK flags"); axs[3].set_xlabel("Seconds since undock command")
for ax in axs:
    ax.set_xlim(2.7,8.65)
    ax.axvspan(3.716784,3.831568,alpha=.17,color="grey")
    ax.grid(alpha=.2)
    ax.legend(loc="upper left",ncol=2,fontsize=8)
fig.tight_layout();fig.savefig(HERE/"r11-contact.png",dpi=150)
with (HERE/"r11-optical.csv").open("w",newline="") as f:
    w=csv.writer(f); w.writerow(["camera_index","sec","center_x_px","center_y_px","hub_axis_image_deg","dock_x_px","dock_y_px"]);w.writerows(records)
