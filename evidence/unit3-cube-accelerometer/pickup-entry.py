"""Track the last level-looking glyph across two consecutive distinct overhead exposures."""
from pathlib import Path
import csv

import cv2
import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
cap = cv2.VideoCapture(str(here / "pickup.mp4"))
frames = []
for n in [30, 32, 34, 38]:
    cap.set(cv2.CAP_PROP_POS_FRAMES, n)
    ok, frame = cap.read()
    assert ok
    frames.append(frame)
cap.release()
gray = [cv2.cvtColor(f, cv2.COLOR_BGR2GRAY) for f in frames]
# Outer white marker corners, in top-left, top-right, bottom-right, bottom-left order.
border = np.float32([[80,270],[119,290],[92,323],[55,298]])
dest = np.float32([[0,0],[23,0],[23,23],[0,23]])
hom = cv2.getPerspectiveTransform(border, dest)
mask = np.zeros_like(gray[0])
cv2.fillConvexPoly(mask, np.int32(border), 255)
points = cv2.goodFeaturesToTrack(gray[0], 40, .01, 3, mask=mask)
tracks = []
for win in [9,15,21]:
    cfg = dict(winSize=(win,win),maxLevel=1)
    nxt, ok, _ = cv2.calcOpticalFlowPyrLK(gray[0],gray[1],points,None,**cfg)
    back, rev, _ = cv2.calcOpticalFlowPyrLK(gray[1],gray[0],nxt,None,**cfg)
    good = (ok.ravel()>0)&(rev.ravel()>0)&(np.linalg.norm(back[:,0]-points[:,0],axis=1)<.5)
    def plane(q):
        return cv2.perspectiveTransform(q[good],hom)[:,0]
    delta = plane(nxt)-plane(points)
    shift = np.median(delta,axis=0)
    tracks.append([win,len(delta),np.linalg.norm(shift),np.median(np.linalg.norm(delta-shift,axis=1))])
with (here/'pickup-entry.csv').open('w') as file:
    w=csv.writer(file,lineterminator='\n')
    w.writerow(['flow_window_px','tracked_features','apparent_planar_shift_mm','median_residual_mm'])
    w.writerows(tracks)
path=np.genfromtxt(here/'pickup-path.csv',delimiter=',',names=True)
sim=np.genfromtxt(here/'free-pickup.csv',delimiter=',',names=True,dtype=None,encoding='utf8')
sim=sim[sim['initialization']=='Last sighting + pitch']
times=[2.86,3.11,3.38,3.64]
fig,axes=plt.subplots(1,4,figsize=(11.2,3.4),layout='constrained')
for i,(ax,f,t) in enumerate(zip(axes,frames,times)):
    if i==0:
        cv2.polylines(f,[border.astype('int32')],True,(0,200,255),1)
    ax.imshow(cv2.cvtColor(f[145:370,0:235],cv2.COLOR_BGR2RGB))
    lift=np.interp(t,path['time_s'],path['lift_mm'])
    tilt=np.interp(t,sim['time_s'],sim['tilt_deg'])
    ax.set_title(f'+{t:.2f} s\nWrist {lift:.1f} mm · sim tilt {tilt:.1f}°',fontsize=10)
    ax.axis('off')
fig.savefig(here/'pickup-entry.png',dpi=160)
print(tracks)
for k in ('cube_dx_mm','cube_dy_mm','tilt_deg'):
    print(k,np.interp(times,sim['time_s'],sim[k]))
print('wrist',np.interp(times,path['time_s'],path['lift_mm']))
