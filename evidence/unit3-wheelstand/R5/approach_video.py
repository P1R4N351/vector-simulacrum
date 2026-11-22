"""Track the shelf-view glyph using its picked frame 85 to before pickup."""
import csv
import json
import subprocess
import sys
from pathlib import Path
import cv2
import matplotlib.pyplot as plt
import numpy as np
from pickup_onset import action_time, read

here = Path(__file__).resolve().parent
start = action_time(here/"actions.csv","pickup for inward transport","sec")
video = here/"carry.mp4"
probe = subprocess.check_output(["ffprobe","-v","error","-select_streams","v:0","-show_entries",
                                 "frame=best_effort_timestamp_time","-of","json",str(video)])
t = np.array([float(row["best_effort_timestamp_time"]) for row in json.loads(probe)["frames"]])
t += read(here/"video-registration.csv")["offset_s"][0] - start
K = np.array([[340.,0,209.5],[0,340.,119.5],[0,0,1]])
raw_k = np.array([[249.2,0,480],[0,249.2,270],[0,0,1]])
rot = cv2.Rodrigues(np.radians([24.,0,0]))[0] @ cv2.Rodrigues(np.radians([0,49.,0]))[0]
maps = cv2.fisheye.initUndistortRectifyMap(raw_k,np.zeros(4),rot,K,(420,240),cv2.CV_32FC1)
cap = cv2.VideoCapture(str(video))
imgs = []
for i in range(86):
    ok, frame = cap.read()
    assert ok
    rect = cv2.bilateralFilter(cv2.remap(frame,*maps,cv2.INTER_LINEAR),5,18,18)
    imgs.append(cv2.addWeighted(rect,1.25,cv2.GaussianBlur(rect,(0,0),1),-.25,0))
cap.release()
gray = [cv2.cvtColor(im,cv2.COLOR_BGR2GRAY) for im in imgs]
g = read(here/"glyph-raise.csv")[0]
points = np.array([[g[f"x{i}"],g[f"y{i}"]] for i in range(1,5)],np.float32)
# Register each frame directly to the picked glyph, initializing on the central marking.
unit = np.array([[-.5,-.5,0],[.5,-.5,0],[.5,.5,0],[-.5,.5,0]])
rows, tiles = [], []
indices = np.flatnonzero(t[:86] >= -.25)
for i in indices:
    source = 85
    match = cv2.matchTemplate(gray[i][125:183,75:155],gray[source][146:161,99:118],cv2.TM_CCOEFF_NORMED)
    _,_,_,loc = cv2.minMaxLoc(match)
    shift = np.array(loc,np.float32)+[75-99,125-146]
    guess = (points+shift).astype(np.float32)
    args = dict(winSize=(9,9), maxLevel=0, flags=cv2.OPTFLOW_USE_INITIAL_FLOW)
    q,valid,_ = cv2.calcOpticalFlowPyrLK(gray[source],gray[i],points,guess,**args)
    p,back,_ = cv2.calcOpticalFlowPyrLK(gray[i],gray[source],q,points.copy(),**args)
    fb = np.linalg.norm(p-points,axis=1).max()
    assert valid.all() and back.all() and fb<1, (i,fb)
    ok,rv,tv = cv2.solvePnP(unit,q.astype(float),K,None)
    assert ok and tv[2]>0
    n = cv2.Rodrigues(rv)[0][:,2]
    rows.append((i,t[i],fb,q,tv.ravel(),n))
    if i in [indices[0], 67,72,78,81,85]:
        tile=imgs[i].copy()
        cv2.polylines(tile,[q.round().astype(np.int32)],True,(0,220,255),1,cv2.LINE_AA)
        tile=tile[90:215,45:225]
        cv2.putText(tile,f"{t[i]:+.2f} s",(4,17),0,.45,(255,255,255),1,cv2.LINE_AA)
        tiles.append(tile)
cv2.imwrite(str(here/"approach-video.jpg"),np.vstack([np.hstack(tiles[:3]),np.hstack(tiles[3:])]))
# Median precommand reference, assumed resting with its top normal vertical.
pre=[row for row in rows if row[1]<0]
base_t,base_n=[np.median([row[k] for row in pre],axis=0) for k in [4,5]]
base_n/=np.linalg.norm(base_n)
scores=[]
for i,time,fb,q,tv,n in rows:
    projected=[]
    for width in (20,23):
        delta = width*(tv-base_t)+22*(n-base_n)
        projected.append(np.linalg.norm(delta-base_n*(base_n@delta)))
    scores.append([i,time,fb,*projected,np.rad2deg(np.arccos(np.clip(base_n@n,-1,1)))])
a=np.array(scores)
fig,ax=plt.subplots(figsize=(8,4.5),layout="constrained")
ax.fill_between(a[:,1],a[:,3],a[:,4],alpha=.45,label="Shelf camera · 20–23 mm glyph")
sdk=read(here/"sightings.csv")
base=sdk[sdk["sec"]<0][-1]
after=sdk[sdk["sec"]>=base["sec"]]
ax.scatter(after["sec"],np.hypot(after["x"]-base["x"],after["y"]-base["y"]),c="k",label="Onboard sighting")
for label,path in zip(("Pose-only replay","Measured tool velocity"),sys.argv[1:],strict=True):
    d=read(Path(path)/"pickup-replay.csv")
    win=(d["sec"]>=a[0,1])&(d["sec"]<=a[-1,1]+.01)
    ax.plot(d["sec"][win],np.hypot(d["dx_mm"],d["dy_mm"])[win],label=label)
ax.set(xlabel="Seconds after pickup command",ylabel="Planar distance from initial cube center (mm)")
ax.legend(fontsize=8)
fig.savefig(here/"approach-video.png",dpi=160)
with (here/"approach-video.csv").open("w") as stream:
    w=csv.writer(stream,lineterminator="\n")
    w.writerow("frame sec max_roundtrip_px center_distance_20mm center_distance_23mm normal_change_deg".split())
    w.writerows(scores)
