"""Check physical RGB pitch against the two vertical edges of the dock marker."""
import argparse
import csv
import json
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
cal = json.loads((here.parents[1]/"web/camera_calibration_vector2.json").read_text())
K = np.array([[cal["fx_px"],0,cal["cx_px"]],[0,cal["fy_px"],cal["cy_px"]],[0,0,1]])
square = np.array([[-1,-1,0],[1,-1,0],[1,1,0],[-1,1,0]],float)
source = here/"dock-pitch-picks.csv"
# Sharp frames with the bright house glyph near image center in the W17 observer.
ids = [78,79,95,97,99,100]


def corners(gray, threshold):
    contours,_ = cv2.findContours((gray>threshold).astype(np.uint8),cv2.RETR_LIST,cv2.CHAIN_APPROX_NONE)
    valid = []
    for c in contours:
        area = cv2.contourArea(c)
        x,y,w,h = cv2.boundingRect(c)
        if not (3500<area<11000 and 50<w<100 and 80<h<130 and 320<x<500):
            continue
        poly = cv2.approxPolyDP(c,0.05*cv2.arcLength(c,True),True)
        if len(poly)!=4:
            continue
        quad = poly[:,0].astype(float)
        quad = quad[np.argsort(np.arctan2(*(quad-quad.mean(axis=0))[:,::-1].T))]
        quad = np.roll(quad,-np.argmin(quad.sum(axis=1)),axis=0)
        lines = []
        for a,b in zip(quad,np.roll(quad,-1,axis=0)):
            u = (b-a)/np.linalg.norm(b-a)
            along = (c[:,0]-a)@u
            n = np.array([-u[1],u[0]])
            pts = c[(along>0.20*np.linalg.norm(b-a)) & (along<0.80*np.linalg.norm(b-a)) & (abs((c[:,0]-a)@n)<8)]
            if len(pts)<12:
                break
            vx,vy,cx,cy = cv2.fitLine(pts,cv2.DIST_L1,0,.01,.01).ravel()
            lines.append([-vy,vx,vy*cx-vx*cy])
        if len(lines)==4:
            cross=np.cross(np.roll(lines,1,axis=0),lines)
            valid.append((area,cross[:,:2]/cross[:,2,None]))
    if valid:
        return max(valid,key=lambda v:v[0])[1]


def extract(folder):
    samples=[json.loads(p.read_text()) for p in sorted(folder.glob("*.json"),key=lambda p:int(p.stem))]
    with (here/"dock-pitch-states.csv").open("w") as f:
        w=csv.writer(f,lineterminator="\n")
        w.writerow(["frame","stamp","head_deg","body_pitch_deg"])
        for i,s in enumerate(samples):
            w.writerow([i,s["stamp"],*np.degrees([s["head"],s["pitch"]])])
    with source.open("w") as f:
        w=csv.writer(f,lineterminator="\n")
        w.writerow(["frame","stamp","threshold","head_deg","body_pitch_deg","corner","u","v"])
        for i in ids:
            s=json.loads((folder/f"{i}.json").read_text())
            gray=cv2.GaussianBlur(cv2.imread(str(folder/f"{i}.jpg"),0),(3,3),0)
            for th in (65,75,85,95):
                q=corners(gray,th)
                if q is not None:
                    for j,uv in enumerate(q):
                        w.writerow([i,s["stamp"],th,*np.degrees([s["head"],s["pitch"]]),j,*uv])


parser=argparse.ArgumentParser()
parser.add_argument("--extract",type=Path)
args=parser.parse_args()
if args.extract:
    extract(args.extract)
data=np.genfromtxt(source,delimiter=",",names=True)
states=np.genfromtxt(here/"dock-pitch-states.csv",delimiter=",",names=True)
fig, ax=plt.subplots(figsize=(7,4),layout="constrained")
for i in ids:
    group=data[data["frame"]==i]
    poses=[]
    for th in np.unique(group["threshold"]):
        g=group[group["threshold"]==th]
        uv=np.column_stack((g["u"],g["v"]))
        ok,r,_=cv2.solvePnP(square,uv,K,None)
        assert ok
        # Second marker axis points down. Its dot with optical forward is
        # positive when the camera looks down.
        poses.append(-np.degrees(np.arcsin(cv2.Rodrigues(r)[0][2,1])))
    if not poses:
        continue
    h,body=g["head_deg"][0],g["body_pitch_deg"][0]
    base=h+body-4
    obs=np.median(poses)
    print(i,len(poses),*[round(v,2) for v in (h,body,obs,min(poses),max(poses),base,base+cal['rgb_pitch_up_deg'])])
    window=states[i-2:i+1]
    if np.ptp(window["head_deg"])>0.25:
        continue
    ax.plot([i,i],[min(poses),max(poses)],color='black')
    for label,y,style in [('Dock marker',obs,'ko'),('Firmware pose',base,'r_'),('RGB correction',base+cal['rgb_pitch_up_deg'],'b_')]:
        ax.plot(i,y,style,ms=10,label=label if i==ids[0] else None)
ax.set(xlabel="Observer frame",ylabel="Camera pitch above horizontal (degrees)")
ax.legend()
fig.savefig(here/"dock-pitch.png",dpi=160)
