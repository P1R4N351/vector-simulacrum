"""Estimate initial axle heading in the cube frame from two independent corrected views."""
import csv
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np

root=Path(__file__).resolve().parent
video=cv2.VideoCapture(str(root/'docked-contact.mp4'))
ok,frame=video.read()
video.release()
assert ok
# Marker corners TL, TR, BR, BL: approximate intersections of outer white edges.
# Front and rear hub centers withheld from the cube-pose solve.
views={
    'A':([[331,267],[390,267],[395,325],[336,325]],[[355,217],[411,216],[396,234],[338,236]],
         [[222,306],[108,304]],474.5),
    'B':([[558,215],[617,215],[611,272],[551,272]],[[535,165],[593,165],[612,181],[550,182]],
         [[730,245],[849,235]],459.5)}
s=11.5
world=np.array([[-s,-22,22+s],[s,-22,22+s],[s,-22,22-s],[-s,-22,22-s],
                [-s,s,44],[s,s,44],[s,-s,44],[-s,-s,44]],float)
pitch=float(np.interp(0,*np.loadtxt(root/'dock-pitch.csv',delimiter=',',skiprows=1)[:,:2].T))
def solve(uv,hubs,cx,f,extra):
    K=np.array([[f,0,cx],[0,f,185.5],[0,0,1.]])
    ok,r,t=cv2.solvePnP(world,uv,K,None,flags=cv2.SOLVEPNP_SQPNP)
    assert ok
    r,t=cv2.solvePnPRefineLM(world,uv,K,None,r,t)
    R=cv2.Rodrigues(r)[0]
    camera=(-R.T@t).ravel()
    assert camera[2] > 44 and (world@R.T+t.ravel())[:,2].min()>0
    ray=np.c_[hubs,np.ones(2)]@np.linalg.inv(K).T@R
    # Front axle as pitch pivot; rear axle is 48 mm aft, 3.75 mm lower on level ground.
    z=14.5+extra+np.array([0,-48*np.sin(pitch)-3.75*np.cos(pitch)])
    points=camera+ray*((z-camera[2])/ray[:,2])[:,None]
    vec=points[0]-points[1]
    angle=(np.degrees(np.arctan2(vec[1],vec[0]))+90)%180-90
    proj=cv2.projectPoints(world,r,t,K,None)[0][:,0]
    return angle,np.linalg.norm(vec),np.sqrt(np.mean(np.sum((proj-uv)**2,axis=1)))
rows=[]
fig,axes=plt.subplots(1,2,figsize=(10,4),layout='constrained')
rng=np.random.default_rng(42)
for ax,(name,(face,top,hubs,cx)) in zip(axes,views.items()):
    uv=np.array(face+top,float); hubs=np.array(hubs,float)
    ax.imshow(cv2.cvtColor(frame,cv2.COLOR_BGR2RGB))
    for quad in (face,top):
        p=np.array(quad+[quad[0]])
        ax.plot(*p.T,'y.-',lw=1)
    ax.plot(*hubs.T,'c.-')
    ax.set(xlim=(min(hubs[:,0].min(),uv[:,0].min())-25,max(hubs[:,0].max(),uv[:,0].max())+25),
           ylim=(max(hubs[:,1].max(),uv[:,1].max())+25,uv[:,1].min()-25),title=f'Camera {name}')
    ax.axis('off')
    for extra in (0,3,6,12):
        a,base,err=solve(uv,hubs,cx,465,extra)
        samples=np.array([solve(uv+rng.uniform(-2,2,uv.shape),hubs+rng.uniform(-2,2,hubs.shape),
                                cx,rng.uniform(441.75,488.25),extra) for _ in range(500)])
        rows.append([name,extra,a,*np.percentile(samples[:,0],[5,95]),base,err])
fig.savefig(root/'starting-heading.png',dpi=175)
with (root/'starting-heading.csv').open('w') as f:
    w=csv.writer(f);w.writerow('view axle_elevation_mm heading_deg sensitivity_p05_deg sensitivity_p95_deg reconstructed_axle_span_mm marker_reprojection_rms_px'.split())
    w.writerows(rows)
