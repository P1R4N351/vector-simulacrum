"""Measure four front aperture corners in the glyph plane of Sat's cube photo."""
import sys
import csv
from pathlib import Path
import cv2
import numpy as np
import matplotlib.pyplot as plt

here=Path(__file__).resolve().parent
im=cv2.imread(sys.argv[1])
assert im.shape[:2]==(1440,1920)
# Work at 640px for the hand landmarks. Fit the four outer WHITE glyph lines
# from their long straight sections on the full-resolution source.
gray=cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
edges=cv2.Canny(gray,50,105)
def fit(points,normal):
    samples=[]
    for p in np.linspace(*points,81):
        probes=np.rint(3*p+np.arange(-9,10)[:,None]*normal).astype(int)
        hits=np.flatnonzero(edges[probes[:,1],probes[:,0]])
        if len(hits):
            samples.append(probes[hits[np.argmin(abs(hits-9))]])
    if len(samples)<30:
        raise ValueError('Insufficient glyph edge coverage')
    vx,vy,x,y=cv2.fitLine(np.array(samples),cv2.DIST_HUBER,0,.01,.01).ravel()
    return np.array([-vy,vx,vy*x-vx*y])
lines=[fit(np.array(p),np.array(n)) for p,n in [
    (((168,185),(288,197)),(0,1)),(((321,230),(324,357)),(1,0)),
    (((180,369),(288,388)),(0,1)),(((146,215),(152,335)),(1,0))]]
def intersection(a,b):
    v=np.cross(a,b)
    return v[:2]/v[2]
corners=np.float32([intersection(lines[i-1],line) for i,line in enumerate(lines)])
metric=np.float32([[-11.5,11.5],[11.5,11.5],[11.5,-11.5],[-11.5,-11.5]])
H=cv2.getPerspectiveTransform(corners,metric)
def transform(p,H):
    return cv2.perspectiveTransform(np.float32(p).reshape(-1,1,2),H)[:,0]
picks=3*np.array([[130,199],[138,341],[346,223],[345,377]])
pos=transform(picks,H)
fig,ax=plt.subplots(figsize=(7,5),layout='constrained')
ax.imshow(cv2.cvtColor(im,cv2.COLOR_BGR2RGB))
ax.plot(*np.vstack((corners,corners[0])).T,color='cyan',lw=1,label='23 mm glyph reference')
ax.scatter(*picks.T,color='orange',s=30,label='Aperture inner corners')
ax.legend();ax.axis('off')
fig.savefig(here/'cube-face-photo.png',dpi=150)
print(pos)
with (here/'cube-face-photo.csv').open('w',newline='') as f:
    w=csv.writer(f,lineterminator='\n')
    w.writerow(('corner','image_x','image_y','face_x_mm','face_z_mm'))
    for name,uv,xz in zip(('left upper','left lower','right upper','right lower'),picks,pos):
        w.writerow((name,*uv,*xz))
