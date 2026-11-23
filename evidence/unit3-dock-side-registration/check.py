"""Register the dock glyph and reconstruct hub centers in the two corrected side views."""
import csv
from itertools import product
from pathlib import Path

import cv2
import numpy as np
import matplotlib.pyplot as plt

root = Path(__file__).resolve().parent
world = np.array([[-23,0,71.5],[23,0,71.5],[23,0,25.5],[-23,0,25.5],[0,-94,0]],float)
intrinsics = np.array([[465,0,499.5],[0,465,185.5],[0,0,1.]])
pins = np.array([[side,-93.205+59.71,13.671] for side in [-9.85,9.85]])
views = [
    ("A", [[79,180],[166,162],[187,253],[104,292],[345,330]], [[254,308],[139,308]], (40,125,405,404)),
    ("B", [[855,90],[951,100],[921,218],[833,187],[668,273]], [[763,243],[886,234]], (600,60,983,337)),
]
rows=[]
fig,axes=plt.subplots(2,2,figsize=(10,6.6),layout="constrained")
for i,(name,picks,hubs,box) in enumerate(views):
    pix=np.array(picks,float)
    for count in (4,5):
        _,rot,t=cv2.solvePnP(world[:count],pix[:count],intrinsics,None,flags=cv2.SOLVEPNP_SQPNP)
        rot,t=cv2.solvePnPRefineLM(world[:count],pix[:count],intrinsics,None,rot,t)
        R=cv2.Rodrigues(rot)[0]
        camera=(-R.T@t).ravel()
        project=lambda arr: cv2.projectPoints(arr,rot,t,intrinsics,None)[0][:,0]
        error=np.linalg.norm(project(world)[-1]-pix[-1])
        rays=R.T@np.linalg.inv(intrinsics)@np.c_[hubs,np.ones(2)].T
        for side in [27,30,33]:
            xyz=camera[:,None]+rays*((np.sign(camera[0])*side-camera[0])/rays[0])
            span=np.linalg.norm(xyz[:,1]-xyz[:,0])
            rows.append([name,count,side,error,span,93.205+xyz[1,0],*project(pins).ravel()])
        ax=axes[i,0]
        pp=project(pins)
        ax.scatter(pp[:,0],pp[:,1],marker='+' if count==4 else 'x',label='Model pins, glyph only' if count==4 else 'Model pins, glyph + lip',s=90)
    for ax,when in zip(axes[i],['approach','seated']):
        left,top,right,bottom=box
        img=plt.imread(root/f'{name}-{when}.png')
        ax.imshow(img,extent=[left-.5,right-.5,bottom-.5,top-.5])
        ax.set(xlim=(left,right),ylim=(bottom,top),title=f'{name} • {when}')
        ax.axis('off')
    axes[i,0].scatter(*pix.T,s=16,facecolors='none',edgecolors='lime',label='Dock picks')
    axes[i,1].scatter(*np.array(hubs).T,marker='+',c='orange',s=80,label='Visible axle centers')
    for ax in axes[i]:ax.legend(fontsize=7,loc='lower left')
fig.savefig(root/'registration.png',dpi=160)
with (root/'registration.csv').open('w') as f:
    w=csv.writer(f,lineterminator='\n')
    w.writerow('view anchor_count side_plane_mm lip_residual_px axle_span_mm front_axle_depth_mm pin1_x pin1_y pin2_x pin2_y'.split())
    w.writerows(rows)
