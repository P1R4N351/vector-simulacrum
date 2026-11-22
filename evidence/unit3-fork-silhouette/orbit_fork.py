"""Check the near-side fork outline against rim-registered native Flip photos."""
from pathlib import Path
import csv
import json
import sys
import cv2
import matplotlib.pyplot as plt
import numpy as np
from geometry import mesh, HERE

archive = Path(sys.argv[1])
registered = HERE.parent/'shoulder-photo-check/flip'
# Picks in a 720-column display crop, in the order upper crest, lowest visible tab.
picks = {60: [(430,166),(434,564)], 195: [(245,155),(244,526)]}
fig,axes = plt.subplots(1,2,figsize=(9,5),layout='constrained')
records=[]
for ax,(heading,xy) in zip(axes,picks.items()):
    d=json.loads((registered/f'flip-{heading}.json').read_text())
    R,t=np.array(d['R']),np.array(d['t'])
    def project(p):
        q=p@R.T+t
        return q[...,:2]/q[...,2,None]*d['f']+d['pp']
    box=project(np.array([[x,y,z] for x in (12,40) for y in (-28,28) for z in (0,45)]))
    a=np.maximum(0,box.min(0).astype(int)-80)
    b=box.max(0).astype(int)+80
    im=cv2.imread(str(archive/f'S-ring{heading}-main.jpg'))
    roi=im[a[1]:b[1],a[0]:b[0]]
    display=cv2.resize(roi,(720,round(720*roi.shape[0]/roi.shape[1])))
    scale=roi.shape[1]/720
    native=np.array(xy)*scale+a
    ax.imshow((display[:,:,::-1]/255)**.55)
    ax.scatter(*np.array(xy).T,marker='+',color='white')
    side=-1 if heading==60 else 1
    # Recover height in vertical parallel planes spanning the visible fork cheek.
    ray=np.c_[(native-d['pp'])/d['f'],np.ones(2)]@R
    cam=-R.T@t
    estimates=[]
    for y1 in (16,20,24):
        for y2 in (16,20,24):
            distances=(side*np.array([y1,y2])-cam[1])/ray[:,1]
            points=cam+ray*distances[:,None]
            height=points[0,2]-points[1,2]
            estimates.append(height)
            records.append([heading,*native.ravel(),y1,y2,height])
    # The rigid STL side half is drawn at nominal lowered wrist height.
    h=32
    wrist=[-41+np.sqrt(66**2-(h-45)**2),0,h]
    near=mesh[(side*mesh[:,:,1]).min(axis=1)>15]
    polys=np.round((project(near+wrist)-a)/scale).astype(np.int32)
    mask=np.zeros(display.shape[:2],np.uint8)
    for poly in polys:
        cv2.fillConvexPoly(mask,poly,1)
    ax.contour(mask,levels=[.5],colors=['#00ced1'],linewidths=1)
    ax.set_title(f'{heading}° · visible height {min(estimates):.1f}–{max(estimates):.1f} mm')
    ax.axis('off')
    print(heading,min(estimates),max(estimates),np.ptp(near[:,:,2]))
fig.suptitle('Printed fork side outline at 32 mm wrist height · white crosses are visible landmarks')
fig.savefig(HERE/'orbit-fork.png',dpi=165)
with (HERE/'orbit-fork.csv').open('w') as f:
    w=csv.writer(f,lineterminator='\n')
    w.writerow(['heading','upper_u','upper_v','lower_u','lower_v','upper_abs_y_mm','lower_abs_y_mm','height_mm'])
    w.writerows(records)
