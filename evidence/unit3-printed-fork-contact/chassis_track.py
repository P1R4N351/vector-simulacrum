"""Track trial 1's head crown against the fixed dock during the recorded push."""
import csv
import sys
from pathlib import Path
import cv2
import matplotlib.pyplot as plt
import numpy as np
from evaluate import here,states

frames=Path(sys.argv[2])
epoch=1795287608.6804-5.498106956481934
ref=cv2.imread(str(frames/'1795287608.6804.jpg'),0)
# Native-pixel crop of crown vents; narrower strip above includes the crown seam.
boxes=[(335,243,420,281),(335,230,421,263),(376,75,474,143)]
def locate(img,box):
    x,y,r,b=box
    template=ref[y:b,x:r]
    sx,sy=x-65,y-125
    sy=max(sy,0)
    best=-1
    for factor in (np.arange(.68,1.061,.02) if box!=boxes[2] else [1.]):
        patch=cv2.resize(template,None,fx=factor,fy=factor)
        scores=cv2.matchTemplate(img[sy:b+65,sx:r+65],patch,cv2.TM_CCOEFF_NORMED)
        _,peak,_,(j,i)=cv2.minMaxLoc(scores)
        if peak>best:
            best=peak
            delta=np.array([sx+j-x+(patch.shape[1]-template.shape[1])/2,
                            sy+i-y+(patch.shape[0]-template.shape[0])/2])
    return delta,best
r=states[states['trial']==1]
records=[]
for p in sorted(frames.glob('*.jpg')):
    t=float(p.stem)-epoch
    if not -1.5<=t<=11.5:
        continue
    img=cv2.imread(str(p),0)
    (c,cs),(s,ss),(dock,ds)=[locate(img,box) for box in boxes]
    odom,head,lift=[np.interp(t+1.5,r['sec'],r[k]) for k in ('x_mm','head_rad','lift_mm')]
    records.append([t,odom,head,lift,*(c-dock),cs,*(s-dock),ss,*dock,ds])
a=np.array(records)
train=(a[:,0]>=-.5)&(a[:,0]<=3)&(a[:,6]>.75)&(a[:,9]>.75)
fig,ax=plt.subplots(2,1,figsize=(8,6),sharex=True,layout='constrained')
ax[0].plot(a[:,0],a[:,1],label='SDK odometry')
pred=[]
for j,label in [(5,'Crown vents'),(8,'Crown seam')]:
    scale,intercept=np.polyfit(a[train,j],a[train,1],1)
    estimate=a[:,j]*scale+intercept
    estimate[a[:,j+1]<.75]=np.nan
    pred.append(estimate)
    ax[0].plot(a[:,0],estimate,'.-',label=label)
    ax[1].plot(a[:,0],estimate-a[:,1],'.-',label=label)
    print(label,'training n',train.sum(),'scale',scale,'train RMS',np.sqrt(np.mean((estimate[train]-a[train,1])**2)))
ax[0].set_ylabel('Forward position (mm)');ax[0].legend()
ax[1].set(xlabel='Seconds after approach',ylabel='Camera minus SDK (mm)')
for panel in ax: panel.axvspan(-.5,3,color='gray',alpha=.12)
fig.savefig(here/'chassis-track.png',dpi=160)
with (here/'chassis-track.csv').open('w',newline='') as f:
    w=csv.writer(f,lineterminator='\n')
    w.writerow('sec odom_mm head_rad lift_mm vents_dx vents_dy vents_corr seam_dx seam_dy seam_corr dock_dx dock_dy dock_corr vents_forward_mm seam_forward_mm'.split())
    w.writerows(np.column_stack([a,*pred]))
print(np.round(np.column_stack([a[:,[0,1,6,9]],*pred])[(a[:,0]>3)&(a[:,0]<7.5)],2))
