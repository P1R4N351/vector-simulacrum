"""Estimate relative camera/SDK receipt timing on the preceding turn; transfer to the cube turn."""
from pathlib import Path
import sys
import csv
import zipfile

import cv2
import numpy as np
import matplotlib.pyplot as plt

out=Path(__file__).resolve().parent/'E'
src=Path(sys.argv[1])
epoch=1795299509.3353744
if src.is_dir():
    with zipfile.ZipFile(out/'chassis-sync-crops.zip','w') as z:
        for p in sorted(src.glob('*.jpg')):
            if 45.6<float(p.stem)-epoch<51.9:
                img=cv2.imread(str(p),cv2.IMREAD_GRAYSCALE)[125:365,230:520]
                ok,b=cv2.imencode('.png',img);assert ok
                z.writestr(p.stem+'.png',b.tobytes())
    src=out/'chassis-sync-crops.zip'
with zipfile.ZipFile(src) as z:
    frames=[(float(Path(n).stem)-epoch,cv2.imdecode(np.frombuffer(z.read(n),np.uint8),0)) for n in sorted(z.namelist())]

def displacement(first,second,mask):
    pts=cv2.goodFeaturesToTrack(first,maxCorners=160,qualityLevel=.004,minDistance=3,mask=mask)
    nxt,ok,_=cv2.calcOpticalFlowPyrLK(first,second,pts,None,winSize=(25,25),maxLevel=3)
    back,ok2,_=cv2.calcOpticalFlowPyrLK(second,first,nxt,None,winSize=(25,25),maxLevel=3)
    keep=ok.ravel().astype(bool)&ok2.ravel().astype(bool)&(np.linalg.norm((pts-back)[:,0],axis=1)<1)
    assert keep.any(), (len(pts),keep.sum())
    return np.percentile(np.linalg.norm((nxt-pts)[keep,0],axis=1),75),keep.sum()

r=np.genfromtxt(out/'robot.csv.gz',delimiter=',',names=True)
t=r['receipt_s'];yaw=np.unwrap(r['yaw_rad'])
fig,axes=plt.subplots(1,2,figsize=(10,3.8),layout='constrained',sharey=True)
rows=[]
for name,xy,color in [('Lower body',(38,130,225,236),'#277359'),('Upper body',(62,43,196,130),'#945736')]:
    mask=np.zeros_like(frames[0][1]);x,y,x2,y2=xy;mask[y:y2,x:x2]=255
    fit=None
    for ax,(a,b),role in zip(axes,[(46.0,47.6),(50.0,51.4)],['Calibration turn','Cube turn']):
        group=[(s,img) for s,img in frames if a-.3<s<b+.3]
        times=np.array([s for s,_ in group])
        motion=[displacement(i,j,mask) for (_,i),(_,j) in zip(group,group[1:])]
        px=np.array([p for p,n in motion])
        # Each frame pair integrates motion over its actual (irregular) receipt interval.
        def predicted(lag):
            return abs(np.diff(np.interp(times+lag,t,yaw)))
        def norm(v):return v/sum(v)
        def error(lag):return np.sqrt(np.mean((norm(px)-norm(predicted(lag)))**2))
        offsets=np.arange(-.35,.351,.005)
        best=offsets[np.argmin([error(s) for s in offsets])]
        if fit is None:fit=best
        mid=(times[1:]+times[:-1])/2
        ax.plot(mid,norm(px),'o-',color=color,label=f'{name} flow')
        ax.plot(mid,norm(predicted(fit)),'--',color=color,label=f'Yaw with {name.lower()} offset')
        if name=='Lower body':ax.plot(mid,norm(predicted(0)),':',color='k',label='Yaw, zero offset')
        rows.append([name,role,len(px),min(n for p,n in motion),fit,best,error(0),error(fit)])
        ax.set(title=role,xlabel='Camera receipt (s)')
axes[0].set_ylabel('Fraction of turn motion per frame interval')
axes[1].legend(fontsize=7)
fig.savefig(out/'chassis-sync.png',dpi=155)
with (out/'chassis-sync.csv').open('w') as f:
    w=csv.writer(f,lineterminator='\n');w.writerow(['region','segment','frame_intervals','minimum_tracks','fit_offset_s','segment_best_s','zero_offset_rmse','transfer_rmse']);w.writerows(rows)
print(rows)
