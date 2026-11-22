"""Estimate lowest cube corner relative to the shelf implied by the resting glyph."""
from itertools import product
from pathlib import Path
import csv
import cv2
import matplotlib.pyplot as plt
import numpy as np

here=Path(__file__).resolve().parent
corners=np.genfromtxt(here/'landing-corners.csv',delimiter=',',skip_header=1)
marker=np.array([[-1.,-1,0],[1,-1,0],[1,1,0],[-1,1,0]])*11.5
vertices=np.array(list(product((-22.15,22.15),(-22.15,22.15),(-.15,44.15))))
K=np.array([[465.,0,499.5-170],[0,465.,185.5-115],[0,0,1.]])
def poses(q, intrinsics=K):
    _,rs,ts,errs=cv2.solvePnPGeneric(marker,q,intrinsics,None,flags=cv2.SOLVEPNP_IPPE)
    return [(cv2.Rodrigues(r)[0],t.ravel(),err.item()) for r,t,err in zip(rs,ts,errs)]
def low(p,down):
    R,t,_=p
    return max((vertices@R.T+t)@down)

rows=[]
for th in (80,90,100):
    s=corners[corners[:,1]==th]
    base=poses(np.median(s[s[:,0]<0,2:],axis=0).reshape(4,2))[0]
    down=base[0][:,1]
    level=low(base,down)
    for r in s:
        for branch,p in enumerate(poses(r[2:].reshape(4,2))):
            rows.append([*r[:2],branch,p[2],level-low(p,down)])
a=np.array(rows)
with (here/'landing-height.csv').open('w') as f:
    w=csv.writer(f,lineterminator='\n');w.writerow(['t','threshold','ippe_branch','corner_reprojection_rms_px','clearance_mm']);w.writerows(rows)
fig,ax=plt.subplots(figsize=(7,3.8),layout='constrained')
for th in (80,90,100):
    b=a[(a[:,1]==th)&(a[:,2]==0)&(a[:,0]>4.95)&(a[:,0]<5.6)]
    ax.plot(b[:,0],np.where(b[:,3] <= .4,b[:,-1],np.nan),'.-',label=f'Gray threshold {th}')
    for lo,hi in [(-1,0),(5.15,5.23),(6,7)]:
        use=(a[:,0]>lo)&(a[:,0]<hi)&(a[:,1]==th)&(a[:,2]==0)
        print(th,lo,use.sum(),np.percentile(a[use,-1],[10,50,90]) if use.any() else None)
ax.axhline(0,color='.45',ls=':');ax.axhline(8,color='.45',ls='--',label='8 mm release hypothesis')
ax.set(xlabel='Seconds from roll command',ylabel='Inferred lowest corner above rest plane (mm)');ax.legend(fontsize=8)
fig.savefig(here/'landing-height.png',dpi=150)

with (here/'landing-height-sensitivity.csv').open('w') as f:
    w=csv.writer(f,lineterminator='\n')
    w.writerow(['threshold','reference','focal_multiplier','prerelease_median_mm'])
    for th,reference,focal in product((80,90,100),('before','after'),(.95,1.,1.05)):
        s=corners[corners[:,1]==th]
        intrinsics=K.copy()
        intrinsics[0,0]*=focal
        intrinsics[1,1]*=focal
        subset=s[s[:,0]<0] if reference=='before' else s[s[:,0]>6]
        base=poses(np.median(subset[:,2:],axis=0).reshape(4,2),intrinsics)[0]
        down=base[0][:,1]
        values=[low(base,down)-low(poses(r[2:].reshape(4,2),intrinsics)[0],down)
                for r in s if 5.15<r[0]<5.23]
        w.writerow([th,reference,focal,np.median(values)])
