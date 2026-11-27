"""Compare local optical fork motion during and after near-target encoder plateaus."""
import csv
from pathlib import Path
import sys
import cv2
import numpy as np
import matplotlib.pyplot as plt

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'crossings'))
from crossing import rows, displacement


def summarize(folder):
    sdk = rows(folder/'sdk.csv')
    states = [r for r in sdk if r['event']=='state']
    t,h,flag = (np.array([float(r[k]) for r in states]) for k in ('relative_s','lift_mm','in_position'))
    cam = np.array([float(r['relative_s']) for r in rows(folder/'camera.csv')])
    v = cv2.VideoCapture(str(folder/'lift.mp4'))
    frames = []
    while True:
        ok,im = v.read()
        if not ok:
            break
        frames.append(cv2.cvtColor(im,cv2.COLOR_BGR2GRAY))
    v.release()
    assert len(frames)==len(cam)
    commands = [float(r['relative_s']) for r in sdk if r['event']=='command' and r['target']=='52']
    for n,start in enumerate(commands,1):
        first = np.flatnonzero((t>=start)&(flag==0))[0]
        ready = first+np.flatnonzero(flag[first:]==1)[0]
        trough = first+np.argmin(h[first:ready+1])
        ref = frames[np.argmin(abs(cam-t[ready]-.2))]
        # Longest observed constant-height interval 1–6 mm above target, before trough.
        spans = []
        i = first
        while i<trough:
            j = i
            while j<trough and abs(h[j+1]-h[i])<.05:
                j+=1
            if 53<=h[i]<=58 and j-i>=2:
                spans.append((t[j]-t[i],i,j))
            i=j+1
        row = dict(session=folder.name,lower=n,recovery_mm=h[ready]-h[trough],
                   pause_s=np.nan,pause_mm=np.nan,drop_mm=np.nan,frames=0,
                   left_range_px=np.nan,right_range_px=np.nan,left_drop_px=np.nan,right_drop_px=np.nan)
        if spans:
            duration,i,j=max(spans)
            idx=np.flatnonzero((cam>=t[i])&(cam<=t[j]))
            row.update(pause_s=duration,pause_mm=h[i],drop_mm=h[i]-h[trough],frames=len(idx))
            k=np.argmin(abs(cam-t[trough]))
            for x,eye in ((209,'left'),(295,'right')):
                y=np.array([displacement(ref,frames[p],x) for p in idx])
                if len(y)>=2 and np.all(np.isfinite(y)):
                    row[eye+'_range_px']=np.ptp(y)
                    row[eye+'_drop_px']=displacement(ref,frames[k],x)-np.median(y)
        yield row


data=[r for name in ('recovery','height-crossover','speed-crossover') for r in summarize(Path('..')/name)]
with open('pauses.csv','w') as f:
    writer=csv.DictWriter(f,data[0],lineterminator='\n')
    writer.writeheader()
    writer.writerows(data)
fig, axes=plt.subplots(1,2,figsize=(10,4),sharey=True,layout='constrained')
for ax,eye in zip(axes,('left','right')):
    for n,r in enumerate(data,1):
        a,b=r[eye+'_range_px'],r[eye+'_drop_px']
        if np.isfinite([a,b]).all():
            ax.plot([n,n],[a,b],color='0.75')
            ax.scatter(n,a,c='tab:blue',marker='o',label='Range during pause' if n==1 else None)
            ax.scatter(n,b,c='tab:orange',marker='^',label='Drop to encoder minimum' if n==1 else None)
    ax.set(title=eye.capitalize()+' camera',xlabel='Trial in recording order')
axes[0].set_ylabel('Vertical fork displacement (px)')
axes[0].legend()
fig.savefig('pauses.png',dpi=160)
