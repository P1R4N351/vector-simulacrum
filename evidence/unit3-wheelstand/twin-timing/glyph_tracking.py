"""Measure apparent motion of a stationary face glyph as cube illumination changes."""
from pathlib import Path
import csv
import subprocess
import sys

import cv2
import matplotlib.pyplot as plt
import numpy as np

from shelf_view import ShelfView

here = Path(__file__).resolve().parent
windows = [(527.80,528.15),(528.53,528.80),(529.02,529.38),(529.77,530.27),
           (530.57,531.06),(531.48,531.72),(532.03,532.36)]


def measure(folder, serial, picks, ax):
    path = folder/(serial+'.nut')
    ts = np.fromstring(subprocess.check_output(['ffprobe','-v','error','-show_entries',
        'packet=pts_time','-of','csv=p=0',str(path)],text=True),sep='\n')
    cap = cv2.VideoCapture(str(path))
    images = []
    for _ in ts:
        ok,frame = cap.read()
        assert ok
        if not images:
            view = ShelfView(serial,frame)
        images.append(cv2.cvtColor(view.apply(frame),cv2.COLOR_BGR2GRAY))
    cap.release()
    baseline = images[np.argmin(abs(ts-528))]
    points = np.float32(view.points(picks)).reshape(-1,1,2)
    term = (cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,30,.01)
    points = cv2.cornerSubPix(baseline,points,(4,4),(-1,-1),term)
    rows = []
    for index,(lo,hi) in enumerate(windows):
        for i in np.where((ts>=lo)&(ts<=hi))[0]:
            nxt,status,_ = cv2.calcOpticalFlowPyrLK(baseline,images[i],points,None,winSize=(15,15),maxLevel=1)
            back,status2,_ = cv2.calcOpticalFlowPyrLK(images[i],baseline,nxt,None,winSize=(15,15),maxLevel=1)
            for j,(p,q,r) in enumerate(zip(points[:,0],nxt[:,0],back[:,0])):
                fb=np.linalg.norm(r-p)
                rows.append([serial,index,i,j,float(q[0]-p[0]),float(q[1]-p[1]),float(fb),
                             int(bool(status[j,0] and status2[j,0] and fb<=1))])
    x,y = np.floor(points.min(axis=0)[0]-15).astype(int)
    X,Y = np.ceil(points.max(axis=0)[0]+15).astype(int)
    ax.imshow(baseline[y:Y,x:X],cmap='gray',vmin=0,vmax=255)
    ax.scatter(points[:,0,0]-x,points[:,0,1]-y,s=50,marker='+',color='cyan')
    ax.axis('off')
    return rows


if __name__=='__main__':
    fig,axes=plt.subplots(2,2,figsize=(8,5),layout='constrained')
    data=[]
    for col,(serial,picks) in enumerate([
        ('222AD609',[(679,534),(726,534),(726,572),(679,572)]),
        ('3815D409',[(673,214),(711,214),(712,249),(673,248)])]):
        rows=measure(Path(sys.argv[1]),serial,picks,axes[0,col])
        axes[0,col].set_title('Camera '+'AB'[col])
        a=np.array([r[1:] for r in rows])
        for k in range(7):
            b=a[a[:,0]==k]
            val=np.hypot(b[:,3],b[:,4])[b[:,-1]==1]
            if len(val):
                axes[1,col].scatter(np.full(len(val),k),val,s=7,alpha=.4)
            print(serial,k,len(val),len(b),np.round(np.percentile(val,[50,95]),2) if len(val) else 'no valid points')
        axes[1,col].set(xticks=range(7),xticklabels=['off','on','off','on','off','on','off'],ylabel='Apparent shift (pixels)')
        data.extend(rows)
    fig.savefig(here/'glyph-tracking.png',dpi=160)
    with (here/'glyph-tracking.csv').open('w') as file:
        w=csv.writer(file,lineterminator='\n')
        w.writerow(['camera','window','frame','corner','dx_px','dy_px','round_trip_px','accepted'])
        w.writerows(data)
