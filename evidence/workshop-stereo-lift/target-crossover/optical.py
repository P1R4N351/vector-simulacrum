"""Track each fork post relative to the middle camera frame of each physical pause."""
import csv
import cv2
import numpy as np
from analyze import read, score

states=[r for r in read('sdk.csv') if r['event']=='state']
t,h,f=(np.array([float(r[k]) for r in states]) for k in ('relative_s','lift_mm','in_position'))
commands=[r for r in read('sdk.csv') if r['event']=='command' and r['target'] in ('48','56')]
cam=np.array([float(r['relative_s']) for r in read('camera.csv')])
cap=cv2.VideoCapture('lift.mp4')
frames=[]
while True:
    ok,im=cap.read()
    if not ok:
        break
    frames.append(cv2.cvtColor(im,cv2.COLOR_BGR2GRAY))
cap.release()
assert len(frames)==len(cam)
with open('optical.csv','w') as stream:
    w=csv.writer(stream,lineterminator='\n')
    w.writerow(('trial','eye','pause_frames','range_during_pause_px','descent_to_minimum_px','min_correlation'))
    for n,r in enumerate(commands,1):
        start=float(r['relative_s'])
        _,end,pause=score(t,h,f,start,float(r['target']))
        if pause is None:
            continue
        _,i,j=pause
        indices=np.flatnonzero((cam>=t[i])&(cam<=t[j]))
        k=j+np.argmin(h[j:end+1])
        trough=int(np.argmin(abs(cam-t[k])))
        ref=frames[indices[len(indices)//2]]
        for x,eye in ((209,'left'),(295,'right')):
            patch=ref[46:67,x-12:x+13]
            values=[]
            peaks=[]
            for p in [*indices,trough]:
                match=cv2.matchTemplate(frames[p][35:102,x-20:x+21],patch,cv2.TM_CCOEFF_NORMED)
                _,peak,_,(u,v)=cv2.minMaxLoc(match)
                peaks.append(peak)
                if not 0<v<46 or u in (0,16) or peak<.85:
                    values.append(np.nan)
                else:
                    a,b,c=match[v-1:v+2,u]
                    values.append(v-11+.5*(a-c)/(a-2*b+c))
            w.writerow((n,eye,len(indices),np.ptp(values[:-1]),values[-1]-np.median(values[:-1]),min(peaks)))
