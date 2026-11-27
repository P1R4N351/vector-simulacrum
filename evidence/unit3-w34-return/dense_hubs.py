"""Re-track W34 from either corrected camera with fixed templates in both temporal directions."""
from pathlib import Path
import csv
import sys
import subprocess
import cv2
import numpy as np

here=Path(__file__).resolve().parent
sys.path.insert(0,str(here.parent/'unit3-wheelstand/twin-timing'))
from shelf_view import ShelfView
capture=Path(sys.argv[1])
camera=sys.argv[2] if len(sys.argv)>2 else 'B'
if camera=='A':
    serial='222AD609'
    early,late,first,last,check=265,331,214,331,325
    early_centers=[(417,329),(303,320),(74,340)]
    late_centers=[(254,308),(139,308),(74,340)]
    keyframes=(302,327)
    crop=np.s_[240:392,45:400]
    prefix='opposed'
elif camera=='B':
    serial='3815D409'
    early,late,first,last,check=319,396,258,396,389
    early_centers=[(579,270),(710,267),(930,155)]
    late_centers=[(763,243),(886,234),(930,155)]
    keyframes=(362,392)
    crop=np.s_[140:310,590:965]
    prefix='dense'
else:
    raise ValueError('Choose A or B')
video=capture/(serial+'.nut')
clocks=np.loadtxt(capture/'clocks.csv',delimiter=',',skiprows=1)
pts=np.fromstring(subprocess.check_output(['ffprobe','-v','error','-show_entries','packet=pts_time','-of','csv=p=0',str(video)],text=True),sep='\n')
from datetime import datetime
times=pts+np.median(clocks[:,1]-clocks[:,0])-datetime.fromisoformat('2026-11-23T17:58:52.363+00:00').timestamp()
cap=cv2.VideoCapture(str(video))
frames={}
for i in range(last+1):
    ok,frame=cap.read()
    if not ok: raise RuntimeError('Incomplete capture')
    if i==0:view=ShelfView(serial,frame)
    if i>=first:frames[i]=cv2.cvtColor(view.apply(frame),cv2.COLOR_BGR2GRAY)
cap.release()

def track(indices, centers):
    first=frames[indices[0]]
    result={}
    templates=[]
    for (x,y),rad in zip(centers,[24,16,13]):
        templates.append(first[y-rad:y+rad+1,x-rad:x+rad+1])
    for i in indices:
        values=[]
        for k,(template,(x,y)) in enumerate(zip(templates,centers)):
            h=template.shape[0]//2
            left,top=x-h-12,y-h-9
            roi=frames[i][top:y+h+10,left:x+h+13]
            score=cv2.matchTemplate(roi,template,cv2.TM_CCOEFF_NORMED)
            _,q,_,(u,v)=cv2.minMaxLoc(score)
            centers[k]=(left+u+h,top+v+h)
            values.extend((*centers[k],q))
        result[i]=values
    return result

# Two independent references bracket the peak; the third sparse manual pick is held out.
fw=track(range(early,last+1),early_centers)
bw=track(range(late,first-1,-1),late_centers)
with (here/(prefix+'-hubs.csv')).open('w') as f:
    w=csv.writer(f)
    w.writerow(['frame','t_s','direction']+[f'{part}_{field}' for part in ['front','rear','dock'] for field in ['x','y','corr']])
    for key,track_data in [('forward',fw),('backward',bw)]:
        for i,v in sorted(track_data.items()):w.writerow([i,times[i],key,*v])
print('Check frame',check,fw[check],bw[check])
# Show each independent backward estimate on the corrected frames nearest the SDK peak and flag.
panels=[]
for idx in keyframes:
    im=cv2.cvtColor(frames[idx],cv2.COLOR_GRAY2BGR)
    for k,color in [(0,(0,255,255)),(3,(255,230,20)),(6,(100,230,100))]:
        x,y=bw[idx][k:k+2]
        cv2.drawMarker(im,(x,y),color,cv2.MARKER_CROSS,16,1)
    panel=im[crop].copy()
    cv2.putText(panel,f'{times[idx]:+.3f} s',(8,20),0,.52,(240,240,240),1,cv2.LINE_AA)
    panels.append(panel)
cv2.imwrite(str(here/(prefix+'-landmarks.jpg')),np.vstack(panels))
