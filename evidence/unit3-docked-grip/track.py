"""Track each face glyph and a near-side wheel hub in the published review clip."""
import csv
from pathlib import Path

import cv2
import numpy as np
import matplotlib.pyplot as plt

here=Path(__file__).resolve().parent
video=cv2.VideoCapture(str(here/'docked-contact.mp4'))
fps=video.get(cv2.CAP_PROP_FPS)
areas={'A glyph':(332,266,63,62),'A hub':(191,274,56,57),
       'B glyph':(552,214,65,59),'B hub':(701,214,57,49)}
tracks=[]
while True:
    ok,frame=video.read()
    if not ok: break
    gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    if not tracks:
        ref=gray.copy()
    row=[]
    for x,y,w,h in areas.values():
        pad=40
        search=gray[y-pad:y+h+pad,x-pad:x+w+pad]
        scores=cv2.matchTemplate(search,ref[y:y+h,x:x+w],cv2.TM_CCOEFF_NORMED)
        _,score,_,(cx,cy)=cv2.minMaxLoc(scores)
        assert 0<cx<2*pad and 0<cy<2*pad
        def vertex(a,b,c):
            return .5*(a-c)/(a-2*b+c) if a-2*b+c else 0
        row.extend((cx-pad+vertex(*scores[cy,cx-1:cx+2]),cy-pad+vertex(*scores[cy-1:cy+2,cx]),score))
    tracks.append(row)
video.release()
t=np.arange(len(tracks))/fps-.5
v=np.array(tracks)
with (here/'glyph-track.csv').open('w') as stream:
    writer=csv.writer(stream)
    writer.writerow(['seconds']+[key+' '+m for key in areas for m in ['dx_px','dy_px','score']])
    writer.writerows(np.column_stack([t,v]))
fig,ax=plt.subplots(3,1,sharex=True,figsize=(8,6),layout='constrained')
for i,view in enumerate('AB'):
    up=v[:,6*i+4]-v[:,6*i+1]
    up-=np.median(up[t<0])
    ax[0].plot(t,up,'.-',ms=3,label=view)
    print(view,'peak/time',up.max(),t[np.argmax(up)],'final',np.median(up[t>2]),'rest span',np.ptp(up[t<0]),'min glyph score',v[:,6*i+2].min())
ax[0].set_ylabel('Glyph rise relative\nto hub (px)');ax[0].legend()
enc=np.genfromtxt(here/'encoder.csv',delimiter=',',names=True)
ax[1].plot(enc['seconds'],enc['lift_mm'],'.-',ms=3);ax[1].set_ylabel('Lift encoder (mm)')
cube=np.genfromtxt(here/'cube.csv',delimiter=',',names=True)
g=np.column_stack([cube[k] for k in ('x_counts','y_counts','z_counts')])/8192
delta=np.linalg.norm(g-np.median(g[cube['packet_receipt_s']<0],axis=0),axis=1)
ax[2].scatter(cube['packet_receipt_s'],delta,s=5)
ax[2].set(ylabel='Cube acceleration\nchange (g)',xlabel='Seconds from 25% raise request',xlim=(-.5,2.4))
for axis in ax:axis.axvline(1.518,color='gray',ls=':',lw=1)
fig.savefig(here/'motion-overlay.png',dpi=160)
print('shock receipt',cube['packet_receipt_s'][np.argmax(delta)],'encoder peak first',enc['seconds'][np.argmax(enc['lift_mm'])])
