"""Track the top glyph in run E and compare it with radio tap notifications."""
import csv
import cv2
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path

out = Path(__file__).resolve().parent/'E'
video = cv2.VideoCapture(str(out/'overhead.mp4'))
# Centers were read from the white printed border before and after rotation.
templates = []
for sec,cx,cy in [(50,133,175),(65,124,169),(96,109,167)]:
    video.set(cv2.CAP_PROP_POS_MSEC,sec*1000)
    ok, frame = video.read()
    assert ok
    templates.append(cv2.cvtColor(frame[cy-12:cy+13,cx-21:cx+21],cv2.COLOR_BGR2GRAY))
video.set(cv2.CAP_PROP_POS_FRAMES, 0)
track = []
while True:
    ok, frame = video.read()
    if not ok:
        break
    roi = cv2.cvtColor(frame[115:245,62:215], cv2.COLOR_BGR2GRAY)
    score,(x,y) = max((v[1],v[3]) for v in (cv2.minMaxLoc(cv2.matchTemplate(roi,template,cv2.TM_CCOEFF_NORMED)) for template in templates))
    track.append((len(track)/5,x+62+21,y+115+12,score))
video.release()
t,x,y,score = np.asarray(track).T
np.savetxt(out/'glyph-track.csv',track,delimiter=',',header='video_s,x_px,y_px,template_score',comments='',fmt='%.3f')
good = score>.80
radio = np.genfromtxt(out/'cube.csv.gz',delimiter=',',names=True)
a = np.linalg.norm([radio[k] for k in ('x_counts','y_counts','z_counts')],axis=0)/8192
taps = [float(e['receipt_s']) for e in csv.DictReader((out/'object-events.csv').open()) if e['event']=='object_tapped']
fig,(ax,bx)=plt.subplots(2,1,sharex=True,figsize=(10,5),layout='constrained')
ax.plot(radio['receipt_s'],a,lw=.7)
ax.set_ylabel('Cube acceleration magnitude (g)')
for values,label in ((x,'Image x'),(y,'Image y')):
    bx.plot(t,np.where(good,values-np.median(values[good & (t<40)]),np.nan),label=label,lw=1)
bx.set(xlabel='Seconds since undock command',ylabel='Top glyph shift (pixels)')
bx.legend()
for tap in taps:
    for pane in (ax,bx):
        pane.axvline(tap,color='#9a5614',ls=':',lw=.9)
fig.suptitle('Shelf oval · four tap notifications and overhead glyph motion')
fig.savefig(out/'tap-overlay.png',dpi=150)
print('accepted',good.mean())
for tap in taps:
    print(tap,[(np.median(x[b]),np.median(y[b])) if np.any(b) else None for b in [good & (t>=tap-2)&(t<tap),good & (t>tap)&(t<=tap+2)]])
