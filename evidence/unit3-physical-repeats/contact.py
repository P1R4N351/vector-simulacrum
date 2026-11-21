"""Pair the actual incidental cube motion with stock events and overhead images."""
from pathlib import Path
import csv
import gzip
import cv2
import numpy as np
import matplotlib.pyplot as plt

here=Path(__file__).resolve().parent
with gzip.open(here/'A/cube.csv.gz','rt') as f:
    a=np.genfromtxt(f,delimiter=',',names=True)
t=a['receipt_s']
g=np.stack([a[k] for k in ('x_counts','y_counts','z_counts')]).T/8192
events=list(csv.DictReader((here/'A/object-events.csv').open()))
cap=cv2.VideoCapture(str(here/'A/overhead.mp4'))
fig=plt.figure(figsize=(11,6),layout='constrained')
axes=fig.subplot_mosaic([['a','b','c','d'],['acc','acc','acc','acc']])
for key,sec in zip("abcd",[60,62.6,64,66]):
    cap.set(cv2.CAP_PROP_POS_MSEC,sec*1000);ok,frame=cap.read()
    assert ok
    axes[key].imshow(cv2.cvtColor(frame,cv2.COLOR_BGR2RGB))
    axes[key].axis('off')
cap.release()
ax=axes['acc']
ax.plot(t,np.linalg.norm(g,axis=1),color='#243d60',lw=.85)
ax.set(xlim=(59,69),xlabel='Seconds after undocking began',ylabel='Cube specific force (g)')
seen=set()
for e in events:
    tm=float(e["receipt_s"])
    if not 59<tm<69:continue
    kind=e["event"]
    ax.axvline(tm,color={'object_moved':'#166749','object_tapped':'#9b5c10','object_stopped_moving':'#754b87'}[kind],
               ls=':',label=kind if kind not in seen else None)
    seen.add(kind)
ax.legend(fontsize=8)
fig.suptitle('Physical cube displacement during the return to dock')
fig.savefig(here/'A/contact.png',dpi=150)
